// macro for preprocessor to include/exclude OLED code
#define USE_OLED 1

#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#if USE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Needs the ESP32 Arduino core 3.x (Boards Manager: esp32 by Espressif Systems)."
#endif

// layer 1 - configuration and constants

enum JointId { BASE, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, GRIPPER, NUM_JOINTS };
enum AxisId  { J1X, J1Y, J2X, J2Y, NUM_AXES };
enum Mode    { MODE_ARM, MODE_WRIST, NUM_MODES };

// joint fixed settings
//ArmController keeps track of target and current position
struct JointConfig {
  const char* name;      // 3 letters, used on OLED / Serial / calibration
  uint8_t     pin;
  float       minDeg;    // software limits (find them in calibration mode)
  float       maxDeg;
  float       restDeg;   // safe pose for arming and "go to rest"
  int         cal0Us;    // pulse width at 0 deg  (bench test: m0)
  int         cal180Us;  // pulse width at 180 deg (bench test: m180)
  float       speedDps;  // max speed, degrees per second
  bool        enabled;   // if it's false, then that joint is ignored or not built yet
};

namespace Config {
  // pins
  constexpr uint8_t PIN_J1X = 33, PIN_J1Y = 32, PIN_J2X = 35, PIN_J2Y = 34;   // ADC1 only
  constexpr uint8_t PIN_GRIP_POT = 36;                                        // gpio 36 is VP on the board
  constexpr uint8_t PIN_J1_BTN = 16, PIN_J2_BTN = 17, PIN_ARM_BTN = 4;
  constexpr uint8_t PIN_LED = 2;
  constexpr uint8_t PIN_SDA = 21, PIN_SCL = 22;
  constexpr uint8_t OLED_ADDR = 0x3C;            // some modules use 0x3D
  constexpr uint8_t OLED_W = 128, OLED_H = 64;

  // ---------------- joint table (edit after calibration) ----------------
  // Allocation: MG90S base/shoulder/elbow/wrist pitch, SG90 wrist roll/gripper
  // Default 544-2400 us 
  constexpr JointConfig JOINTS[NUM_JOINTS] = {
    // name   pin  min   max  rest  cal0 cal180 speed enabled
    { "BAS",  23,    5, 175,   90,  544, 2400,   60, true  },   // MG90S
    { "SHL",  18,    70,  130,    90,  544, 2400,   40, true  },   // MG90S (best-tested)
    { "ELB",  19,   10, 180,  70,  544, 2400,   50, true  },   // MG90S
    { "WRP",  25,    10, 180,   90,  544, 2400,   60, true },   // MG90S
    { "WRR",  26,   10, 170,   90,  544, 2400,   90, true  },   // SG90
    { "GRP",  27,   65, 107,   65,  444, 3000,  180, true  },   // SG90
  };

  // Arming order (first pulse = jump) and rest order (ramped, one at a time)
  constexpr JointId ARM_ORDER[]  = { ELBOW, SHOULDER, WRIST_PITCH, WRIST_ROLL, BASE, GRIPPER };
  constexpr JointId REST_ORDER[] = { GRIPPER, ELBOW, SHOULDER, WRIST_PITCH, WRIST_ROLL, BASE };
  constexpr uint32_t ARM_STEP_MS = 400;

  // ---------------- joystick ----------------
  constexpr int  AXIS_MAP[NUM_MODES][NUM_AXES] = {
    //  J1X    J1Y       J2X         J2Y
    { BASE, SHOULDER, WRIST_ROLL, ELBOW       },   // MODE_ARM
    { BASE, SHOULDER, WRIST_ROLL, WRIST_PITCH },   // MODE_WRIST
  };
  constexpr bool  AXIS_INVERT[NUM_AXES] = { true, true, true, true };
  constexpr float DEADZONE = 0.12f, STICK_ALPHA = 0.25f, EXPO = 2.0f;

  // ---------------- gripper ----------------
  constexpr bool  GRIPPER_USE_POT    = true;   // false = J1 button open/close
  constexpr float GRIPPER_OPEN_DEG   = 65;
  constexpr float GRIPPER_CLOSED_DEG = 120;
  constexpr int   POT_RAW_OPEN       = 150;    // read "pot" in debug line at each knob end
  constexpr int   POT_RAW_CLOSED     = 3950;   // swap the two to reverse the knob
  constexpr float POT_ALPHA          = 0.15f;
  constexpr float POT_DEADBAND_DEG   = 1.0f;
  constexpr float GRIP_CLOSE_DPS     = 60;     // closes slower than it opens

  // ---------------- safety ----------------
  constexpr bool  INTERLOCK_ENABLED            = true;
  constexpr float SHOULDER_HIGH_BELOW_DEG      = 35;    // shoulder target at/below this = "raised"
  constexpr float ELBOW_MIN_WHEN_SHOULDER_HIGH = 100; // prevents elbow and wrist from hitting the ground when shoulder is lowered
  constexpr int   HARD_MIN_US = 500, HARD_MAX_US = 2500;
  constexpr float CAL_SPEED_DPS = 20;                   // slow moves in calibration mode

  // ---------------- timing ----------------
  constexpr uint32_t CONTROL_MS = 20, OLED_MS = 200, DEBUG_MS = 500, STREAM_MS = 50;
  constexpr uint32_t DEBOUNCE_MS = 30, LONG_PRESS_MS = 1000;

  // ---------------- PWM ----------------
  constexpr uint32_t PWM_HZ = 50;
  constexpr uint8_t  PWM_BITS = 14;
  constexpr uint32_t PERIOD_US = 1000000UL / PWM_HZ;
}


// 2nd layer - hardware drivers

// each class keeps track of its own state and exposes a tiny public interface

enum ButtonEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

class DebouncedButton {
 public:
  explicit DebouncedButton(uint8_t pin) : _pin(pin) {}
  void begin() const { pinMode(_pin, INPUT_PULLUP); }

  ButtonEvent update(uint32_t now) {
    bool down = (digitalRead(_pin) == LOW);            // pressed pulls to GND
    if (down != _lastRead) { _lastRead = down; _lastChange = now; }
    if (now - _lastChange < Config::DEBOUNCE_MS) return BTN_NONE;

    if (down && !_stable) { _stable = true; _downSince = now; _longFired = false; }
    else if (down && _stable) {
      if (!_longFired && now - _downSince >= Config::LONG_PRESS_MS) { _longFired = true; return BTN_LONG; }
    } else if (!down && _stable) {
      _stable = false;
      if (!_longFired) return BTN_SHORT;
    }
    return BTN_NONE;
  }

 private:
  uint8_t  _pin;
  bool     _stable = false, _lastRead = false, _longFired = false;
  uint32_t _lastChange = 0, _downSince = 0;
};

class JoystickAxis {
 public:
  JoystickAxis(uint8_t pin, bool invert) : _pin(pin), _invert(invert) {}

  // Sticks must be untouched. A floating (unwired) pin usually reads near 0 or 4095.
  void calibrate() {
    long sum = 0;
    for (int i = 0; i < 64; i++) { sum += analogRead(_pin); delay(2); }
    int c = sum / 64;
    _ok = (c > 800 && c < 3300);
    _center = _ok ? c : 2048;
    _filtered = _center;
    _value = 0;
  }

  void update() {
    _filtered += Config::STICK_ALPHA * (analogRead(_pin) - _filtered);
    if (!_ok) { _value = 0; return; }
    float d = _filtered - _center;
    float span = (d > 0) ? (4095 - _center) : _center;   // real sticks aren't centered at 2048
    float norm = constrain(d / span, -1.0f, 1.0f);
    float mag = fabsf(norm);
    if (mag < Config::DEADZONE) { _value = 0; return; }
    mag = powf((mag - Config::DEADZONE) / (1.0f - Config::DEADZONE), Config::EXPO);  // starts at 0, fine near center
    _value = (norm > 0 ? mag : -mag) * (_invert ? -1.0f : 1.0f);
  }

  float value()  const { return _value; }    // -1..+1
  int   center() const { return _center; }
  bool  ok()     const { return _ok; }
  uint8_t pin()  const { return _pin; }

 private:
  uint8_t _pin;
  bool    _invert;
  int     _center = 2048;
  float   _filtered = 2048, _value = 0;
  bool    _ok = true;
};

class GripperKnob {
 public:
  explicit GripperKnob(uint8_t pin) : _pin(pin) {}

  void begin() { _filteredRaw = analogRead(_pin); _deg = rawToDeg(_filteredRaw); }   // no startup slide

  void update() {
    _filteredRaw += Config::POT_ALPHA * (analogRead(_pin) - _filteredRaw);
    float deg = rawToDeg(_filteredRaw);
    if (fabsf(deg - _deg) >= Config::POT_DEADBAND_DEG) _deg = deg;   // no jitter while holding
  }

  float deg() const { return _deg; }
  int   raw() const { return (int)_filteredRaw; }

 private:
  static float rawToDeg(float raw) {
    float f = (raw - Config::POT_RAW_OPEN) / float(Config::POT_RAW_CLOSED - Config::POT_RAW_OPEN);
    f = constrain(f, 0.0f, 1.0f);
    if (f < 0.02f) f = 0.0f;              // snap the ends so open/closed are exact
    if (f > 0.98f) f = 1.0f;
    return Config::GRIPPER_OPEN_DEG + f * (Config::GRIPPER_CLOSED_DEG - Config::GRIPPER_OPEN_DEG);
  }
  uint8_t _pin;
  float   _filteredRaw = 0, _deg = Config::GRIPPER_OPEN_DEG;
};

// LESSON: static methods = functions grouped under a class name, no object needed.
class ServoPwm {
 public:
  static bool attach(uint8_t pin) { bool ok = ledcAttach(pin, Config::PWM_HZ, Config::PWM_BITS); stop(pin); return ok; }
  static void writeUs(uint8_t pin, int us) {
    us = constrain(us, Config::HARD_MIN_US, Config::HARD_MAX_US);
    const uint32_t maxDuty = (1UL << Config::PWM_BITS) - 1;
    ledcWrite(pin, (uint32_t)((uint64_t)us * maxDuty / Config::PERIOD_US));
  }
  static void stop(uint8_t pin) { ledcWrite(pin, 0); }   // no pulses = servo limp
  static int degToUs(const JointConfig& j, float deg) {
    int us = (int)(j.cal0Us + deg * (j.cal180Us - j.cal0Us) / 180.0f + 0.5f);
    return constrain(us, Config::HARD_MIN_US, Config::HARD_MAX_US);
  }
};

// 3rd layer - arm control
// each phase decides what inputs mean. Arming no longer blocks with
// delay(): it advances one joint every ARM_STEP_MS while loop() keeps running.

enum ArmPhase { PHASE_DISARMED, PHASE_ARMING, PHASE_RUNNING, PHASE_RESTING, PHASE_CALIBRATING };

struct JointState {
  float target  = 0;
  float current = 0;
  bool  live    = false;    // receiving pulses right now
};

class ArmController {
 public:
  void begin() {
    for (int i = 0; i < NUM_JOINTS; i++) {
      const JointConfig& j = Config::JOINTS[i];
      _js[i].target = _js[i].current = j.restDeg;
      _calMin[i] = j.minDeg; _calMax[i] = j.maxDeg; _calRest[i] = j.restDeg; _calTouched[i] = false;
      _enabled[i] = j.enabled;
      if (j.enabled && !ServoPwm::attach(j.pin)) {
        Serial.printf("ERROR: PWM attach failed on GPIO %d (%s) - joint disabled\n", j.pin, j.name);
        _enabled[i] = false;
      }
    }
    pinMode(Config::PIN_LED, OUTPUT);
    digitalWrite(Config::PIN_LED, LOW);
    setStatus("SAFE - press ARM");
  }

  // ---------- commands ----------
  void toggleArm(float knobDeg) { (_phase == PHASE_DISARMED) ? startArming(knobDeg) : disarm(); }

  void startArming(float knobDeg) {
    if (_phase != PHASE_DISARMED) return;
    _gripperClosed = false;
    _gripStartDeg = Config::GRIPPER_USE_POT ? knobDeg : Config::GRIPPER_OPEN_DEG;
    _armIndex = 0;
    _nextArmMs = millis();                 // first joint immediately
    _phase = PHASE_ARMING;
    setStatus("Arming...");
  }

  void disarm() {
    for (int i = 0; i < NUM_JOINTS; i++) { _js[i].live = false; if (_enabled[i]) ServoPwm::stop(Config::JOINTS[i].pin); }
    _phase = PHASE_DISARMED;
    digitalWrite(Config::PIN_LED, LOW);
    setStatus("Disarmed (limp)");
  }

  void startRest() {
    if (_phase != PHASE_RUNNING) { setStatus(_phase == PHASE_DISARMED ? "Arm first" : "Busy"); return; }
    _gripperClosed = false;
    _restStage = 0;
    _phase = PHASE_RESTING;
    setStatus("Going to rest");
  }

  void toggleMode() {
    _mode = (_mode == MODE_ARM) ? MODE_WRIST : MODE_ARM;
    setStatus(_mode == MODE_ARM ? "Mode: ARM (J2Y=elbow)" : "Mode: WRIST (J2Y=pitch)");
  }

  void toggleGripperButton() {
    if (Config::GRIPPER_USE_POT) { setStatus("Gripper: use the knob"); return; }
    if (_phase != PHASE_RUNNING) { setStatus("Arm first"); return; }
    _gripperClosed = !_gripperClosed;
    setStatus(_gripperClosed ? "Gripper closed" : "Gripper open");
  }

  // ---------- calibration mode ----------
  void toggleCalibration() {
    if (_phase == PHASE_CALIBRATING) {
      _phase = PHASE_RUNNING;               // targets get clamped back into the table limits
      setStatus("Calibration OFF");
      Serial.println("  Joints outside their table limits will move back inside them.");
      printTable();
      return;
    }
    if (_phase != PHASE_RUNNING) { setStatus("Arm first (calibrate)"); return; }
    _phase = PHASE_CALIBRATING;
    setStatus("Calibration ON");
    Serial.println("  Sticks ignored, interlock OFF, slow moves. Range 0-180 deg.");
    Serial.println("  j <name>  select    + / -  1 deg    ++ / --  5 deg    goto <deg>");
    Serial.println("  min / max / rest   record current angle    table   print joint table");
    selectJoint(_calJoint);
  }

  bool selectJoint(int id) {
    if (id < 0 || id >= NUM_JOINTS) return false;
    if (!_enabled[id]) { Serial.printf("  %s is disabled in the joint table\n", Config::JOINTS[id].name); return false; }
    _calJoint = id;
    Serial.printf("  selected %s (%d)\n", Config::JOINTS[id].name, id);
    printCalJoint();
    return true;
  }

  void calNudge(float deg) { if (requireCal()) { _js[_calJoint].target = constrain(_js[_calJoint].target + deg, 0.0f, 180.0f); printCalJoint(); } }
  void calGoto(float deg)  { if (requireCal()) { _js[_calJoint].target = constrain(deg, 0.0f, 180.0f); printCalJoint(); } }

  void calRecord(char which) {   // 'n' min, 'x' max, 'r' rest
    if (!requireCal()) return;
    float a = roundf(_js[_calJoint].current);
    if (which == 'n') _calMin[_calJoint] = a;
    if (which == 'x') _calMax[_calJoint] = a;
    if (which == 'r') _calRest[_calJoint] = a;
    _calTouched[_calJoint] = true;
    Serial.printf("  %s %s = %.0f deg (%d us)\n", Config::JOINTS[_calJoint].name,
                  which == 'n' ? "min" : which == 'x' ? "max" : "rest",
                  a, ServoPwm::degToUs(Config::JOINTS[_calJoint], a));
    if (_calMin[_calJoint] >= _calMax[_calJoint]) Serial.println("  WARNING: min is not below max yet");
  }

  void printTable() const {
    Serial.println("\n  // ---- paste over Config::JOINTS (name pin min max rest cal0 cal180 speed enabled) ----");
    for (int i = 0; i < NUM_JOINTS; i++) {
      const JointConfig& j = Config::JOINTS[i];
      Serial.printf("    { \"%s\", %3d, %4.0f, %4.0f, %4.0f, %4d, %4d, %4.0f, %-5s },%s\n",
                    j.name, j.pin, _calMin[i], _calMax[i], _calRest[i], j.cal0Us, j.cal180Us,
                    j.speedDps, j.enabled ? "true" : "false", _calTouched[i] ? "   // <- calibrated" : "");
    }
    Serial.println();
  }

  // ---------- the 50 Hz update ----------
  void update(float dt, const float axis[NUM_AXES], float knobDeg) {
    uint32_t now = millis();

    switch (_phase) {
      case PHASE_DISARMED: return;                        // no pulses at all
      case PHASE_ARMING:   advanceArming(now); break;
      case PHASE_RESTING:  advanceRest(); break;
      case PHASE_RUNNING:  applySticks(axis, dt); applyInterlock(); break;
      case PHASE_CALIBRATING: break;                      // serial drives the selected joint
    }

    // Gripper source: knob (or button), except while calibrating the gripper itself
    bool calGripper = (_phase == PHASE_CALIBRATING && _calJoint == GRIPPER);
    if (_js[GRIPPER].live && !calGripper) {
      _js[GRIPPER].target = Config::GRIPPER_USE_POT ? knobDeg
                          : (_gripperClosed ? Config::GRIPPER_CLOSED_DEG : Config::GRIPPER_OPEN_DEG);
    }

    rampAndOutput(dt);
  }

  // ---------- read-only access for display / serial ----------
  ArmPhase    phase()   const { return _phase; }
  bool        armed()   const { return _phase != PHASE_DISARMED; }
  Mode        mode()    const { return _mode; }
  const char* status()  const { return _status; }
  int         calJoint() const { return _calJoint; }
  bool        enabled(int i) const { return _enabled[i]; }
  float       current(int i) const { return _js[i].current; }
  float       target(int i)  const { return _js[i].target; }
  float       reportedDeg(int i) const { return _enabled[i] ? _js[i].current : Config::JOINTS[i].restDeg; }
  int         currentUs(int i) const { return ServoPwm::degToUs(Config::JOINTS[i], _js[i].current); }

 private:
  void setStatus(const char* s) { _status = s; Serial.printf("> %s\n", s); }

  bool requireCal() { if (_phase != PHASE_CALIBRATING) { Serial.println("  not in calibration mode (type cal)"); return false; } return true; }

  void printCalJoint() const {
    const JointConfig& j = Config::JOINTS[_calJoint];
    Serial.printf("  %s target %.0f deg = %d us   (table: min %.0f max %.0f rest %.0f)\n", j.name,
                  _js[_calJoint].target, ServoPwm::degToUs(j, _js[_calJoint].target), j.minDeg, j.maxDeg, j.restDeg);
  }

  void advanceArming(uint32_t now) {
    constexpr int n = sizeof(Config::ARM_ORDER) / sizeof(Config::ARM_ORDER[0]);
    while (_armIndex < n && now >= _nextArmMs) {
      JointId id = Config::ARM_ORDER[_armIndex++];
      if (!_enabled[id]) continue;                         // skip instantly, no wait
      float start = (id == GRIPPER) ? _gripStartDeg : Config::JOINTS[id].restDeg;
      _js[id].target = _js[id].current = start;
      _js[id].live = true;                                 // first pulse: servo jumps here
      Serial.printf("  armed %s at %.0f deg\n", Config::JOINTS[id].name, start);
      _nextArmMs = now + Config::ARM_STEP_MS;              // one joint at a time
      return;
    }
    if (_armIndex >= n && now >= _nextArmMs) {
      _phase = PHASE_RUNNING;
      digitalWrite(Config::PIN_LED, HIGH);
      setStatus("ARMED");
    }
  }

  void advanceRest() {
    constexpr int n = sizeof(Config::REST_ORDER) / sizeof(Config::REST_ORDER[0]);
    while (_restStage < n) {
      JointId id = Config::REST_ORDER[_restStage];
      if (!_enabled[id] || (id == GRIPPER && Config::GRIPPER_USE_POT)) { _restStage++; continue; }
      float goal = (id == GRIPPER) ? Config::GRIPPER_OPEN_DEG : Config::JOINTS[id].restDeg;
      _js[id].target = goal;
      if (fabsf(_js[id].current - goal) < 0.5f) { _restStage++; continue; }
      return;                                              // wait for this joint
    }
    _phase = PHASE_RUNNING;
    setStatus("At rest");
  }

  void applySticks(const float axis[NUM_AXES], float dt) {
    for (int a = 0; a < NUM_AXES; a++) {
      int id = Config::AXIS_MAP[_mode][a];
      if (id < 0 || id >= NUM_JOINTS || id == GRIPPER || !_enabled[id] || axis[a] == 0) continue;
      _js[id].target += axis[a] * Config::JOINTS[id].speedDps * dt;
    }
  }

  // Uses TARGETS: the elbow is held up before the shoulder even starts rising.
  void applyInterlock() {
    if (!Config::INTERLOCK_ENABLED || !_enabled[SHOULDER] || !_enabled[ELBOW]) return;
    if (_js[SHOULDER].target <= Config::SHOULDER_HIGH_BELOW_DEG &&
        _js[ELBOW].target < Config::ELBOW_MIN_WHEN_SHOULDER_HIGH) {
      _js[ELBOW].target = Config::ELBOW_MIN_WHEN_SHOULDER_HIGH;
    }
  }

  void rampAndOutput(float dt) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      if (!_enabled[i] || !_js[i].live) continue;
      const JointConfig& j = Config::JOINTS[i];
      JointState& s = _js[i];

      bool calThis = (_phase == PHASE_CALIBRATING && i == _calJoint);
      if (calThis) s.target = constrain(s.target, 0.0f, 180.0f);      // explore past the table limits
      else         s.target = constrain(s.target, j.minDeg, j.maxDeg);

      float step = j.speedDps * dt;
      float err = s.target - s.current;
      if (i == GRIPPER && err * (Config::GRIPPER_CLOSED_DEG - Config::GRIPPER_OPEN_DEG) > 0)
        step = fminf(step, Config::GRIP_CLOSE_DPS * dt);                // gentle closing
      if (_phase == PHASE_CALIBRATING) step = fminf(step, Config::CAL_SPEED_DPS * dt);
      s.current += constrain(err, -step, step);

      ServoPwm::writeUs(j.pin, ServoPwm::degToUs(j, s.current));
    }
  }

  JointState  _js[NUM_JOINTS];
  bool        _enabled[NUM_JOINTS] = {};
  ArmPhase    _phase = PHASE_DISARMED;
  Mode        _mode = MODE_ARM;
  const char* _status = "Boot";
  bool        _gripperClosed = false;
  float       _gripStartDeg = Config::GRIPPER_OPEN_DEG;
  int         _armIndex = 0, _restStage = 0;
  uint32_t    _nextArmMs = 0;
  int         _calJoint = SHOULDER;
  float       _calMin[NUM_JOINTS] = {}, _calMax[NUM_JOINTS] = {}, _calRest[NUM_JOINTS] = {};
  bool        _calTouched[NUM_JOINTS] = {};
};

// =====================================================================
// LAYER 4: INTERFACES
// =====================================================================

#if USE_OLED
class StatusDisplay {
 public:
  void begin() {
    Wire.begin(Config::PIN_SDA, Config::PIN_SCL);
    Wire.setClock(400000);
    Serial.println("I2C scan:");
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.printf("  device at 0x%02X\n", a); found++; }
    }
    if (!found) Serial.println("  nothing found - check SDA/SCL/VCC/GND (run oled_test)");
    _ok = _d.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR);
    Serial.println(_ok ? "OLED: OK" : "OLED: begin() failed - arm runs without it");
    if (!_ok) return;
    _d.clearDisplay(); _d.setTextColor(SSD1306_WHITE); _d.setTextSize(1); _d.setCursor(0, 0);
    _d.println("ATLAS ARM v2"); _d.println(); _d.println("Calibrating sticks"); _d.println("- don't touch -");
    _d.display();
  }

  void draw(const ArmController& arm) {
    if (!_ok) return;
    uint32_t t0 = millis();
    _d.clearDisplay();
    _d.setCursor(0, 0);
    if (arm.phase() == PHASE_CALIBRATING) {
      int j = arm.calJoint();
      _d.println("CALIBRATION");
      _d.drawFastHLine(0, 10, Config::OLED_W, SSD1306_WHITE);
      _d.setTextSize(2); _d.setCursor(0, 16);
      _d.printf("%s %3d", Config::JOINTS[j].name, (int)(arm.current(j) + 0.5f));
      _d.setTextSize(1); _d.setCursor(0, 38);
      _d.printf("%4d us  tgt %3d", arm.currentUs(j), (int)(arm.target(j) + 0.5f));
    } else {
      _d.print(arm.armed() ? "ARMED " : "SAFE  ");
      _d.print(arm.mode() == MODE_ARM ? "ARM" : "WRIST");
      int pct = (int)(100.0f * (arm.current(GRIPPER) - Config::GRIPPER_OPEN_DEG) /
                      (Config::GRIPPER_CLOSED_DEG - Config::GRIPPER_OPEN_DEG) + 0.5f);
      _d.printf("  G:%d%%", constrain(pct, 0, 100));
      _d.drawFastHLine(0, 10, Config::OLED_W, SSD1306_WHITE);
      for (int i = 0; i < NUM_JOINTS; i++) {
        _d.setCursor((i / 3) * 64, 14 + (i % 3) * 12);
        _d.print(Config::JOINTS[i].name); _d.print(' ');
        if (arm.enabled(i)) _d.printf("%3d", (int)(arm.current(i) + 0.5f)); else _d.print(" --");
      }
    }
    _d.drawFastHLine(0, 52, Config::OLED_W, SSD1306_WHITE);
    _d.setCursor(0, 55);
    char line[22];
    snprintf(line, sizeof(line), "%s", arm.status());        // 21 chars fit
    _d.print(line);
    _d.display();
    _drawMs = millis() - t0;
  }
  uint32_t drawMs() const { return _drawMs; }

 private:
  // last two args keep I2C at 400 kHz (the library default drops to 100 kHz after each update)
  Adafruit_SSD1306 _d{Config::OLED_W, Config::OLED_H, &Wire, -1, 400000UL, 400000UL};
  bool _ok = false;
  uint32_t _drawMs = 0;
};
#else
class StatusDisplay {                       // same interface, does nothing
 public:
  void begin() { Serial.println("OLED disabled (USE_OLED 0)"); }
  void draw(const ArmController&) {}
  uint32_t drawMs() const { return 0; }
};
#endif

class SerialConsole {
 public:
  SerialConsole(ArmController& arm, JoystickAxis* axes, GripperKnob& knob, StatusDisplay& disp)
    : _arm(arm), _axes(axes), _knob(knob), _disp(disp) {}

  void poll() {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { if (_len) { _buf[_len] = 0; handle(_buf); _len = 0; } }
      else if (_len < sizeof(_buf) - 1) _buf[_len++] = c;
    }
  }

  void periodic(uint32_t now) {
    if (_stream && now - _lastStream >= Config::STREAM_MS) { _lastStream = now; streamLine(); }
    if (_debug && !_stream && now - _lastDebug >= Config::DEBUG_MS) { _lastDebug = now; debugLine(); }
  }

  void printHelp() const {
    Serial.println("Commands: a arm/disarm | r rest | m mode | g gripper (button mode) | d debug");
    Serial.println("          c recalibrate sticks (disarmed) | s/q ROS stream on/off | cal calibration mode");
    Serial.println("          in cal: j <name|0-5>, + - ++ --, goto <deg>, min, max, rest, table");
  }

 private:
  void handle(char* cmd) {
    for (char* p = cmd; *p; p++) *p = tolower(*p);
    char* arg = strchr(cmd, ' ');
    if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

    if      (!strcmp(cmd, "a"))     _arm.toggleArm(_knob.deg());
    else if (!strcmp(cmd, "r"))     _arm.startRest();
    else if (!strcmp(cmd, "m"))     _arm.toggleMode();
    else if (!strcmp(cmd, "g"))     _arm.toggleGripperButton();
    else if (!strcmp(cmd, "d"))     _debug = !_debug;
    else if (!strcmp(cmd, "s"))     { _stream = true; _debug = false; }   // ROS bridge sends this
    else if (!strcmp(cmd, "q"))     _stream = false;
    else if (!strcmp(cmd, "c"))     recalibrate();
    else if (!strcmp(cmd, "cal"))   _arm.toggleCalibration();
    else if (!strcmp(cmd, "+"))     _arm.calNudge(1);
    else if (!strcmp(cmd, "-"))     _arm.calNudge(-1);
    else if (!strcmp(cmd, "++"))    _arm.calNudge(5);
    else if (!strcmp(cmd, "--"))    _arm.calNudge(-5);
    else if (!strcmp(cmd, "goto"))  { if (arg) _arm.calGoto(atof(arg)); else Serial.println("  usage: goto 90"); }
    else if (!strcmp(cmd, "min"))   _arm.calRecord('n');
    else if (!strcmp(cmd, "max"))   _arm.calRecord('x');
    else if (!strcmp(cmd, "rest"))  _arm.calRecord('r');
    else if (!strcmp(cmd, "table")) _arm.printTable();
    else if (!strcmp(cmd, "j"))     selectJoint(arg);
    else if (!strcmp(cmd, "h") || !strcmp(cmd, "?")) printHelp();
    else Serial.printf("  unknown '%s' - type h\n", cmd);
  }

  void selectJoint(const char* arg) {
    if (!arg || !*arg) { Serial.println("  usage: j shl   (or j 1)"); return; }
    if (isdigit(arg[0])) { _arm.selectJoint(atoi(arg)); return; }
    for (int i = 0; i < NUM_JOINTS; i++) {
      const char* n = Config::JOINTS[i].name;
      if (tolower(n[0]) == arg[0] && tolower(n[1]) == arg[1] && tolower(n[2]) == arg[2]) { _arm.selectJoint(i); return; }
    }
    Serial.println("  names: bas shl elb wrp wrr grp");
  }

  void recalibrate() {
    if (_arm.armed()) { Serial.println("  disarm before recalibrating sticks"); return; }
    Serial.println("Calibrating sticks - don't touch them...");
    for (int a = 0; a < NUM_AXES; a++) {
      _axes[a].calibrate();
      Serial.printf("  axis %d (GPIO %d): center %d%s\n", a, _axes[a].pin(), _axes[a].center(),
                    _axes[a].ok() ? "" : "  <-- looks disconnected, axis disabled");
    }
  }

  void streamLine() const {
    Serial.printf("S,%lu,%d", millis(), _arm.armed() ? 1 : 0);
    for (int i = 0; i < NUM_JOINTS; i++) Serial.printf(",%.1f", _arm.reportedDeg(i));
    Serial.println();
  }

  void debugLine() const {
    static const char* PH[] = { "SAFE", "ARMING", "RUN", "REST", "CAL" };
    Serial.printf("[%s %s] sticks %+.2f %+.2f %+.2f %+.2f | ", PH[_arm.phase()],
                  _arm.mode() == MODE_ARM ? "ARM" : "WRIST",
                  _axes[J1X].value(), _axes[J1Y].value(), _axes[J2X].value(), _axes[J2Y].value());
    for (int i = 0; i < NUM_JOINTS; i++)
      if (_arm.enabled(i)) Serial.printf("%s %5.1f  ", Config::JOINTS[i].name, _arm.current(i));
    if (Config::GRIPPER_USE_POT) Serial.printf("| pot %d ", _knob.raw());
    Serial.printf("| oled %lums\n", _disp.drawMs());
  }

  ArmController& _arm;
  JoystickAxis*  _axes;
  GripperKnob&   _knob;
  StatusDisplay& _disp;
  char     _buf[40];
  size_t   _len = 0;
  bool     _debug = true, _stream = false;
  uint32_t _lastStream = 0, _lastDebug = 0;
};

// =====================================================================
// LAYER 5: OBJECTS, setup() AND loop()
// =====================================================================

ArmController   arm;
StatusDisplay   display;
GripperKnob     knob(Config::PIN_GRIP_POT);
DebouncedButton btnJ1(Config::PIN_J1_BTN), btnJ2(Config::PIN_J2_BTN), btnArm(Config::PIN_ARM_BTN);
JoystickAxis    axes[NUM_AXES] = {
  { Config::PIN_J1X, Config::AXIS_INVERT[J1X] }, { Config::PIN_J1Y, Config::AXIS_INVERT[J1Y] },
  { Config::PIN_J2X, Config::AXIS_INVERT[J2X] }, { Config::PIN_J2Y, Config::AXIS_INVERT[J2Y] },
};
SerialConsole   console(arm, axes, knob, display);

uint32_t lastControl = 0, lastOled = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ATLAS ARM - ESP32 v2 ===");
  esp_reset_reason_t why = esp_reset_reason();
  Serial.printf("Reset reason: %d%s\n", (int)why,
                why == ESP_RST_BROWNOUT ? "  <-- BROWNOUT: check servo power, common ground, capacitor (if fitted)" : "");

  analogReadResolution(12);
  btnJ1.begin(); btnJ2.begin(); btnArm.begin();
  arm.begin();                 // attaches PWM, sends NO pulses
  display.begin();
  knob.begin();

  Serial.println("Calibrating joysticks - don't touch them...");
  for (int a = 0; a < NUM_AXES; a++) {
    axes[a].calibrate();
    Serial.printf("  axis %d (GPIO %d): center %d%s\n", a, axes[a].pin(), axes[a].center(),
                  axes[a].ok() ? "" : "  <-- looks disconnected, axis disabled");
  }
  console.printHelp();
  lastControl = millis();
}

void loop() {
  uint32_t now = millis();
  console.poll();

  if (now - lastControl >= Config::CONTROL_MS) {
    float dt = fminf((now - lastControl) / 1000.0f, 0.1f);   // cap: a hiccup can't cause a big jump
    lastControl = now;

    if (btnArm.update(now) == BTN_SHORT) arm.toggleArm(knob.deg());
    if (btnJ1.update(now)  == BTN_SHORT) arm.toggleGripperButton();
    switch (btnJ2.update(now)) {
      case BTN_SHORT: arm.toggleMode(); break;
      case BTN_LONG:  arm.startRest();  break;
      default: break;
    }

    float axisValues[NUM_AXES];
    for (int a = 0; a < NUM_AXES; a++) { axes[a].update(); axisValues[a] = axes[a].value(); }
    if (Config::GRIPPER_USE_POT) knob.update();

    arm.update(dt, axisValues, knob.deg());
  }

  if (now - lastOled >= Config::OLED_MS) { lastOled = now; display.draw(arm); }
  console.periodic(now);
}
