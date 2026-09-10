/*
  ============================================================================
  JUKTI TITANS - Line Follower Robot (LFR)
  ============================================================================
  Hardware :
    Arduino UNO + L298N + 2x N20 gear motors
    HW-871  : 5-channel DIGITAL IR reflectance sensor array
    0.96"   : I2C OLED, SSD1306 driver, using U8g2 library
    3x push buttons (each wired to GND, using internal pull-ups)

  PIN MAP:
    Motors  : ENA D10  IN1 D8  IN2 D7   ENB D9  IN3 D4  IN4 D2
    Sensors : A0 A1 A2 A3 D11   (A0 = LEFTMOST sensor, D11 = RIGHTMOST)
    OLED    : SDA A4   SCL A5   (hardware I2C - built into Uno, no extra wiring)
    Buttons : LEFT D12   MID D3   RIGHT D5   (button -> GND, uses INPUT_PULLUP)

  ----------------------------------------------------------------------------
  HOW THIS ROBOT DRIVES
  ----------------------------------------------------------------------------
  The HW-871 is DIGITAL only -> each sensor just says "black" or "white",
  no shades of gray. So instead of analog weighted averaging, we give each
  of the 5 sensors a fixed weight (far left = -200 ... far right = +200) and
  average the weights of only the sensors that currently see black. That
  gives us a single "error" number: negative = line is to the LEFT of
  center, positive = line is to the RIGHT of center, 0 = centered.

  That error is fed into a simple PD controller (Proportional + Derivative,
  no Integral - with a noisy digital sensor, I-term usually causes more
  problems than it solves, especially the night before a competition).
  The control step runs on a FIXED 200Hz schedule (not "as fast as the loop
  happens to go") so the derivative term behaves predictably, and its output
  is clamped before being split across the two wheels so a sudden error
  jump can't slam one motor to full-forward and the other to full-reverse.
  If 4 or 5 of the 5 sensors are on black at once (a thick line or an
  intersection, not "perfectly centered"), the robot holds its last known
  heading instead of snapping to a fake "centered" reading.

  If ALL sensors read white (line lost - e.g. a gap in the dashed section
  of your track, or a very sharp hairpin), the robot pivots in place toward
  whichever side the line was actually last seen drifting to, until it
  reacquires it or a safety timeout stops the motors.

  The OLED is completely idle while the robot is RUNNING - no I2C traffic,
  no drawing - so the control loop gets 100% of the Arduino's attention.
  The display only updates again once you stop (hold MID) and you're back
  in the menu.

  ----------------------------------------------------------------------------
  ON-ROBOT TUNING MENU (this is the important part)
  ----------------------------------------------------------------------------
  You do NOT need to re-flash the Arduino to change speed or PID values.
  On boot you land in a menu shown on the OLED:

      Base:90    Kp:0.30
      Kd:0.30    Srch:70

    - Short-press MID  -> jumps the cursor to the next parameter
    - LEFT / RIGHT     -> decrease / increase the highlighted parameter
    - HOLD MID (~0.7s) -> SAVES settings to EEPROM and STARTS the run
    - While RUNNING, HOLD MID again -> stops the motors, display comes back,
      and you're back in this menu

  This lets you place the robot on the track, tweak one number, hold MID,
  watch it run, and repeat - fast iteration is how you actually tune these
  robots. See TUNING_GUIDE.md for the step-by-step process.

  ----------------------------------------------------------------------------
  BEFORE ANY OF THIS WORKS: check your sensor polarity!
  ----------------------------------------------------------------------------
  Cheap digital IR boards differ on whether the output pin goes HIGH or LOW
  over black. For about 2 seconds after power-on/reset, the OLED shows
  "Hold LEFT+RIGHT now" - press and hold both buttons together during that
  window (a short hold, ~0.3s, is enough) to enter SENSOR TEST mode: the
  OLED shows live 1/0 for each of the 5 channels so you can confirm which
  state means "sees black". If it's backwards from what this code assumes,
  flip SENSOR_ACTIVE_HIGH below (one line) - no rewiring needed.
  Press MID to exit sensor test mode.
  ============================================================================
*/

#include <U8g2lib.h>
#include <Wire.h>
#include <EEPROM.h>

// ---------------------------------------------------------------------------
// SENSOR POLARITY - flip this if SENSOR TEST mode shows inverted logic
// true  = sensor output reads HIGH when looking at BLACK line
// false = sensor output reads LOW  when looking at BLACK line
// ---------------------------------------------------------------------------
#define SENSOR_ACTIVE_HIGH false

// ---------------------------------------------------------------------------
// MOTOR SIDE MAPPING - flip this if "turn left" actually turns the robot
// right on the real track (faster than re-wiring the motors)
// ---------------------------------------------------------------------------
#define LEFT_MOTOR_IS_A true   // Motor A = ENA/IN1/IN2, Motor B = ENB/IN3/IN4

// ---------------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------------
const uint8_t PIN_ENA = 10, PIN_IN1 = 8, PIN_IN2 = 7;
const uint8_t PIN_ENB = 9,  PIN_IN3 = 4, PIN_IN4 = 2;

const uint8_t SENSOR_PINS[5] = {A0, A1, A2, A3, 11}; // left -> right
const int16_t SENSOR_WEIGHT[5] = {-200, -100, 0, 100, 200};

const uint8_t BTN_LEFT = 12, BTN_MID = 3, BTN_RIGHT = 5;

// ---------------------------------------------------------------------------
// OLED (U8g2, hardware I2C on A4/A5 - default Uno pins, no extra args needed)
// ---------------------------------------------------------------------------
// Using the "_1_" (single page buffer) variant instead of "_F_" (full frame
// buffer) on purpose: the Uno only has 2KB of RAM, and a full 128x64 buffer
// alone eats half of it. The page-buffer variant uses only a few bytes and
// is plenty fast enough for this simple menu + status display.
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ---------------------------------------------------------------------------
// TUNABLE PARAMETERS (adjustable live via the menu, saved to EEPROM)
// ---------------------------------------------------------------------------
struct Settings {
  int   baseSpeed;   // 0-220   forward speed on a straight line
  float kp;           // proportional gain
  float kd;           // derivative gain
  int   searchSpeed; // 0-150   pivot speed used when the line is lost
};

// Kd=2.0 was too aggressive for a DIGITAL sensor: error jumps in big discrete
// steps (e.g. -200 -> +200 when the line crosses from one sensor to another),
// so a big Kd on a raw one-step delta could momentarily demand a huge,
// saturating correction. 0.3 is a much safer starting point - see
// runLineFollow() below for how the output is now clamped too.
Settings cfg = { 90, 0.30f, 0.3f, 70 };   // sane defaults for N20 + L298N

const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_MAGIC_VAL  = 0xA6;   // bumped so old saved (aggressive) Kd
                                       // values from before this fix are
                                       // ignored and the new defaults load
const int EEPROM_DATA_ADDR  = 1;

const int MAX_SPEED   = 220;   // absolute PWM ceiling (0-255)
const int MIN_MOVE_PWM = 55;   // N20 + L298N combo tends to stall below this

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------
enum AppState { MENU, RUNNING };
AppState state = MENU;

const int NUM_PARAMS = 4; // Base, Kp, Kd, Search
int menuIndex = 0;

int lastError = 0;
int lastDirection = 1;   // +1 = line last seen/drifting right, -1 = left.
                          // Tracked explicitly (rather than reusing lastError,
                          // which gets held steady during intersections - see
                          // below) so the lost-line search always pivots
                          // toward where the line actually was.
unsigned long lostLineSince = 0;
const unsigned long LOST_LINE_SAFETY_STOP_MS = 3000; // give up & stop after this

// Fixed-rate control loop. Running the PID math on a fixed interval (rather
// than as fast as the loop happens to spin) makes the derivative term's
// effective timing constant and predictable - instead of literally dividing
// by a measured, jittery dt (which would make Kd swing wildly and be nearly
// impossible to tune for a sensor whose error jumps in big discrete steps).
// 200Hz is far faster than this robot needs to react.
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_INTERVAL_MS = 5;

// ---- simple button debounce/edge + long-press helpers ----
struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChange;
  unsigned long pressedAt;
  bool longFired;
  Button(uint8_t p)
    : pin(p), lastReading(HIGH), stableState(HIGH),
      lastChange(0), pressedAt(0), longFired(false) {}
};
Button bLeft(BTN_LEFT), bMid(BTN_MID), bRight(BTN_RIGHT);
const unsigned long DEBOUNCE_MS = 25;
const unsigned long LONG_PRESS_MS = 700;

// returns true exactly once on a short press (release before long-press time)
bool updateButtonShortPress(Button &b) {
  bool reading = digitalRead(b.pin);
  bool shortPress = false;
  if (reading != b.lastReading) {
    b.lastChange = millis();
  }
  if (millis() - b.lastChange > DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) {
      b.pressedAt = millis();
      b.longFired = false;
    } else {
      // released
      if (!b.longFired && (millis() - b.pressedAt) < LONG_PRESS_MS) {
        shortPress = true;
      }
    }
  }
  b.lastReading = reading;
  return shortPress;
}

// returns true exactly once when held past LONG_PRESS_MS
bool updateButtonLongPress(Button &b) {
  if (b.stableState == LOW && !b.longFired &&
      (millis() - b.pressedAt) >= LONG_PRESS_MS) {
    b.longFired = true;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
void loadSettings() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VAL) {
    EEPROM.get(EEPROM_DATA_ADDR, cfg);
  }
  // clamp in case EEPROM had garbage / old format
  cfg.baseSpeed = constrain(cfg.baseSpeed, 0, MAX_SPEED);
  cfg.searchSpeed = constrain(cfg.searchSpeed, 0, MAX_SPEED);
  if (cfg.kp < 0 || cfg.kp > 5)  cfg.kp = 0.30f;
  if (cfg.kd < 0 || cfg.kd > 20) cfg.kd = 0.3f;
}

void saveSettings() {
  EEPROM.put(EEPROM_DATA_ADDR, cfg);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
}

// ---------------------------------------------------------------------------
void setMotorRaw(uint8_t enaPin, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  }
  // don't let a tiny nonzero PWM just buzz the motor without moving it
  if (speed > 0 && speed < MIN_MOVE_PWM) speed = MIN_MOVE_PWM;
  analogWrite(enaPin, speed);
}

void driveMotors(int leftSpeed, int rightSpeed) {
#if LEFT_MOTOR_IS_A
  setMotorRaw(PIN_ENA, PIN_IN1, PIN_IN2, leftSpeed);
  setMotorRaw(PIN_ENB, PIN_IN3, PIN_IN4, rightSpeed);
#else
  setMotorRaw(PIN_ENB, PIN_IN3, PIN_IN4, leftSpeed);
  setMotorRaw(PIN_ENA, PIN_IN1, PIN_IN2, rightSpeed);
#endif
}

void stopMotors() {
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

// ---------------------------------------------------------------------------
// returns number of sensors currently on black, fills seen[] with raw bits
// (1 = sees black, 0 = sees white) and computes weighted error via out param
int readSensors(int seen[5], int &errorOut) {
  int count = 0;
  long weightedSum = 0;
  for (int i = 0; i < 5; i++) {
    int raw = digitalRead(SENSOR_PINS[i]);
    bool onBlack = SENSOR_ACTIVE_HIGH ? (raw == HIGH) : (raw == LOW);
    seen[i] = onBlack ? 1 : 0;
    if (onBlack) {
      count++;
      weightedSum += SENSOR_WEIGHT[i];
    }
  }
  errorOut = (count > 0) ? (int)(weightedSum / count) : 0;
  return count;
}

// ---------------------------------------------------------------------------
// This is the whole job of the robot while RUNNING. No OLED/I2C activity
// happens in here at all (or anywhere else while RUNNING) - the display is
// completely frozen from the moment you start until you stop, so 100% of
// the control loop's time goes to reading sensors and driving motors.
void runLineFollow() {
  // Fixed-rate control: only run the sensing/PID/motor step every
  // CONTROL_INTERVAL_MS. This is what keeps the derivative term's timing
  // constant (see the comment by CONTROL_INTERVAL_MS above) - it also means
  // this function returns almost instantly on the other loop() iterations,
  // which is fine since loop() just calls straight back into it.
  if (millis() - lastControlUpdate < CONTROL_INTERVAL_MS) return;
  lastControlUpdate = millis();

  int seen[5];
  int rawError;
  int count = readSensors(seen, rawError);

  // Track which side the line has actually been drifting toward, using the
  // real sensor reading (not the possibly-held value below). This is what
  // the lost-line search uses, instead of assuming "right" by default.
  if (rawError > 0) lastDirection = 1;
  else if (rawError < 0) lastDirection = -1;

  int error = rawError;
  if (count >= 4) {
    // 4 or 5 sensors on black at once isn't "perfectly centered" - it's
    // almost certainly a wide/thick line or an intersection. The plain
    // weighted average would swing to ~0 here and then swing back the moment
    // you exit it, which is exactly the kind of sudden error jump that
    // causes a derivative spike. Instead, hold the last real heading steady
    // while crossing it.
    error = lastError;
  }

  if (count > 0) {
    lostLineSince = 0;

    int derivative = error - lastError;
    float output = cfg.kp * error + cfg.kd * derivative;
    // Clamp the correction itself before splitting it across the two
    // wheels, so a big derivative kick can't push one wheel to full-forward
    // and the other to full-reverse - it gets capped first, symmetrically.
    output = constrain(output, (float)-MAX_SPEED, (float)MAX_SPEED);

    int left  = cfg.baseSpeed + (int)output;
    int right = cfg.baseSpeed - (int)output;

    left  = constrain(left, -MAX_SPEED, MAX_SPEED);
    right = constrain(right, -MAX_SPEED, MAX_SPEED);

    driveMotors(left, right);
    lastError = error;
  } else {
    // line fully lost: pivot toward the side it was actually last seen on
    if (lostLineSince == 0) lostLineSince = millis();

    if (millis() - lostLineSince > LOST_LINE_SAFETY_STOP_MS) {
      stopMotors(); // safety: don't let it wander off forever
      return;
    }

    if (lastDirection >= 0) {
      driveMotors(cfg.searchSpeed, -cfg.searchSpeed);
    } else {
      driveMotors(-cfg.searchSpeed, cfg.searchSpeed);
    }
  }
}

// ---------------------------------------------------------------------------
// The AVR toolchain's snprintf() does NOT support %f / %g by default (it
// just prints "?" for any float) unless you add special linker flags. Easiest
// fix: format floats ourselves with plain integer math, using only %d/%s
// (which always work).
void formatFloat2(float v, char *buf, int bufSize) {
  bool neg = v < 0;
  if (neg) v = -v;
  int whole = (int)v;
  int frac = (int)((v - whole) * 100.0f + 0.5f); // round to 2 decimals
  if (frac >= 100) { frac -= 100; whole += 1; }
  snprintf(buf, bufSize, "%s%d.%02d", neg ? "-" : "", whole, frac);
}

void drawMenu() {
  char kpStr[8], kdStr[8];
  formatFloat2(cfg.kp, kpStr, sizeof(kpStr));
  formatFloat2(cfg.kd, kdStr, sizeof(kdStr));

  char l1[24], l2[24];
  snprintf(l1, sizeof(l1), "Base:%-4d  Kp:%s", cfg.baseSpeed, kpStr);
  snprintf(l2, sizeof(l2), "Kd:%-6s Srch:%-4d", kdStr, cfg.searchSpeed);

  // cursor under the selected parameter
  const int colX[4] = {0, 66, 0, 66};
  const int colY[4] = {12, 12, 24, 24};

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, l1);
    u8g2.drawStr(0, 24, l2);
    u8g2.drawStr(colX[menuIndex], colY[menuIndex] + 10, "^");
    u8g2.drawStr(0, 44, "L/R: adjust  MID: next");
    u8g2.drawStr(0, 56, "HOLD MID: SAVE & START");
  } while (u8g2.nextPage());
}

void adjustParam(int dir) {
  switch (menuIndex) {
    case 0: cfg.baseSpeed = constrain(cfg.baseSpeed + dir * 5, 0, MAX_SPEED); break;
    case 1: cfg.kp = constrain(cfg.kp + dir * 0.02f, 0.0f, 5.0f); break;
    case 2: cfg.kd = constrain(cfg.kd + dir * 0.2f, 0.0f, 20.0f); break;
    case 3: cfg.searchSpeed = constrain(cfg.searchSpeed + dir * 5, 0, MAX_SPEED); break;
  }
}

// ---------------------------------------------------------------------------
void sensorTestMode() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "SENSOR TEST - MID to exit");
  } while (u8g2.nextPage());
  delay(400);

  while (true) {
    int seen[5], error;
    readSensors(seen, error);

    char l[24];
    snprintf(l, sizeof(l), "%d %d %d %d %d  (1=black)",
             seen[0], seen[1], seen[2], seen[3], seen[4]);

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 10, "SENSOR TEST - MID to exit");
      u8g2.drawStr(0, 30, l);
      u8g2.drawStr(0, 45, "If backwards, flip");
      u8g2.drawStr(0, 57, "SENSOR_ACTIVE_HIGH in code");
    } while (u8g2.nextPage());

    if (digitalRead(BTN_MID) == LOW) {
      delay(300); // debounce exit
      return;
    }
    delay(60);
  }
}

// ---------------------------------------------------------------------------
void setup() {
  for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);

  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_MID, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  u8g2.begin();

  loadSettings();

  // Give a real window to catch a LEFT+RIGHT hold for sensor test mode.
  // (Checking just once, instantly, the moment setup() runs is too fast for
  // a human to react to right after power-on/upload - that's why it looked
  // like the buttons "did nothing".) Show a countdown prompt for ~2s and
  // require the two buttons to be held together for 300ms within it.
  const unsigned long BOOT_WINDOW_MS = 2000;
  const unsigned long HOLD_REQUIRED_MS = 300;
  unsigned long windowStart = millis();
  unsigned long bothHeldSince = 0;
  bool enterTest = false;

  while (millis() - windowStart < BOOT_WINDOW_MS) {
    bool held = (digitalRead(BTN_LEFT) == LOW) && (digitalRead(BTN_RIGHT) == LOW);
    if (held) {
      if (bothHeldSince == 0) bothHeldSince = millis();
      if (millis() - bothHeldSince >= HOLD_REQUIRED_MS) {
        enterTest = true;
        break;
      }
    } else {
      bothHeldSince = 0;
    }

    unsigned long remainingMs = BOOT_WINDOW_MS - (millis() - windowStart);
    int secsLeft = (remainingMs + 999) / 1000; // ceiling, so it counts 2,2,...,1,1,...
    char line[24];
    snprintf(line, sizeof(line), "starting in %ds...", secsLeft);
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 20, "Hold LEFT+RIGHT now");
      u8g2.drawStr(0, 34, "for SENSOR TEST mode");
      u8g2.drawStr(0, 50, line);
    } while (u8g2.nextPage());
    delay(20);
  }

  if (enterTest) {
    sensorTestMode();
  }
}

// ---------------------------------------------------------------------------
void loop() {
  if (state == MENU) {
    bool leftShort  = updateButtonShortPress(bLeft);
    bool rightShort = updateButtonShortPress(bRight);
    bool midShort   = updateButtonShortPress(bMid);
    bool midLong    = updateButtonLongPress(bMid);

    if (leftShort) adjustParam(-1);
    if (rightShort) adjustParam(+1);
    if (midShort) menuIndex = (menuIndex + 1) % NUM_PARAMS;

    if (midLong) {
      saveSettings();
      lastError = 0;
      lastDirection = 1;
      lostLineSince = 0;
      lastControlUpdate = 0;
      state = RUNNING;
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 30, "GO!");
      } while (u8g2.nextPage());
      delay(400); // brief pause so you can let go of the robot
    } else {
      drawMenu();
    }
  } else { // RUNNING
    bool midLong = updateButtonLongPress(bMid);
    updateButtonShortPress(bMid); // consume short-press edge so it doesn't
                                   // leak into the menu when we return to it
    if (midLong) {
      stopMotors();
      state = MENU;
      delay(300);
    } else {
      runLineFollow();
    }
  }
}
