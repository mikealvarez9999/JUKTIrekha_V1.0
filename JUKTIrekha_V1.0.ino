/* =====================================================================
   LINE FOLLOWING ROBOT — COMPETITION BUILD
   Arduino Nano + L298N + 2x N20 + HW-871 (5ch digital IR)
   ---------------------------------------------------------------------
   Changes from the basic version:
     - GAP HANDLING: when the line disappears, hold the last steering
       for Q milliseconds instead of spinning. Crosses dashed sections.
     - CORNER SLOWDOWN: base speed drops as error grows, so tight turns
       are taken slower without touching your straight-line speed.
     - SEARCH: only after the gap timeout expires does it pivot to hunt.
     - Crossings (all 5 black) need no special code: the weighted
       average lands on 0, which means straight ahead.

   Serial @ 115200, line ending "Newline", ? for help.
   ===================================================================== */

#include <EEPROM.h>

/* ------------------------- PIN MAP ------------------------- */
#define ENA 10
#define IN1 8
#define IN2 7
#define ENB 9
#define IN3 4
#define IN4 2

const uint8_t SENSOR_PIN[5] = { A0, A1, A2, A3, A4 };

#define LED_PIN 13
#define BUTTON_PIN 12

/* ------------------------- CONFIG ------------------------- */
struct Config {
  uint16_t magic;
  float kp;
  float ki;
  float kd;
  int baseSpeed;
  int maxSpeed;
  int minPwm;
  float trim;
  uint8_t invert;
  int gapMs;        // how long to coast blind across a gap
  float cornerSlow; // 0..0.8  how much to slow in tight turns
};

Config cfg;
#define CFG_MAGIC 0xC442
#define CFG_ADDR 0

void loadDefaults() {
  cfg.magic = CFG_MAGIC;
  cfg.kp = 0.14;
  cfg.ki = 0.0000;
  cfg.kd = 2.20;
  cfg.baseSpeed = 100;
  cfg.maxSpeed = 220;
  cfg.minPwm = 45;
  cfg.trim = 0.0;
  cfg.invert = 1;
  cfg.gapMs = 500;
  cfg.cornerSlow = 0.35;
}

/* ------------------------- STATE ------------------------- */
bool running = false;
uint8_t telemetry = 0;
uint16_t loopHz = 200;

uint8_t bits[5];
uint8_t onCount = 0;
int position = 0;
int lastPosition = 0;
float integral = 0;
float lastError = 0;
bool lineSeen = true;
bool allOn = false;

int lastCorrection = 0;         // held while blind
unsigned long lostSince = 0;    // when the line vanished, 0 = not lost
bool searching = false;

int lastL = 0, lastR = 0;
unsigned long lastLoop = 0;

/* ===================================================================
   MOTORS
   =================================================================== */
void driveMotor(uint8_t en, uint8_t a, uint8_t b, int pwm) {
  bool rev = pwm < 0;
  pwm = abs(pwm);
  if (pwm > 255) pwm = 255;
  if (pwm > 0) pwm = cfg.minPwm + (long)(255 - cfg.minPwm) * pwm / 255;

  digitalWrite(a, rev ? LOW : HIGH);
  digitalWrite(b, rev ? HIGH : LOW);
  analogWrite(en, pwm);
}

void setMotors(int l, int r) {
  float lf = l, rf = r;
  if (cfg.trim > 0) lf *= (1.0 - cfg.trim);
  else if (cfg.trim < 0) rf *= (1.0 + cfg.trim);

  l = constrain((int)lf, -cfg.maxSpeed, cfg.maxSpeed);
  r = constrain((int)rf, -cfg.maxSpeed, cfg.maxSpeed);

  lastL = l;
  lastR = r;
  driveMotor(ENA, IN1, IN2, l);
  driveMotor(ENB, IN3, IN4, r);
}

void stopMotors() {
  lastL = lastR = 0;
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/* ===================================================================
   SENSORS
   =================================================================== */
void readSensors() {
  long weighted = 0;
  onCount = 0;

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t v = digitalRead(SENSOR_PIN[i]);
    bits[i] = cfg.invert ? !v : v;
    if (bits[i]) {
      onCount++;
      weighted += (long)i * 1000L;
    }
  }

  allOn = (onCount == 5);

  if (onCount > 0) {
    position = weighted / onCount - 2000;
    lastPosition = position;
    lineSeen = true;
  } else {
    lineSeen = false;
  }
}

/* ===================================================================
   SENSOR MONITOR
   =================================================================== */
void sensorCheck() {
  stopMotors();
  Serial.println(F("\n[CHECK] 15 s live view. Slide the array over the line."));
  Serial.println(F("  # = line, . = floor. Want one # walking left to right.\n"));

  unsigned long t0 = millis();
  while (millis() - t0 < 15000) {
    readSensors();
    Serial.print(F("  ["));
    for (uint8_t i = 0; i < 5; i++) Serial.print(bits[i] ? '#' : '.');
    Serial.print(F("]  count="));
    Serial.print(onCount);
    Serial.print(F("  pos="));
    Serial.println(onCount ? position : 9999);
    digitalWrite(LED_PIN, onCount > 0);
    delay(150);
  }
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("[CHECK] done."));
}

/* ===================================================================
   BALANCE TEST
   =================================================================== */
void balanceTest() {
  Serial.println(F("\n[TEST] Straight run, 2.5 s."));
  Serial.println(F("Curves LEFT  -> B -0.08   Curves RIGHT -> B 0.08"));
  delay(1500);
  setMotors(cfg.baseSpeed, cfg.baseSpeed);
  delay(2500);
  stopMotors();
  Serial.print(F("[TEST] trim = "));
  Serial.println(cfg.trim, 3);
}

/* ===================================================================
   EEPROM
   =================================================================== */
void saveCfg() {
  cfg.magic = CFG_MAGIC;
  EEPROM.put(CFG_ADDR, cfg);
  Serial.println(F("[EEPROM] saved."));
}

void loadCfg() {
  Config tmp;
  EEPROM.get(CFG_ADDR, tmp);
  if (tmp.magic == CFG_MAGIC) {
    cfg = tmp;
    Serial.println(F("[EEPROM] loaded."));
  } else {
    loadDefaults();
    Serial.println(F("[EEPROM] empty -> defaults."));
  }
}

/* ===================================================================
   CONSOLE
   =================================================================== */
void printSettings() {
  Serial.println(F("--- settings ---"));
  Serial.print(F("Kp="));
  Serial.print(cfg.kp, 4);
  Serial.print(F("  Ki="));
  Serial.print(cfg.ki, 5);
  Serial.print(F("  Kd="));
  Serial.println(cfg.kd, 4);
  Serial.print(F("base="));
  Serial.print(cfg.baseSpeed);
  Serial.print(F("  max="));
  Serial.print(cfg.maxSpeed);
  Serial.print(F("  minPwm="));
  Serial.println(cfg.minPwm);
  Serial.print(F("trim="));
  Serial.print(cfg.trim, 3);
  Serial.print(F("  invert="));
  Serial.println(cfg.invert);
  Serial.print(F("gapMs="));
  Serial.print(cfg.gapMs);
  Serial.print(F("  cornerSlow="));
  Serial.println(cfg.cornerSlow, 2);
  Serial.println(F("----------------"));
}

void printHelp() {
  Serial.println(F("\n===== LFR CONSOLE (competition) ====="));
  Serial.println(F(" G go    X stop    C sensor check    M balance test"));
  Serial.println(F(" P Kp   I Ki   D Kd   S base   A max   N minPwm"));
  Serial.println(F(" B trim -1..1 (+ slows LEFT)"));
  Serial.println(F(" V 0|1  line polarity (1 = line reads LOW)"));
  Serial.println(F(" Q <ms> blind coast time across a gap   e.g. Q 500"));
  Serial.println(F(" F <0-0.8> corner slowdown              e.g. F 0.35"));
  Serial.println(F(" T 0-3 telemetry    W save   L load   R defaults   ? help"));
  printSettings();
}

void handleSerial() {
  static char buf[24];
  static uint8_t n = 0;
  static unsigned long lastChar = 0;
  bool complete = false;

  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\n' || c == '\r') { complete = true; break; }
    if (n < sizeof(buf) - 1) buf[n++] = c;
  }
  if (!complete && n > 0 && millis() - lastChar > 60) complete = true;
  if (!complete || n == 0) return;

  buf[n] = 0;
  Serial.print(F("rx> "));
  Serial.println(buf);

  char cmd = toupper(buf[0]);
  float val = atof(buf + 1);

  switch (cmd) {
    case 'G':
      integral = 0;
      lastError = 0;
      lostSince = 0;
      searching = false;
      running = true;
      Serial.println(F(">> RUN"));
      break;
    case 'X':
      running = false;
      stopMotors();
      Serial.println(F(">> STOP"));
      break;
    case 'C': running = false; sensorCheck(); break;
    case 'M': running = false; balanceTest(); break;
    case 'P': cfg.kp = val; printSettings(); break;
    case 'I': cfg.ki = val; integral = 0; printSettings(); break;
    case 'D': cfg.kd = val; printSettings(); break;
    case 'S': cfg.baseSpeed = (int)val; printSettings(); break;
    case 'A': cfg.maxSpeed = (int)val; printSettings(); break;
    case 'N': cfg.minPwm = (int)val; printSettings(); break;
    case 'B': cfg.trim = constrain(val, -1.0, 1.0); printSettings(); break;
    case 'V': cfg.invert = (uint8_t)val; printSettings(); break;
    case 'Q': cfg.gapMs = (int)val; printSettings(); break;
    case 'F': cfg.cornerSlow = constrain(val, 0.0, 0.8); printSettings(); break;
    case 'T': telemetry = (uint8_t)val; break;
    case 'W': saveCfg(); break;
    case 'L': loadCfg(); break;
    case 'R': loadDefaults(); Serial.println(F("defaults restored")); printSettings(); break;
    default: printHelp(); break;
  }
  n = 0;
}

void printTelemetry(float p, float d, int corr, int spd) {
  switch (telemetry) {
    case 1:
      Serial.print('[');
      for (uint8_t k = 0; k < 5; k++) Serial.print(bits[k] ? '#' : '.');
      Serial.print(F("] pos="));
      Serial.print(position);
      Serial.print(F(" cor="));
      Serial.print(corr);
      Serial.print(F(" spd="));
      Serial.print(spd);
      Serial.print(F(" L="));
      Serial.print(lastL);
      Serial.print(F(" R="));
      Serial.print(lastR);
      if (allOn) Serial.print(F("  <CROSS>"));
      if (!lineSeen) Serial.print(searching ? F("  <SEARCH>") : F("  <GAP>"));
      Serial.println();
      break;
    case 2:
      Serial.print(position);
      Serial.print('\t');
      Serial.print(corr);
      Serial.print('\t');
      Serial.print(lastL);
      Serial.print('\t');
      Serial.println(lastR);
      break;
    case 3:
      for (uint8_t k = 0; k < 5; k++) {
        Serial.print(digitalRead(SENSOR_PIN[k]));
        Serial.print('\t');
      }
      Serial.println();
      break;
  }
}

/* ===================================================================
   SETUP / LOOP
   =================================================================== */
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < 5; i++) pinMode(SENSOR_PIN[i], INPUT);

  stopMotors();
  Serial.begin(115200);
  delay(300);
  loadCfg();
  printHelp();
}

void loop() {
  handleSerial();

  unsigned long now = micros();
  if (now - lastLoop < (1000000UL / loopHz)) return;
  lastLoop = now;

  readSensors();

  int correction;
  int speed = cfg.baseSpeed;

  if (lineSeen) {
    lostSince = 0;
    searching = false;

    float error = position;
    float p = cfg.kp * error;

    integral += error;
    integral = constrain(integral, -20000, 20000);
    float i = cfg.ki * integral;

    float d = cfg.kd * (error - lastError);
    lastError = error;

    correction = (int)(p + i + d);
    lastCorrection = correction;

    // corner slowdown: the further off centre, the slower we go
    float off = abs(error) / 2000.0;
    if (off > 1.0) off = 1.0;
    speed = (int)(cfg.baseSpeed * (1.0 - cfg.cornerSlow * off));

    if (telemetry) printTelemetry(p, d, correction, speed);
    setMotors(speed + correction, speed - correction);
    digitalWrite(LED_PIN, HIGH);

  } else {
    if (lostSince == 0) lostSince = millis();

    if (millis() - lostSince < (unsigned long)cfg.gapMs) {
      // GAP: keep doing exactly what we were doing. Straight stays
      // straight, a curve keeps curving. This crosses dashed sections.
      searching = false;
      correction = lastCorrection;
      speed = (int)(cfg.baseSpeed * 0.85);
      setMotors(speed + correction, speed - correction);
    } else {
      // Gap was too long — it is a real loss. Pivot to hunt.
      searching = true;
      int s = (int)(cfg.baseSpeed * 0.75);
      if (lastPosition > 0) setMotors(s, -s);
      else setMotors(-s, s);
      correction = 0;
    }
    digitalWrite(LED_PIN, (millis() / 80) % 2);
    if (telemetry) printTelemetry(0, 0, correction, speed);
  }

  if (!running) {
    stopMotors();
    digitalWrite(LED_PIN, (millis() / 500) % 2);
  }
}
