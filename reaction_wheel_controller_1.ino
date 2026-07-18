/*
 * Reaction Wheel Self-Balancing Stick — PID Controller
 * Target: Arduino Uno R3
 *
 * ARCHITECTURE CHANGE from v3 (full-state feedback) to PID:
 *   - Reference (setpoint) is the calibrated upright position (theta = 0
 *     after thetaOffset is subtracted at startup — same calibration as v3).
 *   - Error = 0 - theta  (positive error = stick leaning negative direction)
 *   - u = Kp*e + Ki*integral(e) + Kd*de/dt
 *   - Integral is clamped (anti-windup) to prevent runaway during large tilts.
 *   - The encoder/omega state is retained in telemetry but is NOT part of
 *     the control law (PID on angle only). You can re-add it as a feed-forward
 *     term later if needed.
 *
 * Everything else is IDENTICAL to v3:
 *   - Calibration logic (3-second average, LED, countdown)
 *   - EEPROM boot counter
 *   - IMU-aware differentiation (dt from IMU timestamps)
 *   - Low-pass filter on theta_dot
 *   - pollImuUntil() non-blocking loop pacing
 *   - All DEBUG flags and telemetry format
 *
 * Hardware pin assignment unchanged from v3.
 * ---------------------------------------------------------------
 */
 
#include <SoftwareSerial.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Adafruit_BNO08x_RVC.h"
 
// ---------------------------------------------------------------
//  PIN DEFINITIONS
// ---------------------------------------------------------------
#define IMU_RX_PIN       4
#define IMU_TX_PIN       5
#define ENCODER_CS_PIN  10
#define MOTOR_ENA_PIN    6
#define MOTOR_IN1_PIN    2
#define MOTOR_IN2_PIN    3
#define CAL_LED_PIN     13
 
// ---------------------------------------------------------------
//  DEBUG FLAGS
// ---------------------------------------------------------------
#define DEBUG_VERBOSE       1
#define DEBUG_RAW_ENCODER   1
#define DEBUG_CONTRIB       1
#define DEBUG_INTERVAL_MS  50
 
// ---------------------------------------------------------------
//  PID GAINS  — tune these
//  Kp: primary restoring force (replaces K1)
//  Ki: eliminates steady-state bias / slow drift
//  Kd: damps oscillation (replaces K2; uses filtered theta_dot)
//
//  Start with Ki = 0 and Kd = 0, tune Kp until it holds, then
//  add Kd to reduce ringing, then a small Ki to remove bias.
// ---------------------------------------------------------------
const float Kp =  5.0f;
const float Ki = 1.0f;   // start at 0, add slowly
const float Kd =  .5; 
 
// Anti-windup: integral is clamped to this band (in radians * seconds)
const float INTEGRAL_LIMIT = 2.5f;
 
const float U_MAX = 5.0f;
 
// ---------------------------------------------------------------
//  CONSTANTS
// ---------------------------------------------------------------
const float COUNTS_PER_REV = 16384.0f;
const float TWO_PI_F       =  6.28318530718f;
const float PI_F           =  3.14159265359f;
const float DEG2RAD        =  0.017453292520f;
const float RAD2DEG        = 57.29577951308f;
 
const unsigned long LOOP_US = 10000UL;   // 100 Hz target
 
// ---------------------------------------------------------------
//  CALIBRATION & FILTER
// ---------------------------------------------------------------
const unsigned long CAL_DURATION_MS = 3000UL;
const unsigned long CAL_MIN_SAMPLES = 20UL;
const float THETA_DOT_ALPHA = 0.3f;
const float THETA_DOT_MAX   = 30.0f;
 
// ---------------------------------------------------------------
//  EEPROM ADDRESS for boot counter
// ---------------------------------------------------------------
#define EEPROM_BOOT_COUNT_ADDR 0   // 2 bytes (uint16_t)
 
// ---------------------------------------------------------------
//  OBJECTS
// ---------------------------------------------------------------
SoftwareSerial       imuSerial(IMU_RX_PIN, IMU_TX_PIN);
Adafruit_BNO08x_RVC rvc;
 
// ---------------------------------------------------------------
//  PLANT STATE  (identical to v3)
// ---------------------------------------------------------------
float theta     = 0.0f;   // stick tilt, rad (calibrated so upright = 0)
float theta_dot = 0.0f;   // filtered angular rate, rad/s
float omega     = 0.0f;   // wheel speed, rad/s (telemetry only)
 
float thetaOffset  = 0.0f;
float thetaPrevImu = 0.0f;
float thetaDotRaw  = 0.0f;
 
float encoderPrev    = 0.0f;
uint16_t encoderRawNow = 0;
 
unsigned long tPrevUs    = 0UL;
unsigned long tPrevImuUs = 0UL;
 
// ---------------------------------------------------------------
//  PID STATE  (new)
// ---------------------------------------------------------------
float pidIntegral   = 0.0f;   // accumulated integral term
float pidError      = 0.0f;   // current error (for telemetry)
float pidP          = 0.0f;   // Kp contribution (for telemetry)
float pidI          = 0.0f;   // Ki contribution (for telemetry)
float pidD          = 0.0f;   // Kd contribution (for telemetry)
 
// ---------------------------------------------------------------
//  DEBUG BOOKKEEPING  (identical to v3)
// ---------------------------------------------------------------
int           lastPwm        = 0;
int           lastDir        = 0;
bool          imuFresh       = false;
unsigned long loopCounter    = 0UL;
unsigned long lastPrintMs    = 0UL;
 
unsigned long imuNewCount    = 0UL;
unsigned long imuHoldCount   = 0UL;
 
unsigned long imuNewWindow   = 0UL;
unsigned long lastHzSampleMs = 0UL;
float         imuHzMeasured  = 0.0f;
 
// ---------------------------------------------------------------
//  FORWARD DECLARATIONS
// ---------------------------------------------------------------
float    readEncoderRad();
float    unwrapDelta(float delta);
float    wrapToPi(float angle);
uint16_t spiTransfer16(uint16_t data);
void     commandMotor(float u);
void     printDebug(float tau, float dt);
void     calibrateZero();
void     pollImuUntil(unsigned long targetUs);
void     printBootCount();
 
// ---------------------------------------------------------------
//  SETUP  (identical to v3)
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { }
 
  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F("Reaction Wheel Controller — PID"));
  Serial.println(F("============================================"));
 
  printBootCount();
 
  Serial.print(F("Gains: Kp=")); Serial.print(Kp, 4);
  Serial.print(F(" Ki="));       Serial.print(Ki, 4);
  Serial.print(F(" Kd="));       Serial.println(Kd, 4);
  Serial.print(F("Anti-windup limit: +/-")); Serial.println(INTEGRAL_LIMIT, 3);
 
  pinMode(CAL_LED_PIN, OUTPUT);
  digitalWrite(CAL_LED_PIN, LOW);
 
  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  analogWrite(MOTOR_ENA_PIN, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
 
  pinMode(ENCODER_CS_PIN, OUTPUT);
  digitalWrite(ENCODER_CS_PIN, HIGH);
  SPI.begin();
  imuSerial.begin(115200);
 
  if (!rvc.begin(&imuSerial)) {
    Serial.println(F("ERROR: BNO085 not found."));
    while (1) { delay(100); }
  }
  Serial.println(F("BNO085 OK"));
 
  encoderPrev = readEncoderRad();
 
  calibrateZero();   // sets thetaOffset — after this, upright = theta 0
 
  tPrevUs        = micros();
  tPrevImuUs     = tPrevUs;
  lastHzSampleMs = millis();
 
  Serial.println(F("============================================"));
  Serial.println(F("Starting control loop..."));
  Serial.println(F("============================================"));
}
 
// ---------------------------------------------------------------
//  BOOT COUNTER  (identical to v3)
// ---------------------------------------------------------------
void printBootCount() {
  uint16_t n = 0;
  EEPROM.get(EEPROM_BOOT_COUNT_ADDR, n);
  if (n == 0xFFFF) { n = 0; }
  n++;
  EEPROM.put(EEPROM_BOOT_COUNT_ADDR, n);
  Serial.print(F("BOOT #")); Serial.println(n);
  Serial.println(F("(upload a sketch to reset this counter)"));
}
 
// ---------------------------------------------------------------
//  CALIBRATION  (identical to v3)
//  Averages IMU roll over CAL_DURATION_MS while stick is upright.
//  Result stored in thetaOffset so that theta = 0 at the upright.
// ---------------------------------------------------------------
void calibrateZero() {
  Serial.println();
  Serial.println(F(">>> CALIBRATION <<<"));
  Serial.println(F("Hold stick upright. Starting in 3s..."));
 
  for (int i = 3; i > 0; i--) {
    Serial.print(F("  ")); Serial.println(i);
    digitalWrite(CAL_LED_PIN, HIGH);
    unsigned long t0 = millis();
    BNO08x_RVC_Data dump;
    while (millis() - t0 < 500UL) { rvc.read(&dump); }
    digitalWrite(CAL_LED_PIN, LOW);
    t0 = millis();
    while (millis() - t0 < 500UL) { rvc.read(&dump); }
  }
 
  Serial.println(F("Calibrating... (LED solid)"));
  digitalWrite(CAL_LED_PIN, HIGH);
 
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  unsigned long count = 0UL;
  unsigned long tStart = millis();
  BNO08x_RVC_Data h;

  while (millis() - tStart < CAL_DURATION_MS) {
    if (rvc.read(&h)) {
      float r = h.roll * DEG2RAD;
      sumSin += sin(r);
      sumCos += cos(r);
      count++;
    }
  }

  thetaOffset = atan2(sumSin, sumCos);
 
  digitalWrite(CAL_LED_PIN, LOW);
 
  if (count < CAL_MIN_SAMPLES) {
    Serial.print(F("WARNING: only "));
    Serial.print(count);
    Serial.println(F(" samples during cal."));
  }
 
  thetaPrevImu = 0.0f;
 
  // Reset PID state so the integral doesn't start with stale data
  pidIntegral = 0.0f;
 
  Serial.print(F("CAL DONE. samples=")); Serial.print(count);
  Serial.print(F(" rate="));             Serial.print(count / 3.0f, 1);
  Serial.print(F(" Hz  offset="));       Serial.print(thetaOffset, 3);
  Serial.println(F(" deg"));
 
  for (int i = 0; i < 3; i++) {
    digitalWrite(CAL_LED_PIN, HIGH); delay(100);
    digitalWrite(CAL_LED_PIN, LOW);  delay(100);
  }
}
 
// ---------------------------------------------------------------
//  IMU POLLING  (identical to v3)
// ---------------------------------------------------------------
void pollImuUntil(unsigned long targetUs) {
  BNO08x_RVC_Data h;
  while ((long)(micros() - targetUs) < 0) {
    rvc.read(&h);
  }
}
 
// ---------------------------------------------------------------
//  MAIN LOOP
// ---------------------------------------------------------------
void loop() {
  unsigned long tNowUs = micros();
  float dtLoop = (float)(tNowUs - tPrevUs) * 1.0e-6f;
 
  if (dtLoop <= 0.0f || dtLoop > 0.5f) {
    tPrevUs = tNowUs;
    return;
  }
 
  // --- 1. IMU  (identical to v3) ---
  BNO08x_RVC_Data heading;
  imuFresh = false;
  if (rvc.read(&heading)) {
    imuFresh = true;
    imuNewCount++;
    imuNewWindow++;
 
    // NEW (offset subtracted before negation, so zero-crossing is correct):
    float thetaRaw = -(heading.roll * DEG2RAD - thetaOffset);;
    float thetaNew = wrapToPi(thetaRaw);
 
    float dtImu = (float)(tNowUs - tPrevImuUs) * 1.0e-6f;
 
    if (dtImu > 0.0f && dtImu < 0.5f) {
      float dTheta = wrapToPi(thetaNew - thetaPrevImu);
      thetaDotRaw = dTheta / dtImu;
 
      if (thetaDotRaw >  THETA_DOT_MAX) { thetaDotRaw =  THETA_DOT_MAX; }
      if (thetaDotRaw < -THETA_DOT_MAX) { thetaDotRaw = -THETA_DOT_MAX; }
 
      theta_dot = THETA_DOT_ALPHA * thetaDotRaw
                + (1.0f - THETA_DOT_ALPHA) * theta_dot;
    }
 
    theta        = thetaNew;
    thetaPrevImu = thetaNew;
    tPrevImuUs   = tNowUs;
  } else {
    imuHoldCount++;
  }
 
  // --- 2. Encoder  (identical to v3) ---
  float encoderNow = readEncoderRad();
  float delta      = unwrapDelta(encoderNow - encoderPrev);
  omega            = delta / dtLoop;
  encoderPrev      = encoderNow;
  tPrevUs          = tNowUs;
 
  // --- 3. PID control law ---
  //
  //  Reference (setpoint) = 0 rad (calibrated upright position).
  //  Error = reference - measurement = 0 - theta = -theta
  //
  //  Positive error means the stick is leaning in the negative
  //  direction and needs a positive restoring torque.
  //
  //  P term: proportional to error
  //  I term: accumulated error × time (with anti-windup clamp)
  //  D term: rate of change of error = -theta_dot
  //          (we use the filtered theta_dot directly; no separate
  //           error differentiation needed since reference is constant)
 
  pidError = -theta;                             // error = setpoint - theta = 0 - theta
 
  // Integrate only when IMU gave fresh data (avoids integrating stale error)
  if (imuFresh) {
    pidIntegral += pidError * dtLoop;
 
    // Anti-windup: clamp the integral accumulator
    if (pidIntegral >  INTEGRAL_LIMIT) { pidIntegral =  INTEGRAL_LIMIT; }
    if (pidIntegral < -INTEGRAL_LIMIT) { pidIntegral = -INTEGRAL_LIMIT; }
  }
 
  float dError = -theta_dot;   // derivative of error (reference is constant)
 
  pidP = Kp * pidError;
  pidI = Ki * pidIntegral;
  pidD = Kd * dError;
 
  float tau      = pidP + pidI + pidD;
  float tauUnsat = tau;
  tau = constrain(tau, -U_MAX, U_MAX);
 
  // --- 4. Motor ---
  commandMotor(tau);
 
  // --- 5. Update 1-second IMU rate window  (identical to v3) ---
  unsigned long nowMs = millis();
  if (nowMs - lastHzSampleMs >= 1000UL) {
    imuHzMeasured  = (float)imuNewWindow * (1000.0f / (float)(nowMs - lastHzSampleMs));
    imuNewWindow   = 0UL;
    lastHzSampleMs = nowMs;
  }
 
  // --- 6. Telemetry ---
  if (nowMs - lastPrintMs >= (unsigned long)DEBUG_INTERVAL_MS) {
    printDebug(tau, dtLoop);
    if (tauUnsat != tau) {
#if DEBUG_VERBOSE == 1
      Serial.print(F("  [SAT] unsat tau = "));
      Serial.println(tauUnsat, 3);
#endif
    }
    lastPrintMs = nowMs;
  }
 
  loopCounter++;
 
  // --- 7. Pace loop (poll IMU during the wait)  (identical to v3) ---
  unsigned long deadline = tNowUs + LOOP_US;
  pollImuUntil(deadline);
}
 
// ---------------------------------------------------------------
//  DEBUG PRINTER
//  Same format as v3 but shows P/I/D contributions instead of
//  K1*theta / K2*theta_dot / K3*omega.
// ---------------------------------------------------------------
void printDebug(float tau, float dt) {
  float thetaDeg = theta * RAD2DEG;
 
#if DEBUG_VERBOSE == 1
  float totalCycles = (float)(imuNewCount + imuHoldCount);
  float lifetimePct = (totalCycles > 0.0f)
                    ? (100.0f * (float)imuNewCount / totalCycles)
                    : 0.0f;
 
  Serial.print(F("t="));           Serial.print(millis());
  Serial.print(F("ms  loop#="));   Serial.print(loopCounter);
  Serial.print(F("  dt="));        Serial.print(dt * 1000.0f, 2);
  Serial.print(F("ms  imu="));     Serial.print(imuFresh ? F("NEW") : F("hold"));
  Serial.print(F("  rate="));      Serial.print(imuHzMeasured, 0);
  Serial.print(F("Hz life="));     Serial.print(lifetimePct, 0);
  Serial.println(F("%"));
 
  Serial.print(F("  STATE  theta="));    Serial.print(thetaDeg, 2);
  Serial.print(F(" deg  error="));       Serial.print(pidError * RAD2DEG, 2);
  Serial.print(F(" deg  theta_dot="));   Serial.print(theta_dot, 3);
  Serial.print(F(" (raw="));             Serial.print(thetaDotRaw, 3);
  Serial.print(F(")  omega="));          Serial.print(omega, 3);
  Serial.println(F(" rad/s"));
 
#if DEBUG_RAW_ENCODER == 1
  Serial.print(F("  ENC    raw="));  Serial.print(encoderRawNow);
  Serial.print(F("/16384  angle=")); Serial.print(encoderPrev, 4);
  Serial.println(F(" rad"));
#endif
 
#if DEBUG_CONTRIB == 1
  Serial.print(F("  PID    P="));        Serial.print(pidP, 3);
  Serial.print(F("  I="));               Serial.print(pidI, 3);
  Serial.print(F("  D="));               Serial.print(pidD, 3);
  Serial.print(F("  integ="));           Serial.println(pidIntegral, 4);
#endif
 
  Serial.print(F("  CMD    tau="));  Serial.print(tau, 3);
  Serial.print(F("  pwm="));         Serial.print(lastPwm);
  Serial.print(F("/255  dir="));
  if      (lastDir > 0) { Serial.println(F("FWD")); }
  else if (lastDir < 0) { Serial.println(F("REV")); }
  else                  { Serial.println(F("COAST")); }
  Serial.println();
 
#else
  // CSV mode (DEBUG_VERBOSE == 0)
  Serial.print(millis());             Serial.print(',');
  Serial.print(thetaDeg, 2);          Serial.print(',');
  Serial.print(pidError * RAD2DEG, 2); Serial.print(',');
  Serial.print(theta_dot, 3);         Serial.print(',');
  Serial.print(omega, 3);             Serial.print(',');
  Serial.print(pidP, 3);              Serial.print(',');
  Serial.print(pidI, 3);              Serial.print(',');
  Serial.print(pidD, 3);              Serial.print(',');
  Serial.print(pidIntegral, 4);       Serial.print(',');
  Serial.print(tau, 3);               Serial.print(',');
  Serial.print(lastPwm);              Serial.print(',');
  Serial.print(lastDir);              Serial.print(',');
  Serial.print(imuHzMeasured, 1);     Serial.print(',');
  Serial.println(imuFresh ? 1 : 0);
#endif
}
 
// ---------------------------------------------------------------
//  ANGLE HELPERS  (identical to v3)
// ---------------------------------------------------------------
float wrapToPi(float angle) {
  while (angle >  PI_F) { angle -= TWO_PI_F; }
  while (angle < -PI_F) { angle += TWO_PI_F; }
  return angle;
}
 
float unwrapDelta(float delta) {
  if (delta >  TWO_PI_F * 0.5f) { delta -= TWO_PI_F; }
  if (delta < -TWO_PI_F * 0.5f) { delta += TWO_PI_F; }
  return delta;
}
 
// ---------------------------------------------------------------
//  ENCODER + SPI  (identical to v3)
// ---------------------------------------------------------------
float readEncoderRad() {
  spiTransfer16(0xFFFF);
  uint16_t raw    = spiTransfer16(0xC000);
  uint16_t counts = raw & 0x3FFF;
  encoderRawNow   = counts;
  return ((float)counts / COUNTS_PER_REV) * TWO_PI_F;
}
 
uint16_t spiTransfer16(uint16_t data) {
  SPI.beginTransaction(SPISettings(1000000UL, MSBFIRST, SPI_MODE1));
  digitalWrite(ENCODER_CS_PIN, LOW);
  uint8_t hi = SPI.transfer((uint8_t)(data >> 8));
  uint8_t lo = SPI.transfer((uint8_t)(data & 0x00FF));
  digitalWrite(ENCODER_CS_PIN, HIGH);
  SPI.endTransaction();
  return ((uint16_t)hi << 8) | (uint16_t)lo;
}
 
// ---------------------------------------------------------------
//  MOTOR  (identical to v3)
// ---------------------------------------------------------------
void commandMotor(float u) {
  // 1. Determine Direction
  if (u > 0.0f) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    lastDir = 1;
  } else if (u < 0.0f) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    lastDir = -1;
    u = -u; // Use magnitude for PWM calculation
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENA_PIN, 0);
    lastDir = 0;
    lastPwm = 0;
    return;
  }

  // 2. Map Magnitude to PWM (including deadzone compensation)
  const int PWM_MIN = 25; 
  // NEW: map directly from u to [PWM_MIN, 255] without an intermediate zero-crossing:
  int pwm = (int)(PWM_MIN + (255 - PWM_MIN) * (u / U_MAX));

  pwm = constrain(pwm, 0, 255);
  analogWrite(MOTOR_ENA_PIN, pwm);
  lastPwm = pwm;
}
 
 