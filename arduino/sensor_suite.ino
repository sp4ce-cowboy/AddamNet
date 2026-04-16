/*
 * ================================================================
 *  SWITCH → SENSOR MAPPING  (2PH132524A 4×4 Button Matrix)
 * ================================================================
 *  Physical layout:
 *
 *    [ S1  ] [ S2  ] [ S3  ] [ S4  ]
 *    [ S5  ] [ S6  ] [ S7  ] [ S8  ]
 *    [ S9  ] [ S10 ] [ S11 ] [ S12 ]
 *    [ S13 ] [ S14 ] [ S15 ] [ S16 ]
 *
 *  Each press TOGGLES the state. Current state is always
 *  visible in the "switches" field of the JSON output.
 *
 *  S1  → GP2Y0A02YK0F   Sharp IR distance (sharp_ir_cm)
 *  S2  → HW-201         IR obstacle detection (obstacle_detected)
 *  S3  → MQ135          Air quality (raw_adc, quality)
 *  S4  → MPU-6050       IMU — also pauses velocity/orientation integration
 *  S5  → HC-SR04        Ultrasonic distance (ultrasonic_cm)
 *  S6  → DHT-22         Temperature and humidity
 *  S7  → (spare)
 *  S8  → (spare)
 *  S9  → (spare)
 *  S10 → (spare)
 *  S11 → (spare)
 *  S12 → (spare)
 *  S13 → (spare)
 *  S14 → (spare)
 *  S15 → (spare)
 *  S16 → MASTER         Toggles entire system. Switching back ON
 *                        resets time_since_run to 0 and zeroes all
 *                        MPU position, velocity, and orientation.
 * ================================================================
 *
 * ================================================================
 *  Arduino Mega — Full Sensor Suite
 *  Outputs JSON to Serial @ 115200 baud, 10 Hz
 * ================================================================
 *  SENSORS & PINS
 *  ──────────────────────────────────────────────────────────────
 *  GP2Y0A02YK0F  Sharp IR Distance      → A0
 *  HW-201        IR Obstacle (digital)  → D2
 *  MQ135         Air Quality (analog)   → A1
 *  MPU-6050      IMU (I2C)              → SDA=20, SCL=21
 *  HC-SR04       Ultrasonic             → TRIG=D3, ECHO=D4
 *  DHT-22        Temp & Humidity        → D5  (+ 10kΩ pullup to VCC)
 *  2PH132524A    4×4 Button Matrix      → Rows D6–D9, Cols D10–D13
 *
 *  REQUIRED LIBRARIES  (install via Arduino Library Manager)
 *  ──────────────────────────────────────────────────────────────
 *  • "DHT sensor library" by Adafruit
 *  • "Adafruit Unified Sensor" by Adafruit   (dependency)
 *  • "Keypad" by Mark Stanley & Alexander Brevig
 *  Wire is built-in.
 *
 *  MPU-6050 CALIBRATION NOTE
 *  ──────────────────────────────────────────────────────────────
 *  On every boot the sketch samples 500 MPU readings over ~1 second
 *  while the sensor MUST be completely still and flat. It computes
 *  gyro and accelerometer bias offsets and subtracts them from all
 *  future readings. This eliminates:
 *    • Gyro drift  (was ~2–4 deg/s constant offset -> phantom rotation)
 *    • Accel bias + gravity (was causing runaway velocity/distance)
 *  "mpu6050_calibrated": true in JSON means calibration completed OK.
 *
 *  DHT-22 NOTE
 *  ──────────────────────────────────────────────────────────────
 *  If temperature/humidity read as null:
 *    1. Confirm a 10kΩ resistor sits between DATA (D5) and VCC.
 *       Module boards have it built in; bare sensors do not.
 *    2. Check the D5 jumper — DHT-22 is very sensitive to loose wires.
 *    3. The sensor takes up to 2 s to produce its first valid reading
 *       after power-on; null on the very first frame is normal.
 * ================================================================
 */

#include <Wire.h>
#include <DHT.h>
#include <Keypad.h>

// ──────────────────────────────────────────────────────────────
//  USER CONSTANTS
// ──────────────────────────────────────────────────────────────
#define TETHER_LENGTH_M       100.0f   // m   — tether_exceeded trips here
#define SAMPLE_INTERVAL_MS    1000      // ms  — JSON output period (10 Hz)
#define DHT_READ_INTERVAL_MS  2000     // ms  — DHT-22 min safe read period
#define ACCEL_DEADZONE        0.08f    // m/s² — suppress residual noise after bias removal
#define GYRO_DEADZONE         0.5f    // deg/s — suppress residual noise after bias removal
#define CAL_SAMPLES           500     // readings averaged during MPU calibration
#define SERIAL_BAUD           115200

// ──────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────────────────────────
#define PIN_SHARP_IR          A0
#define PIN_MQ135             A1
#define PIN_IR_OBSTACLE       2
#define PIN_ULTRASONIC_TRIG   3
#define PIN_ULTRASONIC_ECHO   4
#define PIN_DHT               5

// 4x4 button matrix
const byte MAT_ROWS = 4;
const byte MAT_COLS = 4;
byte rowPins[MAT_ROWS] = {6, 7, 8, 9};
byte colPins[MAT_COLS] = {10, 11, 12, 13};

char keyMap[MAT_ROWS][MAT_COLS] = {
  {'1', '2', '3', '4'},
  {'5', '6', '7', '8'},
  {'9', 'A', 'B', 'C'},
  {'D', 'E', 'F', 'G'}
};

// ──────────────────────────────────────────────────────────────
//  MPU-6050 CONSTANTS
// ──────────────────────────────────────────────────────────────
#define MPU6050_ADDR   0x68
#define ACCEL_SCALE    16384.0f   // LSB/g  (+-2 g range)
#define GYRO_SCALE     131.0f    // LSB/(deg/s) (+-250 deg/s range)
#define GRAVITY_MS2    9.80665f

// ──────────────────────────────────────────────────────────────
//  MQ135 THRESHOLDS  (uncalibrated — tune after 24-48 h burn-in)
// ──────────────────────────────────────────────────────────────
#define MQ_NORMAL_MAX   149
#define MQ_CO2_MAX      299
#define MQ_AMMONIA_MAX  449
#define MQ_NOX_MAX      549
#define MQ_ALCOHOL_MAX  649
#define MQ_BENZENE_MAX  749

// ──────────────────────────────────────────────────────────────
//  SENSOR ENABLE FLAGS  [1-15] = S1-S15, [16] = MASTER
// ──────────────────────────────────────────────────────────────
bool sEnabled[17] = {
  true,  // 0  unused
  true,  // 1  S1  GP2Y0A02YK0F
  true,  // 2  S2  HW-201
  true,  // 3  S3  MQ135
  true,  // 4  S4  MPU-6050
  true,  // 5  S5  HC-SR04
  true,  // 6  S6  DHT-22
  true,  // 7  spare
  true,  // 8  spare
  true,  // 9  spare
  true,  // 10 spare
  true,  // 11 spare
  true,  // 12 spare
  true,  // 13 spare
  true,  // 14 spare
  true,  // 15 spare
  true   // 16 MASTER
};

bool masterEnabled = true;

// ──────────────────────────────────────────────────────────────
//  OBJECTS
// ──────────────────────────────────────────────────────────────
DHT    dht(PIN_DHT, DHT11);
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, MAT_ROWS, MAT_COLS);

// ──────────────────────────────────────────────────────────────
//  TIMING
// ──────────────────────────────────────────────────────────────
unsigned long lastSampleMs = 0;
unsigned long lastDHTMs    = 0;
unsigned long lastMPUMs    = 0;
unsigned long runStartMs   = 0;

// ──────────────────────────────────────────────────────────────
//  SENSOR STATE
// ──────────────────────────────────────────────────────────────
float dhtTemp      = NAN;
float dhtHumidity  = NAN;

float sharpDist_cm = -1.0f;
bool  irObstacle   = false;
float sonarDist_cm = -1.0f;
int   mq135Raw     = 0;

// MPU-6050 calibration offsets (computed once at boot, sensor must be still)
float accelBias[3]  = {0, 0, 0};  // m/s^2 — full measured value at rest (incl. gravity on Z)
float gyroBias[3]   = {0, 0, 0};  // deg/s — measured rotation rate at rest
bool  mpuCalibrated = false;

// MPU-6050 live values (bias-corrected)
float accel[3]    = {0, 0, 0};  // m/s^2  (gravity removed after calibration)
float gyroRate[3] = {0, 0, 0};  // deg/s
float vel[3]      = {0, 0, 0};  // m/s   (integrated)
float pos[3]      = {0, 0, 0};  // m     (integrated)
float orient[3]   = {0, 0, 0};  // deg   roll/pitch/yaw
float totalDist_m = 0.0f;


// ════════════════════════════════════════════════════════════════
//  MPU-6050 — LOW LEVEL READ
//  Returns false if the I2C transaction failed.
// ════════════════════════════════════════════════════════════════
bool mpuReadRaw(int16_t raw[7]) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return false;
  for (int i = 0; i < 7; i++) {
    raw[i] = ((int16_t)Wire.read() << 8) | Wire.read();
  }
  // raw[0..2] = Ax Ay Az,  raw[3] = chip temp (unused),  raw[4..6] = Gx Gy Gz
  return true;
}


// ════════════════════════════════════════════════════════════════
//  MPU-6050 — INIT
// ════════════════════════════════════════════════════════════════
void mpuInit() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // Wake from sleep
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);   // ACCEL_CONFIG: +-2 g
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);   // GYRO_CONFIG: +-250 deg/s
  Wire.write(0x00);
  Wire.endTransmission(true);

  lastMPUMs = millis();
}


// ════════════════════════════════════════════════════════════════
//  MPU-6050 — CALIBRATION
//
//  Call once at boot with the sensor completely still and flat.
//  Collects CAL_SAMPLES readings and computes the mean of each
//  axis as the bias offset to subtract from all future readings.
//
//  Accelerometer: at rest flat, true linear acceleration = 0 on
//  all axes (gravity is not linear movement). So we store the
//  full measured value on every axis as the bias — including
//  gravity on Z — and subtract it all before integration. This
//  cleanly removes both the hardware offset AND gravity in one
//  step, without needing to know the sensor orientation.
//
//  Gyroscope: at rest, true rotation = 0 deg/s. The measured
//  non-zero values (e.g. -3.4 deg/s on Y) are pure bias noise.
// ════════════════════════════════════════════════════════════════
void mpuCalibrate() {
  Serial.println(F("{\"status\":\"MPU_CALIBRATING\","
                   "\"note\":\"Keep sensor still and flat for 1 second\"}"));

  double sumA[3] = {0, 0, 0};
  double sumG[3] = {0, 0, 0};
  int    good    = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    int16_t raw[7];
    if (mpuReadRaw(raw)) {
      for (int j = 0; j < 3; j++) {
        sumA[j] += (double)raw[j]     / ACCEL_SCALE * GRAVITY_MS2;
        sumG[j] += (double)raw[4 + j] / GYRO_SCALE;
      }
      good++;
    }
    delay(2);   // ~2 ms per sample -> ~1 second total for 500 samples
  }

  if (good < CAL_SAMPLES / 2) {
    Serial.println(F("{\"status\":\"MPU_CAL_FAILED\","
                     "\"note\":\"Too many I2C errors. Check wiring.\"}"));
    mpuCalibrated = false;
    return;
  }

  for (int i = 0; i < 3; i++) {
    accelBias[i] = (float)(sumA[i] / good);
    gyroBias[i]  = (float)(sumG[i] / good);
  }

  mpuCalibrated = true;

  Serial.print(F("{\"status\":\"MPU_CAL_DONE\""
                 ",\"accel_bias_ms2\":{\"x\":"));
  Serial.print(accelBias[0], 4);
  Serial.print(F(",\"y\":")); Serial.print(accelBias[1], 4);
  Serial.print(F(",\"z\":")); Serial.print(accelBias[2], 4);
  Serial.print(F("},\"gyro_bias_degs\":{\"x\":"));
  Serial.print(gyroBias[0], 4);
  Serial.print(F(",\"y\":")); Serial.print(gyroBias[1], 4);
  Serial.print(F(",\"z\":")); Serial.print(gyroBias[2], 4);
  Serial.println(F("}}"));
}


// ════════════════════════════════════════════════════════════════
//  MPU-6050 — RESET INTEGRATION STATE
// ════════════════════════════════════════════════════════════════
void mpuReset() {
  for (int i = 0; i < 3; i++) {
    vel[i] = 0; pos[i] = 0; orient[i] = 0;
    accel[i] = 0; gyroRate[i] = 0;
  }
  totalDist_m = 0.0f;
  lastMPUMs   = millis();
}


// ════════════════════════════════════════════════════════════════
//  MPU-6050 — READ & INTEGRATE
//
//  Called every loop iteration for best dead-reckoning resolution.
//
//  After bias subtraction:
//    accel[0..2]    = true linear acceleration (m/s^2, gravity removed)
//    gyroRate[0..2] = true rotation rate (deg/s)
//
//  Note: dead-reckoning from a consumer MEMS IMU still drifts over
//  minutes. For best long-term accuracy, fuse with wheel encoders.
// ════════════════════════════════════════════════════════════════
void mpuReadAndIntegrate() {
  int16_t raw[7];
  if (!mpuReadRaw(raw)) return;

  unsigned long now = millis();
  float dt = (float)(now - lastMPUMs) / 1000.0f;
  lastMPUMs = now;
  if (dt <= 0.0f || dt > 0.5f) return;

  // Convert and subtract calibration bias.
  // For accel: bias captures hardware offset + gravity at rest,
  // so subtracting it removes both simultaneously.
  for (int i = 0; i < 3; i++) {
    accel[i]    = ((float)raw[i]     / ACCEL_SCALE * GRAVITY_MS2) - accelBias[i];
    gyroRate[i] = ((float)raw[4 + i] / GYRO_SCALE)                - gyroBias[i];
  }

  // Gyro integration -> orientation
  for (int i = 0; i < 3; i++) {
    float g = gyroRate[i];
    if (fabsf(g) < GYRO_DEADZONE) g = 0.0f;
    orient[i] += g * dt;
    while (orient[i] >  180.0f) orient[i] -= 360.0f;
    while (orient[i] < -180.0f) orient[i] += 360.0f;
  }

  // Accel integration -> velocity -> position
  accel[0] = roundf(accel[0] * 10.0f) / 10.0f;
  accel[1] = roundf(accel[1] * 10.0f) / 10.0f;
  accel[2] = roundf(accel[2] * 10.0f) / 10.0f;
  float ax = accel[0];
  float ay = accel[1];
  float az = accel[2];
  if (fabsf(ax) < ACCEL_DEADZONE) ax = 0.0f;
  if (fabsf(ay) < ACCEL_DEADZONE) ay = 0.0f;
  if (fabsf(az) < ACCEL_DEADZONE) az = 0.0f;

  

  vel[0] = roundf(vel[0] * 10.0f) / 10.0f;
  vel[1] = roundf(vel[1] * 10.0f) / 10.0f;
  vel[2] = roundf(vel[2] * 10.0f) / 10.0f;
  vel[0] += ax * dt;
  vel[1] += ay * dt;
  vel[2] += az * dt;

  pos[0] += vel[0] * dt;
  pos[1] += vel[1] * dt;
  pos[2] += vel[2] * dt;

  totalDist_m = sqrtf(pos[0]*pos[0] + pos[1]*pos[1]);
}


// ════════════════════════════════════════════════════════════════
//  SHARP IR DISTANCE  (GP2Y0A02YK0F, valid 20-150 cm)
// ════════════════════════════════════════════════════════════════
float readSharpIR() {
  int raw = analogRead(PIN_SHARP_IR);
  if (raw < 10) return -1.0f;
  float volts = raw * (5.0f / 1023.0f);
  if (volts < 0.35f) return -1.0f;
  float d = 53.0f * powf(volts, -1.10f);
  return constrain(d, 20.0f, 150.0f);
}


// ════════════════════════════════════════════════════════════════
//  ULTRASONIC DISTANCE  (HC-SR04, valid 2-400 cm)
// ════════════════════════════════════════════════════════════════
float readUltrasonic() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 25000UL);
  if (duration == 0) return -1.0f;
  float d = duration * 0.01715f;
  return (d > 400.0f) ? -1.0f : d;
}


// ════════════════════════════════════════════════════════════════
//  MQ135 AIR QUALITY CLASSIFICATION
// ════════════════════════════════════════════════════════════════
const char* classifyMQ135(int raw) {
  if (raw <= MQ_NORMAL_MAX)  return "Normal";
  if (raw <= MQ_CO2_MAX)     return "CO2";
  if (raw <= MQ_AMMONIA_MAX) return "Ammonia";
  if (raw <= MQ_NOX_MAX)     return "NOx";
  if (raw <= MQ_ALCOHOL_MAX) return "Alcohol";
  if (raw <= MQ_BENZENE_MAX) return "Benzene";
  return "Smoke";
}


// ════════════════════════════════════════════════════════════════
//  BUTTON MATRIX
// ════════════════════════════════════════════════════════════════
int keyCharToSwitch(char k) {
  if (k >= '1' && k <= '9') return k - '0';
  switch (k) {
    case 'A': return 10; case 'B': return 11; case 'C': return 12;
    case 'D': return 13; case 'E': return 14; case 'F': return 15;
    case 'G': return 16;
  }
  return -1;
}

void handleKeypad() {
  char key = keypad.getKey();
  if (key == NO_KEY) return;
  int sw = keyCharToSwitch(key);
  if (sw < 1 || sw > 16) return;

  if (sw == 16) {
    masterEnabled = !masterEnabled;
    if (masterEnabled) {
      runStartMs = millis();
      mpuReset();
      Serial.println(F("{\"event\":\"MASTER_ON\",\"time_since_run_s\":0.000}"));
    } else {
      Serial.println(F("{\"event\":\"MASTER_OFF\"}"));
    }
  } else {
    sEnabled[sw] = !sEnabled[sw];
  }
}


// ════════════════════════════════════════════════════════════════
//  JSON OUTPUT
// ════════════════════════════════════════════════════════════════
static void pFloat(float v, int decimals = 1) {
  if (isnan(v) || v < 0.0f) Serial.print(F("null"));
  else Serial.print(v, decimals);
}

void printJSON() {
  unsigned long now = millis();

  float uptime_s       = (float)now / 1000.0f;
  float timeSinceRun_s = (float)(now - runStartMs) / 1000.0f;
  float speed_ms       = sqrtf(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
  bool  tetherExceeded = (totalDist_m >= TETHER_LENGTH_M);

  float dSum = 0.0f; int dCnt = 0;
  if (sEnabled[1] && sharpDist_cm > 0) { dSum += sharpDist_cm; dCnt++; }
  if (sEnabled[5] && sonarDist_cm > 0) { dSum += sonarDist_cm; dCnt++; }
  float avgDist = (dCnt > 0) ? dSum / dCnt : -1.0f;

  Serial.print(F("{"));

  // Timestamps
  Serial.print(F("\"uptime_s\":")); Serial.print(uptime_s, 3);
  Serial.print(F(",\"time_since_run_s\":")); Serial.print(timeSinceRun_s, 3);
  Serial.print(F(",\"master_enabled\":"));
  Serial.print(masterEnabled ? F("true") : F("false"));

  // DHT-22
  Serial.print(F(",\"dht22\":{\"enabled\":"));
  Serial.print(sEnabled[6] ? F("true") : F("false"));
  if (sEnabled[6]) {
    Serial.print(F(",\"temperature_c\":"));
    if (isnan(dhtTemp)) Serial.print(F("null")); else Serial.print(dhtTemp, 1);
    Serial.print(F(",\"humidity_pct\":"));
    if (isnan(dhtHumidity)) Serial.print(F("null")); else Serial.print(dhtHumidity, 1);
  }
  Serial.print(F("}"));

  // Distance
  Serial.print(F(",\"distance\":{\"hw201_ir\":{\"enabled\":"));
  Serial.print(sEnabled[2] ? F("true") : F("false"));
  if (sEnabled[2]) {
    Serial.print(F(",\"obstacle_detected\":"));
    Serial.print(irObstacle ? F("true") : F("false"));
  }
  Serial.print(F("}"));
  Serial.print(F(",\"sharp_ir_cm\":"));
  if (sEnabled[1]) pFloat(sharpDist_cm, 1); else Serial.print(F("null"));
  Serial.print(F(",\"ultrasonic_cm\":"));
  if (sEnabled[5]) pFloat(sonarDist_cm, 1); else Serial.print(F("null"));
  Serial.print(F(",\"average_cm\":")); pFloat(avgDist, 1);
  Serial.print(F("}"));

  // MPU-6050
  Serial.print(F(",\"mpu6050\":{\"enabled\":"));
  Serial.print(sEnabled[4] ? F("true") : F("false"));
  Serial.print(F(",\"calibrated\":"));
  Serial.print(mpuCalibrated ? F("true") : F("false"));
  if (sEnabled[4]) {
    Serial.print(F(",\"accel_ms2\":{\"x\":"));
    Serial.print(accel[0], 1);
    Serial.print(F(",\"y\":")); Serial.print(accel[1], 1);
    Serial.print(F(",\"z\":")); Serial.print(accel[2], 1);
    Serial.print(F("}"));

    Serial.print(F(",\"gyro_degs\":{\"x\":"));
    Serial.print(gyroRate[0], 1);
    Serial.print(F(",\"y\":")); Serial.print(gyroRate[1], 1);
    Serial.print(F(",\"z\":")); Serial.print(gyroRate[2], 1);
    Serial.print(F("}"));

    Serial.print(F(",\"velocity_ms\":{\"x\":"));
    Serial.print(vel[0], 1);
    Serial.print(F(",\"y\":")); Serial.print(vel[1], 1);
    Serial.print(F(",\"z\":")); Serial.print(vel[2], 1);
    Serial.print(F(",\"speed\":")); Serial.print(speed_ms, 1);
    Serial.print(F("}"));

    Serial.print(F(",\"orientation_deg\":{\"roll\":"));
    Serial.print(orient[0], 2);
    Serial.print(F(",\"pitch\":")); Serial.print(orient[1], 2);
    Serial.print(F(",\"yaw\":")); Serial.print(orient[2], 2);
    Serial.print(F("}"));

    Serial.print(F(",\"total_distance_m\":")); Serial.print(totalDist_m, 3);
    Serial.print(F(",\"tether_exceeded\":"));
    Serial.print(tetherExceeded ? F("true") : F("false"));
  }
  Serial.print(F("}"));

  // MQ135
  Serial.print(F(",\"mq135\":{\"enabled\":"));
  Serial.print(sEnabled[3] ? F("true") : F("false"));
  if (sEnabled[3]) {
    Serial.print(F(",\"raw_adc\":")); Serial.print(mq135Raw);
    Serial.print(F(",\"quality\":\""));
    Serial.print(classifyMQ135(mq135Raw));
    Serial.print(F("\""));
  }
  Serial.print(F("}"));

  /*
  // Switch states
  Serial.print(F(",\"switches\":{"));
  for (int i = 1; i <= 16; i++) {
    Serial.print(F("\"S")); Serial.print(i); Serial.print(F("\":"));
    Serial.print(sEnabled[i] ? F("true") : F("false"));
    if (i < 16) Serial.print(F(","));
  }
  Serial.print(F("}"));
  */
  
  Serial.println(F("}"));
}


// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000L);

  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  pinMode(PIN_IR_OBSTACLE, INPUT);

  Serial.println(F("{\"status\":\"INITIALIZING\","
                   "\"note\":\"MQ135 needs 24-48h burn-in. Keep MPU still during cal.\"}"));

  dht.begin();
  mpuInit();

  // Give DHT-22 time to power up before the first read attempt
  delay(2000);

  // Calibrate MPU — sensor must be completely still and flat right now
  mpuCalibrate();

  // Warm up DHT — retry a few times since the first read often returns NAN
  for (int attempt = 0; attempt < 5; attempt++) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      dhtTemp     = t;
      dhtHumidity = h;
      break;
    }
    delay(2100);  // DHT-22 needs >2 s between reads
  }

  unsigned long now = millis();
  runStartMs   = now;
  lastMPUMs    = now;
  lastDHTMs    = now;
  lastSampleMs = now;

  Serial.println(F("{\"status\":\"READY\"}"));
}


// ════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Button matrix — always polled
  handleKeypad();

  // MPU-6050 — integrate every loop for max time resolution
  if (masterEnabled && sEnabled[4]) {
    mpuReadAndIntegrate();
  }

  // DHT-22 — slow sensor, read every 2 s, only keep valid reads
  if (masterEnabled && sEnabled[6] && (now - lastDHTMs) >= DHT_READ_INTERVAL_MS) {
    lastDHTMs = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) dhtTemp     = t;
    if (!isnan(h)) dhtHumidity = h;
  }

  // 10 Hz JSON output
  if ((now - lastSampleMs) >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;

    if (masterEnabled) {
      if (sEnabled[1]) sharpDist_cm = readSharpIR();
      if (sEnabled[2]) irObstacle   = !digitalRead(PIN_IR_OBSTACLE);
      if (sEnabled[3]) mq135Raw     = analogRead(PIN_MQ135);
      if (sEnabled[5]) sonarDist_cm = readUltrasonic();
      printJSON();
    } else {
      Serial.print(F("{\"status\":\"MASTER_DISABLED\",\"uptime_s\":"));
      Serial.print((float)now / 1000.0f, 3);
      Serial.println(F("}"));
    }
  }
}
