// ======= Partnerens deler (bevart) =======
#include <camera/camera_core.hpp>
#include "IIR_Filter.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <Wire.h>

// IMU (partnerens valg)
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// GNSS (SparkFun u-blox v3)
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// ======= Vårt rammeverk (bevares som i prosjektet) =======
#include "config/PinConfig.h"
#include "core/StatusLED.h"
#include "core/SafetyInterlock.h"
#include "recovery/RecoveryLogic.h"
#include "logging/DataLogger.h"
#include "util/Time.h"
#include "util/ComplementaryFilter.h"

// ---------------- PINS (partnerens) ----------------
#define LED   GPIO_NUM_4
#define Motor1 GPIO_NUM_0
#define Motor2 GPIO_NUM_13
#define Motor3 GPIO_NUM_2
#define Motor4 GPIO_NUM_4
#define SCL    GPIO_NUM_15
#define SDA    GPIO_NUM_14

// URL for data (ikke brukt i denne loopen, beholdt)
const char* JS1V = "http://192.168.4.1/JS1V";

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535;

// ====== Hjelpefunksjoner (partnerens) ======
void i2cScanAdresses(){
  // OK å kalle begin på nytt med samme pinner på ESP32
  Wire.begin(SDA, SCL);

  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning I2C...");
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device at 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error==4) {
      Serial.print("Unknown error at 0x");
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found");
  else Serial.println("Scan done");
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverName);
  int httpResponseCode = http.GET();
  String payload = "--";
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: "); Serial.println(httpResponseCode);
    payload = http.getString();
  } else {
    Serial.print("Error code: "); Serial.println(httpResponseCode);
  }
  http.end();
  return payload;
}

void WifiInit(){
  WiFi.begin("esp-captive", "xcvbnmkl");
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  analogWrite(LED,10);
  delay(1000);
  analogWrite(LED,0);
}

void DisplayImuSettings(){
  while (!Serial) delay(10);

  Serial.println("Adafruit ICM20948 test!");
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) { delay(10); }
  }
  Serial.println("ICM20948 Found!");

  Serial.print("Accelerometer range: ");
  switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:  Serial.println("+-2G");  break;
    case ICM20948_ACCEL_RANGE_4_G:  Serial.println("+-4G");  break;
    case ICM20948_ACCEL_RANGE_8_G:  Serial.println("+-8G");  break;
    case ICM20948_ACCEL_RANGE_16_G: Serial.println("+-16G"); break;
  }

  Serial.print("Gyro range: ");
  switch (icm.getGyroRange()) {
    case ICM20948_GYRO_RANGE_250_DPS:  Serial.println("250 dps");  break;
    case ICM20948_GYRO_RANGE_500_DPS:  Serial.println("500 dps");  break;
    case ICM20948_GYRO_RANGE_1000_DPS: Serial.println("1000 dps"); break;
    case ICM20948_GYRO_RANGE_2000_DPS: Serial.println("2000 dps"); break;
  }

  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125.0f / (1.0f + accel_divisor);
  Serial.print("Accel ODR ~ "); Serial.print(accel_rate); Serial.println(" Hz");

  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100.0f / (1.0f + gyro_divisor);
  Serial.print("Gyro  ODR ~ "); Serial.print(gyro_rate); Serial.println(" Hz");

  Serial.print("Mag data rate: ");
  switch (icm.getMagDataRate()) {
    case AK09916_MAG_DATARATE_SHUTDOWN: Serial.println("Shutdown"); break;
    case AK09916_MAG_DATARATE_SINGLE:   Serial.println("Single");   break;
    case AK09916_MAG_DATARATE_10_HZ:    Serial.println("10 Hz");    break;
    case AK09916_MAG_DATARATE_20_HZ:    Serial.println("20 Hz");    break;
    case AK09916_MAG_DATARATE_50_HZ:    Serial.println("50 Hz");    break;
    case AK09916_MAG_DATARATE_100_HZ:   Serial.println("100 Hz");   break;
  }
  Serial.println();
}

// ====== Partnerens IIR-filter og hjelpevariabler ======
#include "IIR_Filter.hpp"
IIR_Filter head(0.5f);
float heading = 0.0f;
IIR_Filter g(0.1f);
float grav = 0.0f;

double currentTime = 0;
double DT = 0;

void PrintData(){
  static double prevTime = 0;
  static double elapsedTime = 0;
  static const int interval_ms = 100;

  currentTime = millis();
  DT = currentTime - prevTime;
  elapsedTime += DT;

  if (elapsedTime >= interval_ms){
    // Fiks: bruk riktig Serial.print (ikke "str" + int)
    Serial.print(F("Fix: "));        Serial.println(myGNSS.getFixType());
    Serial.print(F("Satellites: ")); Serial.println(myGNSS.getSIV());

    long alt_mm = myGNSS.getAltitudeMSL(); // mm over MSL
    Serial.print(F("Altitude (m): ")); Serial.println(alt_mm / 1000.0f);

    long lat_e7 = myGNSS.getLatitude();
    long lon_e7 = myGNSS.getLongitude();
    Serial.print(F("Latitude : ")); Serial.println(lat_e7 / 1e7, 7);
    Serial.print(F("Longitude: ")); Serial.println(lon_e7 / 1e7, 7);

    long gs_mms = myGNSS.getGroundSpeed(); // mm/s
    Serial.print(F("Speed (m/s): ")); Serial.println(gs_mms / 1000.0f);

    Serial.print(F("Heading (rad): ")); Serial.println(heading);
    Serial.print(F("Z m/s^2: "));       Serial.println(grav);
    Serial.println();

    elapsedTime = 0;
  }
  prevTime = currentTime;
}

// ================== Vårt subsystem (innkapslet bruk) ==================
StatusLED leds;
SafetyInterlock interlock;
RecoveryLogic recovery;
DataLogger logger;
ComplementaryFilter attitude;

// Numerisk derivat for høyde -> vertikal hastighet
struct Differentiator {
  float prev = 0.0f;
  uint64_t t_prev = 0;
  float operator()(float x, uint64_t t_us) {
    float v = 0.0f;
    if (t_prev != 0) {
      float dt = (t_us - t_prev) / 1e6f;
      if (dt > 1e-4f) v = (x - prev) / dt;
    }
    prev = x; t_prev = t_us; return v;
  }
} d_alt;

struct RmsWindow {
  float acc = 0.0f; uint16_t n = 0;
  void reset(){ acc=0; n=0; }
  void add(float x){ acc += x*x; n++; }
  float rms() const { return (n>0) ? sqrtf(acc / n) : 0.0f; }
} gyro_rms;

static DeltaTime dtimer;
static bool logger_header_done = false;
static bool gnss_baseline_set = false;
static float gnss_alt0_m = 0.0f; // baseline for relativ høyde

// ================== setup/loop ==================
void setup(){
  Serial.begin(115200);
  // WifiInit();  // valgfritt, beholdt men ikke aktivert
  i2cScanAdresses();

  // I2C init (sørg for riktige pinner)
  Wire.begin(SDA, SCL);
  delay(50);

  DisplayImuSettings();

  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS NOT detected at default I2C address. Check wiring."));
    // Ikke freeze – la resten kjøre for benk/logikk
  }

  // Vårt subsystem
  logger.begin();
  leds.begin();
  leds.set(LedState::PROCESSING);
  interlock.begin();
  recovery.reset();

  if (!logger_header_done) { logger.logHeader(); logger_header_done = true; }
}

void loop(){
  // ===== Partnerens sensormåling =====
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // 100 ms aktiviteter (partnerens IIR)
  static uint32_t last_iir_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_iir_ms >= 100) {
    last_iir_ms = now_ms;
    heading = head.filter(atan2f(mag.magnetic.x, mag.magnetic.y));
    grav    = g.filter(accel.acceleration.z);
  }

  myGNSS.checkUblox(); // spill inn nye bytes
  PrintData();         // partnerens konsoll-utskrifter

  // ===== Vårt subsystem (tilstand, holdning, logging) =====
  float dt = dtimer.step_s();
  if (dt <= 0) return;

  interlock.update();

  // Attitude fra ICM-gyro (rad/s) + akselerometer (m/s^2)
  attitude.update(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, dt);
  float roll=0, pitch=0, yaw=0;
  attitude.getEuler(roll, pitch, yaw);

  // Relativ høyde: bruk GNSS MSL - baseline (hvis tilgjengelig); fallback = 0
  float alt_rel_m = 0.0f;
  uint64_t t_us = micros();
  long alt_mm = myGNSS.getAltitudeMSL(); // mm
  if (alt_mm != 0) {
    float alt_m = alt_mm / 1000.0f;
    if (!gnss_baseline_set) { gnss_alt0_m = alt_m; gnss_baseline_set = true; }
    alt_rel_m = alt_m - gnss_alt0_m;
  }

  float v_speed = d_alt(alt_rel_m, t_us);

  // Bevegelses-/stabilitetsmål
  float gyro_norm_dps = sqrtf(gyro.gyro.x*gyro.gyro.x + gyro.gyro.y*gyro.gyro.y + gyro.gyro.z*gyro.gyro.z) * RAD_TO_DEG;
  float accel_norm    = sqrtf(accel.acceleration.x*accel.acceleration.x +
                              accel.acceleration.y*accel.acceleration.y +
                              accel.acceleration.z*accel.acceleration.z);

  // RMS-vindu ~0.5s
  static uint32_t last_rms_ms = 0;
  gyro_rms.add(gyro_norm_dps);
  if (millis() - last_rms_ms > 500) {
    last_rms_ms = millis();
    float stability = gyro_rms.rms();
    gyro_rms.reset();

    // Faseoppdatering
    FlightPhase ph = recovery.update(alt_rel_m, v_speed, gyro_norm_dps);

    // LED-policy
    // (IMU antas OK — hvis ikke, ville DisplayImuSettings ha feilet)
    if (recovery.landed())          leds.set(LedState::READY);
    else if (interlock.isHardwareArmed()) leds.set(LedState::PROCESSING);
    else                            leds.set(LedState::WARNING);

    // Logging (CSV til serie / SD ut fra build-flags)
    logger.logSample(
      t_us,
      static_cast<uint8_t>(ph),
      interlock.isHardwareArmed(),
      alt_rel_m,
      v_speed,
      roll, pitch, yaw,
      gyro_norm_dps,
      accel_norm,
      stability
    );
  }

  leds.update();
  // loop-hastighet: ~kHz granulat; dt styrer estimat/deriv
  delayMicroseconds(1000);
}
