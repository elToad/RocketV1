#include <camera/camera_core.hpp>
#include "IIR_Filter.hpp"
#include <Arduino.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
//IMU
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
//GPS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library:  http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;


//Changed Camera library "sensor_t" to "sensor_ct"

//PINS
#define LED GPIO_NUM_4
#define Motor1 GPIO_NUM_0
#define Motor2 GPIO_NUM_13
#define Motor3 GPIO_NUM_2
#define Motor4 GPIO_NUM_4
#define SCL GPIO_NUM_15
#define SDA GPIO_NUM_14

//URL for data
const char* JS1V = "http://192.168.4.1/JS1V";
// const char* serverNameHumi = "http://192.168.4.1/humidity";
// const char* serverNamePres = "http://192.168.4.1/pressure";

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

void i2cScanAdresses(){
  Wire.begin(SDA,SCL);

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
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
  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();

}

IIR_Filter head(0.5);
float heading;

IIR_Filter g(0.1);
float grav;


double currentTime;
double DT;

void PrintData(){

  static double prevTime = 0;
  static double elapsedTime = 0;
  static int interval = 1*100;

  currentTime = millis();
  DT = currentTime - prevTime;
  elapsedTime += DT;

  if (elapsedTime >= interval){

    Serial.println("Fix : " + myGNSS.getFixType());
    Serial.println("Sattelites : " + myGNSS.getSIV());
    Serial.println("Altitude: " + myGNSS.getAltitude()/1000);
    Serial.println("Latitude : " + myGNSS.getLatitude());
    Serial.println("Longitude : " + myGNSS.getLongitude());
    Serial.println("m/s : " + myGNSS.getGroundSpeed()/1000);


    Serial.print("Heading: ");
    Serial.print(heading);
    Serial.println("");

    Serial.print("Z m/s^2 : ");
    Serial.println(grav);
    Serial.println("");

    elapsedTime = 0;
  }

  prevTime = currentTime;
}


void setup(){
  Serial.begin(115200);

  //CameraSetup();
  i2cScanAdresses();

  DisplayImuSettings();
  //icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  //icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  //icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);

    if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //This will pipe all NMEA sentences to the serial port so we can see them

  //myGNSS.setNMEAOutputPort(Serial);

}


void loop(){
  // int spd = httpGETRequest(JS1V).toInt();
  // spd = map(spd,0,255,1000,2000);
  // M1.writeMicroseconds(spd);
  // Serial.println(spd);

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  
  //when 0 y is pointing north

  static double prevTime = 0;
  static double elapsedTime = 0;
  static int interval = 100;

  currentTime = millis();
  DT = currentTime - prevTime;
  elapsedTime += DT;

  if (elapsedTime >= interval){
    heading = head.filter(atan2(mag.magnetic.x,mag.magnetic.y));
    grav = g.filter(accel.acceleration.z);myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
    myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
    elapsedTime = 0;
  }

  prevTime = currentTime;

 

  PrintData();

  // if (grav <= 0){
  //   analogWrite(LED,5);
  // }
  // else{
  //   analogWrite(LED,0);
  // }


  // Serial.print("\t\tTemperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  // Serial.print("\t\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.println(" uT");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();

  // delay(100);

  //  Serial.print(temp.temperature);
  //
  //  Serial.print(",");
  //
  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //
  //  Serial.print(",");
  //  Serial.print(gyro.gyro.x);
  //  Serial.print(","); Serial.print(gyro.gyro.y);
  //  Serial.print(","); Serial.print(gyro.gyro.z);
  //
  //  Serial.print(",");
  //  Serial.print(mag.magnetic.x);
  //  Serial.print(","); Serial.print(mag.magnetic.y);
  //  Serial.print(","); Serial.print(mag.magnetic.z);

  //  Serial.println();
  //
  //  delayMicroseconds(measurement_delay_us);

}

