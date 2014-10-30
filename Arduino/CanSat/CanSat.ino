#include "Wire.h"
#include "SFE_BMP180.h" // https://github.com/sparkfun/BMP180_Breakout/archive/master.zip
#include "ADXL345.h" // http://bildr.org/2011/03/adxl345-arduino/
#include "TinyGPS_Modified.h"

/**
 * Globals
 */

// Uncomment DEBUG to print debug information to serial
// #define DEBUG

// Uncomment SHOW_RAW_GPS to print raw gps data to serial
// #define SHOW_RAW_GPS

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
#else
  #define DEBUG_PRINT(x)
#endif

#define LOG_DELAY 1000 // Milliseconds to wait between logging sensor values

#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_CONFIG_REG_B 0x01
#define HMC5883L_MODE_REG 0x02
#define HMC5883L_FIRST_DATA_REG 0x03
#define HMC5883L_CONT_MEASUREMENT_MODE 0x00

// This is the default with range +- 1.3 Gauss
#define HMC5883L_SCALE 0x01
#define HMC5883L_GAUSS_PER_LSB 0.0092

// This is the maximum with range +- 8.1
// #define HMC5883L_SCALE 0x07
// #define HMC5883L_GAUSS_PER_LSB 4.35

// Using the ADXL345 in Full Resolution mode, so 0.0039 g / LSB
#define ADLX345_RANGE 16
#define ADXL345_G_PER_LSB 0.0039

#define HIH6130_ADDRESS 0x27

// Milliseconds between read requests. If the sensor is reporting stale data,
// this should be adjusted up.
#define HIH6130_READ_DELAY 100

// Magnetometer values in Gauss
float magX = 0.00, magY = 0.00, magZ = 0.00;

// Accelerometer Values in g
float accelX = 0.00, accelY = 0.00, accelZ = 0.00;

// Pressure Sensor Values in Deg C and Millibars (Same as Hectopascals)
double pressureSensorDegC = 0.00, pressureSensorMillibars = 0.00;

byte pressureSensorState = 0;
unsigned long pressureSensorDelay = 0;
unsigned long pressureSensorStart = 0;

// Humidity Sensor Values in Relative Humidity and Deg C
float humiditySensorRelativeHumidity = 0.00, humiditySensorDegC = 0.00;
byte humiditySensorState = 0;
unsigned long humiditySensorLastRead = 0;

// GPS Values
// Lat and Long in Degrees. Alt in meters, Speed in m/s
float gpsLat = 0.0, gpsLon = 0.0, gpsAlt = 0.0, gpsSpeed = 0.0;
unsigned long gpsAge = 0;

// Milliseconds the last time we logged sensor values.
unsigned long lastLogTime = 0;

/**
 * Devices
 */

// GPS (GP-635T)
TinyGPS gps;

// Pressure / Temp (BMP180)
SFE_BMP180 pressureSensor;

// Accelerometer (ADXL345)
ADXL345 accelerometer;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Wire.begin();

  // Init pressure sensor
  if (pressureSensor.begin() == false) {
    DEBUG_PRINT("Failed to initialize pressure sensor");
  }

  // Init accelerometer
  accelerometer.powerOn();
  accelerometer.setRangeSetting(ADLX345_RANGE);

  // This will ensure 3.9 mg / LSB
  accelerometer.setFullResBit(true);

  initializeMagnetometer();

  pinMode(8, OUTPUT);
}

void loop() {
  unsigned long now = millis();

  readPressureSensor(now);
  readMagnetometer();
  readAccelerometer();
  readHumiditySensor(now);

  // Feed data from Serial1 to the GPS
  while(Serial1.available()) {
    char c = Serial1.read();
    gps.encode(c);

    #ifdef SHOW_RAW_GPS
    Serial.print(c);
    #endif
  }
  readGPS();

  if(now - lastLogTime >= LOG_DELAY) {
    lastLogTime = now;
    String sensorData = "";
    buildSensorDataString(sensorData);

    // This will print the sensor data to Serial
    printSensorData(sensorData);

    // This will log the sensor data to a log file
    logSensorData(sensorData);
  }
}

void buildSensorDataString(String &dataString){
  String separator = ",";

  char buf[100];
  buf[0] = 0;
  char width = 7;
  unsigned char precision = 6;

  // elapsed time timestamp
  dataString += dtostrf(millis() / 1000.0, width, precision, buf);
  dataString += separator;

  // Magnetometer
  dataString += dtostrf(magX, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(magY, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(magZ, width, precision, buf);
  dataString += separator;

  // Accelerometer
  dataString += dtostrf(accelX, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(accelY, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(accelZ, width, precision, buf);
  dataString += separator;

  // Pressure Sensor
  dataString += dtostrf(pressureSensorMillibars, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(pressureSensorDegC, width, precision, buf);
  dataString += separator;

  // Humidity Sensor
  dataString += dtostrf(humiditySensorRelativeHumidity, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(humiditySensorDegC, width, precision, buf);
  dataString += separator;

  // GPS
  dataString += dtostrf(gpsLat, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(gpsLon, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(gpsAlt, width, precision, buf);
  dataString += separator;
  dataString += dtostrf(gpsSpeed, width, precision, buf);
}

void logSensorData(String &sensorData) {
  String commandString = "echo '";
  commandString += sensorData;
  commandString += "' >> /home/root/sensor_logs/log.csv";

  // Super wasteful, but we have a lot of memory
  char buf[1000];
  buf[0] = 0;
  commandString.toCharArray(buf, 1000);
  system(buf);
}

void printSensorData(String &sensorData) {
  Serial.println(sensorData);
}

void readPressureSensor(unsigned long now) {
  switch (pressureSensorState) {
    case 0: // Start temp
      pressureSensorDelay = pressureSensor.startTemperature();
      if(pressureSensorDelay > 0){
        pressureSensorStart = now;
        pressureSensorState = 1;
      }else{
        DEBUG_PRINT("Failed to start temp reading from pressure sensor");
        pressureSensorState = 0; // Start over
      }
      break;
    case 1: // Read temp - Start pressure
      if (now - pressureSensorStart >= pressureSensorDelay){
        if (pressureSensor.getTemperature(pressureSensorDegC) == false) {
          DEBUG_PRINT("Failed to read temp from pressure sensor");
          pressureSensorState = 0;
        }else{
          pressureSensorDelay = pressureSensor.startPressure(3);
          if (pressureSensorDelay > 0){
            pressureSensorStart = now;
            pressureSensorState = 2;
          }else{
            DEBUG_PRINT("Failed to start pressure reading from pressure sensor");
            pressureSensorState = 0;
          }
        }
      }
      break;
    case 2: // Read pressure
      if (now - pressureSensorStart >= pressureSensorDelay){
        if (pressureSensor.getPressure( pressureSensorMillibars, pressureSensorDegC ) == false) {
          DEBUG_PRINT("Failed to read pressure from pressure sensor");
        }
        pressureSensorState = 0;
      }
      break;
    default:
      // Shouldn't get here
      DEBUG_PRINT("Error: Invalid pressureSensorState");
      pressureSensorState = 0;
      break;
  }
}

void initializeMagnetometer() {
  // From Sparkfun Example Code:
  // http://sfecdn.s3.amazonaws.com/datasheets/Sensors/Magneto/HMC5883.pde

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(HMC5883L_ADDRESS); // Open communication with HMC5883
  Wire.write(HMC5883L_MODE_REG); // Select mode register
  Wire.write(HMC5883L_CONT_MEASUREMENT_MODE); // Continuous measurement mode
  Wire.endTransmission();

  // Set scale - From data sheet
  Wire.beginTransmission(HMC5883L_ADDRESS); // Open communication with HMC5883
  uint8_t scaleValue = HMC5883L_SCALE;
  // This goes in CRB7, CRB6, and CRB5
  scaleValue = scaleValue << 5;
  Wire.write(HMC5883L_CONFIG_REG_B); // Select config register B
  Wire.write(scaleValue); // Set scale value
  Wire.endTransmission();
}

void readMagnetometer() {
  // From Sparkfun Example Code:
  // http://sfecdn.s3.amazonaws.com/datasheets/Sensors/Magneto/HMC5883.pde

  // Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(HMC5883L_FIRST_DATA_REG); // Select register 3, X MSB register
  Wire.endTransmission();


  // Read data from each axis, 2 registers per axis
  int16_t x, y, z;
  Wire.requestFrom(HMC5883L_ADDRESS, 6);
  if (6<=Wire.available()) {
    x = Wire.read()<<8; // X msb
    x |= Wire.read();   // X lsb
    z = Wire.read()<<8; // Z msb
    z |= Wire.read();   // Z lsb
    y = Wire.read()<<8; // Y msb
    y |= Wire.read();   // Y lsb

    magX = x * HMC5883L_GAUSS_PER_LSB;
    magY = y * HMC5883L_GAUSS_PER_LSB;
    magZ = z * HMC5883L_GAUSS_PER_LSB;
  }
}

void readAccelerometer() {
  int x, y, z;
  accelerometer.readAccel( &x, &y, &z );
  accelX = ((int16_t)x) * ADXL345_G_PER_LSB;
  accelY = ((int16_t)y) * ADXL345_G_PER_LSB;
  accelZ = ((int16_t)z) * ADXL345_G_PER_LSB;
}

void readHumiditySensor(unsigned long now) {
  // Adapted from http://www.phanderson.com/arduino/hih6130.html
  byte Hum_H, Hum_L, Temp_H, Temp_L, status, error;
  uint16_t H_dat, T_dat;

  switch (humiditySensorState) {
    case 0: // Request Measurement
      Wire.beginTransmission(HIH6130_ADDRESS);
      Wire.write(0);
      error = Wire.endTransmission();
      if(error == 0){
        humiditySensorState = 1;
      }
      break;
    case 1: // Read Temp & Humidity
      if (now - humiditySensorLastRead >= HIH6130_READ_DELAY) {
        Wire.requestFrom(HIH6130_ADDRESS, 4);
        Hum_H = Wire.read();
        Hum_L = Wire.read();
        Temp_H = Wire.read();
        Temp_L = Wire.read();
        Wire.endTransmission();

        // Take the left 2 bits as status
        status = (Hum_H >> 6) & 0x03;
        // Take the rest as humidity
        Hum_H = Hum_H & 0x3f;

        if (status == 1) {
          DEBUG_PRINT(
            "Polling humidity sensor faster than it can update. Stale data - adjust delay"
          );
        }else if (status == 3) {
          DEBUG_PRINT("Humidity sensor diagnostic condition. Ignoring data");
          break;
        }

        H_dat = (((uint16_t)Hum_H) << 8) | Hum_L;
        T_dat = (((uint16_t)Temp_H) << 8) | Temp_L;

        // Same as >> 2. Getting rid of the two rightmost bits
        T_dat = T_dat / 4;

        // Convert to Relative Humidity and Deg C
        humiditySensorRelativeHumidity = (float) H_dat * 6.10e-3;
        humiditySensorDegC = (float) T_dat * 1.007e-2 - 40.0;
        humiditySensorLastRead = now;
        humiditySensorState = 0;
      }
      break;
    default:
      // Shouldn't get here
      DEBUG_PRINT("Error: Invalid humiditySensorState");
      humiditySensorState = 0;
      break;
  }
}

void initializeGPS() {
  // Adapted from:
  // https://github.com/mjholmes/basic_tracker/blob/master/Code/gps.ino
  // This sets the GPS to Airborne < 1G mode. We don't expect to have a GPS fix
  // during launch, only when falling. So we want the more accurate airborne
  // mode.

  // Make sure the GPS is in airborne mode. See u-blox protocol spec
  // section 31.10.2 for details of the CFG-NAV5 command and section 24
  // (page 82) for UBX packet structure. u-center binary console can be
  // used to sniff this data rather than having to work it all out.
  uint8_t cfgNav5[] = {
    0xB5, 0x62, 0x06, 0x24, // Sync chars mu and b plus class and id values
    0x24, 0x00, // Length of data (in bytes, little endian) - 36dec
    0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, // Data
    0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x16, 0xDC // 16bit Checksum, 8bit Fletcher (RFC1146) calculated over class,
  };           // id, length and data (not sync chars).

  for(int i = 0; i < sizeof(cfgNav5)/sizeof(uint8_t); i++) {
    Serial1.write(cfgNav5[i]);
  }
}

void readGPS() {
  digitalWrite(8, HIGH);
  delay(100);

  // Get Lat and Long - Check for invalid values
  gps.f_get_position(&gpsLat, &gpsLon, &gpsAge);

  if(gpsAge == TinyGPS::GPS_INVALID_AGE) {
    DEBUG_PRINT("No GPS Fix");

    digitalWrite(8, LOW);
    delay(100);
  } else if (gpsAge > 5000) {
    DEBUG_PRINT("GPS Fix is older than 5 sec. Possible stale data");

    digitalWrite(8, LOW);
    delay(100);
  }

  if(gpsLat == TinyGPS::GPS_INVALID_F_ANGLE) {
    DEBUG_PRINT("GPS: Invalid Lat");
    gpsLat = 0.0;
  }
  if(gpsLon == TinyGPS::GPS_INVALID_F_ANGLE) {
    DEBUG_PRINT("GPS: Invalid Long");
    gpsLon = 0.0;
  }

  // Get Altitude
  gpsAlt = gps.f_altitude();

  if(gpsAlt == TinyGPS::GPS_INVALID_F_ALTITUDE) {
    DEBUG_PRINT("GPS: Invalid Alt");
    gpsAlt = 0.0;
  }

  // Get Speed
  gpsSpeed = gps.f_speed_mps();

  if(gpsSpeed == TinyGPS::GPS_INVALID_F_SPEED) {
    DEBUG_PRINT("GPS: Invalid Speed");
    gpsSpeed = 0.0;
  }
}

// From:
// https://github.com/arduino/Arduino/blob/a2e7413d229812ff123cb8864747558b270498f1/hardware/arduino/sam/cores/arduino/avr/dtostrf.c
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

