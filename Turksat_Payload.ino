#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Wire.h>
#include <utility/imumaths.h>


#define MS 10
#define MS_B 1000
#define buzzer 25



float dt = MS / 1000.0;
float dt_b = MS_B / 1000.0;
double prevLineerZ = 0;
///////////HYPSOMETRIC FORMULA & SIMP//////////
float temp = 288.15;
float sbt = 0.0065;
float Po = 101325;
float sbt_pow = 0.1902;
double TEMP = 15;
long int SIMP;
int altitude_simp = 0;
///////////HYPSOMETRIC FORMULA & SIMP/////////

unsigned long next_t = 0L;

//////////////  EEPROM Defines //////////////
#define state_low_address 10
#define state_high_address 11

#define satellite_low_address 12
#define satellite_high_address 13

#define package_count_low_address 20
#define package_count_high_address 21

#define referance_altitude_low_address 24
#define referance_altitude_high_address 25
//////////////  EEPROM Defines //////////////


//////////////  INIT //////////////
TinyGPSPlus gps;
File myFile;
Adafruit_BMP280 bmp;
const int chipSelect = BUILTIN_SDCARD;
//////////////  INIT //////////////



//////////////  Time Variables //////////////
long long int presentTime = millis();
long long int prevTime = 0;
long long int altitudeTime = millis();
long long int prevAltitudeTime = 0;
long long int prevLastStateTimer = 0;
long long int lastStateTimer;
long long int velocityTimer = millis();
long long int velocityPrevTime = 0;
////////////// Time Variables //////////////


////////////// Flight Variables //////////////
float velocity = 0.00 , liaX = 0.00, liaY = 0.00 , liaZ = 0.00;
float referanceAltitude = 0.00;
const float seaLevel = 1029;
float prevAltitude;
double gpsLatitude;
double gpsLongitude;
float altitudeDifference = 0.00;
float C_Altitude = 0.00;
float C_Pressure = 0.00;
float gpsAltitude;
float pressure = 0.00;
char flightMode = 'F';
uint8_t parachuteCounter = 0;
boolean sendVideo = false;
boolean receivedVideo = false;
float vin = 0 ;
float vout = 0;
int baroInt = 0;
int velocityInt = 0;
double velocityAltitude = 0.00;
double prevVelocityAltitude = 0.00;
double prevVelocity = 0.00;
////////////// Flight Variables //////////////


////////////// Check Variables //////////////
boolean lastStateVariable = false;
bool dataFlow = true;
char errorCode[5] = {'0', '0', '0', '0', '0'};
uint8_t cbnStatus = 0;
////////////// Check Variables //////////////




////////////// Telemetry Variables //////////
String package;
int packageCounter = 1;
String receivedComs[5];
uint8_t satelliteStatusInteger = 0;
uint8_t flightState = 0;
////////////// Telemetry Variables ///////////


////////////// Sen0253 Variables //////////////
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, 0x28);
float roll = 0.00;
float pitch = 0.00;
float yaw = 0.00;
float temperature = 0.00;
float altitude = 0.00;
uint8_t counter = 0;
double baroVelocity = 0.00;
////////////// Sen0253 Variables //////////////


void errorcheck() {
  errorCode[0] = '0';
  errorCode[1] = '0';
  errorCode[2] = '0';
  errorCode[3] = '0';
  errorCode[4] = '0';

  if ((flightState == 3)  && (velocity <= 12.00 || velocity >= 14.00))
    errorCode[0] = '1';


  if ((flightState == 4) && (velocity <= 6.00 || velocity >= 8.00))
    errorCode[1] = '1' ;


  if (C_Pressure == 0.00)
    errorCode[2] = '1';

  if (gpsLatitude == 0.00 || gpsLongitude == 0.00)
    errorCode[3] = '1';

  if (flightState == 4 && altitudeDifference < 3.00)
    errorCode[4] = '1';

}

void prevAltitudetudeCheck() {
  if (altitudeTime - prevAltitudeTime > 450) {
    prevAltitudeTime = altitudeTime;
    prevAltitude = altitude;
    displayCalStatus();
  }
}

void voltage() {
  vin = (analogRead(A17) * 8.2) / (1023);
  vout = ((vin)) ;
}



void telemetry() {
  myFile = SD.open("TMUY_209122_TLM.csv", FILE_WRITE);
  altitudeDifference = abs(altitude - C_Altitude);
  package += packageCounter;
  package += ",";
  if (sendVideo) {
    satelliteStatusInteger = 7;
    sendVideo = false;
  }
  if (receivedVideo) {
    satelliteStatusInteger = 6;
    receivedVideo = false;
  }
  package += satelliteStatusInteger;
  package += ",";
  for (int i = 0 ; i < 4 ; i++)
    package += errorCode[i];
  package += "'";  
  package += ",";
  if (day() < 10)
    package += "0";
  package += day();
  package += "/";
  if (month() < 10)
    package += "0";
  package += month();
  package += "/";
  package += year();
  package += ".";
  if (hour() < 10)
    package += "0";
  package += hour();
  package += ":";
  if (minute() < 10)
    package += "0";
  package += minute();
  package += ":";
  if (second() < 10)
    package += "0";
  package += second();
  package += ",";
  package += abs(pressure);
  package += ",";
  package += abs(C_Pressure);
  package += ",";
  package += abs(altitude);
  package += ",";
  package += abs(C_Altitude);
  package += ",";
  package += altitudeDifference;
  package += ",";
  package += abs(velocity);
  package += ",";
  package += temperature;
  package += ",";
  package += vout;
  package += ",";
  package += String(gpsLatitude , 6);
  package += ",";
  package += String(gpsLongitude , 6);
  package += ",";
  package += gpsAltitude;
  package += ",";
  package += pitch;
  package += ",";
  package += roll;
  package += ",";
  package += yaw;
  package += ",";
  package += "209122";
  package += ",";
  Serial5.println(package);
  myFile.println(package);
  Serial.println(package);
  Serial5.flush();
  myFile.close();
  packageCounter++;
  eeprom_write(package_count_low_address, package_count_high_address , packageCounter);
  package = "";
  velocity = 0.00;
  C_Pressure = 0.00;
  C_Altitude = 0.00;
  if (flightState == 4 && altitude < 430)
    parachuteCounter++;
  if ((int)altitude == (int)prevAltitude && flightState == 4)
    counter++;
}

void sendTelemetry() {
  if (presentTime - prevTime > 970 && dataFlow == true) {
    prevTime = presentTime;
    errorcheck();
    telemetry();
    if (flightState != 5)
      digitalWrite(buzzer, LOW);

  }
}

void speedCalculation() {
  baroInt = trunc(velocityAltitude * 10);
  velocityAltitude = baroInt / 10.0;

  if (velocityTimer - velocityPrevTime >= MS_B) {
    baroVelocity = ((velocityAltitude - prevAltitude) / dt_b);
    prevVelocity = baroVelocity;
    prevVelocityAltitude = velocityAltitude;
    velocityPrevTime = velocityTimer;
  }
  
  else if (millis() - next_t >= MS) {
    velocity = prevVelocity + ((liaZ + prevLineerz) / 2) * dt;
    prevLineerZ = liaZ;
    prevVelocity = velocity;
    next_t = MS;
  }
  velocityInt = trunc(velocity * 10);
  velocity = velocityInt / 10;
}




void eeprom_write(int low, int high, int values)
{
  byte lowbyte = lowByte(values);
  byte highbyte = highByte(values);
  EEPROM.update(low, lowbyte);
  EEPROM.update(high, highbyte);
}


int eeprom_read(int read_low, int read_high)
{
  byte  low = EEPROM.read(read_low);
  byte high = EEPROM.read(read_high);
  int recovery = low + (high << 8);
  return recovery;
}


void wipeEEPROM() {
  for (int i = 0 ; i <= 25 ; i++) {
    EEPROM.update(i, 0);
  }
  referanceAltitude = 0.00;
  altitude_simp = 0;
  satelliteStatusInteger = 0;
  packageCounter = 1;
  flightState = 0;
  dataFlow = true;
  flightMode = 'F';
  lastStateVariable = false;
  counter = 0;
  SD.remove("TMUY_209122_TLM.csv");
  digitalWrite(buzzer, HIGH);
}

void coms() {
  if (Serial.available() > 0) {
    Serial.setTimeout(20);
    for (uint8_t i = 0 ; i < 5 ; i++) {
      receivedComs[i] = Serial.readStringUntil(',');
    }

    if (receivedComs[0] == "TY") {
      C_Altitude = receivedComs[1].toFloat();
      C_Pressure = receivedComs[2].toFloat();
    }

    if (receivedComs[0] == "209122" && receivedComs[1] == "GONDERME") {
      sendVideo = true;
      digitalWrite(buzzer, HIGH);
    }


    if (receivedComs[0] == "209122" && receivedComs[1] == "ALMA") {
      receivedVideo = true;
      digitalWrite(buzzer, HIGH);
    }

    if (receivedComs[0] == "209122" && receivedComs[1] == "CBN") {
      if (flightMode == 'F') {
        Serial3.println("209122,CBN");
        digitalWrite(buzzer, HIGH);
        referanceAltitude = bmp.readAltitude(seaLevel);
        altitude = bmp.readAltitude(seaLevel) - referanceAltitude;
        eeprom_write(referance_altitude_low_address , referance_altitude_high_address , referanceAltitude);
        SD.remove("TMUY_209122_TLM.csv");
        flightState = 1;
        packageCounter = 1;
      }

      if (flightMode == 'S') {
        Serial3.println("209122,CBN");
        digitalWrite(buzzer, HIGH);
        altitude = altitude_simp - referanceAltitude;
        referanceAltitude = altitude_simp - altitude;
        SD.remove("TMUY_209122_TLM.csv");
        flightState = 1;
        packageCounter = 1;
      }
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "wipeEEPROM") {
      wipeEEPROM();
      digitalWrite(buzzer, HIGH);
      Serial3.println("209122,wipeEEPROM");
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "SIM") {
      flightMode = 'S';
      digitalWrite(buzzer, HIGH);
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "SIMP") {
      SIMP = receivedComs[2].toInt();
      altitude_simp = (((pow(Po / SIMP, sbt_pow) - 1) * (temp)) / sbt);
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "Flight") {
      flightMode = 'F';
      digitalWrite(buzzer, HIGH);
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "AYIR")
      Serial3.println("209122,AYIR");

    Serial.flush();
  }

  if (Serial3.available() > 0) {
    Serial3.setTimeout(20);
    for (uint8_t i = 0 ; i < 3 ; i++) {
      receivedComs[i] = Serial3.readStringUntil(',');
    }

    if (receivedComs[0] == "209122" && receivedComs[1] == "GONDERME") {
      digitalWrite(buzzer, HIGH);
      sendVideo = true;
    }

    if (receivedComs[0] == "209122" && receivedComs[1] == "ALMA") {
      digitalWrite(buzzer, HIGH);
      receivedVideo = true;
    }

    if (receivedComs[0] == "209122" && receivedComs[1] == "CBN") {
      Serial3.println("209122,CBN,");
      digitalWrite(buzzer, HIGH);
      referanceAltitude = bmp.readAltitude(seaLevel);
      altitude = bmp.readAltitude(seaLevel) - referanceAltitude;
      eeprom_write(referance_altitude_low_address , referance_altitude_high_address , referanceAltitude);
      SD.remove("TMUY_209122_TLM.csv");
      flightState = 1;
      packageCounter = 1;

    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "wipeEEPROM") {
      Serial3.println("209122,wipeEEPROM,");
      digitalWrite(buzzer, HIGH);
      wipeEEPROM();
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "AYIR") {
      digitalWrite(buzzer, HIGH);
      Serial3.println("209122,AYIR,");
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "SIM") {
      flightMode = 'S';
      digitalWrite(buzzer, HIGH);
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "SIMP") {
      SIMP = receivedComs[2].toInt();
      altitude_simp = (((pow(Po / SIMP, sbt_pow) - 1) * (temp)) / sbt);
    }
    if (receivedComs[0] == "209122" && receivedComs[1] == "Flight") {
      flightMode = 'F';
      digitalWrite(buzzer, HIGH);
    }
    Serial3.flush();
  }

  if (Serial5.available() > 0) {
    Serial5.setTimeout(20);
    for (uint8_t i = 0 ; i < 3 ; i++) {
      receivedComs[i] = Serial5.readStringUntil(',');
    }
    if (receivedComs[0] == "TY") {
      C_Altitude = receivedComs[1].toFloat();
      C_Pressure = receivedComs[2].toFloat();
    }
    Serial5.flush();
  }
}


void GPS() {
  do {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated())
    {
      gpsLatitude = gps.location.lat();
      gpsLongitude = gps.location.lng();
    }
    if (gps.altitude.isUpdated())
      gpsAltitude = gps.altitude.meters();
  } while (Serial2.available() > 0);
}



void dataProcess() {
  if (flightMode == 'F') {
    sensors_event_t event;
    sensors_event_t linearAccelData;
    myIMU.getEvent(&linearAccelData , Adafruit_BNO055::VECTOR_LINEARACCEL);
    myIMU.getEvent(&event);
    roll = event.orientation.z;
    pitch = event.orientation.y;
    yaw = event.orientation.x;

    sensors_event_t linearAccel;
    myIMU.getEvent(&linearAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    liaX = linearAccel.acceleration.x;
    liaY = linearAccel.acceleration.y;
    liaZ = linearAccel.acceleration.z;

    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(seaLevel) - referanceAltitude;
    velocityAltitude = bmp.readAltitude(seaLevel) - referanceAltitude;
  }

  if (flightMode == 'S') {
    sensors_event_t event;
    sensors_event_t linearAccelData;
    myIMU.getEvent(&linearAccelData , Adafruit_BNO055::VECTOR_LINEARACCEL);
    myIMU.getEvent(&event);
    altitude = altitude_simp - referanceAltitude;
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    roll = event.orientation.z;
    pitch = event.orientation.y;
    yaw = event.orientation.x;
  }

}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

#define TIME_HEADER  "T"   // Header tag for serial time sync message
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

void displayCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mag);

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}




void setup() {
  Serial.begin(19200);
  Serial3.begin(19200);
  Serial2.begin(9600);
  Serial5.begin(19200);
  bmp.begin(0x76, BMP280_CHIPID);
  myIMU.begin(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
  myIMU.setExtCrystalUse(true);
  bmp.setSampling
  (Adafruit_BMP280::MODE_NORMAL,
   Adafruit_BMP280::SAMPLING_X2,
   Adafruit_BMP280::SAMPLING_X16,
   Adafruit_BMP280::FILTER_X16,
   Adafruit_BMP280::STANDBY_MS_500);
  SD.begin(chipSelect);
  pinMode(buzzer, OUTPUT);
  pinMode(A17, INPUT);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  myFile = SD.open("TMUY_209122_TLM.csv", FILE_WRITE);
  if (myFile) {
    Serial.println("Dosya acildi");
  }
  else {
    Serial.println("Dosya acilamadi");
  }
  myFile.close();
  setSyncProvider(getTeensy3Time);

  if (eeprom_read(package_count_low_address, package_count_high_address) > 0)
    packageCounter = eeprom_read(package_count_low_address , package_count_high_address);

  if (eeprom_read(referance_altitude_low_address , referance_altitude_high_address > 0))
    referanceAltitude = eeprom_read(referance_altitude_low_address , referance_altitude_high_address);

  if (eeprom_read(state_low_address , state_high_address) > 0) {
    Serial.print("Son Kalinan Flight State:");
    flightState = eeprom_read(state_low_address , state_high_address);
    Serial.print(flightState);
  }

  if (eeprom_read(satellite_low_address , satellite_high_address) >= 0) {
    Serial.print("Son kalinan satelliteStatusInteger:");
    satelliteStatusInteger = eeprom_read(satellite_low_address , satellite_high_address);
    Serial.println(satelliteStatusInteger);
  }
}

void loop() {
  presentTime = millis();
  altitudeTime = millis();
  lastStateTimer = millis();
  velocityTimer = millis();
  speedCalculation();
  prevAltitudetudeCheck();
  dataProcess();
  coms();
  sendTelemetry();
  GPS();
  voltage();
  if (flightState == 0 || (flightState == 1 && altitude < 10)) {
    satelliteStatusInteger = 0;
    eeprom_write(satellite_low_address , satellite_high_address , satelliteStatusInteger);  // elif harika biri

  }

  if (altitude > 10 && flightState == 1) {
    satelliteStatusInteger = 1;
    flightState = 2;
    eeprom_write(state_low_address , state_high_address , flightState);
    eeprom_write(satellite_low_address , satellite_high_address , satelliteStatusInteger);
  }

  if (altitude - prevAltitude < -10 && flightState == 2) {
    satelliteStatusInteger = 2;
    flightState = 3;
    eeprom_write(state_low_address , state_high_address , flightState);
    eeprom_write(satellite_low_address , satellite_high_address , satelliteStatusInteger);
  }

  if (altitude < 430 && flightState == 3) {
    satelliteStatusInteger = 3;
    flightState = 4;
    eeprom_write(state_low_address , state_high_address , flightState);
    eeprom_write(satellite_low_address , satellite_high_address , satelliteStatusInteger);
    Serial3.println("209122,AYIR,");
  }

  if ((flightState == 4 && ((int)altitude != (int)prevAltitude)) && parachuteCounter > 3) {
    satelliteStatusInteger = 4;
    eeprom_write(satellite_low_address , satellite_high_address , satelliteStatusInteger);
  }

  if (flightState == 4 && counter == 15) {
    satelliteStatusInteger = 5;
    flightState = 5;
    digitalWrite(buzzer, HIGH);
  }

  if (flightState == 5) {
    if (!lastStateVariable) {
      prevLastStateTimer = lastStateTimer;
      lastStateVariable = true;
    }

    if (lastStateTimer - prevLastStateTimer > 9999) {
      dataFlow = false;
      myFile.close();
    }
  }
}
