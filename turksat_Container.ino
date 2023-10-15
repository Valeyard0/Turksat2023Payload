#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

#define stateAddress 10    //Durumların EEPROM'a kaydedileceği adres
#define commandAddress 11  //Komutların EEPROM'a kaydedileceği adres
#define mosfetPin 3 

Adafruit_BMP280 bmp;

String package = "";

File dataSd;

bool dataFlow = true;
bool lastStateControl = false;

//////////////// Flight Variables ///////////////////
int state = 0 ;
float previousAltitude = 0;
int counter = 0;
int altitude ;
int pressure ;

//////////////// EEPROM Variables ///////////////////
int address;
int value;

//////////////// Time Variables ///////////////////
long long int currentTime;  //Genel zaman millis
long long int previousTime = 0;

long long int previousAltitudeCurrentTime;  //Önceki yüksekliği kontrol etmek için zaman millis
long long int previousAltitudePrevTime = 0;

long long int lastStateTimer;             // Son durumu zamanlamak için
long long int lastStatePrevTimer = 0;  

long long int seperationTime;  //Ayrılma için zamanlayıcı
long long int seperationPrevTime = 0;


//int buzzerPin = #;
//int sdPin = #;
//int mosfet = #;


bool seperationTimeControl = false ;
bool seperation = false ;

void dataProcess() {
  altitude = bmp.readAltitude();
  pressure = bmp.readPressure();
  previousAltitude = altitude;

  if (currentTime - previousTime > 900 && dataFlow == true) {
    previousTime = currentTime;
    //dataSd = SD.open(data.csv, FILE_WRITE);
    package += ",";
    package += pressure;
    package += ",";
    package += altitude;
    package += ",";
    Serial.print(package);
    package = "";

    //dataSd.println(package);
    //dataSd.close();
  }
}


void eepromWrite(int address, int value) {
  EEPROM.write(address, value);
}


void functionSeperationShutDown() {
  if (seperationTimeControl == false) 
  {
      seperationPrevTime = seperationTime;
      seperationTimeControl = true;
  }

  if (seperationTime - seperationPrevTime > 999 && seperation == true) {
    analogWrite(mosfet, 0);
    seperation = false;
    state = 3;
    eepromWrite(stateAddress, state);
  }
}


void functionLanding() {
  if (state == 3 && (int)altitude == (int)previousAltitude)
    counter++;
}


void prevAltitudeCheck() {
  if (previousAltitudeCurrentTime - previousAltitudeCurrentTime > 332) {
    previousAltitudeCurrentTime = previousAltitudeCurrentTime;
    previousAltitude = altitude;
  }
}



////////////////////////////////////////////////////////////////////////////////SETUP & LOOP////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);

  bmp.begin(0x76, BMP280_CHIPID);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  //pinMode(buzzerPin, OUTPUT);
  //pinMode(sdPin, OUTPUT);
  //pinMOde(mosfet, OUTPUT);

  if (EEPROM.read(stateAddress) > 0) {
    state = EEPROM.read(stateAddress);
  }
}



void loop() {
  currentTime = millis();
  previousAltitudeCurrentTime = millis();
  seperationTime = millis();
  
  dataProcess();
  functionLanding();

  if (altitude < previousAltitude && state == 1) 
  {
    state = 2;
    eepromWrite(stateAddress, state);
  }

  else if (state == 2 && altitude < 410) 
  {
    seperation = true;

    if (seperation == true) {
      //analogWrite(mosfet, 150);
      seperationTimeControl = true;
      functionSeperationShutDown();
    }

  }

  else if (state == 3 && previousAltitude == altitude) {
    lastStateTimer = millis();
    //digitalWrite(buzzerPin, HIGH);

    if (lastStateControl == false) 
    {
      lastStatePrevTimer = lastStateTimer;
      lastStateControl = true;
    }

    else if (lastStateTimer - lastStatePrevTimer > 9999) 
    {
      dataFlow = false;
    }

    else if (lastStateTimer - lastStatePrevTimer < 9999) 
    {
      dataProcess();
    }
  }
}

