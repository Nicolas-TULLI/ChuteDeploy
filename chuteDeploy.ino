/*
  Parachute deployment system intended for water rocket

  sources
  - available aduino examples
  - Adafruit_BMP280_Library      modified bmp280 IÂ²C address for alternate one
  - Advanced_I2C.ino     Brian R Taylor      brian.taylor@bolderflight.com    arduino examples
  - Datalogger.ino       by Tom Igoe                arduino examples
*/

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
//#include "MPU9250.h"
#include "Latch.h"
#include "Cli.h"
#include "FlightDatas.h"
#include "Smoother.h"

//MPU9250 IMU(Wire, 0x68);                // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
Adafruit_BMP280 bmp;                    // BMP280 from adafruit
Cli cli;
FlightDatas fds;

File logFile;
String logFileName = "DATA";      // base name for flight data log file. 6 char max. will be "uppercased", truncated to 6 char and suffixed with "_[0-9]*"
bool headed = false;

Latch latch;                            // create Latch object to control the latch servo
const int latchPin = 5;                 // the number of the latch's servo signal pin
const int latchOpenPos = 3;             // variable for open latch position setting
const int latchClosedPos = 90;          // variable for closed latch position setting

const bool debug = true;
const bool doDisplayGy91Data = false;

const int armButtonPin = 2;             // the number of the arm pushbutton pin
const int launchDetectorPin = 3;        // the number of the launch detector pin
//const int tiltButtonPin = 4;            // the number of the tilt button pin
const int chipSelect = 4;               // SD card chipSelect pin
const int inFlightLedPin = 6;           // the number of the in flight state led's pin
//const int armedLedPin = LED_BUILTIN;    // the number of the armed state led's pin
const int qnh = 1021;

//int IMUstatus;                         // tracks IMU's state
int BMP280Status;                      // tracks BMP280's state

//////////////////////
// SETUP

void setup() {
  Serial.begin(9600);               // initialize serial communication at 9600 bits per second
  pinMode(inFlightLedPin, OUTPUT);  // initialize in flight led pin as an output.
  //pinMode(armedLedPin, OUTPUT);     // initialize armed led pin as an output.

  pinMode(armButtonPin, INPUT_PULLUP);
  pinMode(launchDetectorPin, INPUT_PULLUP);

  // setting qnh
  fds.setQnh(qnh);

  // Latch setup
  latch.init(latchPin, latchOpenPos, latchClosedPos); // attaches the servo on pin 9 to the servo object
  latch.openLatch();                     // puts the servo in the opened position

  // Cli setup
  cli.init(latch, fds);

  // SD card setup
  Serial.print(F("Initializing SD card..."));
  if (SD.begin(chipSelect)) {                        // see if the card is present and can be initialized
    Serial.println(F(" Card initialized."));
  } else {
    Serial.println(F(" Card failed, or not present."));
  }

  // log file setup
  logFileName = getAvailableFileName(logFileName);
  Serial.println(String("Data file : ") + String(logFileName));
  //  logFile = SD.open(logFileName, FILE_WRITE);
  //  logHeader();

  // BMP280 setup
  //initializing BMP280 with IÂ²C alternate address 0x76 instead of 0x77
  BMP280Status = bmp.begin(0x76);       // start com with BMP280
  if (BMP280Status < 1) {               // check BMP280 status
    Serial.print(F("BMP280 initialization unsuccessful, check wiring ! Status: "));
    Serial.println(BMP280Status);
  } else {
    Serial.println(F("BMP280 initialization successful."));
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  //                  Adafruit_BMP280::STANDBY_MS_500   /* Standby time. */
                  //                                    Adafruit_BMP280::STANDBY_MS_125
                  Adafruit_BMP280::STANDBY_MS_1
                 );
}

//////////////////////
// LOOP

void loop() {
  /* execute the current flight phase methods
      each method is responsible of switching to the next
  */

  fds.setAlt(bmp.readAltitude(fds.getQnh()));

  debugAltimeter();
  //  display_gy91_data();
  //  logHeader();        // can't hav this to work
  logDataInFile();

  switch (fds.getFlightPhase()) {
    case 0: preflight();
      break;
    case 1: readyToLaunch();
      break;
    case 2: inFlight();
      break;
    case 3: chuteDeployed();
      break;
    case 4: //landed();
      break;
  }
}

//////////////////////
// METHODS
// FLIGHT PHASES
void preflight() {
  cli.readCommand();
  if ( digitalRead(armButtonPin) == LOW &&
       digitalRead(launchDetectorPin) == LOW
     ) {
    latch.closeLatch();
    //    digitalWrite(armedLedPin, HIGH);
    fds.setFlightPhase(1);
    Serial.println(F("To phase 1"));
  }
}

void readyToLaunch() {
  cli.readCommand();

  if (digitalRead(launchDetectorPin) == HIGH) {
    digitalWrite(inFlightLedPin, HIGH);
    //digitalWrite(armedLedPin, LOW);
    fds.setFlightPhase(2);
    Serial.println(F("To phase 2"));
  }
}

void inFlight() {
  if (fds.isApogee()) {
    latch.openLatch();
    //    digitalWrite(armedLedPin, LOW);
    digitalWrite(inFlightLedPin, LOW);
    fds.setFlightPhase(3);
    Serial.println(F("To phase 3"));
  }
}



void chuteDeployed(){
  if ( digitalRead(armButtonPin) == LOW ) {
    Serial.print("blinking " + String(round(fds.getMaxAlt())));
    dispLatch(round(fds.getMaxAlt()));
  }
}

// Log handling

//void logHeader() {
//  if(!headed){
//    logFile = SD.open(logFileName, FILE_WRITE);
//    logFile.print("Alt;SmoothedAlt;LoopTime;MaxAlt;MinAlt;");
//    logFile.print("vario;smVario;getMaxVario;getMinVario;isApogee;");
//    logFile.print(F("\n"));
//    logFile.close();
//    headed = true;
//  }
//
//}

void logDataInFile() {
  logFile = SD.open(logFileName, FILE_WRITE);
  logFile.print(String(millis()) + F(";"));
  //  logFile.print(String(fds.getLoopTime()) + F(";"));
  logFile.print(String(fds.getFlightPhase()) + F(";"));
  logFile.print(String(fds.getAlt()) + F(";"));
  logFile.print(String(fds.getSmoothedAlt()) + F(";"));
  logFile.print(String(fds.getMaxAlt()) + F(";"));
  logFile.print(String(fds.getMinAlt()) + F(";"));
  logFile.print(String(fds.vario()) + F(";"));
  logFile.print(String(fds.smVario()) + F(";"));
  logFile.print(String(fds.getMaxVario()) + F(";"));
  logFile.print(String(fds.getMinVario()) + F(";"));
  logFile.print(String(fds.isApogee()) + F(";"));
  logFile.print(F("\n"));
  logFile.close();
  //  Serial.print(String(fds.getAlt()) + F("\n"));
}

String getAvailableFileName(String fName) {
  File dir =  SD.open("/");
  fName.toUpperCase();
  fName = fName.substring(0, 6);
  int fileCount = 0;
  while (File entry =  dir.openNextFile()) {
    if (!entry.isDirectory() && String(entry.name()).startsWith(fName)) {
      fileCount++;
    }
    entry.close();
  }
  return (String(fName) + "_" + String(fileCount));
}


// display an int with a led...
// or the latch as the builtin led pin is used by SD card reader SPI ctx...

//void blinkOnce() {
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(125);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  delay(125);
//}

//void blinkLong() {
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(2000);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  delay(500);
//}

//void blinkZero() {
//  for (int i = 0 ; i < 10 ; i++) {
//    digitalWrite(LED_BUILTIN, HIGH);
//    delay(20);
//    digitalWrite(LED_BUILTIN, LOW);
//    delay(20);
//  }
//}

void dispLatch(int integer) {
  String strinteger = String(integer);
  int origClosedPos = latch.getClosedPos();
  
  latch.setClosedPos(origClosedPos / 2);
  latch.closeLatch();
  delay(2000);
//  blinkLong();

  Serial.println("---------------------------");
  for (int i = 0 ; i < strinteger.length() ; i++) {
    int digit = strinteger.charAt(i) - 48 ;     // TODO : find a proper way to convert this
    Serial.println(digit);
    
    if (digit == 0) {
      latch.openLatch();
      delay(150);
      latch.closeLatch();
//      blinkZero();
    } else {
      for (int j = 1 ; j <= digit ; j++) { 
        latch.setClosedPos((origClosedPos / 2) + (j * (origClosedPos / 20) ));            
        latch.closeLatch();
        delay(400);
//        blinkOnce();
      }
      delay(400);
      latch.setClosedPos(origClosedPos / 2);
      latch.closeLatch();
    }
    delay(1000);
  }
  latch.setClosedPos(origClosedPos);
  latch.openLatch();
}


// sensors data 

void debugAltimeter(){
  if (true) {
//    Serial.print(String(millis()) + F("\t")); /* Adjusted to local forecast! */
//    Serial.print(String(fds.getAlt()) + F("\t")); /* Adjusted to local forecast! */
//    Serial.print(String(fds.getSmoothedAlt()) + F("\t"));
//    Serial.print(String(fds.getMaxAlt()) + F("\t"));
//    Serial.print(String(fds.getMinAlt()) + F("\t"));
    Serial.print(String(fds.vario()) + F("\t"));
    Serial.print(String(fds.smVario()) + F("\t"));
    Serial.print(String(fds.getMaxVario()) + F("\t"));
//    Serial.print(String(fds.getMinVario()) + F("\t"));
//    Serial.print(String(fds.getLoopTime()) + F("\t"));
    Serial.print(String(fds.isApogee()) + F("\t"));
    Serial.print(F("\n"));
  }
}

//void display_gy91_data() {
//  if (doDisplayGy91Data) {
//    IMU.readSensor();       // read the sensor
//
//    //  display MPU9250 data
//    if (true) {
//      //    Serial.print(F("Acc_XYZ "));
//      Serial.print(IMU.getAccelX_mss(), 6);
//      Serial.print("\t");
//      Serial.print(IMU.getAccelY_mss(), 6);
//      Serial.print("\t");
//      Serial.print(IMU.getAccelZ_mss(), 6);
//      Serial.print("\t");
//    }
//    if (true) {
//      //    Serial.print(F("Gyro_XYZ "));
//      Serial.print(IMU.getGyroX_rads(), 6);
//      Serial.print("\t");
//      Serial.print(IMU.getGyroY_rads(), 6);
//      Serial.print("\t");
//      Serial.print(IMU.getGyroZ_rads(), 6);
//      Serial.print("\t");
//    }
//    if (true) {
//      //    Serial.print(F("Mag_XYZ "));
//      Serial.print(IMU.getMagX_uT(), 6);
//      Serial.print("\t");
//      Serial.print(IMU.getMagY_uT(), 6);
//      Serial.print("\t");
//      Serial.print(IMU.getMagZ_uT(), 6);
//      Serial.print("\t");
//
//      Serial.print(IMU.getTemperature_C(), 6);
//      Serial.print(F("\t"));
//    }
//
//    // BMP280L
//    if (true) {
//      //      Serial.print(F("Temp(Â°C)\t "));
//      Serial.print(bmp.readTemperature(), 6);
//      Serial.print(F("\t"));
//
//      //      Serial.print(F("=Pressure(Pa) "));
//      Serial.print(bmp.readPressure(), 6);
//      Serial.print(F("\t"));
//      //      Serial.print(F("=Approx_alt(m) "));
//      Serial.print(bmp.readAltitude(fds.getQnh()), 6); /* Adjusted to local forecast! */
//
//      Serial.print(F("\t"));
//      //    int lastMs;                            // keep track of the last loop for the variometer
//    }
//
//    // compute horizon
//    if (false){
//      // roll
//      Serial.print(atan2(IMU.getAccelX_mss() , IMU.getAccelZ_mss()));
//      Serial.print(F("\t"));
//      // pitch
//      Serial.print(atan2(IMU.getAccelY_mss() , IMU.getAccelZ_mss()));
//      Serial.print(F("\t"));
//    }
//    Serial.print(F("\n"));
//  }
//}
