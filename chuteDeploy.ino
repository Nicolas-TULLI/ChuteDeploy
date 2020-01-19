/*
  Parachute deployment system intended for water rocket

  sources
  - available aduino examples
  - Adafruit_BMP280_Library      modified bmp280 IÂ²C address for alternate one
  - Advanced_I2C.ino     Brian R Taylor      brian.taylor@bolderflight.com
*/

//#include <Servo.h>
#include "MPU9250.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "Latch.h"


Latch latch;                            // create Latch object to control the latch servo
MPU9250 IMU(Wire, 0x68);                // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
Adafruit_BMP280 bmp;                    // BMP280 from adafruit

const bool debug = true;
const bool displayFlightStats = false;

const int armButtonPin = 2;             // the number of the arm pushbutton pin
const int launchDetectorPin = 3;        // the number of the launch detector pin
const int tiltButtonPin = 4;            // the number of the tilt button pin
const int latchServoPin = 5;            // the number of the latch's servo signal pin
const int inFlightLedPin = 6;           // the number of the in flight state led's pin
const int armedLedPin = LED_BUILTIN;    // the number of the armed state led's pin

int openLatchPos = 2;                  // variable for open latch position setting
int closedLatchPos = 90;               // variable for closed latch position setting
int flightPhase = 0;                   // variable to track flight phases
int IMUstatus;                         // tracks IMU's state
int BMP280Status;                      // tracks BMP280's state
int qnh = 1013;                        // QNH
float rawVal[50];                      // raw altitudes to smoothen
int lastMs;                            // keep track of the last loop for the variometer
float lastAlt;
float aprxAlt;
float maxAlt;
float minAlt;

//////////////////////
// SETUP

void setup() {
  Serial.begin(9600);               // initialize serial communication at 9600 bits per second
  pinMode(inFlightLedPin, OUTPUT);  // initialize in flight led pin as an output.
  pinMode(armedLedPin, OUTPUT);     // initialize armed led pin as an output.

  pinMode(armButtonPin, INPUT_PULLUP);
  pinMode(launchDetectorPin, INPUT_PULLUP);

  // Servo setup
  latch.init(latchServoPin, openLatchPos, closedLatchPos); // attaches the servo on pin 9 to the servo object
  latch.openLatch();                     // puts the servo in the opened position

  // MPU9250 setup
  IMUstatus = IMU.begin();             // start communication with IMU
  if (IMUstatus < 1) {                 // check IMU status
    sDisp(F("IMU initialization unsuccessful, check IMU wiring ! Status: "));
    sDisp(IMUstatus);
  } else {
    sDisp(F("IMU initialization successful."));
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);         // setting the accelerometer full scale range to +/-8G
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);       // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ); // setting DLPF bandwidth to 20 Hz
  IMU.setSrd(19);                                     // setting SRD to 19 for a 50 Hz update rate

  // BMP280 setup
  //initializing BMP280 with IÂ²C alternate address 0x76 instead of 0x77
  BMP280Status = bmp.begin(0x76);       // start com with BMP280
  if (BMP280Status < 1) {               // check BMP280 status
    sDisp(F("BMP280 initialization unsuccessful, check wiring ! Status: "));
    sDisp(BMP280Status);
  } else {
    sDisp(F("BMP280 initialization successful."));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  //                  Adafruit_BMP280::STANDBY_MS_500   /* Standby time. */
//                                    Adafruit_BMP280::STANDBY_MS_125
                  Adafruit_BMP280::STANDBY_MS_1
                 );

  while (!Serial) {}
}

//////////////////////
// LOOP

void loop() {
  /* execute the current flight phase methods
      each method is responsible of switching to the next
  */
  display_gy91_data();
  switch (flightPhase) {
    case 0: preflight();
      break;
    case 1: ready_to_launch();
      break;
    case 2: in_flight();
      break;
    case 3: //chute_deployed();
      break;
    case 4: //landed();
      break;
  }
}

//////////////////////
// METHODS

// serial display methods
void sDisp(String str) {
  if (debug) {
    Serial.println(str);
  }
}
void sDisp(int str) {
  if (debug) {
    Serial.println(str);
  }
}

// Commands interface
void read_command() {
  if (Serial.available()) {
    String inStr = Serial.readString();           // reading input from serial port
    if (inStr.startsWith("qnh")) {             // looking for QNH command
      setQnh(inStr);
    } else if (inStr.startsWith("o")) {           // looking for open position command
      set_open_latch_pos(inStr);
    } else if (inStr.startsWith("c")) {           // looking for closed position command
      set_closed_latch_pos(inStr);
    }
  }

}

void setQnh(String inStr){
   if (inStr.startsWith("qnh")) {             // looking for QNH command
      qnh = inStr.substring(3, 7).toInt();
      maxAlt = aprxAlt ;
      minAlt = aprxAlt ;
      sDisp("setting qnh to " + String(qnh));
   }
}
/**
   op adds 10 to the openLatch var
   om removes 10 from the openLatch var
*/
void set_open_latch_pos(String inStr) {
  if (flightPhase == 0) {
    if (inStr.equals("op\n")) {
      latch.setOpenPos(latch.getOpenPos() + 5);
      sDisp("setting opened course to " + String(latch.getOpenPos()));
    } else if (inStr.equals("om\n")) {
      latch.setOpenPos(latch.getOpenPos() - 5);
      sDisp("setting opened course to " + String(latch.getOpenPos()));
    } else {
      sDisp("input error [" + String(inStr) + "]");
    }
    latch.openLatch();                           // moves the latch to setted position
  } else {
    sDisp("input error : not in preflight");
  }
}

/**
   op adds 10 to the openLatch var
   om removes 10 from the openLatch var
*/
void set_closed_latch_pos(String inStr) {
  if (flightPhase == 1 ) {
    if (inStr == "cp\n") {
      closedLatchPos += 5;
      sDisp("setting closed course to " + String(closedLatchPos));
    } else if (inStr.equals("cm\n")) {
      closedLatchPos -= 5;
      sDisp("setting closed course to " + String(closedLatchPos));
    } else {
      sDisp("input error [" + String(inStr) + "]");
    }
    latch.closeLatch();                         // moves the latch to setted position
  } else {
    sDisp("input error : not armed");
  }
}

// FLIGHT PHASES
void preflight() {
  read_command();
  if ( digitalRead(armButtonPin) == LOW &&
       digitalRead(launchDetectorPin) == LOW
     ) {
    latch.closeLatch();
    digitalWrite(armedLedPin, HIGH);
    flightPhase = 1;
    sDisp(F("To phase 1"));
  }
}

void ready_to_launch() {
  read_command();
  if (digitalRead(launchDetectorPin) == HIGH) {
    digitalWrite(inFlightLedPin, HIGH);
    digitalWrite(armedLedPin, LOW);
    flightPhase = 2;
    sDisp(F("To phase 2"));
  }
}

void in_flight() {
  if (digitalRead(tiltButtonPin) == HIGH) {
   latch.openLatch();
    digitalWrite(armedLedPin, LOW);
    digitalWrite(inFlightLedPin, LOW);
    flightPhase = 3;
    sDisp(F("To phase 3"));
  }
}

// sensors data handling
void display_gy91_data() {
  if (displayFlightStats) {
//    IMU.readSensor();       // read the sensor

    //  display MPU9250 data
    if (false) {
      //    Serial.print(F("Acc_XYZ "));
      Serial.print(IMU.getAccelX_mss(), 6);
      Serial.print("\t");
      Serial.print(IMU.getAccelY_mss(), 6);
      Serial.print("\t");
      Serial.print(IMU.getAccelZ_mss(), 6);
      Serial.print("\t");
    }
    if (false) {
      //    Serial.print(F("Gyro_XYZ "));
      Serial.print(IMU.getGyroX_rads(), 6);
      Serial.print("\t");
      Serial.print(IMU.getGyroY_rads(), 6);
      Serial.print("\t");
      Serial.print(IMU.getGyroZ_rads(), 6);
      Serial.print("\t");
    }
    if (false) {
      //    Serial.print(F("Mag_XYZ "));
      Serial.print(IMU.getMagX_uT(), 6);
      Serial.print("\t");
      Serial.print(IMU.getMagY_uT(), 6);
      Serial.print("\t");
      Serial.print(IMU.getMagZ_uT(), 6);
      Serial.print("\t");

      Serial.print(IMU.getTemperature_C(), 6);
      Serial.print(F("\t"));
    }

    // BMP280L
    if (false) {
      //      Serial.print(F("Temp(Â°C)\t "));
      Serial.print(bmp.readTemperature(), 6);
      Serial.print(F("\t"));

      //      Serial.print(F("=Pressure(Pa) "));
      Serial.print(bmp.readPressure(), 6);
      Serial.print(F("\t"));
      //      Serial.print(F("=Approx_alt(m) "));
      Serial.print(bmp.readAltitude(qnh), 6); /* Adjusted to local forecast! */
     
      Serial.print(F("\t"));
      //    int lastMs;                            // keep track of the last loop for the variometer
    }

    // computed
    if (true) {



      
      // roll
      //      Serial.print(atan2(IMU.getAccelX_mss() , IMU.getAccelZ_mss()));
      //      Serial.print(F("\t"));
      // pitch
      //      Serial.print(atan2(IMU.getAccelY_mss() , IMU.getAccelZ_mss()));
      //      Serial.print(F("\t"));
      //      Serial.print((bmp.readAltitude(qnh)), 6); /* Adjusted to local forecast! */
      //      Serial.print(F("\t"));
      aprxAlt = bmp.readAltitude(qnh);
      int now = millis();
      Serial.print(String(((aprxAlt - lastAlt) / (now - lastMs))*1000) + F("\t"));
      Serial.print(String(now - lastMs) + F("\t"));
      lastAlt = aprxAlt;
      lastMs = now;
      
      maxAlt >= aprxAlt ? : maxAlt = aprxAlt;
      minAlt <= aprxAlt ? : minAlt = aprxAlt;
      
      Serial.print(String(maxAlt) + F("\t"));
      Serial.print(String(minAlt) + F("\t"));
      Serial.print(String(aprxAlt) + F("\t")); /* Adjusted to local forecast! */
      Serial.print(String(smooth(aprxAlt)) + F("\t"));
    }
    Serial.print(F("\n"));
  }
}

float smooth(float val) {
  float sum = 0;
  // shifting array one element down and adding last val on top
  for (int i = sizeof(rawVal) / sizeof(rawVal[0]) - 1 ; i > 0 ; i--) {
    rawVal[i] = rawVal[i - 1];
  }
  rawVal[0] = val;
  // summing up the array and averaging it
  for (int i = sizeof(rawVal) / sizeof(rawVal[0]) - 1 ; i > -1 ; i--) {
    sum += rawVal[i];
  }
  return sum / (sizeof(rawVal) / sizeof(rawVal[0]));
}
