/*
 Parachute deployment system intended for water rocket

 sources
 - available aduino examples
 - Adafruit_BMP280_Library      modified bmp280 I²C address for alternate one
 - Advanced_I2C.ino     Brian R Taylor      brian.taylor@bolderflight.com
*/

#include <Servo.h>
#include "MPU9250.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>


Servo latchServo;                       // create servo object to control the latch servo
MPU9250 IMU(Wire,0x68);                 // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
Adafruit_BMP280 bmp;                    // BMP280 from adafruit

const bool debug = false;
const bool displayFlightStats = true;

const int armButtonPin = 2;             // the number of the arm pushbutton pin
const int launchDetectorPin = 3;        // the number of the launch detector pin
const int tiltButtonPin = 4;            // the number of the tilt button pin
const int latchServoPin = 5;            // the number of the latch's servo signal pin
const int inFlightLedPin = LED_BUILTIN; // the number of the in flight state led's pin
const int armedLedPin = 6;              // the number of the armed state led's pin

int openLatchPos = 80;                  // variable for open latch position setting
int closedLatchPos = 180;               // variable for closed latch position setting
int flightPhase = 0;                    // variable to track flight phases
int IMUstatus;                          // tracks IMU's state
int BMP280Status;                       // tracks BMP280's state

//////////////////////
// SETUP

void setup() {
  Serial.begin(9600);               // initialize serial communication at 9600 bits per second
  pinMode(inFlightLedPin, OUTPUT);  // initialize in flight led pin as an output.
  pinMode(armedLedPin, OUTPUT);     // initialize armed led pin as an output.

  // Servo setup
  latchServo.attach(latchServoPin); // attaches the servo on pin 9 to the servo object
  open_latch();                     // puts the servo in the opened position

  // MPU9250 setup
  IMUstatus = IMU.begin();             // start communication with IMU
  if (IMUstatus < 1) {                 // check IMU status
    sDisp(F("IMU initialization unsuccessful, check IMU wiring ! Status: "));
    sDisp(IMUstatus);
  }else{
    sDisp(F("IMU initialization successful."));
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);         // setting the accelerometer full scale range to +/-8G 
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);       // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ); // setting DLPF bandwidth to 20 Hz
  IMU.setSrd(19);                                     // setting SRD to 19 for a 50 Hz update rate

  // BMP280 setup
  //initializing BMP280 with I²C alternate address 0x76 instead of 0x77
  BMP280Status = bmp.begin(0x76);       // start com with BMP280
  if (BMP280Status < 1) {               // check BMP280 status
    sDisp(F("BMP280 initialization unsuccessful, check wiring ! Status: "));
    sDisp(BMP280Status);
  }else{
    sDisp(F("BMP280 initialization successful."));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  while (!Serial){}
}

//////////////////////
// LOOP

void loop() {
  /* execute the current flight phase methods
   *  each method is responsible of switching to the next 
   */
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
void sDisp(String str){if(debug){Serial.println(str);}}
void sDisp(int str){if(debug){Serial.println(str);}}

// SERVO MANIPULATION
void move_latch(int pos){
  latchServo.write(pos);            // set latch to open 
  delay(15);                        // waits for the servo to get there
}

void open_latch(){move_latch(openLatchPos);}
void close_latch(){move_latch(closedLatchPos);}

/**
 * reads command from serial input
 * op adds 10 to the openLatch var
 * om removes 10 from the openLatch var
 */
void set_open_latch_pos(){
  if ( Serial.available() )
  {
    String inStr = Serial.readString();             // reading input from serial port
    if(inStr.equals("op\n")){
      openLatchPos += 10;
      sDisp("setting opened course to " );
      sDisp(openLatchPos);
    }else if(inStr.equals("om\n")){
      openLatchPos -= 10;
      sDisp("setting opened course to ");
      sDisp(openLatchPos);
    }else{
      sDisp("input error [");
      sDisp(inStr);
      sDisp("]");
    }
    sDisp("\n");
  }
  move_latch(openLatchPos);                           // moves the latch to setted position
}

/**
 * reads command from serial input
 * op adds 10 to the openLatch var
 * om removes 10 from the openLatch var
 */
void set_closed_latch_pos(){
  if ( Serial.available() )
  {
    String inStr = Serial.readString();
    if(inStr == "cp\n"){
      closedLatchPos += 10;
      sDisp("setting closed course to ");
      sDisp(closedLatchPos);
    }else if(inStr.equals("cm\n")){
      closedLatchPos -= 10;
      sDisp("setting closed course to ");
      sDisp(closedLatchPos);
    }else{
      sDisp("input error [");
      sDisp(inStr);
      sDisp("]");
    }
    sDisp("\n");
  }
  move_latch(closedLatchPos);                         // moves the latch to setted position
}

void display_gy91_data(){
  IMU.readSensor();       // read the sensor

//  display MPU9250 data
//  Serial.print(F("Acc_XYZ "));
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");

//  Serial.print(F("Gyro_XYZ "));
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");

//  Serial.print(F("Mag_XYZ "));
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");  
  Serial.print(IMU.getTemperature_C(),6);
  Serial.print(F("\t"));

  // BMP280L
//  Serial.print(F("Temp(°C)\t "));
  Serial.print(bmp.readTemperature());
//  Serial.print(F("\t"));

//  Serial.print(F("=Pressure(Pa) "));
  Serial.print(bmp.readPressure());
  Serial.print(F("\t"));
//
//  Serial.print(F("=Approx_alt(m) "));
  Serial.print((bmp.readAltitude(1034))); /* Adjusted to local forecast! */
  
  Serial.print(F("\n"));
}

// FLIGHT PHASES
void preflight(){
  set_open_latch_pos();
  display_gy91_data();
  if( digitalRead(armButtonPin) == HIGH && 
      digitalRead(launchDetectorPin) == HIGH
     ){
    close_latch();
    digitalWrite(armedLedPin, HIGH);
    flightPhase = 1;
  }
}

void ready_to_launch(){
  set_closed_latch_pos();
  if(digitalRead(launchDetectorPin) == LOW){
    digitalWrite(inFlightLedPin, HIGH);
    flightPhase = 2;
  }
}

void in_flight(){
  display_gy91_data();
  if(digitalRead(tiltButtonPin) == HIGH){
    open_latch();
    digitalWrite(armedLedPin, LOW);
  }
}
