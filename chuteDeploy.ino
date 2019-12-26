/*
 Parachute deployment system intended for water rocket

 based on available aduino examples
*/

#include <Servo.h>

Servo latchServo;                       // create servo object to control the latch servo

const int lowTrimPotPin = 0;            // analog pin used to connect the potentiometer adjusting the open angle of the latch's servo
const int latchServoPin = 6;            // the number of the latch's servo signal pin
const int armButtonPin = 2;             // the number of the arm pushbutton pin
const int tiltButtonPin = 3;            // the number of the tilt button pin
const int launchDetectorPin = 4;        // the number of the launch detector pin
const int inFlightLedPin = LED_BUILTIN; // the number of the in flight state led's pin
const int armedLedPin = 5;              // the number of the armed state led's pin

int val;                                // variable to read the value from the analog pin
int openLatchPos = 0;                   // variable for open latch position setting
int closedLatchPos = 180;               // variable for closed latch position setting
int flightPhase = 0;                    // variable to track flight phases

void setup() {
  latchServo.attach(latchServoPin); // attaches the servo on pin 9 to the servo object
  pinMode(inFlightLedPin, OUTPUT);  // initialize in flight led pin as an output.
  pinMode(armedLedPin, OUTPUT);     // initialize armed led pin as an output.
//  Serial.begin(9600);               // initialize serial communication at 9600 bits per second
}

void loop() {
  /* execute the current flight phase methods
   *  each method is responsible of switching to the next */
  switch (flightPhase) {
    case 0: preflight();
    break;
    case 1: ready_to_launch();
    break;
    case 2: in_flight();
    break;
  }
}

void preflight(){
  set_open_latch_pos();
  if(digitalRead(armButtonPin) == HIGH){
    close_latch();
    digitalWrite(armedLedPin, HIGH);
    flightPhase = 1;
  }
}

void ready_to_launch(){
  if(digitalRead(launchDetectorPin) == LOW){
    digitalWrite(inFlightLedPin, HIGH);
    flightPhase = 2;
  }
}

void in_flight(){
  if(digitalRead(tiltButtonPin) == HIGH){
    open_latch();
  }
}

void set_open_latch_pos(){
  openLatchPos = analogRead(lowTrimPotPin);           // reads the value of the potentiometer (value between 0 and 1023)
  openLatchPos = map(openLatchPos, 0, 1023, 0, 180);  // scale it to use it with the servo (value between 0 and 180)
//  Serial.println(openLatchPos);                       // writes servo value to serial out
  move_latch(openLatchPos);                          // moves the latch to setted position
}

void move_latch(int pos){
  latchServo.write(pos);            // set latch to open 
  delay(15);                        // waits for the servo to get there
}

void open_latch(){move_latch(openLatchPos);}
void close_latch(){move_latch(closedLatchPos);}
