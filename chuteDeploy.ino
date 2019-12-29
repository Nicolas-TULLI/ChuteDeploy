/*
 Parachute deployment system intended for water rocket

 based on available aduino examples
*/

#include <Servo.h>

Servo latchServo;                       // create servo object to control the latch servo

const int armButtonPin = 2;             // the number of the arm pushbutton pin
const int launchDetectorPin = 3;        // the number of the launch detector pin
const int tiltButtonPin = 4;            // the number of the tilt button pin
const int latchServoPin = 5;            // the number of the latch's servo signal pin
const int inFlightLedPin = LED_BUILTIN; // the number of the in flight state led's pin
const int armedLedPin = 6;              // the number of the armed state led's pin

int openLatchPos = 0;                   // variable for open latch position setting
int closedLatchPos = 180;               // variable for closed latch position setting
int flightPhase = 0;                    // variable to track flight phases

//////////////////////
// SETUP

void setup() {
  latchServo.attach(latchServoPin); // attaches the servo on pin 9 to the servo object
  pinMode(inFlightLedPin, OUTPUT);  // initialize in flight led pin as an output.
  pinMode(armedLedPin, OUTPUT);     // initialize armed led pin as an output.
  Serial.begin(9600);               // initialize serial communication at 9600 bits per second
}

//////////////////////
// LOOP

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
    case 3: //chute_deployed();
    break;
    case 4: //landed();
    break;
  }
}

//////////////////////
// METHODS

// SERVO MANIPULATION
void move_latch(int pos){
  latchServo.write(pos);            // set latch to open 
  delay(15);                        // waits for the servo to get there
}

void open_latch(){move_latch(openLatchPos);}
void close_latch(){move_latch(closedLatchPos);}

void set_open_latch_pos(){
  if ( Serial.available() )
  {
    String inStr = Serial.readString();
    if(inStr.equals("op\n")){
      openLatchPos += 10;
      Serial.print("setting opened course to " );
      Serial.print(openLatchPos);
    }else if(inStr.equals("om\n")){
      openLatchPos -= 10;
      Serial.print("setting opened course to ");
      Serial.print(openLatchPos);
    }else{
      Serial.print("input error [");
      Serial.print(inStr);
      Serial.print("]");
    }
    Serial.print("\n");
  }
  move_latch(openLatchPos);                           // moves the latch to setted position
}

void set_closed_latch_pos(){
  if ( Serial.available() )
  {
    String inStr = Serial.readString();
    if(inStr == "cp\n"){
      closedLatchPos = closedLatchPos + 10;
      Serial.print("setting closed course to ");
      Serial.print(closedLatchPos);
    }else if(inStr.equals("cm\n")){
      closedLatchPos = closedLatchPos - 10;
      Serial.print("setting closed course to ");
      Serial.print(closedLatchPos);
    }else{
      Serial.print("input error [");
      Serial.print(inStr);
      Serial.print("]");
    }
    Serial.print("\n");
  }
  move_latch(closedLatchPos);                         // moves the latch to setted position
}

// FLIGHT PHASES
void preflight(){
  set_open_latch_pos();
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
  if(digitalRead(tiltButtonPin) == HIGH){
    open_latch();
    digitalWrite(armedLedPin, LOW);
  }
}
