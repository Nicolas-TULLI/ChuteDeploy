/*
 * 
 */
#include "Arduino.h"
#include "Latch.h"

Latch::Latch(int pin)
{
  _pin = pin;
  pinMode(pin, OUTPUT);
}

void setOpenPos(int pos){
  _openPos = pos;
}

void setClosedPos(int pos){
  _closedPos = pos;
}

void move_latch(int pos) {
  latchServo.write(pos);            // set latch to open
  delay(15);                        // waits for the servo to get there
}

void open_latch() {
  move_latch(openLatchPos);
}

void close_latch() {
  move_latch(closedLatchPos);
}
