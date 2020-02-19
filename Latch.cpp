#include "Arduino.h"
#include "Latch.h"


//////////////////////
// constructor

Latch::Latch() {}

//////////////////////
// methods
void Latch::init(int pin, int openPos, int closedPos) {
  pinMode(pin, OUTPUT);
  _pin = pin;
  _openPos = openPos;
  _closedPos = closedPos;
  _servo.attach(_pin);
}

void Latch::_moveLatch(int pos) {
  _servo.write(pos);            // set latch to open
  delay(15);                    // waits for the servo to get there
}

void Latch::openLatch() {
  _moveLatch(_openPos);
}

void Latch::closeLatch() {
  _moveLatch(_closedPos);
}

//////////////////////
// getters/setters

int Latch::getOpenPos() {
  return _openPos;
}

void Latch::setOpenPos(int pos) {
  _openPos = pos;
}

int Latch::getClosedPos() {
  return _closedPos;
}

void Latch::setClosedPos(int pos) {
  _closedPos = pos;
}
