#include "Arduino.h"
#include "Cli.h"


//////////////////////
// constructor

Cli::Cli() {}

//////////////////////
// methods
void Cli::init(Latch &latch, int &qnh) {
  _latch = &latch;
  _qnh = &qnh;
}

void Cli::readCommand() {
  if (Serial.available()) {
    String inStr = Serial.readString();           // reading input from serial port
    if (inStr.startsWith("qnh")) {                // looking for QNH command
      _setQnh(inStr);
    } else if (inStr.startsWith("o")) {           // looking for open position command
      _setLatchOpenPos(inStr);
    } else if (inStr.startsWith("c")) {           // looking for closed position command
      _setLatchClosedPos(inStr);
    }
  }
}

void Cli::_setQnh(String in) {
  if (in.startsWith("qnh")) {             // looking for QNH command
    *_qnh = in.substring(3, 7).toInt();
//    maxAlt = aprxAlt ;
//    minAlt = aprxAlt ;
  }
}

void Cli::_setLatchOpenPos(String in) {
  //  if (flightPhase == 0) {
  if (in.equals("op\n")) {
    _latch -> setOpenPos(_latch -> getOpenPos() + 5);
  } else if (in.equals("om\n")) {
    _latch -> setOpenPos(_latch -> getOpenPos() - 5);
  }
  _latch -> openLatch();                           // moves the latch to setted position
}

void Cli::_setLatchClosedPos(String in) {
  //  if (flightPhase == 1) {
  if (in.equals("cp\n")) {
    _latch -> setClosedPos(_latch -> getClosedPos() + 5);
  } else if (in.equals("cm\n")) {
    _latch -> setClosedPos(_latch -> getClosedPos() - 5);
  }
  _latch -> closeLatch();                           // moves the latch to setted position

}
