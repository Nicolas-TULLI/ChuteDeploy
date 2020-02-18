#include "Arduino.h"
#include "Cli.h"


//////////////////////
// constructor

Cli::Cli() {}

//////////////////////
// methods
void Cli::init(Latch *latch, FlightDatas *fds) {
  _latch = latch;
  _fds = fds;
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
  if (in.startsWith("qnh")) {                     // looking for QNH command
    _fds -> setQnh(in.substring(3, 9).toFloat());
  }
}

void Cli::_setLatchOpenPos(String in) {
  if (_fds -> getFlightPhase() == 0) {
    if (in.equals("op\n")) {
      _latch -> setOpenPos(_latch -> getOpenPos() + 5);
    } else if (in.equals("om\n")) {
      _latch -> setOpenPos(_latch -> getOpenPos() - 5);
    }
    _latch -> openLatch();                           // moves the latch to setted position
  }
}

void Cli::_setLatchClosedPos(String in) {
  if (_fds -> getFlightPhase() == 1) {
    if (in.equals("cp\n")) {
      _latch -> setClosedPos(_latch -> getClosedPos() + 5);
    } else if (in.equals("cm\n")) {
      _latch -> setClosedPos(_latch -> getClosedPos() - 5);
    }
    _latch -> closeLatch();                           // moves the latch to setted position
  }
}
