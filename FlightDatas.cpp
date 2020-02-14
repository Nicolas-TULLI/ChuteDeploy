#include "Arduino.h"
#include "FlightDatas.h"

//////////////////////
// constructor

FlightDatas::FlightDatas() {}

//////////////////////
// methods

// QNH
int FlightDatas::getQnh() {
  return _qnh;
}

void FlightDatas::setQnh(int qnh) {
  _qnh = qnh;
  _reset = 1;
}

void FlightDatas::setAlt(float alt) {

  resetMinMax(alt);

  _lastMs = _now;
  _now = millis();

  _lastAlt = _aprxAlt;
  _aprxAlt = alt;

  _lastVarioAlt = _aprxVarioAlt;
  _aprxVarioAlt = getSmoothedAlt();

  // alt smoother
  _smAlt.addSmVal(alt);

  // min max recording
  _maxVario >= smVario() ? : _maxVario = smVario();
  _minVario <= smVario() ? : _minVario = smVario();
  _maxAlt >= getSmoothedAlt() ? : _maxAlt = getSmoothedAlt();
  _minAlt <= getSmoothedAlt() ? : _minAlt = getSmoothedAlt();
}

bool FlightDatas::isFalling() {
  if (_fallCounter >= _fallingLoops) {
    return true;
  }
  else if (_maxVario > _maxVarioConfirmAscent &&
           smVario() < 0 &&
           _launchPhase <= _flightPhase) {
    _fallCounter++;
    return false;
  } else {
    return false;
  }
}


float FlightDatas::getAlt() {
  return _aprxAlt;
}

float FlightDatas::getSmoothedAlt() {
  return _smAlt.getSmVal();
}

float FlightDatas::getMaxAlt() {
  return _maxAlt;
}

float FlightDatas::getMinAlt() {
  return _minAlt;
}

// Min Max
void FlightDatas::resetMinMax(float rstVal) {
  // check reset state
  switch (_reset) {
    case 1:
      _reset++;
      _smAlt.rstSmTo(rstVal);
      _maxAlt = rstVal;
      _minAlt = rstVal;
      break;
    case 2:
      _maxVario = 0;
      _minVario = 0;
      _reset = 0;
      _fallCounter = 0;
      break;
  }
}

int FlightDatas::getLoopTime() {
  return _now - _lastMs;
}

float FlightDatas::vario() {
  return (_aprxAlt - _lastAlt) / (_now - _lastMs) * 1000;
}

float FlightDatas::smVario() {
  return (_aprxVarioAlt - _lastVarioAlt) / (_now - _lastMs) * 1000;
}

float FlightDatas::getMaxVario() {
  return _maxVario;
}

float FlightDatas::getMinVario() {
  return _minVario;
}

int FlightDatas::getFlightPhase() {
  return _flightPhase;
}

void FlightDatas::setFlightPhase(int phase) {
  _flightPhase = phase;
  if (_launchPhase == _flightPhase) {
    _reset = 1;
  }
}
