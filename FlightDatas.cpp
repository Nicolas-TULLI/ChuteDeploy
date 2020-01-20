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
  _reset = true;
}

void FlightDatas::setAlt(float alt) {
  // check reset state
  if (_reset) {
    resetMinMax(alt);
    _reset = false;
  }

  _lastMs = _now;
  _now = millis();
  _lastAlt = _aprxAlt;
  _aprxAlt = alt;
  
  _lastVarioAlt = _aprxVarioAlt;
  _aprxVarioAlt = getSmoothedAlt();
    
  _smAlt.addSmVal(alt);
    
  _maxAlt >= getSmoothedAlt() ? : _maxAlt = getSmoothedAlt();
  _minAlt <= getSmoothedAlt() ? : _minAlt = getSmoothedAlt();
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
void FlightDatas::resetMinMax(float rstVal){
  _smAlt.rstSmTo(rstVal);
  _maxAlt = rstVal;
  _minAlt = rstVal;
}

int FlightDatas::getLoopTime(){
  return _now - _lastMs;
}

float FlightDatas::vario(){
  return (_aprxAlt - _lastAlt) / (_now - _lastMs)*1000;
}

float FlightDatas::smVario(){
  return (_aprxVarioAlt - _lastVarioAlt) / (_now - _lastMs)*1000;
}
