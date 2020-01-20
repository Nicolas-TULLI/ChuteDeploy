#include "Smoother.h"

//////////////////////
// constructor

Smoother::Smoother() {}

//////////////////////
// methods

void Smoother::addSmVal(float val) {
  // shifting array one element downand adding last val on top
  for (int i = sizeof(_rawVals) / sizeof(_rawVals[0]) - 1 ; i > 0 ; i--) {
    _rawVals[i] = _rawVals[i - 1];
  }
  _rawVals[0] = val;
}

float Smoother::getSmVal() {
  float sum = 0;
  for (int i = sizeof(_rawVals) / sizeof(_rawVals[0]) - 1 ; i > -1 ; i--) {
    sum += _rawVals[i];
  }
  return sum / (sizeof(_rawVals) / sizeof(_rawVals[0]));
}

void Smoother::rstSmTo(float val) {
  for (int i = sizeof(_rawVals) / sizeof(_rawVals[0]) - 1 ; i > -1 ; i--) {
    _rawVals[i] = val;
  }
}
