#ifndef Smoother_h
#define Smoother_h

//#include "Arduino.h"

class Smoother
{
  public:
    Smoother::Smoother();
    void addSmVal(float val);
    void rstSmTo(float val);
    float getSmVal();
  private:
    float _rawVals[35];           // set smoother resolution here
};

#endif
