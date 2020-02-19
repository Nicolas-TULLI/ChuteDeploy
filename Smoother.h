/*
   stores history of a value and give a mean value
*/
#ifndef Smoother_h
#define Smoother_h

class Smoother
{
  public:
    Smoother::Smoother();
    void addSmVal(float val);     // adding a value to the smoother
    void rstSmTo(float val);      // reset the smoother array filling it with a value
    float getSmVal();             // get the mean value of the array
  private:
    float _rawVals[35];           // set smoother resolution in the array size
};

#endif
