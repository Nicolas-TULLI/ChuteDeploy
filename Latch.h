/*
 * 
 */
#ifndef Latch_h
#define Latch_h

#include "Arduino.h"


class Latch
{
  public:
    Latch(int pin);
    void setOpenPos(int pos);
    void setClosedPos(int pos);
    void moveLatch(int pos);
    void openLatch();
    void closeLatch();
  private:
    int _pin;
    int _openPos;                  // variable for open latch position setting
    int _closedPos;               // variable for closed latch position setting
};

#endif
