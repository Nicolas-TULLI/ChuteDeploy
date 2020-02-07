/*
 * ease the use of a servo motor as a door latch
 */
 
#ifndef Latch_h
#define Latch_h

#include "Arduino.h"
#include <Servo.h>

class Latch
{
  public:
    Latch::Latch();
    void init(int pin, int openPos, int closedPos);
    void openLatch();
    void closeLatch();
    int getOpenPos();
    void setOpenPos(int pos);
    int getClosedPos();
    void setClosedPos(int pos);
  private:
    void _moveLatch(int pos);
    Servo _servo;
    int _pin;
    int _openPos;                  // variable for open latch position setting
    int _closedPos;               // variable for closed latch position setting
};

#endif
