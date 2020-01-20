#ifndef Cli_h
#define Cli_h

#include "Arduino.h"
#include "Latch.h"

class Cli
{
  public:
    Cli::Cli();
    void readCommand();
    void init(Latch &latch, int &qnh);
  private:
    Latch *_latch;
    int *_qnh;
    void _setQnh(String in);
    void _setLatchOpenPos(String in);
    void _setLatchClosedPos(String in);
};

#endif
