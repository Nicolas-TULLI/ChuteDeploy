/*
   sort of "commande line interface" to configure the system.
   mostly set the open and closed position of the latch mechanism
   and set the Qnh
*/

#ifndef Cli_h
#define Cli_h

#include "Arduino.h"
#include "Latch.h"
#include "FlightDatas.h"

class Cli
{
  public:
    Cli::Cli();
    void readCommand();
    void init(Latch *latch, FlightDatas *fds);
  private:
    Latch *_latch;
    FlightDatas *_fds;
    void _setQnh(String in);
    void _setLatchOpenPos(String in);
    void _setLatchClosedPos(String in);
};

#endif
