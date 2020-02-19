/*
   regroup flight datas as the active flight phase, altitude history,
   and computed ones as a "smoothed" altitude reading, the vetical speed (vario).
   isFalling() tries to detect fall after apogee
*/

#ifndef FlightDatas_h
#define FlightDatas_h

#include "Smoother.h"

class FlightDatas
{
  public:
    FlightDatas::FlightDatas();
    int getFlightPhase();
    void setFlightPhase(int phase);
    void resetMinMax(float rstVal);
    int getLoopTime();
    bool isFalling();

    float getQnh();
    void setQnh(float qnh);
    void setAlt(float alt);
    float getAlt();
    float getSmoothedAlt();

    float getMaxAlt();
    float getMinAlt();

    float vario();
    float smVario();
    float getMaxVario();
    float getMinVario();

  private:
    Smoother _smAlt;

    int _flightPhase = 0;   // variable to track flight phases
    float _qnh = 1013;      // QNH
    int _lastMs;            // time elapsed since last loop for the variometer
    int _now;
    int _reset = 0;         // used to follow up reset state, need two actualy

    float _aprxAlt;
    float _lastAlt;
    float _maxAlt;
    float _minAlt;

    float _aprxVarioAlt;
    float _lastVarioAlt;    // used to smoothen vario without smoother
    float _maxVario;
    float _minVario;

    float _maxVarioConfirmAscent = 1 ;  // max vario to be reached to confirm ascent in m/s
    int _fallingLoops = 10;             // number of loop falling before setting apogee
    int _fallCounter = 0;               // count number of falling loops
    int _launchPhase = 2;               // to check if we are airborne to reset min/max at launch
};

#endif
