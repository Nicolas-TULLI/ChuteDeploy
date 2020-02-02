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
    int getQnh();
    void setQnh(int qnh);
    void setAlt(float alt);
    float getAlt();
    float getSmoothedAlt();
    int getLoopTime();
    float getMaxAlt();
    float getMinAlt();
    float vario();
    float smVario();
    float getMaxVario();
    float getMinVario();
    bool isApogee();
    
  private:
    Smoother _smAlt;
//    Smoother _smVario;
    int _flightPhase = 0;   // variable to track flight phases
    int _qnh = 1013;        // QNH
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

    float _maxVarioConfirmAscent = 0.6 ;  // max vario to be reached to confirm ascent
    int _fallCountIsApogee = 10;          // number of loop falling before setting apogee
    int _fallCounter = 0;                 // count number of falling loops
    int _lauchPhase = 2;                  // to check if we are airborne to reset min/max at launch
};

#endif
