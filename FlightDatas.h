#ifndef FlightDatas_h
#define FlightDatas_h

#include "Smoother.h"

class FlightDatas
{
  public:
    FlightDatas::FlightDatas();
    int flightPhase = 0;                   // variable to track flight phases

    int getQnh();
    void setQnh(int qnh);
    void resetMinMax(float rstVal);
    
    void setAlt(float alt);
    float getAlt();
    float getSmoothedAlt();
    int getLoopTime();
    float getMaxAlt();
    float getMinAlt();
    float vario();
    float smVario();
    
  private:
    Smoother _smAlt;
    Smoother _smVario;
    int _qnh = 1013;                        // QNH
    bool _reset = false;
    float _aprxAlt;
    float _lastAlt;
    float _aprxVarioAlt;
    float _lastVarioAlt;
    float _maxAlt;
    float _minAlt;
    int _lastMs;                            // time elapsed since last loop for the variometer
    int _now;
};

#endif
