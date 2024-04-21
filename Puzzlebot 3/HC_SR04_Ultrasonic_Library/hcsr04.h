#ifndef hcsr04_H
#define hcsr04_H
#include "mbed.h"


 
class HCSR04 {
  public:
    HCSR04(PinName t, PinName e);
    long echo_duration();
    float distance();
 
    private:
        DigitalOut trig;
        DigitalIn echo;
        Timer timer;
        long duration;
        float distance_cm;
};
 
#endif