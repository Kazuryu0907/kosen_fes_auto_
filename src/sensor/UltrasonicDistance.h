#include "mbed.h"

class UltrasonicDistance{
public:
    UltrasonicDistance(PinName trig,PinName p):trig(trig),io(p),abletimeout(false){}
    UltrasonicDistance(PinName trig,PinName p,int timeout):trig(trig),io(p),_timeout(timeout),abletimeout(true){}
    double getDistance();
private:
    int getPulse();
    DigitalInOut io;
    DigitalOut trig;
    int _timeout;
    Timer pulsetime,runtime;
    bool abletimeout;
    unsigned long int Len_mm_X100;
    unsigned long int Len_Integer;
};