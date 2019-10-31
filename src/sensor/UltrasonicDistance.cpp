#include "UltrasonicDistance.h"

double UltrasonicDistance::getDistance(){
    trig = 1;
    wait_us(10);
    trig = 0;
    int pulse = getPulse();
    if((pulse < 60000) && (pulse > 1)) {
        Len_mm_X100 = (pulse*34)/2;
        Len_Integer = Len_mm_X100/100;
        return(Len_Integer);
    }else{
        return(-1);
    }
}


int UltrasonicDistance::getPulse(){
    runtime.reset();
    runtime.start();
    pulsetime.reset();
    io.input();
    while(io == 1){
        if(abletimeout)if(runtime.read_us() > _timeout) return(-1);
    }
    while(io == 0){
        if(abletimeout)if(runtime.read_us() > _timeout) return(-1);
    }
    pulsetime.start();
    while(io == 1){
        if(abletimeout)if(runtime.read_us() > _timeout) return(-1);
    }
    pulsetime.stop();
    return(pulsetime.read_us());
}

