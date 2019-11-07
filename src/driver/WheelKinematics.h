#include "math.h"
#include "mbed.h"

#ifdef Arduino
#define SCALE_UNIT uint8_t
#else
#define SCALE_UNIT double
#endif

#define PI (3.14159265359)
#define ToRadian(degree) ((degree) / 180 * PI)

class WheelKinematics {
public:
  typedef enum {
    Omni3WD,
    Omni4WD,
    Mechanum4WD
  } KinematicsType;

  WheelKinematics(WheelKinematics::KinematicsType type) : kinematicsType(type) {};
  WheelKinematics(WheelKinematics::KinematicsType type, SCALE_UNIT maxAllocateOutput) : kinematicsType(type), maxAllocateOutput(maxAllocateOutput) {};

  KinematicsType getKinematicsType() { return kinematicsType; }
  void getScale(double, double, double, double, SCALE_UNIT*);
  void controlMotor(PwmOut *pwm,SCALE_UNIT*);
  void controlMotor(PwmOut *pwm,SCALE_UNIT*,int e);
private:
  KinematicsType kinematicsType;
  SCALE_UNIT maxAllocateOutput;
  int wheelsize = 0;
  void getOmniKinematics3wdScale(double, double, double, double, SCALE_UNIT*);
  void getOmniKinematics4wdScale(double, double, double, double, SCALE_UNIT*);
  void getMechanumKinematics4wdScale(double, double, double, double, SCALE_UNIT*);
  
};