#include "mbed.h"
#include "sensor/MPU9250.h"
#include "sensor/QEI.h"
#include "driver/PIDController.h"
#include "driver/MWodometry.h"
#include "driver/WheelKinematics.h"
#include "other/CheckFin.h"

#define SIZEOF(a) (sizeof(a)/sizeof(&a)-1)

int arrayLength = 10;
typedef enum{
  Bath=1,
  Sheet
}MechanismType;

struct
{
  PinName XAxisAPulse = PF_9;
  PinName XAxisBPulse = PF_8;
  PinName XAxisIndexPulse = NC;
  PinName YAxisAPulse = PB_0;
  PinName YAxisBPulse = PA_4;
  PinName YAxisIndexPulse = NC;
} OdometryPin;

struct
{
  const int encoderPPRLow = 48;
  const int encoderPPRHigh = 1024;
  // mini(50) omni
  const double encoderAttachedWheelRadius = 5.25;
  // max:0.7, recommend:0.64 //DEFAULT 0.5
  const double estimateDriveMaxPWM = 0.5;
  const double estimateDriveMinPWM = 0.10;
} Robot;

PwmOut RFA(PB_4);
PwmOut RFB(PB_5);
PwmOut LFA(PC_8);
PwmOut LFB(PC_9);
PwmOut RBA(PD_12);
PwmOut RBB(PD_13);
PwmOut LBA(PD_14);
PwmOut LBB(PD_15);

struct
{
  double MaxPwm = 0.2;
}Pwms;

PwmOut WheelPins[8] = {
  RFA,RFB,LFA,LFB,RBA,RBB,LBA,LBB
};

double driverPWMOutput[4]; 

struct
{
  PinName IMUSDA = PB_11;
  PinName IMUSCL = PB_10;
} I2CPin;

struct baudRate
{
  const long HardwareSerial = 256000;
  const long I2C = 400000;
  const long SoftwareSerial = 115200;
} SerialBaud;

Timer TimerForQEI;            //エンコーダクラス用共有タイマー
Timer TimerForMove;
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C);

QEI encoderXAxis(OdometryPin.XAxisAPulse,
                 OdometryPin.XAxisBPulse,
                 OdometryPin.XAxisIndexPulse,
                 Robot.encoderPPRHigh,
                 &TimerForQEI,
                 QEI::X2_ENCODING);
QEI encoderYAxis(OdometryPin.YAxisAPulse,
                 OdometryPin.YAxisBPulse,
                 OdometryPin.YAxisIndexPulse,
                 Robot.encoderPPRHigh,
                 &TimerForQEI,
                 QEI::X2_ENCODING);


MWodometry odometryXAxis(encoderXAxis,
                         Robot.encoderPPRHigh,
                         Robot.encoderAttachedWheelRadius);
MWodometry odometryYAxis(encoderYAxis,
                         Robot.encoderPPRHigh,
                         Robot.encoderAttachedWheelRadius);

WheelKinematics wheelKinematics(WheelKinematics::Mechanum4WD,Pwms.MaxPwm);

PIDController pidObX(0.03, 0.0001, 0);
PIDController pidObY(0.03, 0.0001, 0);
PIDController pidObYaw(0.01, 0.005, 0);

Serial serial(USBTX, USBRX);

CheckFin chObX(20,10);
CheckFin chObY(20,10);

float TargetXYy[][3] = {{0,100.0f,0},{0,-100.0f,0}};//X Y MechanismType
int currentPoint = 0; 


void setup(){
  pidObX.setOutputLimit(Pwms.MaxPwm);
  pidObY.setOutputLimit(Pwms.MaxPwm);
  pidObX.setOutputLimit(Pwms.MaxPwm);

  serial.printf("%s","setupIMU:");
  IMU.setup();
  serial.printf("%s\n","END");

  for(int i = 0;i<8;i++)WheelPins[i].period_ms(1);
  
  TimerForMove.start();
}  



void update(double *currentXLocation,double *currentYLocation){
  
  IMU.update();
  double xTemp = odometryXAxis.getDistance();
  odometryXAxis.setDistance(0);
  double yTemp = odometryYAxis.getDistance();
  odometryYAxis.setDistance(0);

  *currentXLocation += xTemp * cos(ToRadian(IMU.getYaw()));
  *currentYLocation += xTemp * sin(ToRadian(IMU.getYaw()));
  *currentXLocation -= yTemp * sin(ToRadian(IMU.getYaw()));
  *currentYLocation += yTemp * cos(ToRadian(IMU.getYaw()));
  
  

  pidObX.update(TargetXYy[currentPoint][0],*currentXLocation);
  pidObY.update(TargetXYy[currentPoint][1],*currentYLocation);
  pidObYaw.update(0.0,IMU.getYaw());

  double pidX = pidObX.getTerm();
  double pidY = pidObY.getTerm();
  double pidYaw = pidObYaw.getTerm();

  chObX.update(pidX);
  chObY.update(pidY);

  if(chObX.isEnd() && chObY.isEnd())
  {
    currentPoint++;
    chObX.reset();
    chObY.reset();
    *currentXLocation = 0;
    *currentYLocation = 0;
  }

  serial.printf("%f%s%f%s%f%s%f%s%f\n",IMU.getYaw(),":",pidX,":",pidY,"  ",*currentXLocation,":",*currentYLocation);
  wheelKinematics.getScale(pidX,pidY,pidYaw,IMU.getYaw(),driverPWMOutput);
  wheelKinematics.controlMotor(WheelPins,driverPWMOutput);


   switch ((int)TargetXYy[currentPoint][2])
   {
   case Bath:

   break;

   case Sheet:

   break;
   }

}


int main(){
    double currentXLocation,currentYLocation;
    setup();
    while(1){
      update(&currentXLocation,&currentYLocation);
  } 

}

