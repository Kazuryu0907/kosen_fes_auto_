#include "mbed.h"
#include "sensor/MPU9250.h"
#include "sensor/QEI.h"
#include "driver/PIDController.h"
#include "driver/MWodometry.h"
#include "driver/WheelKinematics.h"


struct
{
  PinName XAxisAPulse = PF_9;
  PinName XAxisBPulse = PF_8;
  PinName XAxisIndexPulse = NC;
  PinName YAxisAPulse = PA_4_ALT0;
  PinName YAxisBPulse = PB_0_ALT0;
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

WheelKinematics wheelKinematics(WheelKinematics::Mechanum4WD, 0.3);

PIDController pidObX(0.1, 0.05, 0);
PIDController pidObY(0.1, 0.05, 0);
PIDController pidObYaw(0.01, 0.005, 0);

Serial serial(USBTX, USBRX);

float TargetXY[][2] = {0,100.0f};
int currentPoint = 0; 

enum {
  UP,
  LEFT
  }MovingStatus;
  

void update(float *currentXLocation,float *currentYLocation){
  IMU.update();
  double xTemp = odometryXAxis.getDistance();
  odometryXAxis.setDistance(0);
  double yTemp = odometryYAxis.getDistance();
  odometryYAxis.setDistance(0);

  *currentXLocation += xTemp * cos(ToRadian(IMU.getYaw()));
  *currentYLocation += xTemp * sin(ToRadian(IMU.getYaw()));
  *currentXLocation -= yTemp * sin(ToRadian(IMU.getYaw()));
  *currentYLocation += yTemp * cos(ToRadian(IMU.getYaw()));

  serial.printf("%f%s%f\n",*currentYLocation,":",*currentYLocation);

  pidObX.update(TargetXY[currentPoint][0],*currentXLocation);
  pidObY.update(TargetXY[currentPoint][1],*currentYLocation);

  double pidX = pidObX.getTerm();
  double pidY = pidObY.getTerm();
  double pidYaw = pidObYaw.getTerm();

  wheelKinematics.getScale(pidX,pidY,pidYaw,IMU.getYaw(),driverPWMOutput);
  wheelKinematics.controlMotor(WheelPins,driverPWMOutput);
}


int main(){
    float currentXLocation,currentYLocation;

    pidObX.setOutputLimit(0.30f);
    pidObY.setOutputLimit(0.30f);
    pidObX.setOutputLimit(0.30f);
    serial.printf("%s","setupIMU:");
    IMU.setup();
    serial.printf("%s\n","END");
    for(int i = 0;i<8;i++)WheelPins[i].period_ms(1);
    while(1){
      update(&currentXLocation,&currentYLocation);
  } 

}

