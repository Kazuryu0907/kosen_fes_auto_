#include "mbed.h"
#include "sensor/MPU9250.h"
#include "sensor/QEI.h"
#include "driver/PIDController.h"
#include "driver/MWodometry.h"
#include "driver/WheelKinematics.h"
#include "other/CheckFin.h"

#define SIZEOF(a) (sizeof(a)/sizeof(&a)-1)

#define MANUAL

#define DEBUG

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
  PinName LimRight = PG_10;
  PinName LimLeft = PG_15;
}LimitPin;

typedef enum
{
  STCONN,CONN,NOCONN
}PS4Status;

struct
{
  const int encoderPPRLow = 48;
  #ifdef MANUAL
  const int encoderPPRHigh = 48;
  #else
  const int encoderPPRHigh = 1024;
  #endif
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

PwmOut HGA(PE_11);
PwmOut HGB(PE_9);
PwmOut ENA(PE_5);
PwmOut ENB(PE_6);
PwmOut MechanismPins[]{
  HGA,HGB,ENA,ENB
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

struct
{
  int LocationX;
  int LocationY;
  int Yaw;
  bool TRIANGLE;
  bool CIRCLE;
  bool CROSS;
  bool SQUARE;
  PS4Status Status = NOCONN;
  int HangerY;
  int R1;
  int L1;
  int preR1;
  double RollPwm;
}ManualVaris;

Timer TimerForQEI;            //エンコーダクラス用共有タイマー
Timer TimerForMove;
Timer TimerForRoll;

MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C);

I2C i2cControl(I2CPin.IMUSDA,I2CPin.IMUSCL);

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
PIDController pidObYaw(0.03, 0.005, 0);

Serial serial(USBTX, USBRX);
Serial SerialControl(PD_5,PD_6);

CheckFin chObX(20,10);
CheckFin chObY(20,10);

DigitalIn LimitRight(LimitPin.LimRight);
DigitalIn LimitLeft(LimitPin.LimLeft);


float TargetXYy[][3] = {{0,100.0f,0},{0,-100.0f,0}};//X Y MechanismType
int currentPoint = 0; 

bool updateMechanismEnc(MWodometry *p,int encoderPPRHigh){
  if(p->getPulses() < encoderPPRHigh)return(1);
  else
  {
    p->setDistance(0);
    return(0);
  }
  serial.printf("%d\n",p->getPulses());
}
void ReceivePacket()
{
  char buf[8];
  IMU.i2c_->read(0x08<<1,buf,8);
  if(buf[0] == 'S')ManualVaris.Status = CONN;
  else if(buf[0] == 'E')ManualVaris.Status = NOCONN;
  if(ManualVaris.Status == CONN){
    ManualVaris.LocationY = abs(buf[1] - 127) < 10?0:-buf[1] + 127;
    ManualVaris.LocationX = abs(buf[2] - 127) < 10?0:buf[2] - 127;
    ManualVaris.Yaw = -buf[3] + buf[4];
    ManualVaris.TRIANGLE = buf[5] & 0b1000;
    ManualVaris.CIRCLE = buf[5] & 0b0100;
    ManualVaris.CROSS = buf[5] & 0b0010;
    ManualVaris.SQUARE = buf[5] & 0b0001;
    ManualVaris.HangerY = -(abs(buf[6] - 127) < 10?0:buf[6] - 127);
    ManualVaris.R1 = buf[7] & 0b10;
    ManualVaris.L1 = buf[7] & 0b01;
  }else{
    ManualVaris.LocationY = 0;
    ManualVaris.LocationX = 0;
    ManualVaris.Yaw = 0;
    ManualVaris.TRIANGLE = 0;
    ManualVaris.CIRCLE = 0;
    ManualVaris.CROSS = 0;
    ManualVaris.SQUARE = 0;
    ManualVaris.HangerY = 0;
    ManualVaris.R1 = 0;
    ManualVaris.L1 = 0;
  }
}

void Mechanisms(double *pidYaw){
  static double thisyaw;
  static bool isHigh = false;
  if(ManualVaris.TRIANGLE)
  {
    isHigh = true;
  }
  if(isHigh)
  {
    if(updateMechanismEnc(&odometryXAxis,12))ManualVaris.RollPwm = 0.1;//続行
    else
    {
      ManualVaris.RollPwm = 0;
      isHigh = false;
    }
  } 

  if(ManualVaris.CIRCLE)
  {
    
  }
  if(ManualVaris.CROSS)
  {
    pidObYaw.update(0,IMU.getYaw());
    *pidYaw = pidObYaw.getTerm();
  }
  if(ManualVaris.SQUARE)
  {

  }
  if(*pidYaw)
  {
    thisyaw = IMU.getYaw();//reset
    TimerForRoll.reset();
  }else
  {
    if(TimerForRoll.read_ms() > 100){
    pidObYaw.update(thisyaw,IMU.getYaw());
    *pidYaw = pidObYaw.getTerm();
    }
  }
  if(ManualVaris.R1 && ManualVaris.L1){
    IMU.reset();
    thisyaw = 0;
  }
}

void updateMechanism(){
  double mechanismPWMOutput[2];
  mechanismPWMOutput[0] = (double)ManualVaris.HangerY*0.00787*0.3;
  mechanismPWMOutput[1] = ManualVaris.RollPwm;
  serial.printf("%f:%f\n",mechanismPWMOutput[0],mechanismPWMOutput[1]);
  wheelKinematics.controlMotor(MechanismPins,mechanismPWMOutput,0,2);
}

void setup(){
  pidObX.setOutputLimit(Pwms.MaxPwm);
  pidObY.setOutputLimit(Pwms.MaxPwm);
  pidObX.setOutputLimit(Pwms.MaxPwm);
  serial.printf("%s","setupIMU:");
  IMU.setup();
  serial.printf("%s\n","END");
  for(int i = 0;i<8;i++)WheelPins[i].period_ms(1);
  for(int i = 0;i<2;i++)MechanismPins[i].period_ms(1);
  TimerForMove.start();
  TimerForRoll.start();
}  


void update(int XLocation,int YLocation,int Yaw){

  double pidYaw;
  pidYaw = (double)Yaw*0.001;
  Mechanisms(&pidYaw);
  serial.printf("%f %d:%d:%d  %d:%d\n",IMU.getYaw(),XLocation,YLocation,Yaw,ManualVaris.HangerY,ManualVaris.R1);
  wheelKinematics.getScale((double)XLocation*0.002,(double)YLocation*0.002,pidYaw,IMU.getYaw(),driverPWMOutput);
  updateMechanism();
  wheelKinematics.controlMotor(WheelPins,driverPWMOutput);
}

#ifndef MANUAL
void update(double *currentXLocation,double *currentYLocation){
  
  
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

  if(abs(*currentXLocation) > abs(TargetXYy[currentPoint][0])*0.1)chObX.update(*currentXLocation);
  if(abs(*currentYLocation) > abs(TargetXYy[currentPoint][1])*0.1)chObY.update(*currentYLocation);

  if(chObX.isEnd() && chObY.isEnd())
  {
    printf("%s\n","//////////////////next//////////////////");
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
#endif

int main(){
    double currentXLocation,currentYLocation;
    setup();
    while(1){
      IMU.update();
      #ifdef MANUAL
        ReceivePacket();
        update(ManualVaris.LocationX,ManualVaris.LocationY,ManualVaris.Yaw);
        //serial.printf("%d%s%d%s%d%s%d%s%d%s%d%s%d\n",ManualVaris.LocationX,":",ManualVaris.LocationY,":",ManualVaris.Yaw,":",ManualVaris.TRIANGLE,":",ManualVaris.CIRCLE,":",ManualVaris.CROSS,":",ManualVaris.SQUARE);
      #else
        update(&currentXLocation,&currentYLocation);
      #endif
  } 

}

