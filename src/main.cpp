#include "mbed.h"
#include "sensor/MPU9250.h"
#include "sensor/QEI.h"
#include "driver/PIDController.h"
#include "driver/MWodometry.h"
#include "driver/WheelKinematics.h"
#include "other/CheckFin.h"

#define SIZEOF(a) (sizeof(a)/sizeof(&a)-1)

//#define MANUAL

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

struct
{
  int LocationX;
  int LocationY;
  int Yaw;
  bool TRIANGLE;
  bool CIRCLE;
  bool CROSS;
  bool SQUARE;
  int count;
  float offset_X = 0;
  float offset_Y = 0;
  int sampling = 100;
}ManualVaris;

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
Serial SerialControl(PD_5,PD_6);

CheckFin chObX(20,10);
CheckFin chObY(20,10);

DigitalIn LimitRight(LimitPin.LimRight);
DigitalIn LimitLeft(LimitPin.LimLeft);

/*
int points_blue[][3] = {
    {UP,5900-a,0},
    {LEFT,3575,1},
    {RIGHT,3575,0},
    {BACK,5900-a,0},
    {UP,5100-roboy-a,0},
    {LEFT,3575,-1},
    {RIGHT,3575,0},
    {BACK,5100-roboy-a,0}
    };
*/
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


void update(u_int8_t XLocation,u_int8_t YLocation,u_int8_t Yaw){
  pidObX.update(XLocation,0);
  pidObY.update(YLocation,0);
  pidObYaw.update(Yaw*0.05,0);//iF X is LOW

  double pidX = pidObX.getTerm();
  double pidY = pidObY.getTerm();
  double pidYaw = pidObYaw.getTerm();

  serial.printf("%f%s%f%s%f%s%d%s%d\n",IMU.getYaw(),":",pidX,":",pidY,"  ",XLocation,":",YLocation);
  wheelKinematics.getScale(pidX,pidY,pidYaw,IMU.getYaw(),driverPWMOutput);
  wheelKinematics.controlMotor(WheelPins,driverPWMOutput);
}


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

typedef enum{
  X = 1,Y,R,L,B
}EnumPacket;

void ReceivePacket()
{
  static int countPacket = 0;
  if(SerialControl.readable())
  {
    u_int8_t val = SerialControl.getc();
    serial.printf("%d\n",val);
    switch(countPacket)
    {
      case X:
        if(ManualVaris.count > ManualVaris.sampling)ManualVaris.offset_X += val;
        else if(ManualVaris.count == ManualVaris.sampling)ManualVaris.offset_X /= ManualVaris.sampling;
        else  ManualVaris.LocationX = (int)val - ManualVaris.offset_X;
      break;
      case Y:
        if(ManualVaris.count > ManualVaris.sampling)ManualVaris.offset_Y += val;
        else if(ManualVaris.count == ManualVaris.sampling)ManualVaris.offset_Y /= ManualVaris.sampling;
        else  ManualVaris.LocationY = (int)val - ManualVaris.sampling;
      break;
      case R:
        if(val < 3)val = 0;
        ManualVaris.Yaw = -val;  
      break;
      case L:
        if(val < 3)val = 0;
        ManualVaris.Yaw += val;  
      break;
      case B:
        if(val & 0b1000)ManualVaris.TRIANGLE = 1;
        else ManualVaris.TRIANGLE = 0;
        if(val & 0b0100)ManualVaris.CIRCLE = 1;
        else ManualVaris.CIRCLE = 0;
        if(val & 0b0010)ManualVaris.CROSS = 1;
        else ManualVaris.CROSS = 0;
        if(val & 0b0001)ManualVaris.SQUARE = 1;
        else ManualVaris.SQUARE = 0;
      break;
    }
    countPacket++;
    if(val == 255)countPacket = 1;
  }
}

void Mechanisms(){
  if(ManualVaris.TRIANGLE)
  {

  }
  if(ManualVaris.CIRCLE)
  {

  }
  if(ManualVaris.CROSS)
  {

  }
  if(ManualVaris.SQUARE)
  {

  }
}
int main(){
    double currentXLocation,currentYLocation;
    #ifndef MANUAL
      setup();
    #endif
    //SerialControl.format(7,1);
    SerialControl.baud(SerialBaud.SoftwareSerial);
    while(1){
      //IMU.update();
      #ifdef MANUAL
        //ReceivePacket();
        //Mechanisms();
        if(SerialControl.readable())serial.printf("%s\n","able");//serial.printf("%c\n",SerialControl.getc());
        else serial.printf("%s\n","unable");
        //update(ManualVaris.LocationX,ManualVaris.LocationY,ManualVaris.Yaw);
        #ifdef DEBUG
          //serial.printf("%d%s%d%s%d%s%d%s%d%s%d%s%d\n",ManualVaris.LocationX,":",ManualVaris.LocationY,":",ManualVaris.Yaw,":",ManualVaris.TRIANGLE,":",ManualVaris.CIRCLE,":",ManualVaris.CROSS,":",ManualVaris.SQUARE);
        #endif
      #else
        update(&currentXLocation,&currentYLocation);
      #endif
  } 

}

