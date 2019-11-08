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

DigitalIn DinUnfoldLim(PF_6);
PwmOut HGA(PE_11);//1/2
PwmOut HGB(PE_9);//1/1
PwmOut ENTA(PE_5);//9/1
PwmOut ENSA(PE_6);//9/2
PwmOut LMA(PA_3);//2/4

PwmOut MechanismPins[]{//機構用pwmピン指定
  HGA,HGB,ENTA,ENSA,LMA
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
  double RollTowelPwm;
  double RollSheetPwm;
  bool Share;
  bool Options;
  double UnfoldPwm;
}ManualVaris;//arduinoからくるDualShockの信号

Timer TimerForQEI;            //エンコーダクラス用共有タイマー
Timer TimerForMove;
Timer TimerForRoll;
Timer TimerForCir;

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
//Serial SerialControl(PD_5,PD_6);

CheckFin chObX(20,10);
CheckFin chObY(20,10);

//DigitalIn LimitRight(LimitPin.LimRight);
//DigitalIn LimitLeft(LimitPin.LimLeft);


float TargetXYy[][3] = {{0,100.0f,0},{0,-100.0f,0}};//X Y MechanismType
int currentPoint = 0; 

bool updateMechanismEnc(MWodometry *p,int encoderPPRHigh){//バスタオル機構のロリコンがencoderPPRHigh分回ったかどうかの判定
  if(abs(p->getPulses()) < encoderPPRHigh)return(1);
  else
  {
    p->setDistance(0);
    return(0);
  }
  serial.printf("%d\n",p->getPulses());
}

bool updateMechanismLim(DigitalIn *p,int HorL){//HorLになったら止める
  int read = p->read();
  if(read != HorL)return(1);
  else return(0);
}
void ReceivePacket()//arduinoから信号を受け取る
{
  char buf[9];
  IMU.i2c_->read(0x08<<1,buf,9);
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
    ManualVaris.Share = buf[8] & 0b10;
    ManualVaris.Options = buf[8] & 0b01;
  }else{
    //接続状態じゃなかったら初期化
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
    ManualVaris.Share = 0;
    ManualVaris.Options = 0;
  }
}

void Mechanisms(double *pidYaw){//コントローラーのボタンが押されてた時の処理
  static double thisyaw;
  static bool isHighTri = false;
  static bool isHighCir = false;
  static bool preisHighCir = false;
  static bool isHighShareOptions = false;
  if(ManualVaris.TRIANGLE)
  {
    isHighTri = true;
  }
  if(isHighTri)
  {
    if(updateMechanismEnc(&odometryXAxis,12))ManualVaris.RollTowelPwm = 0.5;//まだ回る
    else
    {
      ManualVaris.RollTowelPwm = 0;//止める
      isHighTri = false;
    }
  } 

  if(ManualVaris.CIRCLE)
  {
    if(!preisHighCir)
    {
      isHighCir = true;
      TimerForCir.reset();
      TimerForCir.start();
    }
  }
  if(isHighCir)
  {
    if(TimerForCir.read_ms() < 800){
      ManualVaris.RollSheetPwm = 0.1;
    }else{
      TimerForCir.stop();
      ManualVaris.RollSheetPwm = 0;
      isHighCir = false;
    } 
  }

  if(ManualVaris.CROSS)
  {
    pidObYaw.update(0,IMU.getYaw());//初期角度に回転
    *pidYaw = pidObYaw.getTerm();
  }
  if(ManualVaris.SQUARE)
  {

  }
  if(*pidYaw)//コントローラーから回転の信号が来てるかどうか
  {
    thisyaw = IMU.getYaw();//reset
    TimerForRoll.reset();
  }else
  {               //来てなかったら初期角度を維持
    if(TimerForRoll.read_ms() < 500)thisyaw = IMU.getYaw();
    pidObYaw.update(thisyaw,IMU.getYaw());
    *pidYaw = pidObYaw.getTerm();
  }
  if(ManualVaris.R1 && ManualVaris.L1){//初期角度を設定
    IMU.reset();
    thisyaw = 0;
  }

  if(ManualVaris.Share && ManualVaris.Options)
  {
    isHighShareOptions = true;
  }
  serial.printf("%d\n",isHighShareOptions);
  if(isHighShareOptions)
  {
    if(updateMechanismLim(&DinUnfoldLim,1))ManualVaris.UnfoldPwm = 0.3;//1になるまで回り続ける
    else
    {
      ManualVaris.UnfoldPwm = 0;
      isHighShareOptions = false;
    }
  }
  preisHighCir = isHighCir;
}

void updateMechanism(){//機構用pwm出力
  double mechanismPWMOutput[4];
  mechanismPWMOutput[0] = (double)ManualVaris.HangerY*0.00787*0.3;//max0.3に抑える
  mechanismPWMOutput[1] = ManualVaris.RollTowelPwm;
  mechanismPWMOutput[2] = ManualVaris.RollSheetPwm;
  mechanismPWMOutput[3] = ManualVaris.UnfoldPwm;
  serial.printf("%f:%f:%f:%f\n",mechanismPWMOutput[0],mechanismPWMOutput[1],mechanismPWMOutput[2],mechanismPWMOutput[3]);
  wheelKinematics.controlMotor(MechanismPins,mechanismPWMOutput,0);
}

void setup(){
  pidObX.setOutputLimit(Pwms.MaxPwm);
  pidObY.setOutputLimit(Pwms.MaxPwm);
  pidObX.setOutputLimit(Pwms.MaxPwm);
  serial.printf("%s","setupIMU:");
  IMU.setup();
  serial.printf("%s\n","END");
  for(int i = 0;i<8;i++)WheelPins[i].period_ms(1);
  for(int i = 0;i<5;i++)MechanismPins[i].period_ms(1);
  TimerForMove.start();
  TimerForRoll.start();
}  


void update(int XLocation,int YLocation,int Yaw){

  double pidYaw;
  pidYaw = (double)Yaw*0.001;
  Mechanisms(&pidYaw);
  //serial.printf("%f %d:%d:%d  %d:%d\n",IMU.getYaw(),XLocation,YLocation,Yaw,ManualVaris.HangerY,ManualVaris.R1);
  serial.printf("%d:%d\n",ManualVaris.Share,ManualVaris.Options);
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

