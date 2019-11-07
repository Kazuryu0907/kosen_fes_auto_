//PS4のヘッダーファイル
#include<PS4BT.h>    //無線
#include<Wire.h>
USB Usb;
BTD Btd(&Usb);       //無線
PS4BT PS4(&Btd);     //無線
#define DEBUG

#define ADDR 0x08
char Status = 'E';
void requestEvent();
void setup() {
  //通信速度
  Serial.begin(115200);
  Wire.begin(ADDR);
  Wire.onRequest(requestEvent);
  //PS4 BT関係
  while (!Serial);
  if (Usb.Init() == -1) {
    Serial.print(F("\nPS4 Do Not Connect \n"));
    //PS4が認識できないときLED全点灯
    while (1);
  }
  Serial.print(F("\nPS4 OK Program  Start\n"));
}
uint8_t StickX;
uint8_t StickY;
uint8_t AnalogR2;
uint8_t AnalogL2;
uint8_t Buttons;
uint8_t Hanger;
uint8_t DigitalRL1;
void SendPacket();
uint8_t preDigitalRL1;
unsigned long timer;
unsigned long TimerForRumble;
uint8_t ResetAngle;
bool Runble = false;
int16_t Accel;
ButtonEnum buttons[4] = {TRIANGLE,CIRCLE,CROSS,SQUARE};
void loop() {
  
  Usb.Task();

  if (PS4.connected()) 
  { //通信中に実行
    // put your main code here, to run repeatedly:
    Status = 'S';
    Buttons = 0;
    StickX =  PS4.getAnalogHat(LeftHatY);
    StickY =  PS4.getAnalogHat(LeftHatX);
    AnalogR2 = PS4.getAnalogButton(R2);
    AnalogL2 = PS4.getAnalogButton(L2);
    Hanger = PS4.getAnalogHat(RightHatY);
    DigitalRL1 = PS4.getButtonPress(R1) << 1 | PS4.getButtonPress(L1);
    //Accel = PS4.getSensor(aX);
    ResetAngle = 0;
    if(DigitalRL1 == 0b11)//high high 
    {
        if(preDigitalRL1 != 0b11)timer = millis();
        if(timer + 100 < millis())//0.5秒以上
        {
          ResetAngle = DigitalRL1;
          TimerForRumble = millis();
          PS4.setRumbleOn(RumbleHigh);
          Runble = true;
        }
    }else{
        timer = millis();
      }
    for(int i = 0;i<4;i++){
      Buttons = Buttons | (int)PS4.getButtonPress(buttons[i]) << 3 - i;
    }
    if(Runble)
    {
      if(TimerForRumble + 100 < millis())
      {
        Runble = false;
        PS4.setRumbleOff();
      } 
    }
    
    #ifdef DEBUG
      Serial.print("X:");
      Serial.print(StickX);
      Serial.print("Y:");
      Serial.print(StickY);
      Serial.print("R2:");
      Serial.print(AnalogR2);
      Serial.print("L2:");
      Serial.print(AnalogL2);
      Serial.print("Buttons:");
      Serial.print(Buttons,BIN);
      Serial.print("Hanger:");
      Serial.print(Hanger);
      Serial.print("R1:");
      Serial.print(DigitalRL1);
      Serial.print("AC:");
      Serial.println(Accel);
    #endif
      //接続を切る
      preDigitalRL1 = DigitalRL1;
    if (PS4.getButtonClick(PS)) {
      Status = 'E';
      PS4.disconnect();
    }
    
  }
}
//スティック

void requestEvent()
{
  Wire.write(Status);
  Wire.write(StickX);
  Wire.write(StickY);
  Wire.write(AnalogR2);
  Wire.write(AnalogL2);
  Wire.write(Buttons);
  Wire.write(Hanger);
  Wire.write(ResetAngle);
}
