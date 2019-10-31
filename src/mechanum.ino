//PS4のヘッダーファイル
#include<PS4BT.h>    //無線
USB Usb;
BTD Btd(&Usb);       //無線
PS4BT PS4(&Btd);     //無線

////////セットアップ////////
void setup() {
  //通信速度
  Serial.begin(115200);

  //PS4 BT関係
  while (!Serial);
  if (Usb.Init() == -1) {
    Serial.print(F("\nPS4 Do Not Connect \n"));
    //PS4が認識できないときLED全点灯
    while (1);
  }
  Serial.print(F("\nPS4 OK Program  Start\n"));
}


void loop() {
  Usb.Task();

  if (PS4.connected()) 
  { //通信中に実行
    // put your main code here, to run repeatedly:
    uint8_t StickX =  PS4.getAnalogHat(LeftHatY) - 127;
    uint8_t StickY =  PS4.getAnalogHat(LeftHatX) - 127;
    uint8_t AnalogR2 = PS4.getAnalogButton(R2);
    uint8_t AnalogL2 = PS4.getAnalogButton(L2);
    uint8_t Buttons = PS4.getButtonPress(TRIANGLE) << 3 || PS4.getButtonPress(CIRCLE) << 2 || PS4.getButtonPress(CROSS) << 1 || PS4.getButtonPress(SQUARE);
    Serial.print("X:");
    Serial.print(StickX);
    Serial.print("Y:");
    Serial.print(StickY);
    Serial.print("R2:");
    Serial.print(AnalogR2);
    Serial.print("L2:");
    Serial.print(AnalogL2);
      //接続を切る
    if (PS4.getButtonClick(PS)) {
      PS4.disconnect();
    }
  }
}
//スティック
