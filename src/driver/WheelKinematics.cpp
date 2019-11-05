#include "WheelKinematics.h"


void WheelKinematics::getScale(double x, double y, double yaw, double yawAngle, SCALE_UNIT *scale)
{
    switch (kinematicsType)
    {
    case Omni3WD:
        getOmniKinematics3wdScale(x, y, yaw, yawAngle, scale);
        wheelsize = 3;
        break;
    case Omni4WD:
        getOmniKinematics4wdScale(x, y, yaw, yawAngle, scale);
        wheelsize = 4;
        break;
    case Mechanum4WD:
        getMechanumKinematics4wdScale(x, y, yaw, yawAngle, scale);
        wheelsize = 4;
        break;
    }

    //計算上の最大出力を求める
    SCALE_UNIT max = 0;
    for (int i = 0; i < wheelsize; i++)
    {
        if (max < abs(scale[i]))
            max = abs(scale[i]);
    }

    //最大出力が許容最大出力を超えていたら再計算する
    double rate = 0;
    if (maxAllocateOutput < max)
    {
        rate = maxAllocateOutput / max;
        for (int i = 0; i < wheelsize; i++)
        {
            scale[i] = scale[i] * rate;
        }
    }
}

void WheelKinematics::getOmniKinematics3wdScale(double x, double y, double yaw, double yawAngle, SCALE_UNIT *scale)
{
    //逆運動学を使って各軸の移動量からモータの回転方向・量を計算する
    //モーターの正転は反時計回り scale[0]はFL
    scale[0] = x * cos(ToRadian(yawAngle + 240)) + y * sin(ToRadian(yawAngle + 240)) + yaw;
    scale[1] = x * cos(ToRadian(yawAngle +   0)) + y * sin(ToRadian(yawAngle +   0)) + yaw;
    scale[2] = x * cos(ToRadian(yawAngle + 120)) + y * sin(ToRadian(yawAngle + 120)) + yaw;
}

void WheelKinematics::getOmniKinematics4wdScale(double x, double y, double yaw, double yawAngle, SCALE_UNIT *scale)
{
    //逆運動学を使って各軸の移動量からモータの回転方向・量を計算する
    //モーターの正転は反時計回り scale[0]はFL
    scale[0] = x * cos(ToRadian(yawAngle + 225)) + y * sin(ToRadian(yawAngle + 225)) + yaw;
    scale[1] = x * cos(ToRadian(yawAngle + 315)) + y * sin(ToRadian(yawAngle + 315)) + yaw;
    scale[2] = x * cos(ToRadian(yawAngle +  45)) + y * sin(ToRadian(yawAngle +  45)) + yaw;
    scale[3] = x * cos(ToRadian(yawAngle + 135)) + y * sin(ToRadian(yawAngle + 135)) + yaw;
}

void WheelKinematics::getMechanumKinematics4wdScale(double x, double y, double yaw, double yawAngle, SCALE_UNIT *scale)
{
    //逆運動学を使って各軸の移動量からモータの回転方向・量を計算す
    //RF LF RB LB
    scale[0] = x * cos(ToRadian(yawAngle + 135)) + y * sin(ToRadian(yawAngle + 135)) + yaw;
    scale[1] = x * cos(ToRadian(yawAngle + 45)) + y * sin(ToRadian(yawAngle + 45)) - yaw;
    scale[2] = x * cos(ToRadian(yawAngle + 45)) + y * sin(ToRadian(yawAngle + 45)) + yaw;
    scale[3] = x * cos(ToRadian(yawAngle +  135)) + y * sin(ToRadian(yawAngle +  135)) - yaw;
}

void WheelKinematics::controlMotor(PwmOut *WheelPins,SCALE_UNIT *driverPWMOutput)
{
    for(int i = 0;i<4;i++){
      if(driverPWMOutput[i] > 0){
        WheelPins[i*2] = driverPWMOutput[i];
        WheelPins[i*2+1] = 0;
      }else{
        WheelPins[i*2] = 0;
        WheelPins[i*2+1] = -driverPWMOutput[i];
      }
    }
}

void WheelKinematics::controlMotor(PwmOut *WheelPins,SCALE_UNIT *driverPWMOutput,int s,int e)
{
    for(int i = s;i<e;i++){
      if(driverPWMOutput[i] > 0){
        WheelPins[i*2] = driverPWMOutput[i];
        WheelPins[i*2+1] = 0;
      }else{
        WheelPins[i*2] = 0;
        WheelPins[i*2+1] = -driverPWMOutput[i];
      }
    }
}