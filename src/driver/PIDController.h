
// 制御対象: controlled object.
// 制御量:   controlled variable.
// 目標値:   desired value.
// 操作量:   manipulated variable.

class PIDController
{
public:
  PIDController(double pGain, double iGain, double dGain); // enable all gain and manipulated variable.
  PIDController(double pGain);                             // enable only propotional gain and manipulated variable.

  typedef enum gainType
  {
    proportional,
    integral,
    differential,
  } gainType;

  void update(double target, double current);

  double getTerm();

  double getTerm(gainType);

  void modifyGain(gainType, double);

  void setOutputLimit(double);

private:
  struct
  {
    double gain, term, outMax;
    bool isEnable;
  } p, i, d;

  double errorStack; // 毎回のターゲット値と現在値の差の蓄積(I制御で使用)
  double prevError;  // 前回の偏差
  double maxValue;   // 容認する最大出力
};
