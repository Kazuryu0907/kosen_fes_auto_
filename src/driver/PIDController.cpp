#include "PIDController.h"

// 制御対象: controlled object.
// 制御量:   controlled variable.
// 目標値:   desired value.
// 操作量:   manipulated variable.

PIDController::PIDController(double pGain, double iGain, double dGain)
{
  this->p.gain = pGain;
  this->i.gain = iGain;
  this->d.gain = dGain;
  this->i.isEnable = true;
  this->d.isEnable = true;
}

PIDController::PIDController(double pGain)
{
  this->p.gain = pGain;
  this->i.isEnable = false;
  this->d.isEnable = false;
}

void PIDController::update(double target, double current)
{

  // 偏差計算
  double error = target - current;
  errorStack += error;

  // 各操作量計算
  p.term = error * p.gain;
  i.term = errorStack * i.gain;
  d.term = (error - prevError) * d.gain;

  // 合計操作量
  double totalTerm = p.term;
  if (i.isEnable)
  {
    totalTerm += i.term;
  }
  if (d.isEnable)
  {
    totalTerm += d.term;
  }

  //constrain
  i.term > maxValue ? i.term = maxValue : i.term < -maxValue ? i.term = -maxValue : i.term; //ハードウェアの制限でi.termが無限に累積して復帰が遅くならないよう
  d.term > maxValue ? d.term = maxValue : d.term < -maxValue ? d.term = -maxValue : d.term; //外乱の制御量が大きくならないよう
  totalTerm > maxValue ? totalTerm = maxValue : totalTerm < -maxValue ? totalTerm = -maxValue : totalTerm;

  //次回計算用に今回の偏差値を格納
  prevError = error;
}

double PIDController::getTerm()
{
  // 合計操作量
  double totalTerm = p.term;
  if (i.isEnable)
  {
    totalTerm += i.term;
  }
  if (d.isEnable)
  {
    totalTerm += d.term;
  }

  return totalTerm;
}

double PIDController::getTerm(gainType parameter)
{
  switch (parameter)
  {
  case proportional:
    return p.term;
  case integral:
    return i.term;
  case differential:
    return d.term;
  }
  return 0;
}

void PIDController::modifyGain(gainType parameter, double value)
{
  switch (parameter)
  {
  case proportional:
    p.gain = value;
    break;
  case integral:
    i.gain = value;
    break;
  case differential:
    d.gain = value;
    break;
  }
}

void PIDController::setOutputLimit(double estimateValue)
{
  maxValue = estimateValue;
}