#include "CheckFin.h"
#include "mbed.h"
Serial ss(USBTX,USBRX);
void CheckFin::update(double term)
{
  shift(array);
  array[0] = term;
  ss.printf("%f\n",round(*array,-5) - round(*(array+arrayLength-1),-5));
  if(round(array[0],-5) == round(array[arrayLength-1],-5))countFin++;
  if(countFin > 10)check = true;
  else check = false;
}

void CheckFin::reset()
{
    for(int i = 0;i<arrayLength;i++)array[i] = 0;
    countFin = 0;
    check = false;
}

bool CheckFin::isEnd()
{
    return(check);
}

double CheckFin::round(double src, int n)
{
    double dst;
    
    dst = src * pow(10, -n - 1);          /*処理を行う桁を10-1 の位にする*/
    dst = (double)(int)(dst + 0.5);
    
    return    dst * pow(10, n + 1);       /*処理を行った桁を元に戻す*/
}

void CheckFin::shift(double *array)
{
  int temp;
  for(int i = 0;i<arrayLength;i++)
  {
    temp = arrayLength-i;
    *(array + temp-1) = *(array + temp-2);
  }
}