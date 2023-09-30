#include "calc.h"

int CalcAmp(unsigned int Vm) {
  float I_calc = Vm * maxADCVolt / ADCSamples - ZeroCorrection + 0.007;
  I_calc = I_calc / mVperAmp;
  if (I_calc < 0) I_calc = 0;
  int I = (int)(I_calc * 1000);
  return I;
}

////Расчёт максимального тока
//unsigned int  IMaxEr_AMP = 30;
//
//unsigned int set_IMaxEr() {
//  unsigned int IMaxEr = (IMaxEr_AMP * mVperAmp + ZeroCorrection) * ADCSamples / maxADCVolt;
//  return IMaxEr;
//  }

//+++++++++++++++++++++++