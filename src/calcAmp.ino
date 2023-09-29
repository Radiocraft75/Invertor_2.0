const double ADCSamples = 1024;   //1024 samples
const double maxADCVolt = 5.0;    //5 Volts
const double ZeroCorrection = 0;  //Calibration coefficient
const double mVperAmp = 0.06;

int CalcAmp(unsigned int Vm) {
  float I_calc = Vm * maxADCVolt / ADCSamples - ZeroCorrection;
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
