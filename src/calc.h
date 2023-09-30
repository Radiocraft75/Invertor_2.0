#ifndef calc_h
#define calc_h

const double ADCSamples = 1024;   //1024 samples
const double maxADCVolt = 5.0;    //5 Volts
const double ZeroCorrection = 2.5;  //Calibration coefficient
const double mVperAmp = 0.100;//0.185;

int CalcAmp(unsigned int Vm);

#endif