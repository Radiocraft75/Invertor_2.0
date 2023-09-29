// Подключаем библиотеки:
#include <Wire.h>                  // подключаем библиотеку для работы с шиной I2C
#include <iarduino_I2C_connect.h>  // подключаем библиотеку для соединения arduino по шине I2C

iarduino_I2C_connect I2C2;

byte REG_Array[11];
//REG_Array[0] = 0; // 0 STOP 1 SET 2 START
//REG_Array[1] = 0; // 0-Bi   1-L   2-R
//REG_Array[2] = 0; // PowerSet >> 8
//REG_Array[3] = 0; // PowerSet
//REG_Array[4] = 0; // Uamp  >> 8
//REG_Array[5] = 0; // Uamp
//REG_Array[6] = 0; // I100  >> 8
//REG_Array[7] = 0; // I100
//REG_Array[8] = 0; // Error
//REG_Array[9] = 0; // Duty100  >> 8
//REG_Array[10] = 0; // Duty100
byte OldMode = 10;

unsigned int PowerSet = 0;
unsigned int PowerNow = 0;
double Duty = 0;
unsigned int Duty1000 = 0;
unsigned int Iamp = 0;
unsigned int Uamp = 0;

byte k = 0;
byte oldk = 0;
unsigned int Um = 0;
double Gist = 0.02;

#define DUAL 0
#define LEFT 1
#define RIGHT 2

//++++++++++++++++++++++++++++++++++++++++++++
#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER1_OVF_vect) {
  delayMicroseconds(10);   
  Um = analogRead(A0);  
  if (k < 9) k++;
  else k = 0;
}

void short_circuit() {
  DDRD &= ~(1 << DDD3);
  OCR2B = 0;
  REG_Array[8] = 99;
  REG_Array[0] = 0;
}

void timer1_OUT(byte b){
  switch (b) {
    case 0:{
      TCCR1A |= ((1 << COM1A1) | (1 << COM1B1));
      break;        
    }
    case 1:{
      TCCR1A |= ((0 << COM1A1) | (0 << COM1B1));
      digitalWrite(9, HIGH);
      break;
    }
    case 2:{
      TCCR1A |= ((0 << COM1A1) | (0 << COM1B1));
      digitalWrite(9, LOW);
      break;
    }
  }
}

void timer_1_set() {      // инициализация Timer1  
  cli();  // отключить глобальные прерывания
  TCCR1A = 0;   // установить регистры в 0
  TCCR1B = 0;
  ICR1 = 780; //Соответствует 20Гц
  OCR1A = 390;
  //OCR1B = 80;
  // set none-inverting mode
  //TCCR1A |= ((1 << COM1A1) | (1 << COM1B1));
  // Fast PWM mode
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  // START the timer with no prescaler
  TCCR1B |= (1 << CS10);//8
  TCCR1B |= (0 << CS11);
  TCCR1B |= (1 << CS12);

  //TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера
  TIMSK1 |= (1 << TOIE1);  // включить прерывание по совпадению таймера
  //TIMSK1 |= (1 << 5);  // включить прерывание по совпадению таймера
  sei(); // включить глобальные прерывания
  DDRB |= (1 << DDB1);

//  // TIMER 1 for interrupt frequency 20 Hz:
//cli(); // stop interrupts
//TCCR1A = 0; // set entire TCCR1A register to 0
//TCCR1B = 0; // same for TCCR1B
//TCNT1  = 0; // initialize counter value to 0
//// set compare match register for 20 Hz increments
//OCR1A = 12499; // = 16000000 / (64 * 20) - 1 (must be <65536)
//// turn on CTC mode
//TCCR1B |= (1 << WGM12);
//// Set CS12, CS11 and CS10 bits for 64 prescaler
//TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
//// enable timer compare interrupt
//TIMSK1 |= (1 << OCIE1A);
//sei(); // allow interrupts
}

void timer_2_set() {      // инициализация Timer1
  // TIMER 2 for interrupt frequency 10000 Hz:
  cli();  
  TCCR2A = 0x23;
  TCCR2B = 0;
  TCCR2B |= (1 << WGM22);
  TCCR2B |= (1 << CS21);
  OCR2A = 200;
  OCR2B = 0;
  sei();

//TCCR2A = 0; // set entire TCCR2A register to 0
//TCCR2B = 0; // same for TCCR2B
//TCNT2  = 0; // initialize counter value to 0
//// set compare match register for 10000 Hz increments
//OCR2A = 199; // = 16000000 / (8 * 10000) - 1 (must be <256)
//// turn on CTC mode
//TCCR2B |= (1 << WGM21);
//// Set CS22, CS21 and CS20 bits for 8 prescaler
//TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20);
//// enable timer compare interrupt
//TIMSK2 |= (1 << OCIE2A);
}

void Err_short_circuit() {
  Serial.println("Аппаратная защита по току");
  Serial.println("Таймаут 5 сек");
  delay(5000);
  REG_Array[8] = 0;
  OCR2B = 0;
  //DDRD |= (1 << DDD3);
  Serial.println("Restart");
  return;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void printValue(String name, String value, String unit) {
  Serial.print(name);
  Serial.print(" - ");
  Serial.print(value);
  Serial.print(unit);
  Serial.print('\t');
}

void Serial_Log() {
//  Serial.print("  k - "); Serial.print(k);
//  Serial.print("  U "); Serial.print(Uamp);
//  Serial.print("V  Iamp "); Serial.print(Iamp);
//  Serial.print("mA  DUTY "); Serial.print(Duty * 100);
//  Serial.print("%  Error "); Serial.print(REG_Array[8]); Serial.print(" ");
//  Serial.print("  PowerSet "); Serial.print(PowerSet);
//  Serial.print("  PowerNow "); Serial.println(PowerNow);

  printValue("k", (String) k, "");
  printValue("U", (String) Uamp, "V");
  printValue("Iamp", (String) Iamp, "mA");
  printValue("DUTY", (String) (Duty * 100), "%");
  printValue("Error", (String) REG_Array[8], "");
  printValue("PowerNow", (String) PowerNow, "W");
  Serial.println();
}



//+++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++


void PID() {
  if ((PowerNow < (PowerSet - PowerSet * Gist)) && (OCR2B < 200)) {
    OCR2B++;
  }
  if ((PowerNow > (PowerSet + PowerSet * Gist)) && (OCR2B > 0)) {
    OCR2B--;
  }
}




void setup() {
  //Wire.setClock(400000);  // устанавливаем скорость передачи данных по шине I2C = 400кБит/с
  Wire.begin(0x01);       // инициируем подключение к шине I2C в качестве ведомого (slave) устройства, с указанием своего адреса на шине.
  I2C2.begin(REG_Array);  // инициируем возможность чтения/записи данных по шине I2C, из/в указываемый массив
  attachInterrupt(0, short_circuit, FALLING);
  Serial.begin(115200);
  Serial.println("Start program");
}

void loop() {
  if (REG_Array[8] == 99) {  //Проверка на короткое замыкание
    Err_short_circuit();
  }

  //Отключить вывод если нет генерации
  // if (OCR2B == 0) {
  //  DDRD &= ~(1 << DDD3);
  // }
  // else {
  //  DDRD |= (1 << DDD3);
  // }

  if (REG_Array[0] != OldMode) {
    OldMode = REG_Array[0];
    switch (REG_Array[0]) {
      case 0:
        { //Stop
          Serial.println("*****Stop*****");
          DDRB &= ~(1 << DDB1);
          DDRD &= ~(1 << DDD3);
          OCR2B = 0;
          break;
        }
      case 1:
        { //Settings
          Serial.println("*****Setting*****");
          PowerSet = REG_Array[2] << 8;
          PowerSet += REG_Array[3];
          Serial.print("  PowerSet = ");
          Serial.println(PowerSet);
          Uamp = REG_Array[4] << 8;
          Uamp += REG_Array[5];
          Serial.print("  Uamp = ");
          Serial.println(Uamp);
          switch (REG_Array[1]) {
            case 0:
              { //BiPolar
                Serial.println("BiPolar");
                timer_1_set();
                timer_2_set();
                timer1_OUT(DUAL);
                break;
              }
            case 1:
              { //+Polar
                Serial.println("+Polar");
                timer_1_set();
                timer_2_set();
                timer1_OUT(RIGHT);
                break;
              }
            case 2:
              { //-Polar
                Serial.println("-Polar");
                timer_1_set();
                timer_2_set();
                timer1_OUT(LEFT);
                break;
              }
          }
          Serial.println("Save settings");
          break;
        }
      case 2:
        { //Start
          Serial.println("*****Start*****");
          DDRB |= (1 << DDB1);
          DDRD |= (1 << DDD3);
          break;
        }
    }
  }

  if (oldk != k) {
    oldk = k;
    //Вычисление
    Iamp = CalcAmp(Um);
    Duty = (double)OCR2B / OCR2A;
    PowerNow = (int)((long)Iamp * Uamp * Duty / 1000);

    REG_Array[6] = Iamp >> 8;
    REG_Array[7] = Iamp;

    Duty1000 = int(1000 * Duty);
    REG_Array[9] = Duty1000 >> 8;
    REG_Array[10] = Duty1000;

    if (REG_Array[0] == 2) {
      Serial_Log();

      PID();
    }
  }
}


