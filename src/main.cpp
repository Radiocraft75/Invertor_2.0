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

void PID() {
  if ((PowerNow < (PowerSet - PowerSet * Gist)) && (OCR2B < 200)) {
    OCR2B++;
  }
  if ((PowerNow > (PowerSet + PowerSet * Gist)) && (OCR2B > 0)) {
    OCR2B--;
  }
}
