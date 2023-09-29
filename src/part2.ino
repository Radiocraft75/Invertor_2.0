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
