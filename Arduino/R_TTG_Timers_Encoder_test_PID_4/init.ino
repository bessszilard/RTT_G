//#include <Arduino.h>
#include "init_header.h"
//#include <WProgram.h>

void Encoder_Init(){
  pinMode(Enc_RA, INPUT);
  pinMode(Enc_RB, INPUT);
  pinMode(Enc_LA, INPUT);
  pinMode(Enc_LB, INPUT); 
}

void Motor_Init(){
  pinMode(Left_Motor_PWM_Pin, OUTPUT);
  pinMode(Left_Motor_4A,  OUTPUT);
  pinMode(Left_Motor_3A,  OUTPUT);

  pinMode(Right_Motor_PWM_Pin, OUTPUT);  
  pinMode(Right_Motor_1A,  OUTPUT);  
  pinMode(Right_Motor_2A,  OUTPUT); 
}

void LED_Init(){
  pinMode(LED42, OUTPUT);
  pinMode(LED44, OUTPUT);
  pinMode(LED46, OUTPUT);
  pinMode(LED48, OUTPUT);
  pinMode(LED50, OUTPUT);
  pinMode(LED52, OUTPUT);

  digitalWrite(LED42, LOW);
  digitalWrite(LED44, LOW);
  digitalWrite(LED46, LOW);
  digitalWrite(LED48, LOW);
  digitalWrite(LED50, LOW);
  digitalWrite(LED52, LOW);
}
void timer4_Init(unsigned int prescaler){

  TCCR4A = 0;
  TCCR4B = 0;

  //  timer4_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  timer4_counter = 0;

  TCNT4 = timer4_counter;   // preload timer

    switch(prescaler){
  case 1: // 1 prescaler
    TCCR4B |= (1 << CS10);
    break;

  case 8:  // 8 prescaler
    TCCR4B |= (1 << CS11);
    break;

  case 64: // 64 prescaler 
    TCCR4B |= (1 << CS10);
    TCCR4B |= (1 << CS11);    
    break;

  case 256: // 256 prescaler 
    TCCR4B |= (1 << CS12);    
    break;

  case 1024: // 64 prescaler 
    TCCR4B |= (1 << CS10);
    TCCR4B |= (1 << CS12);    
    break;

  default:
    TCCR4B |= (1 << CS10);
    break;
  }
  TIMSK4 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void timer3_Init(unsigned int prescaler){

  TCCR3A = 0;
  TCCR3B = 0;

  //  timer3_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  timer3_counter = 0;

  TCNT3 = timer3_counter;   // preload timer

    switch(prescaler){
  case 1: // 1 prescaler
    TCCR3B |= (1 << CS10);
    break;

  case 8:  // 8 prescaler
    TCCR3B |= (1 << CS11);
    break;

  case 64: // 64 prescaler 
    TCCR3B |= (1 << CS10);
    TCCR3B |= (1 << CS11);    
    break;

  case 256: // 256 prescaler 
    TCCR3B |= (1 << CS12);    
    break;

  case 1024: // 64 prescaler 
    TCCR3B |= (1 << CS10);
    TCCR3B |= (1 << CS12);    
    break;

  default:
    TCCR3B |= (1 << CS10);
    break;
  }
  TIMSK3 |= (1 << TOIE1);   // enable timer overflow interrupt
}




