  //#include <Arduino.h>
  #include "init_header.h"
  #include <WProgram.h>
  
  void Accelero_Init(){
    pinMode(Acc_X_Pin, INPUT);
    pinMode(Acc_Y_Pin, INPUT);
    pinMode(Acc_Z_Pin, INPUT);
    analogReference(INTERNAL2V56);  
  }
  
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
  void Lamp_Init(){
    pinMode(Lamp_Pin, OUTPUT);
  }  

  void timer3_Init(unsigned int prescaler){
    TCCR3A = 0;
    TCCR3B = 0;
  
    TCNT3 = timer3_counter;   // preload timer
    switch(prescaler){
    case 1:    // 1 prescaler
      TCCR3B |= (1 << CS10);
      break;
    case 8:    // 8 prescaler
      TCCR3B |= (1 << CS11);
      break;
    case 64:   // 64 prescaler 
      TCCR3B |= (1 << CS10);
      TCCR3B |= (1 << CS11);    
      break;
    case 256:  // 256 prescaler 
      TCCR3B |= (1 << CS12);    
      break;
    case 1024: // 64 prescaler 
      TCCR3B |= (1 << CS10);
      TCCR3B |= (1 << CS12);    
      break;
    default:   // 1 prescaler
      TCCR3B |= (1 << CS10);
      break;
    }
    TIMSK3 |= (1 << TOIE1);   // enable timer overflow interrupt
  }
  
  void timer4_Init(unsigned int prescaler){
    TCCR4A = 0;
    TCCR4B = 0;
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
    default:  // 1 prescaler
      TCCR4B |= (1 << CS10);
      break;
    }
    TIMSK4 |= (1 << TOIE1);   // enable timer overflow interrupt
  }
  
  void LEDS(unsigned int STATUS){
    switch(STATUS){
    case OFF:
      digitalWrite(LED42, LOW);
      digitalWrite(LED44, LOW);
      digitalWrite(LED46, LOW);
      digitalWrite(LED48, LOW);
      digitalWrite(LED50, LOW);
      digitalWrite(LED52, LOW);
      break;
    case ON:
      digitalWrite(LED42, HIGH);
      digitalWrite(LED44, HIGH);
      digitalWrite(LED46, HIGH);
      digitalWrite(LED48, HIGH);
      digitalWrite(LED50, HIGH);
      digitalWrite(LED52, HIGH);
      break;
    case BLINK:
      digitalWrite(LED42, !digitalRead(LED42));
      digitalWrite(LED44, !digitalRead(LED44));
      digitalWrite(LED46, !digitalRead(LED46));
      digitalWrite(LED48, !digitalRead(LED48));
      digitalWrite(LED50, !digitalRead(LED50));
      digitalWrite(LED52, !digitalRead(LED52));
      break;
    }
  }
  
  

