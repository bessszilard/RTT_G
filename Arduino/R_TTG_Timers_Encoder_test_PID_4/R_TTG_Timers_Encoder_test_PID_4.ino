  #include "init_header.h"
  //#include <WProgram.h>
  
  #define Enc_RA 18
  #define Enc_RB 16
  
  #define Enc_LA 19
  #define Enc_LB 17
  
  unsigned int timer4_counter = 0;
  unsigned int timer3_counter = 0;
  unsigned int i = 0;
  unsigned int j = 0;
  
  unsigned char L_Motor_dir = 0;
  unsigned char R_Motor_dir = 0;
  int  L_Motor_Pos = 0;
  int  R_Motor_Pos = 0;
  
  int l_enc = 0;
  unsigned int l_enc_temp;
  int r_enc = 0;
  unsigned int r_enc_temp;
  
  bool timer4_flag = 0;
  bool timer3_flag = 0;
  
  //unsigned int time_elapsed;
  unsigned int L_min_time = 65535;
  unsigned int R_min_time = 65535;
  
  long unsigned int goal_Pos = 0;
  unsigned int Pos_inc = 18;
  /*
  int Kp = 3;
  int Kpper = 1;    // never be 0
  
  int Kpi = 2;
  int Kpiper = 3;    // never be 0
  
  int Ki = 1;
  int Kiper = 50;    // never be 0
   */
  
  int Kp = 3;
  int Kpper = 1;    // never be 0
  
  int Kpi = 2;
  int Kpiper = 3;    // never be 0
  
  int Ki = 0;
  int Kiper = 50;    // never be 0
  
  
   
  int stab_imp = 5;
  long unsigned int Left_Motor_pos = 0;
  long unsigned int Right_Motor_pos = 0;
  int Left_I_error = 0;
  int Right_I_error = 0;
  
  int I_error = 0;
  int PI_error;
  
  int error;
  
  int Left_Motor_PWM = 0;
  int Right_Motor_PWM = 0;
  
  void Left_Motor_(unsigned int dir);
  void Right_Motor_(unsigned int dir);
  
  unsigned int l_motor_interrupts;
  unsigned int r_motor_interrupts;
  
  void setup(){
    LED_Init();
    Encoder_Init();
    Motor_Init();
  
    attachInterrupt(5, RA_interrupt, CHANGE);  //Pin 18
    attachInterrupt(4, LA_interrupt, CHANGE);  //Pin 19
  
    Serial.begin(9600);
    Serial3.begin(9600);
  
    noInterrupts();           // disable all interrupts
    timer4_Init(64);    //timer4(prescaler); 8, 64, 256, 1024
    // timer4_counter = 40535;    // 10Hz - PID + samples
    timer4_counter = 53035;  //20 Hz
  
    timer3_Init(256);
    timer3_counter = 15535;
    //    timer3_counter = 15535;    // 5 Hz - increment
    interrupts();             // enable all interrupts
  
  }
  
  //////////////////////////Loop//////////////////////////
  void loop(){
    unsigned char inByte = Serial3.read();
  
    if(inByte == 'W'){
      Left_Motor_(FOWARD);
      Right_Motor_(FOWARD);
      //     Left_Motor_PWM = 200;
      //     Right_Motor_PWM = 200;
    }
    else if(inByte == 'S'){
      Left_Motor_(BACKWARD);
      Right_Motor_(BACKWARD);
      //   Left_Motor_PWM = 200;
      //   Right_Motor_PWM = 200;
    }
    else if(inByte == 'A'){
      Left_Motor_(BACKWARD);
      Right_Motor_(FOWARD);
      //   Left_Motor_PWM = 200;
      //   Right_Motor_PWM = 200;
    }
    else if(inByte == 'D'){
      Left_Motor_(FOWARD);
      Right_Motor_(BACKWARD);
      //      Left_Motor_PWM = 200;
      //      Right_Motor_PWM = 200;
    }
    else if(inByte == 'H'){
      Left_Motor_ (STOP);
      Right_Motor_(STOP);
  
      if(l_enc !=0 && r_enc !=0){
        Left_Motor_PWM = 0;
        Right_Motor_PWM = 0;
        Left_I_error = 0;
        Right_I_error = 0;
        I_error = 0;
      } 
    }
 if(timer3_flag && (l_motor_interrupts !=0) || (r_motor_interrupts !=0)){
        Serial3.write(0xFF);
        delay(10);
        Serial3.write(l_motor_interrupts);
        delay(10);
        Serial3.write(r_motor_interrupts);
        delay(10);
 }
  }////////////////////////Loop End///////////////////////
  
  void Right_Motor_(unsigned int dir){
    switch(dir){
    case FOWARD:
      digitalWrite(Right_Motor_1A, HIGH);
      digitalWrite(Right_Motor_2A, LOW); 
      break;
    case BACKWARD:
      digitalWrite(Right_Motor_1A, LOW);
      digitalWrite(Right_Motor_2A, HIGH);
      break;
    case STOP:
      digitalWrite(Right_Motor_1A, LOW);
      digitalWrite(Right_Motor_2A, LOW);
      break;
    }
  }
  
  void Left_Motor_(unsigned int dir){
    switch(dir){
    case FOWARD:
      digitalWrite(Left_Motor_3A, HIGH);
      digitalWrite(Left_Motor_4A, LOW); 
      break;
    case BACKWARD:
      digitalWrite(Left_Motor_3A, LOW);
      digitalWrite(Left_Motor_4A, HIGH);
      break;
    case STOP:
      digitalWrite(Left_Motor_3A, LOW);
      digitalWrite(Left_Motor_4A, LOW);
      break;
    }
  }
  bool LA;
  bool LB;
  /////////////////Interrupts/////////////////
  //Input Change Noticed
  void LA_interrupt(){
    /* LA = digitalRead(Enc_LA);
     LB = digitalRead(Enc_LB);
     
     if(LA)
     if(LB) L_Motor_dir = FOWARD;
     else   L_Motor_dir = BACKWARD;
     else
     if(LB) L_Motor_dir = BACKWARD; 
     else   L_Motor_dir = FOWARD;
     */
    l_enc += 1;
    TCNT3 = timer3_counter;      // moving
  }
  
  bool RA;
  bool RB;
  void RA_interrupt(){
    /*  RA = digitalRead(Enc_RA);
     RB = digitalRead(Enc_RB);
     
     if(RA)
     if(RB) R_Motor_dir = FOWARD;
     else   R_Motor_dir = BACKWARD;
     else
     if(RB) R_Motor_dir = BACKWARD;
     else   R_Motor_dir = FOWARD;
     */
    r_enc += 1;
    TCNT3 = timer3_counter;    // moving
  }
  
  //timer interrupts
  ISR(TIMER3_OVF_vect){  // R_Motor timer
    TCNT3 = timer3_counter;   // preload timer
    timer3_flag = 1;
    digitalWrite(LED50, digitalRead(LED50) ^ 1);
    // if(l_enc !=0 && r_enc !=0){
    I_error = 0;
    //  Right_Motor_PWM = Left_Motor_PWM;
    Left_Motor_PWM = 0;
    Right_Motor_PWM = 0;
    // }
  }
  unsigned int L_PWM_Write;
  unsigned int R_PWM_Write;
  
  ISR(TIMER4_OVF_vect){  // L_Motor timer
    TCNT4 = timer4_counter;   // preload timer
    l_motor_interrupts = l_enc;
    r_motor_interrupts = r_enc;
  
    //Left_Motor_pos += l_motor_interrupts;
    //Right_Motor_pos += r_motor_interrupts;
  
    PI_error = ((int)l_enc  - (int)r_enc);
    I_error += ((int)l_enc  - (int)r_enc);
    /*  if((int)l_enc - (int)r_enc)
     I_error = 0;
     */
  
    Left_Motor_PWM  += Kp * (stab_imp - l_enc - Kpi * PI_error / Kiper - Ki * I_error / Kiper) / Kpper ;
    Right_Motor_PWM += Kp * (stab_imp - l_enc + Kpi * PI_error / Kiper + Ki * I_error / Kiper) / Kpper ;
    //Right_Motor_PWM -= Ki * I_error / Kiper;// + Kdif *(old_error - I_error) / Kdifper;
    /*
            if(l_enc > 2 || r_enc > 2){
     Left_Motor_PWM  += Kdif * (r_enc - l_enc);
     Right_Motor_PWM += Kdif * (l_enc - r_enc);//+ Ki * I_error / Kiper;  
     }*/
    L_PWM_Write = Left_Motor_PWM;
    R_PWM_Write = Right_Motor_PWM;
  
    if(Left_Motor_PWM  > 255) Left_Motor_PWM = 255;
    if(Right_Motor_PWM > 255) Right_Motor_PWM = 255;
  
    if(Left_Motor_PWM  < 0)  L_PWM_Write = 0;
    if(Right_Motor_PWM < 0)  R_PWM_Write = 0;
  
    /*  if(Right_Motor_PWM < 0) Right_Motor_PWM = 0;
     if(Right_Motor_PWM > 255) Right_Motor_PWM = 0;
     */
    analogWrite(Left_Motor_PWM_Pin, Left_Motor_PWM);
    analogWrite(Right_Motor_PWM_Pin, Right_Motor_PWM);
  
    timer4_flag = 1;
    l_enc = 0;
    r_enc = 0;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

