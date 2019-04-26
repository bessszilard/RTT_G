  #include "init_header.h"
  #include <WProgram.h>
  #include <Servo.h>
  #include <avr/wdt.h>
  
  unsigned int timer0_counter = 0;
  unsigned int timer3_counter = 0;
  unsigned int timer4_counter = 0;
  
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
  
  // bool timer0_flag2 = 0;
  bool timer3_flag = 0;
  bool timer4_flag = 0;
  
  bool moving_flag = 0;
  int Kp = 3;
  int Kpper = 1;    // never be 0 
  int Kpi = 200;// 2;
  int Kpiper = 2;    // never be 0
  int Ki = 0;
  int Kiper = 150;    // never be 0
  
  int Kp_Acc = 0;//1;
  int Kpper_Acc = 100;
  
  int stab_imp = 2;
  int curve = 0;
  
  int Kp_curve = 1;
  int Kpper_curve = 2;
  
  int I_error = 0;
  int IAcc_error = 0;
  int PI_error;
  int Acc_error;
  
  int Left_Motor_PWM = 0;
  int Right_Motor_PWM = 0;
  unsigned int l_motor_interrupts;
  unsigned int r_motor_interrupts;
  unsigned int l_temp;
  unsigned int r_temp;
  
  // servo  
  Servo servo38;
  int pos = 90;
  int def_pos = 95;
  bool Go_flag = 0;
  bool start_search_flag = 1;
  
  // turining
  int  pos_L = 0;
  int  pos_L_temp = 1;
  int  pos_L_rot = 65;          // how much impuls
  unsigned int rot_counter = 0;
  
  bool rotate_complete = false;
  
  // lighting
  unsigned int lighting_PWM;
  
  // fucntiions
  void Left_Motor_(unsigned int dir);
  void Right_Motor_(unsigned int dir);
  void Command_S(unsigned int inByte);
  
  void setup() {
    wdt_enable(WDTO_4S);
  
    LED_Init();
    Encoder_Init();
    Motor_Init();
    Lamp_Init();
    attachInterrupt(5, LA_interrupt, CHANGE);  // Pin 18
    attachInterrupt(4, RA_interrupt, CHANGE);  // Pin 19
  
    Serial.begin (9600);
    Serial3.begin(9600);
  
    servo38.attach(38);
  
    pos = def_pos;
    servo38.write(pos);
  
    noInterrupts();           // disable all interrupts
    // sampling encoders interrupts
    // PID control
    timer4_Init(64);         // timer4(prescaler); 8, 64, 256, 1024
    timer4_counter = 53035;  // 20 Hz
  
    // 64 40535 10  Hz 
    // 64 53035 20  Hz
    // 64 63035 100 Hz
    // sampling accalerometer
    interrupts();            // enable all interrupts  
  }
  //////////////////////////Loop//////////////////////////
  void loop() {
    if(Serial3.available()) {
      unsigned int inByte = Serial3.read();
      Command_S(inByte);
    }    
    wdt_reset(); 
  }////////////////////////Loop End///////////////////////
  
  bool LA;
  bool LB;
  ////////////////////////Interrupts/////////////////////
  //Input Change Noticed
  void LA_interrupt() {
    LA = digitalRead(Enc_LA);
    LB = digitalRead(Enc_LB);
  
    /* if(LA)
     if(LB) L_Motor_dir = FOWARD;
     else   L_Motor_dir = BACKWARD;
     else
     if(LB) L_Motor_dir = BACKWARD; 
     else   L_Motor_dir = FOWARD;
     */
  
    if(LA)
      if(LB) pos_L += 1;
      else   pos_L -= 1;
    else
      if(LB) pos_L -= 1; 
      else   pos_L += 1;
    l_enc += 1;
  
    //  TCNT3 = timer3_counter;      // moving
  }
  
  bool RA;
  bool RB;
  void RA_interrupt() {
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
    //   TCNT3 = timer3_counter;    // moving
  }

  ISR(TIMER4_OVF_vect) {  // L_Motor timer
  
    TCNT4 = timer4_counter;   // preload timer
    if(moving_flag) {
      digitalWrite(LED42, HIGH);
      l_motor_interrupts = l_enc;
      r_motor_interrupts = r_enc;
  
  
  
      PI_error = ((int)l_enc  - (int)r_enc);
      I_error += ((int)l_enc  - (int)r_enc);
      //  II_error += I_error;
  
      Left_Motor_PWM  += Kp * (stab_imp - l_enc - Kpi * PI_error / Kiper) + curve;//*/ - Ki * I_error / Kiper) / Kpper;
      Right_Motor_PWM += Kp * (stab_imp - r_enc + Kpi * PI_error / Kiper) - curve;//*/ + Ki * I_error / Kiper) / Kpper;
  
      //      Left_Motor_PWM = 200;
      //      Right_Motor_PWM = 200;
  
      if(Left_Motor_PWM  > 255) Left_Motor_PWM = 255;
      if(Right_Motor_PWM > 255) Right_Motor_PWM = 255;
      if(Left_Motor_PWM  < 0)  Left_Motor_PWM = 0;
      if(Right_Motor_PWM < 0)  Right_Motor_PWM = 0;
  
      analogWrite(Left_Motor_PWM_Pin, Left_Motor_PWM);
      analogWrite(Right_Motor_PWM_Pin, Right_Motor_PWM);
  
  
    }
    l_enc = 0;
    r_enc = 0;
    timer4_flag = 1;
  }
  /*
                          void serialEvent3() {
   inByte = Serial3.read();
   Command_S();
   }
   */
  ////////////////////////Interrupts/////////////////////
  //////////////////// Other_unctions///////////////////
  void Right_Motor_(unsigned int dir) {
    switch(dir) {
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
  
  void Left_Motor_(unsigned int dir) {
    switch(dir) {
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
  
  void Command_S(unsigned int inByte) {
    if(moving_flag == 0) {     // robot is not moving
      Left_Motor_PWM  = 0;
      Right_Motor_PWM = 0;
      curve = 0;
      stab_imp = 5;
    }
    if(Serial3.available() && inByte != 'R') {
      start_search_flag = true;
      rot_counter = 0;
      digitalWrite(LED52,!digitalRead(LED52));
    }
    if(inByte == 'W') {        // go foward
      Left_Motor_(FOWARD);
      Right_Motor_(FOWARD);
      moving_flag = 1;
      curve = 0;
      stab_imp = 5;  
      //      Robot(GO,FOWARD,STRAIGHT,5);
    }
    else if(inByte == 'S') {  // go backward
      Left_Motor_(BACKWARD);
      Right_Motor_(BACKWARD);
      moving_flag = 1;
      curve = 0;
      stab_imp = 5;
    }
    else if(inByte == 'A') {  // turn left
      Left_Motor_(BACKWARD);
      Right_Motor_(FOWARD);
      moving_flag = 1;
      curve = 0;
      stab_imp = 5;
    }
    else if(inByte == 'D') {  // turn right
      Left_Motor_(FOWARD);
      Right_Motor_(BACKWARD);
      moving_flag = 1;
      curve = 0;
      stab_imp = 5;
    }
    else if(inByte == 'H') {  // stop moving
      Left_Motor_ (STOP);
      Right_Motor_(STOP);
  
      moving_flag = 0;
  
      if(l_enc !=0 && r_enc !=0) {  
        Left_Motor_PWM = 0;
        Right_Motor_PWM = 0;
        IAcc_error = 0;
        I_error  = 0;
      } 
    }
    else if(inByte == 'h') {  // stop moving and than correct position
      if(abs(pos - def_pos) <= 5) {
        LEDS(BLINK);
        moving_flag = 0;
      }
      else if(pos - def_pos > 5) {       // robot have to turn right    
        Left_Motor_(FOWARD);       // 'D'
        Right_Motor_(BACKWARD);
        moving_flag = 1;
        stab_imp = 2;
        curve = 0;
  
        if(pos > 100)  delay(30);  // faster
        else           delay(10);
      }
      else if(pos - def_pos < -5) { // robot have to turn left    
        Left_Motor_(BACKWARD);     // 'A'
        Right_Motor_(FOWARD);
        moving_flag = 1;
  
        stab_imp = 2;
        curve = 0;
        if(pos < 80)  delay(30);  // faster
        else          delay(10);  
      }   
      Left_Motor_ (STOP);
      Right_Motor_(STOP);
      //    moving_flag = 0;
    }
  
    else if(inByte == 'e') {  // servo fast turn right
      pos -= 2;              // fast right turn
      if(pos < 0) pos = 0;
      servo38.write(pos);
  
      Left_Motor_PWM  = 100;
      Right_Motor_PWM = 100;
      /*      moving_flag = 0;
       Left_Motor_ (STOP);
       Right_Motor_(STOP);  
       */  }
    else if(inByte == 'd') {  // servo turn right
      pos -= 1;              // right turn
      if(pos < 0) pos = 0;
      servo38.write(pos);
  
      Left_Motor_PWM  = 100;
      Right_Motor_PWM = 100;
  
      /*      moving_flag = 0;
       Left_Motor_ (STOP);
       Right_Motor_(STOP);  
       */  }
    else if(inByte == 'q') { // servo fast turn left
      pos += 2;             // fast left turn
      if(pos > 180) pos = 180;
      servo38.write(pos);
      //     moving_flag = 0;
  
      Left_Motor_PWM  = 100;
      Right_Motor_PWM = 100;
      //      Left_Motor_ (STOP);
      //      Right_Motor_(STOP);  
    }
    else if(inByte == 'a') { // servo turn left
      pos += 1;             // left turn
      if(pos > 180) pos = 180;
      servo38.write(pos);
  
      Left_Motor_PWM  = 100;
      Right_Motor_PWM = 100;
    }
    else if(inByte == 'o') {        // ball found - positing robot << >>
      if(pos - def_pos > 1) {       // robot have to turn right    
        Left_Motor_(FOWARD);       // 'D'
        Right_Motor_(BACKWARD);
        moving_flag = 1;
        stab_imp = 2;
        curve = 0;
  
        if(pos > 100)  delay(30);  // faster
        else           delay(10);
      }
      else if(pos - def_pos < -1) { // robot have to turn left    
        Left_Motor_(BACKWARD);     // 'A'
        Right_Motor_(FOWARD);
        moving_flag = 1;
  
        stab_imp = 2;
        curve = 0;
        if(pos < 80)  delay(30);  // faster
        else          delay(10);  
      }   
      Left_Motor_ (STOP);
      Right_Motor_(STOP);
      moving_flag = 0;
    }
    else if(inByte == '1' || inByte == '2' || inByte == '3') {  // ball found
      if(def_pos - 1 <= pos && pos <= def_pos + 1) { // robot is in ball direction, goind foward
        if      (inByte == '1') stab_imp = 1;       // speed_1
        else if (inByte == '2') stab_imp = 2;       // speed_2
        else if (inByte == '3') stab_imp = 3;       // speed_3
  
          Left_Motor_(FOWARD);
        Right_Motor_(FOWARD);
        moving_flag = 1;
        curve = 0;
      }
      else if(pos - def_pos > 1) {                   // robot have to turn right    
        Left_Motor_(FOWARD);  // 'D'
        Right_Motor_(BACKWARD);
        curve = 0;
        moving_flag = 1;
        stab_imp = 2;
  
        if(pos > 100)  delay(60);
        else  delay(20);
        Left_Motor_ (STOP);
        Right_Motor_(STOP);
      }
      else if(pos - def_pos < -1) {                 // robot have to turn left    
        Left_Motor_(BACKWARD);  // 'A'
        Right_Motor_(FOWARD);
        curve = 0;
        moving_flag = 1;
        stab_imp = 2;
  
        if(pos < 80)  delay(60);
        else  delay(20);  
        Left_Motor_ (STOP);
        Right_Motor_(STOP);  
      }   
    }
    else if(inByte == 'O') {                        // curve
      Left_Motor_(FOWARD);
      Right_Motor_(FOWARD);
  
      curve = Kp_curve * (pos - def_pos) / Kpper_curve;
      stab_imp = 2;
      moving_flag = 1;
  
      if(pos - def_pos > 45) {
        Left_Motor_(FOWARD);  // 'D'
        Right_Motor_(STOP);
      }
      if(pos - def_pos < -45) {
        Left_Motor_(STOP);  // 'D'BACKWARD
        Right_Motor_(FOWARD);
      }
    }
    else if(inByte == 'R') {  // searching for the ball
      //if(timer3_flag) {
        pos = def_pos;
        servo38.write(def_pos);
        Left_Motor_(BACKWARD);  // 'A'
        Right_Motor_(FOWARD);
        curve = 0;
        moving_flag = 1;
        stab_imp = 2;
  
        delay(45);  
        Left_Motor_ (STOP);
        Right_Motor_(STOP);  
    //  }
    }
    else if(inByte == 0) analogWrite(Lamp_Pin, 0); 
    else if(inByte == 1) analogWrite(Lamp_Pin, 25); 
    else if(inByte == 2) analogWrite(Lamp_Pin, 255 - 7*25); 
    else if(inByte == 3) analogWrite(Lamp_Pin, 255 - 6*25); 
    else if(inByte == 4) analogWrite(Lamp_Pin, 255 - 5*25); 
    else if(inByte == 5) analogWrite(Lamp_Pin, 255 - 4*25); 
    else if(inByte == 6) analogWrite(Lamp_Pin, 255 - 3*25); 
    else if(inByte == 7) analogWrite(Lamp_Pin, 255 - 2*25); 
    else if(inByte == 8) analogWrite(Lamp_Pin, 255 - 1*25); 
    else if(inByte == 9) analogWrite(Lamp_Pin, 255); 
  
//    analogWrite(Lamp_Pin, 255);
  }
  
  
  

