  /*#ifndef INIT_HEADER_H
  #define INIT_HEADER_H*/
  
  // Pins
  // Accelerometer
  #define Acc_X_Pin 56  // A2
  #define Acc_Y_Pin 57  // A3
  #define Acc_Z_Pin 58  // A4  
  
  // Encoders
  #define Enc_LA 18
  #define Enc_LB 16
  
  #define Enc_RA 19
  #define Enc_RB 17
  
  //LEDS
  #define LED42 42
  #define LED44 44
  #define LED46 46
  #define LED48 48
  #define LED50 50
  #define LED52 52
  
  // Motor
  #define Right_Motor_PWM_Pin 9
  #define Right_Motor_2A 10
  #define Right_Motor_1A 11
  
  #define Left_Motor_PWM_Pin 12
  #define Left_Motor_3A 55  // A1
  #define Left_Motor_4A 54  // A0
  
  // constans
  #define FOWARD 1
  #define BACKWARD 2
  #define STOP 0
  
  #define OFF 0
  #define ON 1
  #define BLINK 2
  
  #define Lamp_Pin 5
  
  extern unsigned int timer3_counter;
  extern unsigned int timer4_counter;
  
  void Accelero_Init();    // also set analog voltage reference 2V56  
  void Encoder_Init();     // encoder inputs 
  void LED_Init();         // sets LED_Pins to OUTPUT
  void Motor_Init();       // sets Motor1_PinS, and Motor2_Pins as output
  void Lamp_Init();
    
  void timer3_Init(unsigned int prescaler);
  void timer4_Init(unsigned int prescaler);
  
  void LEDS(unsigned int STATUS);
  //#endif
  
