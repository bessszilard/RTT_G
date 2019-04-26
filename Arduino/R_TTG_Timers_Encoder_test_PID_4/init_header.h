/*#ifndef INIT_HEADER_H
#define INIT_HEADER_H
*/
//LEDS
#define LED42 42
#define LED44 44
#define LED46 46
#define LED48 48
#define LED50 50
#define LED52 52

#define FOWARD 1
#define BACKWARD 2
#define STOP 0

//Motor
#define Left_Motor_PWM_Pin 9
#define Left_Motor_4A 11
#define Left_Motor_3A 10

#define Right_Motor_PWM_Pin 12
#define Right_Motor_1A 54  // A0
#define Right_Motor_2A 55  // A1

//int Motor1_1A = A0;
//int Motor1_2A = A1;


extern unsigned int timer3_counter;
extern unsigned int timer4_counter;

void Encoder_Init();
void timer3_Init(unsigned int prescaler);
void timer4_Init(unsigned int prescaler);

//#endif

