// Encoder pins
#define ENCODER_LEFT_MOTOR_A 18         //YELLOW A channel for encoder of left motor  
#define ENCODER_LEFT_MOTOR_B 5        //GREEN B channel for encoder of left motor

#define ENCODER_RIGHT_MOTOR_A 23        //YELLOW A channel for encoder of right motor
#define ENCODER_RIGHT_MOTOR_B 4        //GREEN B channel for encoder of right motor

#define  encoder_cpr  500             //Encoder ticks or counts per rotation

#define PI 3.14159265358979323846

volatile double encoderCount_A=0;
volatile int lastEncoderState_A=0;

volatile double encoderCount_B=0;
volatile int lastEncoderState_B=0;

float encoder_angular_A=0;
float encoder_angular_B=0;

float encoder_linearVelocity=0;
float encoder_angularVelocity=0;


volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

