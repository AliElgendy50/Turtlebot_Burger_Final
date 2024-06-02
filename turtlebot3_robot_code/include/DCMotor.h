// The #pragma once directive ensures that the header file is included only once in a compilation unit.
#pragma once
#include <Arduino.h>

// Define the pin connections for two DC motors
#define ENA 33  // PWM pin for motor A
#define IN1 25  // Control pin 1 for motor A
#define IN2 26  // Control pin 2 for motor A


#define ENB  13 // PWM pin for motor B
#define IN3  14 // Control pin 1 for motor B
#define IN4  12 // Control pin 2 for motor B

// Define the maximum PWM value and the PWM data type based on whether 16-bit PWM is used

#ifdef _16BIT_PWM_
  #define MAX_VALUE 0xFFFF
  #define INT_PWM   uint16_t
#else
  #define MAX_VALUE 0xFF
  #define INT_PWM   uint8_t
#endif

// Define the DCMotor class
class DCMotor {
public:

  // Declare the constructors and methods for controlling the motor
  DCMotor();
  DCMotor(int,int,int);
  void CW(INT_PWM);
  void CCW(INT_PWM);
  void Stop();
private:  
  // Declare the control pins for the motor
  int INL; 
  int INH;
  int EN;

  // Declare the methods for initializing the pins and protecting the output
  void initPins();
  INT_PWM protectOutput(INT_PWM);
};

// Define the default constructor
DCMotor::DCMotor()
  : DCMotor::DCMotor(IN1, IN2, EN) {
  }

// Define the constructor that takes the control pins as arguments
DCMotor::DCMotor(int INL, int INH, int EN) {
  this->INL = INL;
  this->INH = INH;
  this->EN = EN;
  DCMotor::initPins();
}


// Define the method for initializing the pins
void DCMotor::initPins() {

  pinMode(this->INL, OUTPUT);
  pinMode(this->INH, OUTPUT);
  pinMode(this->EN, OUTPUT);
  DCMotor::Stop();
}

// Define the method for stopping the motor
void DCMotor::Stop() {
  // Motor no gira
  digitalWrite (INL, LOW); 
  digitalWrite (INH, LOW);
  analogWrite (EN, LOW);

}

// Define the method for turning the motor clockwise
void DCMotor::CW(INT_PWM val) {
  
  // Motor turns forward or CW
  digitalWrite(INL, LOW);
  digitalWrite (INH, HIGH);
  analogWrite(EN, protectOutput(val));
}


// Define the method for turning the motor counterclockwise
void DCMotor::CCW(INT_PWM val) {

  // Motor turns in the inverse direction or CCW
  digitalWrite(INH, LOW);
  digitalWrite (INL, HIGH);
  analogWrite(EN, protectOutput(val));
}

// Define the method for protecting the output
INT_PWM DCMotor::protectOutput(INT_PWM val) {

  // Limit the PWM value to the maximum value
  val > MAX_VALUE? val = MAX_VALUE : val;

  return val;
}