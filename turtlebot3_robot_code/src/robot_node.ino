#include "DifferentialDriveRobot.h"
#include <BluetoothSerial.h>
#include "Encoder.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

#define Servo_pin 27
Servo myservo;
int angle = 0;
char flag=0;

#define TRIGGER_PIN1 2
#define ECHO_PIN1 35
Ultrasonic ultrasonic1(TRIGGER_PIN1, ECHO_PIN1);

const float wheelRadius = 0.04;
const float wheelBase = 0.13;

unsigned long previousMillis = 0; 
const long interval = 1000; 
const long interval2 = 50;
unsigned long currentMillis = 0;

BluetoothSerial SerialBT;

class BluetoothHardware {
public:
  BluetoothHardware() : connected(false) {}

  void init() {
    SerialBT.begin("ESP32_BT");
  }

  int read() {
    if (SerialBT.available())
      return SerialBT.read();
    else
      return -1;
  }

  void write(uint8_t* data, int length) {
    for(int i=0; i<length; i++)
      SerialBT.write(data[i]);
  }

  unsigned long time() {
    return millis();
  }

  bool connected;
};

Adafruit_MPU6050 mpu;
ros::NodeHandle_<BluetoothHardware> nh;

DifferentialDriveRobot *my_robot;

geometry_msgs::Twist(lol_msg);
ros::Publisher lol_pub("cmd_vel", &lol_msg);

void robot_callback(const geometry_msgs::Twist& msg) {
  my_robot->move(msg.linear.x, msg.angular.z);
  double lin=msg.linear.x;
  double ang=msg.angular.z;
  // Calculate the right and left wheel velocities
	double u_r = (lin + ang * 0.13/2.0) / 0.04;
	double u_l = (lin - ang * 0.13/2.0) / 0.04;
  lol_msg.linear.y=u_r;
  lol_msg.angular.y=u_l;
  lol_msg.linear.x =map(static_cast<INT_PWM>(-u_r), 0, 250, 0, 255);
  lol_msg.angular.z = map(static_cast<INT_PWM>(-u_l), 0, 250, 0, 255);
  lol_pub.publish(&lol_msg);

}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &robot_callback);
geometry_msgs::Twist cmd_msg;
ros::Publisher cmd_pub("cmd_vel", &cmd_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

std_msgs::String odom_str_msg;
ros::Publisher raw_odom_pub("raw_odom", &odom_str_msg);

std_msgs::Float64 distance_msg1;
ros::Publisher distance_pub1("sonar", &distance_msg1);

void handleEncoder_A() {
  int currentState_A = digitalRead(ENCODER_LEFT_MOTOR_A);
  int currentState_B = digitalRead(ENCODER_LEFT_MOTOR_B);
  if ((lastEncoderState_A == LOW) && (currentState_A == HIGH)) {
    if (currentState_B == LOW) {
      encoderCount_A++;
    } else {
      encoderCount_A--;
    }
  }
  lastEncoderState_A = currentState_A;
}

void handleEncoder_B() {
  int currentState_A = digitalRead(ENCODER_RIGHT_MOTOR_A);
  int currentState_B = digitalRead(ENCODER_RIGHT_MOTOR_B);
  if ((lastEncoderState_B == LOW) && (currentState_B == HIGH)) {
    if (currentState_A == LOW) {
      encoderCount_B++;
    } else {
      encoderCount_B--;
    }
  }
  lastEncoderState_B = currentState_B;
}


void setup() {
  SerialBT.begin("ESP32_BT");
  myservo.attach(Servo_pin);
  myservo.write(0);

  my_robot = new DifferentialDriveRobot(
    new DCMotor(IN1, IN2, ENA),
    new DCMotor(IN3, IN4, ENB)
  );

  my_robot->updateParameters(0.2, 1);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(ENCODER_LEFT_MOTOR_A, INPUT);
  pinMode(ENCODER_RIGHT_MOTOR_A, INPUT);
  pinMode(ENCODER_LEFT_MOTOR_B, INPUT);
  pinMode(ENCODER_RIGHT_MOTOR_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_MOTOR_A), handleEncoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_MOTOR_B), handleEncoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_MOTOR_A), handleEncoder_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_MOTOR_B), handleEncoder_B, CHANGE);

  nh.initNode();
  nh.advertise(cmd_pub);
  nh.advertise(imu_pub);
  nh.advertise(distance_pub1);
  nh.advertise(raw_odom_pub);
  nh.advertise(lol_pub);
  nh.subscribe(sub);
}

void loop() {
  currentMillis = millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (currentMillis - previousMillis >= interval2) {
    float distance = ultrasonic1.read();
    String distanceString = String(distance, 10);  
    distance_msg1.data = distanceString.toFloat();
    //distance_msg1.data = distance;
    distance_pub1.publish(&distance_msg1);
    if (flag == 0) {
      angle += 1;
      myservo.write(angle);
      if (angle >= 180) {
        flag = 1;
      }
    } else if (flag == 1) {
      angle -= 1;
      myservo.write(angle);
      if (angle == 0) {
        flag = 0;
      }
    }
  }

  if (currentMillis - previousMillis >= interval) {
    float encoder_angular_A = (encoderCount_A * 2 * PI) / encoder_cpr;
    float encoder_angular_B = (encoderCount_B * 2 * PI) / encoder_cpr;
    float encoder_linearVelocity = (wheelRadius / 2) * (encoder_angular_A + encoder_angular_B);

    String odom_data = "ENC_A:" + String(encoder_angular_A, 4) + 
                       ",ENC_B:" + String(encoder_angular_B, 4) + 
                       ",VEL:" + String(encoder_linearVelocity, 4) + 
                       ",GYRO_Z:" + String(g.gyro.z, 4);
    odom_str_msg.data = odom_data.c_str();
    raw_odom_pub.publish(&odom_str_msg);

    encoderCount_A = 0;
    encoderCount_B = 0;

    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;
    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    
    imu_pub.publish(&imu_msg);

    previousMillis = currentMillis;
  }

  nh.spinOnce(); 
  delay(1);
}
