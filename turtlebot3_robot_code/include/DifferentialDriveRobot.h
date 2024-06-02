// The #pragma once directive ensures that the header file is included only once in a compilation unit.
#pragma once


/**********************************Include the necessary libraries***************************************/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DCMotor.h"


/**********************************Define the DifferentialDriveRobot class***************************************/
class DifferentialDriveRobot
{
	// Declare the left and right motors
	DCMotor *motor_left;
	DCMotor *motor_right;

	// Declare the wheel radius and distance between wheels
	double wheel_radius;
	double wheel_distance;

	// Declare the bounds for the right and left motors
	double bound_right;
	double bound_left;



	// Declare the maximum speed and turn rate
	float max_speed;
	float max_turn;
public:

	// Declare the constructors
	DifferentialDriveRobot();
	DifferentialDriveRobot(DCMotor*, DCMotor*);
	DifferentialDriveRobot(DCMotor*, DCMotor*, double, double);

	// Declare the move and updateParameters methods
	void move(double,double);
	void updateParameters(float, float);
};

// Define the default constructor
DifferentialDriveRobot::DifferentialDriveRobot()
	: DifferentialDriveRobot::DifferentialDriveRobot(
		new DCMotor(IN1,IN2,ENA),
		new DCMotor(IN3,IN4,ENB), 0.04, 0.13) {}

// Define the constructor that takes two motors
DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right)
	: DifferentialDriveRobot::DifferentialDriveRobot(
		motor_left, motor_right, 0.04, 0.13) {}

// Define the constructor that takes two motors, a wheel radius, and a wheel distance
DifferentialDriveRobot::DifferentialDriveRobot
	(DCMotor *motor_left, DCMotor *motor_right, double rad, double dist) {
	this->motor_left = motor_left;
	this->motor_right = motor_right;
	this->wheel_radius = rad;
	this->wheel_distance = dist;
}

// Define the move method, which moves the robot based on a linear and angular velocity

void DifferentialDriveRobot::move(double lin, double ang) {

	/**
	 * 
	 * ang < 0: moves CW (u_l > u_r)
	 *
	 * To change this, you have to change the sign of the second term
	 *
	 */

	//remove error from readings
	// Calculate the right and left wheel velocities
	double u_r = (lin + ang * this->wheel_distance/2.0) / this->wheel_radius;
	double u_l = (lin - ang * this->wheel_distance/2.0) / this->wheel_radius;

	// Move the right motor
	if (u_r > 0) {
		motor_right->CCW(
			map(static_cast<INT_PWM>(u_r), 0, this->bound_right, 0, MAX_VALUE));
	} else {

		motor_right->CW(
			map(static_cast<INT_PWM>(-u_r), 0, this->bound_right, 0, MAX_VALUE));
	}

	// Move the left motor
	if (u_l > 0) {
		
		motor_left->CW(
			map(static_cast<INT_PWM>(u_l), 0, this->bound_left, 0, MAX_VALUE));
	}
	else
	{
		motor_left->CCW(
			map(static_cast<INT_PWM>(-u_l), 0, this->bound_left, 0, MAX_VALUE));
	}	
}


// Define the updateParameters method, which updates the maximum speed and turn rate
void DifferentialDriveRobot::updateParameters(float max_speed, float max_turn) {
	
	this->max_speed = max_speed;
	this->max_turn = max_turn;
	this->bound_right =250;
	this->bound_left = 250;
		// (this->max_speed - this->max_turn * this->wheel_distance/2.0) / this->wheel_radius;

}
