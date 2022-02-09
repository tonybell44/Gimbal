#ifndef GIMBAL_CONTROLS_H
#define GIMBAL_CONTROLS_H
#include "Gimbal_Controls.h"
#endif

#ifndef ARDUINO_H
#define ARDUINO_H
#include <Arduino.h>
#endif

#include <math.h>

//Default Constructor
Motor::Motor(int iin1, int iin2, int eena, float ddead_up, float ddead_low, int ccpr, int proportion)
{
	in1 = iin1;
	in2 = iin2;
	ena = eena;
	dead_up = ddead_up;
	dead_low = ddead_low;
	current_count = 0;
	previous_count = 0;
	cascade_call_count = 0;
	pos_vel_proportion = proportion;
	cpr = ccpr;
	kp = 2.5; //Default value
	kv = 1.8; //Default value

	velocity_command = 0;  //if it doesnt work, comment these two out 
	position_error = 0;
}

void Motor::motor_setup(){
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(ena, OUTPUT);
	return;
}

void Motor::change_control_constants(float kpos, float kvel){
	kp = kpos;
	kv = kvel;
	return;
}

void Motor::calc_velocity(double Ts){
	current_velocity = (1*3.141592*((current_count - previous_count)/cpr))/Ts;
	return;
}

float Motor::deadband_compensation(float lin_scale)
{
	//Serial.print(" lin scale:");
	//Serial.print(lin_scale);

	float out_voltage;
	
	if (lin_scale > 0) {
		out_voltage = ((lin_scale/10) * (dead_up - dead_low)) + dead_low;
		if (out_voltage > 12) {
			out_voltage = 12;
		}
	}
	else if (lin_scale < 0) {
		out_voltage = (((-1*lin_scale)/10) * (dead_low - dead_up)) - dead_low; //note hard code lol
		if (out_voltage < -12) {
			out_voltage = -12;
		}
	}
	else {return 0;}
	
	//Serial.print(" Out_volts: ");
	//Serial.print(out_voltage);
	//Serial.print("\n");

	return out_voltage;
}

void Motor::drive_motor(float volt)
{
	// we account for deadband by limiting the voltage between dead_low and dead_up
	float speed = floor((abs(volt)/12) * 255);
	//Serial.print(" Speed:");
	//Serial.print(speed);
	analogWrite(ena, speed);
		
	// Note, for our purpose, when GND of motor is plugged into Out1, volt<0 causes counter clockwise rotation, [state something about the orientation of the IMU]
	if (volt <= 0){
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
	}
	else {
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
	}
	return;
}

void Motor::CascadeControl(float current_position, float desired_position){
	//current and prev will be in microseconds
	
	// look into velocity feedforward
	// Position Loop
	if ((cascade_call_count % pos_vel_proportion) == 0){
		position_error = desired_position - current_position; // error will range from -pi to pi 
		velocity_command = position_error * kp; 
		//Serial.print("      Velocity Command: ");
		//Serial.print(velocity_command);
		//Serial.print("Called\n");
	}
	
	//Serial.print("Position error: ");
	//Serial.print(position_error);
	//Serial.print("      Velocity Command outside: ");
	//Serial.print(velocity_command);
	//Serial.print("      Kp: ");
	//Serial.print(kp);
	//Serial.print("\n");

	//Velocity Loop	
	float velocity_error = velocity_command - current_velocity;
	float output = velocity_error * kv;
	drive_motor(deadband_compensation(output));
	cascade_call_count = cascade_call_count + 1;

	//Serial.print("Call Count: ");
	//Serial.print(cascade_call_count);
	//Serial.print("Mod:");
	//Serial.print(cascade_call_count % pos_vel_proportion);
	//Serial.print("Logic:");
	//Serial.print((cascade_call_count % pos_vel_proportion) == 0);
	//Serial.print(" Output:");
	//Serial.print(output);
	//Serial.print("\n");

	return;
}

void Motor::Tune_Velocity_Loop(float kvel){
	float velocity_error = 0 - current_velocity;
	float output = velocity_error * kvel;
	drive_motor(deadband_compensation(output));
	return;
}

void Motor::Tune_Position_Loop(float kpos, float kvel, float current_position, float desired_position){
	if ((cascade_call_count % pos_vel_proportion) == 0){
		position_error = desired_position - current_position; // error will range from -pi to pi 
		velocity_command = position_error * kpos; 
	}
	
	float velocity_error = velocity_command - current_velocity;
	float output = velocity_error * kvel;
	drive_motor(deadband_compensation(output));
	cascade_call_count = cascade_call_count + 1;
	return;
}
		    