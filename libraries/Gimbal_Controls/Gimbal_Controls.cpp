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
	ki = 2; //Test Value

	velocity_command = 0;  //if it doesnt work, comment these two out 
	position_error = 0;
}

void Motor::motor_setup(){
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(ena, OUTPUT);
	return;
}

void Motor::change_control_constants(float kpos, float kvel, float kint){
	kp = kpos;
	kv = kvel;
	ki = kint;
	return;
}

void Motor::calc_velocity(double Ts){
	current_velocity = ((current_count - previous_count)/cpr)/Ts;
	
	/*
	Serial.print("Previous Count: ");
    Serial.print(previous_count);
    Serial.print("\n");
	Serial.print("Current Count: ");
    Serial.print(current_count);
    Serial.print("\n");
	*/
	return;
}

// Consider changing 12 to dead_up
float Motor::deadband_compensation(float voltage_command)
{
	float out_voltage;
	
	if (voltage_command > 0) {
		out_voltage = ((voltage_command/10) * (dead_up - dead_low)) + dead_low;
		if (out_voltage > dead_up) {
			out_voltage = dead_up;
		}
	}
	else if (voltage_command < 0) {
		out_voltage = (((-1 * voltage_command)/10) * (dead_low - dead_up)) - dead_low;
		if (out_voltage < -1 * dead_up) {
			out_voltage = -1 * dead_up;
		}
	}
	else {return 0;}
	
	Serial.print(" Out_volts: ");
	Serial.print(out_voltage);
	Serial.print("\n");

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

void Motor::CascadeControl(float current_position, float desired_position, float Ts){
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
	float voltage_integral = voltage_integral + (velocity_error * Ts);
	float voltage_command = (velocity_error * kv) + (voltage_integral * ki);
	drive_motor(deadband_compensation(voltage_command));
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

// is the velocity loop really doing what it should be doing?
void Motor::Tune_Velocity_Loop(float velocity, float Ts, float kvel, float kint){
	float velocity_error = velocity - current_velocity;
	float voltage_integral = voltage_integral + (velocity_error * Ts);
	float voltage_command = (velocity_error * kvel) + (voltage_integral * kint);
	drive_motor(deadband_compensation(voltage_command));
	return;
}

void Motor::Tune_Position_Loop(float kpos, float kvel, float kint, float current_position, float desired_position, float Ts){
	if ((cascade_call_count % pos_vel_proportion) == 0){
		position_error = desired_position - current_position; // error will range from -pi to pi 
		velocity_command = position_error * kpos; 
	}

	float velocity_error = velocity_command - current_velocity;
	float voltage_integral = voltage_integral + (velocity_error * Ts);
	float voltage_command = (velocity_error * kvel) + (voltage_integral * kint);
	drive_motor(deadband_compensation(voltage_command));
	cascade_call_count = cascade_call_count + 1;
	return;
}
		    