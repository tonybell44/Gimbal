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
Motor::Motor(int iin1, int iin2, int eena, float ddead_up, float ddead_low, int ccpr)
{
	in1 = iin1;
	in2 = iin2;
	ena = eena;
	dead_up = ddead_up;
	dead_low = ddead_low;
	current_count = 0;
	previous_count = 0;
	cpr = ccpr;
}

void Motor::motor_setup(){
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(ena, OUTPUT);
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
		out_voltage = (((lin_scale)/10) * (dead_low - dead_up)) - dead_low; //note hard code lol
		out_voltage = (out_voltage * -1);
		if (out_voltage < -12) {
			out_voltage = -12;
		}
	}
	else {return 0;}
	
	
	//Serial.print(" Out_volts:");
	//Serial.print(out_voltage);
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
		} else
			digitalWrite(in1, LOW);
			digitalWrite(in2, HIGH);
		}
	
}

float Motor::PID(float current_position, float desired_position){
	
	// Proportional (look into velocity feedforward)
	// Position Loop
	float kp = 3;
	float position_error = desired_position - current_position; // error will range from -pi to pi 
	float velocity_command = position_error * kp; 
	// Velocity Loop
	float kv = 1;
	float velocity_error = velocity_command - current_velocity;
	float output = velocity_error * kv;
	drive_motor(deadband_compensation(output));
	//Serial.print(" PID Output:");
	//Serial.print(output);
	
}

void Motor::calc_velocity(double Ts){
	
	current_velocity = (2*3.141592*((current_count - previous_count)/cpr))/Ts;

}



		    