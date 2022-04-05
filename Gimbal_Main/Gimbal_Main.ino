#include "MPU6050.h"
#include "Complementary_Filter.h"
#include "Gimbal_Controls.h"
#include <math.h>

/*
 * This implementation must have the following connections to the arduino board (Mega):
 * [add connections]
 * 
 * Also, this program uses micros() instead of millis().
 * It will behave normally for 1-2 ms in an interrupt
 * before behaving eratically. It will overflow after 71 minutes, however. 
 * 
 * Look Into RTOSS for timing
 * 
 * Consider using an analog IMU since I2C is slow; there is also a method to increase the speed of I2C
 * 
 * On these motors, spinning clockwise increases the count on the interrupt 
 * 
 * What to do next:
 * 
 * 1) Implement PID control scheme
 * 
 * 2) Implement Kalman Filtering as proposed in "A correction in feedback loop applied to two-axis gimbal stabilization"
 * 
 * 3) Extend functionality to three motors (remember to change "count" and the interrupts to work with more than one motor/encoder)
 * 
 * Roll Motor Connections: ENC_A = 2, ENC_B = 3, eena = 4 (H drive Enable), iin1 = 32, iin2 = 34
 * 
 */

// Global Variables for the IMU
float accel[3] = {0,0,0};             //IMU Data- {x,y,z} respectively
float gyro[3] = {0,0,0};              //IMU Data- {x,y,z} respectively
float pitch = 0;                      //x axis position of IMU
float roll = 0;                       //y axis position of IMU
float yaw = 0;                        //z axis position of IMU
float desired_pitch = 0;              //desired x axis position of IMU
float desired_roll = 0;               //desired y axis position of IMU
float desired_yaw = 0;                //desired z axis position of IMU
MPU6050 imu = MPU6050(20, 21);        //May not be neccessary to declare the pin numbers

// Global Variables for Encoders
const byte ENC_A = 2;          //A pin -> the interrupt pin 0 (pin 2 on MEGA)
const byte ENC_B = 3;          //B pin -> the digital pin 3 (Also an Interrupt on MEGA)
byte encoder0PinALast;
volatile float count;                   //the number of the pulses recieved from the encoder

// Global Variables for Timing (State Machine)
unsigned long current;                
unsigned long prev;
double Ts = .005;                     //Ts is in seconds; time between every new state (for cascade control, we are going to use this to control the velocity loop)

//Global Variables for Motors
Motor roll_Motor = Motor(32,34,4,12,0.8,8384,4,1/8.69173967493,Ts); //for these motors, deadband is 0.5 Volts, and the BJT H-bridges drop 1.4 Volts, I think MOSFET H-Bridges drop 0.4 Volts. With the current encoder function, one rotation is 8384
// For dirty derivative: 8.69173967493 is the max rads/second the motor can go. this is the crossover frequency (i think) and thus the bandwidth, and thus is 1/sigma. so sigma is 1/8.69173967493

//Setup and initialize all neccessary modules/variables
void setup() {
  Serial.begin(115200);
  roll_Motor.motor_setup();
  //roll_Motor.change_control_constants(5,.16,.32);
  roll_Motor.change_control_constants(7.20,1,.1);
  imu.initialize();
  EncoderInit();                      //Initialize the module for the encoder
  current = micros();                 //milliseconds since arduino start
  prev = current;

  // Following two lines are for velocity tuning; run the motor in case you want to check 0 velocity in loop
  //roll_Motor.Tune_Velocity_Loop(3.1415, 5, 0);
  //delay(2000);
}

void loop() {
  // A state machine controls timing so that the delay function need not be used
  current = micros();
  unsigned long vTs = current - prev; //this variable exists to see how far the actual time step differs from Ts
  if((current - prev) >= (Ts*1000000.0))
  {
    //Get all accel and gyroscope values from the IMU and update the global variables
    imu.update(); //this function is very slow
    imu.get_all_accel(accel);
    imu.get_all_ang_vel(gyro);

//    Serial.print("accel:");
//    Serial.print(accel[1]);
//    Serial.print(" | ");
//
//    Serial.print("gyro:");
//    Serial.print(gyro[1]);
//    Serial.print("\n");
    
    //Use a complementary filter on IMU data to determine the pitch, roll, and yaw
    Complementary(accel, gyro, &roll, &pitch, &yaw, Ts);

    //Update encoder counts and calculate velocity using the encoder only (does not calculate for the IMU)
    roll_Motor.previous_count = roll_Motor.current_count;
    roll_Motor.current_count = count;
    roll_Motor.calc_velocity_dirty(); //updates current velocity (internal to motor object)

    //Serial.print("vTs ");
    //Serial.print(vTs);
    //Serial.print("\n");

    //Serial.print("Velocity ");
    //Serial.print(roll_Motor.current_velocity);
    //Serial.print("\n");

    //Pass the velocity calculated from the encoders and the gyroscope data to the Kalman Filter (maybe work on this in the future)
    //velocity = Kalman(gyro[1], 0, roll_Motor.current_velocity, 0);
    
    //The Current Position will be passed to a PID function, and the motor is driven in this function
    //roll_Motor.PIDControl(roll, desired_roll);
    
    roll_Motor.CascadeControl(roll, desired_roll, gyro[0]);
    //roll_Motor.Tune_Velocity_Loop(3.1415, Ts, 10, 0);
    
    prev = current;
  }
  Serial.print(" Roll:");
  Serial.println(roll);
  Serial.println("\n");
  //Serial.println("Counts: ");
  //Serial.println(count); 
  //Serial.println("\n");
}

// Below are the functions pertaining to the Encoder (using interruptions)
void EncoderInit()
{
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), pulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), pulseB, CHANGE);
}

void pulseA(){
  int valA = digitalRead(ENC_A);
  int valB = digitalRead(ENC_B);

  if (valA == HIGH) { // A Rise
    if (valB == LOW) {
      count++;  // CW
    }
    else {
      count--;  // CCW
    }
  }
  else { // A fall
    if (valB == HIGH) {
      count ++;  // CW
    }
    else {
      count --;  //CCW
    }
  }
}

void pulseB(){
  int valA = digitalRead(ENC_A);
  int valB = digitalRead(ENC_B);

  if (valB == HIGH) { // B rise
    if (valA == HIGH) {
      count ++; // CW
    }
    else {
      count --; // CCW
    }
  }
  else { //B fall
    if (valA == LOW) {
      count ++; // CW
    }
    else {
      count --; // CCW
    }
  }
}
