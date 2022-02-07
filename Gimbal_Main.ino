#include "MPU6050.h"
#include "Complementary_Filter.h"
#include "Gimbal_Controls.h"
#include <math.h>

/*
 * This implementation must have the following connections to the arduino board (Mega):
 * 
 * Also, this program uses micros() instead of millis().
 * It will behave normally for 1-2 ms in an interrupt
 * before behaving eratically. It will overflow after 71 minutes, however. 
 * 
 * Look Into RTOSS for timing 
 * 
 * Look into falling edge for encoder to increase resolution. The current resolution, however,
 * has enough accuaracy to behave suffieciently. 
 * 
 * What to do next:
 * 1) Integrate encoder counts and counts per rotation into the motor class, they are currently global variables (almost done)
 * 
 * 2) Finish the Proportional Control feedback loop (test in real life)
 * 
 * 3) Work on Integral and Derivitive blocks 
 * 
 * 4) Extend functionality to three motors (remember to change "count" and the interrupts to work with more than one motor/encoder)
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
const byte encoder0pinA = 2;          //A pin -> the interrupt pin 0 (pin 2 on MEGA)
const byte encoder0pinB = 4;          //B pin -> the digital pin 4
byte encoder0PinALast;
volatile int count;                   //the number of the pulses recieved from the encoder

// Global Variables for Timing (State Machine)
unsigned long current;                
unsigned long prev;
double Ts = 0.001;                     //Ts is in seconds; time between every new state

//Global Variables for Motors
Motor roll_Motor = Motor(32,36,7,12,0,2096); 

//Setup and initialize all neccessary modules/variables
void setup() {
  Serial.begin(9600);
  roll_Motor.motor_setup();
  imu.initialize();
  EncoderInit();                      //Initialize the module for the encoder
  current = micros();                 //milliseconds since arduino start
  prev = current;
}

void loop() {
  // A state machine controls timing so that the delay function need not be used
  current = micros();
  imu.update(); 
  if((current - prev) >= (Ts*1000000.0))
  {
    //Get all accel and gyroscope values from the IMU and update the global variables
    imu.get_all_accel(accel);
    imu.get_all_ang_vel(gyro);

    //Use a complementary filter on the pitch, roll, and yaw
    Complementary(accel, gyro, &pitch, &roll, &yaw, Ts);

    //Update the motor object with current encoder inputs so it can calculate the velocity (we may want to consider letting the calc_velocity function update the counts)
    roll_Motor.current_count = count;
    roll_Motor.calc_velocity(Ts); //updates current velocity (internal to motor object)
    roll_Motor.previous_count = count; //this must be updated after velocity is calculated and updated
    
    //The Current Position will be passed to a PID function, and the motor is driven in this function
    roll_Motor.PID(roll, desired_roll);
    
    prev = current;
  }
  
  //Serial.print(" Roll:");
  //Serial.println(roll);
  //Serial.println(count); 
}

// Below are the functions pertaining to the Encoder (using interruptions)
void EncoderInit()
{
  pinMode(encoder0pinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), do_count, RISING);
}

void do_count()
{
 if (digitalRead(encoder0pinB) == HIGH){
  count++;
 }
 else{
  count--;
 }
}
