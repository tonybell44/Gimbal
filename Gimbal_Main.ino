#include "MPU6050.h"
#include "Complementary_Filter.h"
#include "Gimbal_Controls.h"
#include <math.h>

/*
 * This implementation must have the following connections to the arduino board (Mega)
 * 
 * Also, this program uses micros() instead of millis() because the former does
 * not rely on interupts to work. It will behave normally for 1-2 ms in an interrupt
 * before behaving eratically. It willoverflow after 71 minutes, however. 
 * 
 * Look Into RTOSS for timing 
 * 
 * Look into falling edge for encoder to increase resolution
 */

// Global Variables for the IMU
float accel[3] = {0,0,0};             //IMU Data- x,y,z respectively
float gyro[3] = {0,0,0};              //IMU Data- x,y,z respectively
float pitch = 0;                      //x axis position
float roll = 0;                       //y axis position
float yaw = 0;                        //z axis position
MPU6050 imu = MPU6050(20, 21);

// Global Variables for Encoder
const byte encoder0pinA = 2;          //A pin -> the interrupt pin 0 (pin 2 on MEGA)
const byte encoder0pinB = 4;          //B pin -> the digital pin 4
byte encoder0PinALast;
volatile int count;                   //the number of the pulses
int current_count;
int previous_count;
int cpr = 2096;                       //the encoder we are using outputs 2096 counts per revolution

// Global Variables for Timing (State Machine)
unsigned long current;
unsigned long prev;
double Ts = 0.01;                     //Ts is in seconds

//Global Variables for Motor
Motor roll_Motor = Motor(32,36,7,12,0);

void setup() {
  Serial.begin(9600);
  //pinMode (13, OUTPUT);
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
    //Get all accel and gyroscope values and set them in the global variables
    imu.get_all_accel(accel);
    imu.get_all_ang_vel(gyro);

    //Use a complementary filter on the pitch, roll, and yaw
    Complementary(accel, gyro, &pitch, &roll, &yaw, Ts);

    //Velocity of the motors are calculated using the encoder
    //current_count = count;
    //roll_Motor.calc_velocity(cpr, previous_count, current_count, Ts);
    //previous_count = current_count;
    
    //The Current Position will be passed to a PID function, and the motor is driven in this function
    roll_Motor.PID(roll, 0);
    
    prev = current;
  }
  
  
  Serial.print(" Roll:");
  Serial.println(roll);
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
