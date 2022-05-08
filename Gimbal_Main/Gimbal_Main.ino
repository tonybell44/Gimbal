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
 * Pitch Motor Connections: ENC_A = 5, ENC_B = 6, eena = 7 (H drive Enable), iin1 = 36, iin2 = 38
 * 
 * Yaw Motor Connections: ENC_A = 8, ENC_B = 9, eena = 10 (H drive Enable), iin1 = 40, iin2 = 42
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
const byte ENC_A = 2;                   //A pin -> the interrupt pin 0 (pin 2 on MEGA)
const byte ENC_B = 3;                   //B pin -> the digital pin 3 (Also an Interrupt on MEGA)

//const byte ENC_C = 5;               
//const byte ENC_D = 6; 

//const byte ENC_E = 8;               
//const byte ENC_F = 9; 

byte encoder0PinALast;
volatile float count1;                   //the number of the pulses recieved from the encoder (roll motor)
volatile float count2;                   //(pitch motor)
volatile float count3;                   //(yaw motor)

// Global Variables for Timing (State Machine)
unsigned long current;                
unsigned long prev;
double Ts = .005;                       //Ts is in seconds; time between every new state (for cascade control, we are going to use this to control the velocity loop)

//Global Variables for Motors
Motor roll_Motor = Motor(32,34,4,255,0,2448,4,1/19.05899541,Ts); //make sure to test deadband. With the current encoder function, one rotation is 2448
// For dirty derivative: 19.05899541 is the max rads/second the motor can go. this is the crossover frequency (i think) and thus the bandwidth, and thus is 1/sigma. so sigma is 1/19.05899541
// pitch_Motor = Motor(32,34,4,240,140,2448,4,1/19.05899541,Ts);
//Motor yaw_Motor = Motor(32,34,4,240,140,2448,4,1/19.05899541,Ts);

//Setup and initialize all neccessary modules/variables
void setup() {
  Serial.begin(115200);
  roll_Motor.motor_setup();
  //pitch_Motor.motor_setup();
  //yaw_Motor.motor_setup();
  roll_Motor.change_control_constants(10,.5,.1); // 8 0 0 
  //pitch_Motor.change_control_constants(7.20,1,.1);
  //yaw_Motor.change_control_constants(7.20,1,.1);
  imu.initialize();
  EncoderInit1();                      //Initialize the module for the encoders

  //EncoderInit2(); 
  //EncoderInit3(); 
  
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
//    Serial.print(gyro[0]);
//    Serial.print("\n");
    
    //Use a complementary filter on IMU data to determine the pitch, roll, and yaw
    Complementary(accel, gyro, &roll, &pitch, &yaw, Ts);

    //Update encoder counts and calculate velocity using the encoder only (does not calculate for the IMU)
    roll_Motor.previous_count = roll_Motor.current_count;
    roll_Motor.current_count = count1;
    roll_Motor.calc_velocity_dirty(); //updates current velocity (internal to motor object)

//    pitch_Motor.previous_count = pitch_Motor.current_count;
//    pitch_Motor.current_count = count2;
//    pitch_Motor.calc_velocity_dirty(); //updates current velocity (internal to motor object)
//
//    yaw_Motor.previous_count = yaw_Motor.current_count;
//    yaw_Motor.current_count = count3;
//    yaw_Motor.calc_velocity_dirty(); //updates current velocity (internal to motor object)

    //Serial.print("vTs ");
    //Serial.print(vTs);
    //Serial.print("\n");

//    Serial.print("Velocity ");
//    Serial.print(roll_Motor.current_velocity);
//    Serial.print("\n");
    
    roll_Motor.CascadeControl(-1 * roll, desired_roll, gyro[0]); //-1 * roll is a hacky way of inverting the motor (but it works!)
    //roll_Motor.Tune_Velocity_Loop(3.1415, Ts, 9.69, 5);

    //pitch_Motor.CascadeControl(pitch, desired_pitch, gyro[1]);
    //yaw_Motor.CascadeControl(yaw, desired_yaw, gyro[2]);
    
    prev = current;
  }
  //Serial.print("Roll:");
  //Serial.print(roll);
//  Serial.print("|");
//  Serial.print("Pitch:");
//  Serial.print(pitch);
//  Serial.print("|");
//  Serial.print("Yaw:");
//  Serial.print(yaw);
//  Serial.print("\n");
  //Serial.println("Counts: ");
  //Serial.println(count1); 
  //Serial.println("\n");
}

// Below are the functions pertaining to the Encoder (using interruptions)
//----------------------------------------------------------------------
void EncoderInit1()
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
      count1++;  // CW
    }
    else {
      count1--;  // CCW
    }
  }
  else { // A fall
    if (valB == HIGH) {
      count1++;  // CW
    }
    else {
      count1--;  //CCW
    }
  }
}

void pulseB(){
  int valA = digitalRead(ENC_A);
  int valB = digitalRead(ENC_B);

  if (valB == HIGH) { // B rise
    if (valA == HIGH) {
      count1++; // CW
    }
    else {
      count1--; // CCW
    }
  }
  else { //B fall
    if (valA == LOW) {
      count1++; // CW
    }
    else {
      count1--; // CCW
    }
  }
}


//----------------------------------------------------------------------
/*
void EncoderInit2()
{
  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_C), pulseC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D), pulseD, CHANGE);
}

void pulseC(){
  int valC = digitalRead(ENC_C);
  int valD = digitalRead(ENC_D);

  if (valC == HIGH) { // C Rise
    if (valD == LOW) {
      count2++;  // CW
    }
    else {
      count2--;  // CCW
    }
  }
  else { // C fall
    if (valD == HIGH) {
      count2++;  // CW
    }
    else {
      count2--;  //CCW
    }
  }
}

void pulseD(){
  int valC = digitalRead(ENC_C);
  int valD = digitalRead(ENC_D);

  if (valD == HIGH) { // D rise
    if (valC == HIGH) {
      count2++; // CW
    }
    else {
      count2--; // CCW
    }
  }
  else { //D fall
    if (valC == LOW) {
      count2++; // CW
    }
    else {
      count2--; // CCW
    }
  }
}

//----------------------------------------------------------------------

void EncoderInit3()
{
  pinMode(ENC_E, INPUT_PULLUP);
  pinMode(ENC_F, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_E), pulseE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_F), pulseF, CHANGE);
}

void pulseE(){
  int valE = digitalRead(ENC_E);
  int valF = digitalRead(ENC_F);

  if (valE == HIGH) { // E Rise
    if (valF == LOW) {
      count3++;  // CW
    }
    else {
      count3--;  // CCW
    }
  }
  else { // E fall
    if (valF == HIGH) {
      count3++;  // CW
    }
    else {
      count3--;  //CCW
    }
  }
}

void pulseF(){
  int valE = digitalRead(ENC_E);
  int valF = digitalRead(ENC_F);

  if (valF == HIGH) { // F rise
    if (valE == HIGH) {
      count3++; // CW
    }
    else {
      count3--; // CCW
    }
  }
  else { //F fall
    if (valE == LOW) {
      count3++; // CW
    }
    else {
      count3--; // CCW
    }
  }
}

*/
