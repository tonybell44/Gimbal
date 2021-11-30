#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H
#include "Complementary_Filter.h"
#endif

#include <math.h>
#define M_PI 3.14159265359

/*
 * The following code is for the complementary filter. It takes an array of accelorometer data
 * and gyroscope date. For the sake of organization, please make accData position 0,1,2 be x,y,z
 * respectively, and follow the same convention for gyrData when calling this function
 */

void Complementary(float accData[3], float gyrData[3], float *pitch, float *roll, float *yaw, double dt)
{
  float pitchAcc;
  float rollAcc;
  float yawAcc;
  
  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += gyrData[0] * (M_PI/180) * dt; // Angle around the X-axis
  *roll += gyrData[1] * (M_PI/180) * dt;	 // Angle around the Y-axis
  *yaw += gyrData[2] * (M_PI/180) * dt;   // Angle around the Z-axis

  // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    // This accounts for bullshit forces
    //int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    
     //if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
     //{
      //To obtain the angular position with the accelerometer, 
      //we are going to determine the position of the gravity vector 
      //(g-force) which is always visible on the accelerometer. (Why we use atan2f)
      pitchAcc = atan2f(accData[1], accData[2]);
      *pitch = *pitch * 0.7 + pitchAcc * 0.3;
	  
	  rollAcc = atan2f(accData[0], accData[2]);
	  *roll = *roll * 0.7 + rollAcc * 0.3;
	  
	  yawAcc = atan2f(accData[0],accData[1]);
	  *yaw = *yaw * 0.7 + yawAcc * 0.3;
//     }
}