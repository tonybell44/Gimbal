// Complementary_Filter takes in acceleration and gyroscope value arrays, the pitch, roll, and yaw, and the refresh rate as inputs

void Complementary(float accData[3], float gyrData[3], float *pitch, float *roll, float *yaw, double dt);