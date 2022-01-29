

class Motor {
	public:
		Motor(int iin1, int iin2, int eena, float ddead_up, float ddead_low);
		int in1; //arduino pin connected to in1 of H Bridge; Make sure this is a digital pin
		int in2; //arduino pin connected to in2 of H Bridge; Make sure this is a digital pin
		int ena; //arduino pin connected to ena of H Bridge; Make sure this is a PWM pin
		float dead_up; //upper voltage of deadband
		float dead_low; //lower voltage of deadband
		float current_velocity; //current velocity of motor
		void drive_motor(float volt); // speed is based of input voltage; dir = 1 is clockwise
		float deadband_compensation(float lin_scale); //Adjust input for deadband, input linear scale from -10 -> 10 (0 no movement, 10 fastest, -10 fastest in other direction)
		void motor_setup(); //Make sure to call this in setup in order to initialize the pins
		float PID(float current_position, float desired_position); // does propotional control
		void calc_velocity(int cpr, int previous_count, int current_count, double Ts); //calculates velocoity of motor based on encoder counts, and counts per rotation of motor
	
};