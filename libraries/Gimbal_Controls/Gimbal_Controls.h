

class Motor {
	public:
		Motor(int iin1, int iin2, int eena, float ddead_up, float ddead_low, int ccpr, int proportion, float sig, float Tstep);

		//Variables
		int in1; //arduino pin connected to in1 of H Bridge; Make sure this is a digital pin
		int in2; //arduino pin connected to in2 of H Bridge; Make sure this is a digital pin
		int ena; //arduino pin connected to ena of H Bridge; Make sure this is a PWM pin
		int dead_up; //upper pwm of deadband
		int dead_low; //lower pwm of deadband
		float current_count; //current number of encoder pulses
		float previous_count; //previous number of encoder pulses before update
		int cpr; //counts per revolution (output of gearbox)
		int cascade_call_count; //keeps track of amount of times the CascadeControl function hs been called
		int pos_vel_proportion; //defines after how many cascade calls should the proportion (primary controller) update 
		float current_velocity; //current velocity of motor
		float kp; //Position constant for cascade control and PID control
		float kv; //Velocity constant for cascade control and derivative constant for PID control
		float ki; //Velocity integrator gain for cascade control and integral constant for PID control
		float Ts; //Timestep
		float sigma; //Used in dirty derivative calculation. 1/sigma is the bandwidth of the differentiator. Bandwidth of a closed loop system is approx. equal to crossover freq of open loop system.
		float beta; //Used in dirty derivative calculation.
		float zeta; //Used in dirty derivative calculation.
		float prev_derivative; //The previous derivative (velocity) of the system
		float prev_proportion; //The previous proportion of the system
		
		float error_integral;

		//Methods
		void change_control_constants(float kpos, float kvel,float kint); //allows user to change the constants used in CascadeControl
		void drive_motor(float volt); //speed is based of input voltage; dir = 1 is clockwise
		void drive_motor_dead(float volt); // drive with deadband in pwm 
		float deadband_compensation(float voltage_command); //Adjust input for deadband, input linear scale from -10 -> 10 (0 no movement, 10 fastest, -10 fastest in other direction)
		void motor_setup(); //Make sure to call this in setup in order to initialize the pins
		void CascadeControl(float current_position, float desired_position, float gyro); //does cascade control
		//void PIDControl(float current_position, float desired_position, float derivative); //does PID control
		void calc_velocity(); //calculates velocity of motor based on encoder counts and counts per rotation of the motor
		void calc_velocity_dirty(); //calculates velocity of motor based on encoder counts and counts per rotation of the motor, using the dirty derivative
		void Tune_Velocity_Loop(float velocity, float Ts, float kvel, float kint); //allows user to alter kv to find the brink of instability 
		void Tune_Position_Loop(float kpos, float kvel, float kint, float current_position, float desired_position, float Ts); //allows user to alter kp to find overshoot (very similar to CascadeControl)
		void test_pwm(int pwm);
		
		//The following must be declared for the CascadeControl function if statement to work
		float velocity_command;
		float position_error;
		float voltage_integral;
	
	
		//PID Testing
		float proportional(float current_position, float desired_position);
		float derivative();
		void integral(float current_position, float desired_position);
		//float square(float number);
		void PIDControl(float current_position, float desired_position); //does PID control
};