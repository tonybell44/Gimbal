

class Motor {
	public:
		Motor(int iin1, int iin2, int eena, float ddead_up, float ddead_low, int ccpr, int proportion);

		//Variables
		int in1; //arduino pin connected to in1 of H Bridge; Make sure this is a digital pin
		int in2; //arduino pin connected to in2 of H Bridge; Make sure this is a digital pin
		int ena; //arduino pin connected to ena of H Bridge; Make sure this is a PWM pin
		float dead_up; //upper voltage of deadband
		float dead_low; //lower voltage of deadband
		float current_count; //current number of encoder pulses
		float previous_count; //previous number of encoder pulses before update
		int cpr; //counts per revolution (output of gearbox)
		int cascade_call_count; //keeps track of amount of times the CascadeControl function hs been called
		int pos_vel_proportion; //defines after how many cascade calls should the proportion (primary controller) update 
		float current_velocity; //current velocity of motor
		float kp; //Position constant for cascade control
		float kv; //Velocity constant for cascade control
		float ki; //Velocity integrator gain

		//Methods
		void change_control_constants(float kpos, float kvel,float kint); //allows user to change the constants used in CascadeControl
		void drive_motor(float volt); //speed is based of input voltage; dir = 1 is clockwise
		float deadband_compensation(float voltage_command); //Adjust input for deadband, input linear scale from -10 -> 10 (0 no movement, 10 fastest, -10 fastest in other direction)
		void motor_setup(); //Make sure to call this in setup in order to initialize the pins
		void CascadeControl(float current_position, float desired_position, float Ts); // does cascade control
		void calc_velocity(double Ts); //calculates velocity of motor based on encoder counts, and counts per rotation of motor
		void Tune_Velocity_Loop(float velocity, float Ts, float kvel, float kint); //allows user to alter kv to find the brink of instability 
		void Tune_Position_Loop(float kpos, float kvel, float kint, float current_position, float desired_position, float Ts); //allows user to alter kp to find overshoot (very similar to CascadeControl)
		
		//The following must be declared for the CascadeControl function if statement to work
		float velocity_command;
		float position_error;
	
};