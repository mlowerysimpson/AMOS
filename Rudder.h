#define RUDDER_LIMIT 90.0 //maximum angular limit of servo motor is +/- RUDDER_LIMIT
#define MIN_RUDDER_ANGLE -70.0 //minimum allowed angle of air rudder in degrees
#define MAX_RUDDER_ANGLE 70.0 //maximum allowed angle of air rudder in degrees
#define ZERO_RUDDER_PULSE_TIME 1520 //pulse time in microseconds required for sending the zero degree angle to the rudder
#define MAX_RUDDER_PULSE_TIME 2500 //maximum pulse time in microseconds, corresponds to maximum possible rudder angle
#define MIN_RUDDER_PULSE_TIME 500 //mininum pulse time in microseconds, corresponds to minimum possible rudder angle
#define RUDDER_PWM_CLK_DIV 64 //divisor to use for 19.2 MHz PWM clock
#define RUDDER_PWM_CLK_FREQ 19200000 //19.2 MHz PWM clock frequency
#define RUDDER_TIME_INCREMENT_US 3.33 //timing for data portion of clock pulses has this resolution in microseconds
#define RUDDER_DEAD_BAND 3 // +/- value (in uS) around ZERO_PULSE_TIME for which the rudder will stay at zero degrees
#define RUDDER_PWM_PIN 40  //gpio pin to use for PWM for rudder (requires Alt0 alternative pin function)
#define RUDDER_SERVO_BIAS 26 //bias angle of servo motor that puts the rudder at a zero degree angle relative to the boat

class Rudder {
public:
	Rudder();//Rudder constructor
	~Rudder();
	float GetAngle();//returns the angle of the rudder (in degrees)
	void SetAngle(float fSpeed, float fAngle);//set angle of rudder (in degrees)
	static void OneTimeSetup();//setup procedure to be performed once per power-cycle of the rudder
		 
private:
	float m_fAngle;//the angle of the rudder in degrees
};
