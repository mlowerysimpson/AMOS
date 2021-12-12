#include <wiringPi.h>

#define MIN_AIRPROP_SPEED 0.0 //minimum air propeller speed (i.e. stopped)
#define MAX_RECOMMENDED_AIRPROP_SPEED 9//a speed typically a bit lower than the maximum that is more suitable for conserving power when sunlight is in limited supply
#define MAX_AIRPROP_SPEED 10.0 //arbitrary units
#define STOP_AIR_PULSE_TIME 1000 //pulse time required for sending a "stop" signal to the air propeller
#define MAX_AIR_PULSE_TIME 1400 //maximum pulse time in microseconds, corresponds to maximum propeller speed (may need to tweak this somewhat depending on motor / speed controller used)
#define AIR_PWM_CLK_DIV 64 //divisor to use for 19.2 MHz PWM clock
#define AIR_PWM_CLK_FREQ 19200000 //19.2 MHz PWM clock frequency
#define AIR_TIME_INCREMENT_US 3.33 //timing for data portion of clock pulses has this resolution in microseconds
#define AIR_PROP_PWM_PIN 12  //gpio pin to use for PWM for propeller (requires Alt0 alternative pin function)

//parameters for avoiding resonance speeds
//#define AVOID_RESONANCE 1 //need to define the following 3 lines with suitable values for setups (eg. damaged propeller) that can resonate at certain speeds
//#define RESONANCE_LOW_SPEED 2.5 //
//#define RESONANCE_HIGH_SPEED 6.9

class AirProp {
public:
	AirProp();//AirProp constructor
	~AirProp();
	bool isInSafetyMode();//returns true if the propeller is currently turned off and in safety mode, otherwise returns false
	void SetObstacleSafetyMode(bool bSafetyMode);//function used for disabling propeller, for example in the event that an obstacle is nearby.
	void Initialize();//initializes propeller by sending a "stopped signal".  Calling program should then pause for a few seconds to make sure that initialization has properly occurred.
	void SetSpeed(float fSpeed);//set speed of propeller (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED).
	void Stop();//send a "stopped signal" to the propeller
	float GetSpeed();//get the current speed of the propeller
	static void OneTimeSetup();//setup procedure to be performed once per power-cycle of the propeller
	void DoMaxSpeedCal();//apply maximum speed signal to propeller
		 
private:
	//data
	bool m_bCalMode;//set to true whenever a calibration is being performed
	bool m_bSafetyMode;//if this flag is true, then propeller should not be turned on
	float m_fSpeed;//the speed of the propeller (arbitrary units)

	//functions
#ifdef AVOID_RESONANCE
	float AvoidResonance(float fSpeed);//function returns a modified (lower) speed in order to avoid resonance with the structure surrounding the air propeller
#endif
};
