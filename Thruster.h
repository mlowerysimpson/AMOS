#pragma once
#include <wiringPi.h>
#include "AirProp.h"
#include "Rudder.h"

#define MIN_THRUSTER_SPEED -10.0 //actually a maximum speed in the "reverse" direction (arbitrary units)
#define MAX_THRUSTER_SPEED 10.0 //arbitrary units
#define MIN_PULSE_TIME 1280 //minimum pulse time in uS... can actually go lower for more vroom vroom... see documentation from BlueRobotics
#define MAX_PULSE_TIME 1720 //maximum pulse time in uS... can actually go higher for more vroom vroom... see documentation from BlueRobotics
#define STOP_PULSE_TIME 1500 //pulse time required for sending a "stop" signal to the T200
#define PWM_CLK_DIV 64 //divisor to use for 19.2 MHz PWM clock
#define PWM_CLK_FREQ 19200000 //19.2 MHz PWM clock frequency
#define TIME_INCREMENT_US 3.33 //timing for data portion of clock pulses has this resolution in microseconds
#define DEAD_BAND 25 // +/- value (in uS) around STOP_PULSE_TIME for which the T200 will not be moved
#define LEFT_THRUST_PIN 41  //gpio pin to use for PWM for left thruster (requires Alt0 alternative pin function, (Compute Module 3))
#define RIGHT_THRUST_PIN 40 //gpio pin to use for PWM for right thruster (requires Alt0 alternative pin function, (Compute Module 3))
#define SINGLE_THRUSTER_PIN 12 //gpio pin used on Raspberry Pi 3B+ when a single T200 water thruster is used

class Thruster {
public:
	Thruster(bool bLeftThruster, bool bRightThruster, bool bAirThruster, bool bSingleWaterThruster);//thruster constructor
	Thruster(char* szPrefsFilename);//thruster constructor with the path to a preferences file as the single parameter
	~Thruster();
	bool isAirBoat();//return true if this is an airboat, otherwise return false
	bool isBoatWithRudder();//return true if boat is using a rudder
	bool isInSafetyMode();//returns true if the thrusters are currently turned off and in safety mode, otherwise returns false
	void SetObstacleSafetyMode(bool bSafetyMode);//function used for disabling thrusters, for example in the event that an obstacle is nearby.
	void Initialize();//initializes thruster by sending a "stopped signal" Should hear two tones indicating initialization. Calling program should then pause for a few seconds to make sure that initialization has properly occurred.
	void SetSpeed(float fSpeed);//set speed of thruster (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
	void Stop();//send a "stopped signal" to thruster(s)
	void StopLeft();//stop the left thruster
	void StopRight();//stop the right thruster
	float GetLSpeed();//get the current speed of the left propeller
	float GetRSpeed();//get the current speed of the right propeller
	float GetSpeed();//get the speed of the propeller (air or single propeller)
	float GetAirSpeed();//get the speed of the air propeller (if available)
	float GetRudderAngle();//get the angle of the rudder (for air propeller or single water propeller) (if available) in degrees
	void SetLeftRightSpeed(float fLeftSpeed, float fRightSpeed);//set speed of left and right thrusters (speeds are in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
	void SetPropSpeedAndRudderAngle(float fSpeed, float fAngle);//set speed of propeller and the angle of the rudder
	void OneTimeSetup();//setup procedure to be performed once per power-cycle of the T200 thrusters
		 
private:
	void SetLeftSpeed(float fSpeed);//set speed of left thruster (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
	void SetRightSpeed(float fSpeed);//set speed of right thruster (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
	bool m_bSafetyMode;//if this flag is true, then thrusters should not be turned on
	bool m_bLeftThruster;//true if the left thruster is being used, otherwise false
	bool m_bRightThruster;//true if the right thruster is being used, otherwise false
	bool m_bSingleWaterThruster;//true if just a single water thruster is being used, with a rudder
	float m_fLSpeed;//the speed of the left propeller (arbitrary units)
	float m_fRSpeed;//the speed  of the right propeller (arbitrary units)
	AirProp *m_airProp;//the object used to control power to the air propeller (if available, i.e. if the bAirThruster flag is true in the Thruster constructor)
	Rudder *m_rudder;//the object used to control the direction of the rudder (if available)
};
