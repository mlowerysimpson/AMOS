#include <stdio.h>
#include <wiringPi.h>
#include "AirProp.h"
#include "Rudder.h"

/**
 * @brief Construct a new Rudder:: Rudder object used for controlling the angle of the airboat rudder(s)
 * 
 */
Rudder::Rudder() {//Rudder constructor
	m_fAngle=0;
}

/**
 * @brief Destroy the Rudder:: Rudder object
 * 
 */
Rudder::~Rudder() {//destructor
	SetAngle(0,0);//make sure rudder is in the zero position prior to destroying object
}

/**
 * @brief set angle of rudder (in degrees) angle must be between MIN_RUDDER_ANGLE and MAX_RUDDER_ANGLE, if it is not, then the angle is set to zero degrees.
 * 
 * @param fAngle the desired angle of the rudder in degrees (counter-clockwise positive when looking down on the rudder from above).
 * @param fSpeed the speed of the air propeller (arbitrary units). It is necessary to specify this because the rudder angle should be relatively shallow at high propeller speeds.
 */
void Rudder::SetAngle(float fSpeed, float fAngle) {//set angle of rudder (in degrees)
	if (fAngle>MAX_RUDDER_ANGLE) {//invalid angle
		fAngle = MAX_RUDDER_ANGLE;
	}
	else if (fAngle<MIN_RUDDER_ANGLE) {//invalid angle
		fAngle = MIN_RUDDER_ANGLE;
	}
	m_fAngle = fAngle;
	
	if (fSpeed>3) {
		//need to make sure that fAngle is within range of MIN_TRIM_RUDDER_ANGLE to MAX_TRIM_RUDDER_ANGLE
		if (fAngle<MIN_TRIM_RUDDER_ANGLE) {
			fAngle = MIN_TRIM_RUDDER_ANGLE;
		}
		else if (fAngle>MAX_TRIM_RUDDER_ANGLE) {
			fAngle = MAX_TRIM_RUDDER_ANGLE;
		}
	}

	int nDelayVal = ZERO_RUDDER_PULSE_TIME;
	fAngle+=RUDDER_SERVO_BIAS;//add on rudder servo bias
	if (fAngle>0) {
		nDelayVal+=RUDDER_DEAD_BAND;
		nDelayVal+=(int)((MAX_RUDDER_PULSE_TIME - ZERO_RUDDER_PULSE_TIME - RUDDER_DEAD_BAND)*fAngle / RUDDER_LIMIT);
	}
	else if (fAngle<0) {
		nDelayVal-=RUDDER_DEAD_BAND;
		nDelayVal+=(int)((ZERO_RUDDER_PULSE_TIME - RUDDER_DEAD_BAND - MIN_RUDDER_PULSE_TIME)*fAngle / RUDDER_LIMIT);
	}
	int nPWMData = (int)(nDelayVal / RUDDER_TIME_INCREMENT_US);
	//printf("nPWMData = %d\n",nPWMData);
	pwmWrite(RUDDER_PWM_PIN,nPWMData);
}

/**
 * @brief static method that should be called prior to creating and using a Rudder object
 * 
 */
void Rudder::OneTimeSetup() {//setup procedure to be performed once per power-cycle of the air propeller
	wiringPiSetupGpio();//need to call this before doing anything with GPIO
	pinMode(RUDDER_PWM_PIN,PWM_OUTPUT);
	pwmSetClock(RUDDER_PWM_CLK_DIV);//set PWM clock divisor
	pwmSetMode(PWM_MODE_MS);//set to "mark space" mode	
}

/**
 * @brief returns the angle of the rudder in degrees
 * 
 * @return float returns the angle of the rudder in degrees
 */
float Rudder::GetAngle() {
	return m_fAngle;
}

