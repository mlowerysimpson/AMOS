#include <stdio.h>
#include "AirProp.h"

AirProp::AirProp() {//AirProp constructor
	m_bSafetyMode = false;
	m_fSpeed=0;
	m_bCalMode=false;
}

AirProp::~AirProp() {//destructor
	Stop();//make sure propeller is stopped before deleting object
	//test
	printf("Deleted AirProp object.\n");
	//end test
}

void AirProp::Initialize() {//initializes propeller by sending a "stopped signal"  Calling program should then pause for a few seconds to make sure that initialization has properly occurred.
	Stop();
	delay(3000);//pause for 3 seconds
}

void AirProp::SetSpeed(float fSpeed) {//set speed of propeller (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED). 
	if (m_bSafetyMode) {
		Stop();
		return;
	}
	if (fSpeed>MAX_RECOMMENDED_AIRPROP_SPEED&&!m_bCalMode) {//invalid speed
		fSpeed = MAX_RECOMMENDED_AIRPROP_SPEED;
	} 
	else if (fSpeed<MIN_AIRPROP_SPEED) {//invalid speed
		Stop();	
		return;
	}
	if (fSpeed==0.0) {
		Stop();
		return;
	}
	m_fSpeed = fSpeed;
	int nDelayVal = STOP_AIR_PULSE_TIME;
	//adjust speed used to determine pulse time (if necessary) to avoid possible resonance with propeller cage / support
#ifdef AVOID_RESONANCE
	fSpeed = AvoidResonance(fSpeed);
#endif
	nDelayVal+=(int)((MAX_AIR_PULSE_TIME - STOP_AIR_PULSE_TIME)*fSpeed / MAX_AIRPROP_SPEED);
	int nPWMData = (int)(nDelayVal / AIR_TIME_INCREMENT_US);
	//printf("nPWMData = %d\n",nPWMData);
	pwmWrite(AIR_PROP_PWM_PIN,nPWMData);
}


void AirProp::Stop() {//send a "stopped signal" to propeller
	int nPWMData = (int)(STOP_AIR_PULSE_TIME / AIR_TIME_INCREMENT_US);
	m_fSpeed=0;
	//printf("Stop = %d\n",nPWMData);
	pwmWrite(AIR_PROP_PWM_PIN,nPWMData);
}


void AirProp::OneTimeSetup() {//setup procedure to be performed once per power-cycle of the air propeller
	wiringPiSetupGpio();//need to call this before doing anything with GPIO
	pinMode(AIR_PROP_PWM_PIN,PWM_OUTPUT);
	pwmSetClock(AIR_PWM_CLK_DIV);//set PWM clock divisor
	pwmSetMode(PWM_MODE_MS);//set to "mark space" mode	
}


//SetObstacleSafetyMode: function used for disabling air propeller, for example in the event that an obstacle is nearby
//bSafetyMode: set to true to disable propeller; propeller will not move until this function is called again and bSafetyMode is set to false
void AirProp::SetObstacleSafetyMode(bool bSafetyMode) {
	m_bSafetyMode = bSafetyMode;
	if (m_bSafetyMode) {
		Stop();
	}
}

//isInSafetyMode: returns true if the air propeller is currently turned off and in safety mode, otherwise return false
bool AirProp::isInSafetyMode() {
	return m_bSafetyMode;
}

//GetSpeed: returns the speed of the propeller
float AirProp::GetSpeed() {
	return m_fSpeed;
}

void AirProp::DoMaxSpeedCal() {//apply maximum speed signal to propeller 
	m_bCalMode=true;
	SetSpeed(MAX_AIRPROP_SPEED);
	m_bCalMode=false;
}

#ifdef AVOID_RESONANCE
float AirProp::AvoidResonance(float fSpeed) {//function returns a modified (lower) speed in order to avoid resonance with the structure surrounding the air propeller
	if (fSpeed<=RESONANCE_LOW_SPEED) return fSpeed;
	if (fSpeed>=RESONANCE_HIGH_SPEED) return fSpeed;
	return RESONANCE_LOW_SPEED;
}
#endif