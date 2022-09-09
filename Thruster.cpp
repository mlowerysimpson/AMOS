#include <stdio.h>
#include "Thruster.h"
#include "filedata.h"
#include <math.h>
#include <algorithm>

/**
 * @brief Construct a new Thruster:: Thruster object
 * 
 * @param bLeftThruster set to true if a water propeller on the left side of the boat is available, otherwise set to false
 * @param bRightThruster set to true if a water propeller on the right side of the boat is available, otherwise set to false
 * @param bAirThruster set to true if an air propeller is present (i.e. this is an airboat)
 */
Thruster::Thruster(bool bLeftThruster, bool bRightThruster, bool bAirThruster, bool bSingleWaterThruster) {//thruster constructor
	m_bLeftThruster = bLeftThruster;
	m_bRightThruster = bRightThruster;
	m_bSafetyMode = false;
	m_bSingleWaterThruster = bSingleWaterThruster;
	m_fLSpeed=0;
	m_fRSpeed=0;
	m_airProp=nullptr;
	m_rudder=nullptr;
	if (bAirThruster) {
		m_airProp = new AirProp();
		m_rudder = new Rudder();
	}
}

/**
 * @brief Thruster constructor with the path to a preferences file as the single parameter
 *
 * @param szPrefsFilename the path to the preferences file containing all of the program settings, including what type of thruster is being used
 */
Thruster::Thruster(char* szPrefsFilename) {
	//default values
	bool bAirThruster = true;
	m_bLeftThruster = false;
	m_bRightThruster = false;
	m_bSafetyMode = false;
	m_bSingleWaterThruster = false;
	m_fLSpeed = 0;
	m_fRSpeed = 0;
	m_airProp = nullptr;
	m_rudder = nullptr;
	if (szPrefsFilename != nullptr) {
		filedata prefsFile(szPrefsFilename);
		int nDualThrusters = prefsFile.getInteger("[thrusters]", "dual_thrusters");
		if (nDualThrusters > 0) {
			m_bLeftThruster = true;
			m_bRightThruster = true;
		}
		else {
			int nSingleWaterThruster = prefsFile.getInteger("[thrusters]", "single_water_thruster");
			if (nSingleWaterThruster > 0) {
				m_bSingleWaterThruster = true;
			}
			else {//assume air thrusterif (bAirThruster) {
				m_airProp = new AirProp();
			}
			m_rudder = new Rudder();
		}
	}
}

/**
 * @brief Destroy the Thruster:: Thruster object
 * 
 */
Thruster::~Thruster() {//destructor
	Stop();//make sure thrusters are stopped before deleting object
	if (m_airProp) {
		delete m_airProp;
		m_airProp = nullptr;
	}
	if (m_rudder) {
		delete m_rudder;
		m_rudder = nullptr;
	}
}

/**
 * @brief initializes thruster by sending a "stopped signal". The function then pauses for 3 seconds (some thrusters require this after startup)
 * 
 */
void Thruster::Initialize() {//
	Stop();
	delay(3000);//pause for 3 seconds
}

/**
 * @brief set speed of both thrusters to be the same value (fSpeed, expressed in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
 * 
 * @param fSpeed the speed to assign to both thrusters, expressed in arbitrary units, between MIN_SPEED and MAXSPEED.
 */
void Thruster::SetSpeed(float fSpeed) {
	if (m_airProp) {
		m_airProp->SetSpeed(fSpeed);
		m_rudder->SetAngle(fSpeed,0);
		m_fLSpeed = m_airProp->GetSpeed();
		m_fRSpeed = m_airProp->GetSpeed();
		return;
	}	
	if (m_bSafetyMode) {
		Stop();
		return;
	}
	else if (m_bSingleWaterThruster) {
		m_fLSpeed = fSpeed;
		m_fRSpeed = fSpeed;
		m_rudder->SetAngle(fSpeed, 0);
		int nDelayVal = STOP_PULSE_TIME;
		if (fSpeed > 0) {
			nDelayVal += DEAD_BAND;
			nDelayVal += (int)((MAX_PULSE_TIME - STOP_PULSE_TIME - DEAD_BAND) * fSpeed / MAX_THRUSTER_SPEED);
		}
		else if (fSpeed < 0) {
			nDelayVal -= DEAD_BAND;
			nDelayVal -= (int)((STOP_PULSE_TIME - MIN_PULSE_TIME + DEAD_BAND) * fSpeed / MIN_THRUSTER_SPEED);
		}
		int nPWMData = (int)(nDelayVal / TIME_INCREMENT_US);
		pwmWrite(SINGLE_THRUSTER_PIN, nPWMData);
		return;
	}
	if (fSpeed>MAX_THRUSTER_SPEED) {//invalid speed
		Stop();
		return;
	}
	else if (fSpeed<MIN_THRUSTER_SPEED) {//invalid speed
		Stop();	
		return;
	}
	if (fSpeed==0.0) {
		Stop();
		return;
	}
	m_fLSpeed = fSpeed;
	m_fRSpeed = fSpeed;
	int nDelayVal = STOP_PULSE_TIME;
	if (fSpeed>0) {
		nDelayVal+=DEAD_BAND;
		nDelayVal+=(int)((MAX_PULSE_TIME - STOP_PULSE_TIME - DEAD_BAND)*fSpeed / MAX_THRUSTER_SPEED);
	}
	else {
		nDelayVal-=DEAD_BAND;
		nDelayVal-=(int)((STOP_PULSE_TIME - MIN_PULSE_TIME + DEAD_BAND)*fSpeed / MIN_THRUSTER_SPEED);
	}
	int nPWMData = (int)(nDelayVal / TIME_INCREMENT_US);
	if (m_bLeftThruster) {
		pwmWrite(LEFT_THRUST_PIN,nPWMData);
	}
	if (m_bRightThruster) {
		pwmWrite(RIGHT_THRUST_PIN,nPWMData);
	}
}

/**
 * @brief set speed of left and right thrusters (speeds are in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
 * 
 * @param fLeftSpeed the amount of speed to allocate to the left side. If fLeftSpeed > fRightSpeed produces a turning effect to the right
 * @param fRightSpeed the amount of speed to allocate to the right side. If fRightSpeed > fLeftSpeed produces a turning effect to the left
 */
void Thruster::SetLeftRightSpeed(float fLeftSpeed, float fRightSpeed) {//set speed of left and right thrusters (speeds are in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
	if (m_airProp) {//air propeller
		//negative speeds are not allowed for air propellers
		if (fLeftSpeed<0) fLeftSpeed=0;
		if (fRightSpeed<0) fRightSpeed=0;
		if (fLeftSpeed>MAX_AIRPROP_SPEED) fLeftSpeed = MAX_AIRPROP_SPEED;
		if (fRightSpeed>MAX_AIRPROP_SPEED) fRightSpeed = MAX_AIRPROP_SPEED;
		float fMaxSpeed = std::max(abs(fLeftSpeed),abs(fRightSpeed));
		float fSpeedDif = fLeftSpeed - fRightSpeed;
		float fRudderAngle = 0;
		if (fMaxSpeed>0) {
			fRudderAngle = MAX_RUDDER_ANGLE * fSpeedDif / MAX_AIRPROP_SPEED;
		}
		if (fRudderAngle>MAX_RUDDER_ANGLE) fRudderAngle = MAX_RUDDER_ANGLE;
		else if (fRudderAngle<MIN_RUDDER_ANGLE) fRudderAngle = MIN_RUDDER_ANGLE;
		m_airProp->SetSpeed(fMaxSpeed);
		m_rudder->SetAngle(fMaxSpeed, fRudderAngle);
		return;
	}
	else if (m_bSingleWaterThruster) {//single water thruster with rudder
		float fSpeedDif = fLeftSpeed - fRightSpeed;
		float fRudderAngle = 0;
		float fMaxSpeed = std::max(abs(fLeftSpeed), abs(fRightSpeed));
		if (fMaxSpeed > 0) {
			fRudderAngle = MAX_WATER_RUDDER_ANGLE * fSpeedDif / MAX_THRUSTER_SPEED;
		}
		if (fRudderAngle > MAX_WATER_RUDDER_ANGLE) fRudderAngle = MAX_WATER_RUDDER_ANGLE;
		else if (fRudderAngle < MIN_WATER_RUDDER_ANGLE) fRudderAngle = MIN_WATER_RUDDER_ANGLE;
		this->SetSpeed(fMaxSpeed);
		m_rudder->SetAngle(fMaxSpeed, fRudderAngle);
		return;
	}
	SetLeftSpeed(fLeftSpeed);
	SetRightSpeed(fRightSpeed);	
}

/**
 * @brief set speed of left thruster (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
 * 
 * @param fSpeed in arbitrary units, must be between MIN_SPEED and MAXSPEED
 */
void Thruster::SetLeftSpeed(float fSpeed) {
	if (m_bSafetyMode) {
		Stop();
		return;	
	}
	m_fLSpeed = fSpeed;

	if (!m_bLeftThruster) return;
	if (fSpeed>MAX_THRUSTER_SPEED) {//invalid speed
		Stop();	
		return;
	}
	else if (fSpeed<MIN_THRUSTER_SPEED) {//invalid speed
		Stop();
		return;
	}
	if (fSpeed==0.0) {
		StopLeft();
		return;
	}
	int nDelayVal = STOP_PULSE_TIME;
	if (fSpeed>0) {
		nDelayVal+=DEAD_BAND;
		nDelayVal+=(int)((MAX_PULSE_TIME - STOP_PULSE_TIME - DEAD_BAND)*fSpeed / MAX_THRUSTER_SPEED);
	}
	else {
		nDelayVal-=DEAD_BAND;
		nDelayVal-=(int)((STOP_PULSE_TIME - MIN_PULSE_TIME + DEAD_BAND)*fSpeed / MIN_THRUSTER_SPEED);
	}
	int nPWMData = (int)(nDelayVal / TIME_INCREMENT_US);
	pwmWrite(LEFT_THRUST_PIN,nPWMData);
}

void Thruster::SetRightSpeed(float fSpeed) {//set speed of right thruster (fSpeed is in arbitrary units, must be between MIN_SPEED and MAXSPEED). Negative speeds are in reverse.
	if (m_bSafetyMode) {
		Stop();
		return;
	}
	m_fRSpeed = fSpeed;

	if (!m_bRightThruster) {
		Stop();	
		return;
	}
	if (fSpeed>MAX_THRUSTER_SPEED) {//invalid speed
		Stop();	
		return;
	}
	else if (fSpeed<MIN_THRUSTER_SPEED) {
		Stop();	
		return;//invalid speed
	}
	if (fSpeed==0.0) {
		StopRight();
		return;
	}
	int nDelayVal = STOP_PULSE_TIME;
	if (fSpeed>0) {
		nDelayVal+=DEAD_BAND;
		nDelayVal+=(int)((MAX_PULSE_TIME - STOP_PULSE_TIME - DEAD_BAND)*fSpeed / MAX_THRUSTER_SPEED);
	}
	else {
		nDelayVal-=DEAD_BAND;
		nDelayVal-=(int)((STOP_PULSE_TIME - MIN_PULSE_TIME + DEAD_BAND)*fSpeed / MIN_THRUSTER_SPEED);
	}
	int nPWMData = (int)(nDelayVal / TIME_INCREMENT_US);
	pwmWrite(RIGHT_THRUST_PIN,nPWMData);
}

/**
 * @brief send a "stopped signal" to all thrusters, to stop them from turning
 * 
 */
void Thruster::Stop() {
	m_fLSpeed=0;
	m_fRSpeed=0;
	if (m_airProp) {
		m_airProp->Stop();
		return;
	}
	int nPWMData = (int)(STOP_PULSE_TIME / TIME_INCREMENT_US);
	if (m_bSingleWaterThruster) {
		pwmWrite(SINGLE_THRUSTER_PIN, nPWMData);
		return;
	}
	if (m_bLeftThruster) {
		pwmWrite(LEFT_THRUST_PIN,nPWMData);
	}
	if (m_bRightThruster) {
		pwmWrite(RIGHT_THRUST_PIN,nPWMData);
	}
}

/**
 * @brief stop the left thruster
 * 
 */
void Thruster::StopLeft() {//stop the left thruster
	if (!m_bLeftThruster) {
		return;
	}
	m_fLSpeed=0;
	int nPWMData = (int)(STOP_PULSE_TIME / TIME_INCREMENT_US);
	pwmWrite(LEFT_THRUST_PIN,nPWMData);
}

/**
 * @brief stop the right thruster
 * 
 */
void Thruster::StopRight() {//stop the right thruster
	if (!m_bRightThruster) {
		return;
	}
	m_fRSpeed=0;
	int nPWMData = (int)(STOP_PULSE_TIME / TIME_INCREMENT_US);
	pwmWrite(RIGHT_THRUST_PIN,nPWMData);
}

/**
 * @brief static method to perform one-time setup for configuring pulse width modulated (PWM) outputs for the thrusters
 * 
 */
void Thruster::OneTimeSetup() {//setup procedure to be performed once per power-cycle of the T200 thrusters
	wiringPiSetupGpio();//need to call this before doing anything with GPIO
	if (m_bSingleWaterThruster) {
		pinMode(SINGLE_THRUSTER_PIN, PWM_OUTPUT);
	}
	else if (m_bLeftThruster && m_bRightThruster) {
		pinMode(LEFT_THRUST_PIN, PWM_OUTPUT);
		pinMode(RIGHT_THRUST_PIN, PWM_OUTPUT);
	}
	pwmSetClock(PWM_CLK_DIV);//set PWM clock divisor
	pwmSetMode(PWM_MODE_MS);//set to "mark space" mode	
}

/**
 * @brief function used for disabling thrusters, for example in the event that an obstacle is nearby
 * 
 * @param bSafetyMode set to true to disable thrusters; thrusters will not move until this function is called again and bSafetyMode is set to false
 */
void Thruster::SetObstacleSafetyMode(bool bSafetyMode) {
	m_bSafetyMode = bSafetyMode;
	if (m_airProp) {
		m_airProp->SetObstacleSafetyMode(bSafetyMode);
		return;
	}
	if (m_bSafetyMode) {
		if (m_bSingleWaterThruster) {
			Stop();
		}
		else {
			StopLeft();
			StopRight();
		}
	}
}


/**
 * @brief returns true if the thrusters are currently turned off and in safety mode, otherwise returns false
 * 
 * @return true if the thrusters are currently turned off and in safety mode
 * @return false if the thrusters are not in safety mode
 */
bool Thruster::isInSafetyMode() {
	return m_bSafetyMode;
}

/**
 * @brief returns the speed of the left propeller
 * 
 * @return float the speed of the left propeller
 */
float Thruster::GetLSpeed() {
	return m_fLSpeed;
}


/**
 * @brief returns the speed of the right propeller
 * 
 * @return float the speed of the right propeller
 */
float Thruster::GetRSpeed() {
	return m_fRSpeed;
}


/**
 * @brief return true if this is an airboat, otherwise return false
 * 
 * @return true if this is an airboat, i.e. m_airProp is non-null
 * @return false if this is not an airboat, i.e. m_airProp is null
 */
bool Thruster::isAirBoat() {//return true if this is an airboat, otherwise return false
	return (m_airProp!=nullptr);
}


/**
 * @brief get the speed of the air propeller (if available)
 * 
 * @return float the speed of the air propeller (arbitrary units)
 */
float Thruster::GetAirSpeed() {//get the speed of the air propeller (if available)
	if (m_airProp==nullptr) {//not an airboat
		return 0;
	}
	return m_airProp->GetSpeed();
}


/**
 * @brief get the angle of the rudder (if available) in degrees
 * 
 * @return float the angle of the rudder in degrees
 */
float Thruster::GetRudderAngle() {//get the angle of the rudder (if available) in degrees
	if (m_rudder==nullptr) {
		return 0;
	}
	float fRudderAngle = m_rudder->GetAngle();
	return fRudderAngle;
}

/**
 * @brief set speed of propeller (air or single water) and the angle of the rudder
 * 
 * @param fSpeed the speed to assign to the propeller (arbitrary units, should be between 0 and MAX_RECOMMENDED_AIRPROP_SPEED for an air propeller or MAX_THRUSTER_SPEED for a single water propeller)
 * @param fRudderAngle the angle (in degrees) to assign to the rudder. fRudderAngle should be between MIN_RUDDER_ANGLE and MAX_RUDDER_ANGLE. Positive angles result in a clockwise turning moment to the boat.
 */
void Thruster::SetPropSpeedAndRudderAngle(float fSpeed, float fRudderAngle) {//set speed of propeller and the angle of the rudder
	if (m_rudder==nullptr) return;
	if (m_airProp != nullptr) {
		m_airProp->SetSpeed(fSpeed);
	}
	else {
		this->SetSpeed(fSpeed);
	}
	m_rudder->SetAngle(fSpeed, fRudderAngle);
}

bool Thruster::isBoatWithRudder() {//return true if boat is using a rudder
	return (m_rudder != nullptr);
}

float Thruster::GetSpeed() {//get the speed of the propeller (air or single propeller)
	if (m_airProp) {
		return m_airProp->GetSpeed();
	}
	return std::max(m_fLSpeed, m_fRSpeed);
}