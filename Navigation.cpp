//Navigation.cpp
//Implementation for Navigation class
#include "Navigation.h"
#include "Thruster.h"
#include "ShipLog.h"
#include "filedata.h"
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#define TIMEOUT_TIME_US 500000//timeout period in microseconds (needs to be faster than the update rate of the GPS device, in order for socket connection to stay alive
#define METERS_PER_SEC_TO_KTS 1.94384//conversion factor from meters per second to knots
#define EARTH_RADIUS_EQUATOR_M 6378150.0//radius of Earth at the equator in m
#define EARTH_RADIUS_POLE_M 6356890.0//radius of Earth at the poles in m
#define CLOSE_ENOUGH_M 20.0//close enough distance in m for making decisions about whether or not we are close enough to a target destination


/**
 * @brief Construct a new Navigation:: Navigation object
 * 
 * @param dMagDeclination the magnetic declination in degrees. Set it to zero if you want headings to be relative to magnetic north.
 * @param i2c_mutex mutex controlling access to the i2c bus
 */
Navigation::Navigation(double dMagDeclination, pthread_mutex_t *i2c_mutex) {
	m_currentGPSData=nullptr;
	m_nMaxPriority = LOW_PRIORITY;
	m_i2c_mutex = i2c_mutex;
	m_uiPreviousTrimTime=0;
	m_fEstimatedCompassError=0;
	m_uiLastCompassCheckTime=0;
	m_bGotValidGPSData=false;
	m_dMagDeclination = dMagDeclination;
	m_gps_rec = new gpsmm("localhost",DEFAULT_GPSD_PORT);//create GPS receiver object
	m_bInitializedGPS=true;
	if (m_gps_rec->stream(WATCH_ENABLE | WATCH_JSON)==NULL) {
		std::cerr << "No GPSD running.\n";
		m_bInitializedGPS=false;
	}
	else {
		printf("gps receiver created ok.\n");
	}
	m_gpsTime=NULL;
	m_dMaxSpeed=MAX_THRUSTER_SPEED;
	m_dLatitude=0.0;
	m_dLongitude=0.0;
	m_dSpeed=0.0;
	m_dTrack=0.0;
	m_dYawRate = 0.0;
	m_dFilteredYawRate = 0.0;
	m_bCompassErrorShown=false;
	m_imu=nullptr;
	memset(&m_imuData,0,sizeof(IMU_DATASAMPLE));//initialize IMU data to zeros
	//seed random number generator
	srand((unsigned int)time(NULL));
	for (int i=0;i<SAMPLE_BUFSIZE;i++) {
		m_navSamples[i]=NULL;
	}
	for (int i=0;i<COMPASS_BUFSIZE;i++) {
		m_compassBuf[i]=NULL;
	}
	m_nSampleIndex=0;
	m_nNumSamples=0;
	m_nCompassBufIndex=0;
	m_compassAdjustment = LoadCompassAdjustments();//get compass adjustments (if any) from compass_cal.txt file
}

Navigation::~Navigation() {
	if (m_currentGPSData!=nullptr) {
		delete m_currentGPSData;
		m_currentGPSData=nullptr;
	}
	if (m_gps_rec) {
		delete m_gps_rec;
		m_gps_rec=nullptr;
	}
	if (m_gpsTime) {
		delete m_gpsTime;
		m_gpsTime=NULL;
	}
	if (m_imu) {
		delete m_imu;
		m_imu=nullptr;
	}
	for (int i=0;i<SAMPLE_BUFSIZE;i++) {
		if (m_navSamples[i]) {
			delete m_navSamples[i];
			m_navSamples[i]=NULL;
		}
	}
	for (int i=0;i<COMPASS_BUFSIZE;i++) {
		if (m_compassBuf[i]) {
			delete m_compassBuf[i];
			m_compassBuf[i]=NULL;
		}
	}

	if (m_compassAdjustment) {
		if (m_compassAdjustment->actual_compass_vals) {
			delete []m_compassAdjustment->actual_compass_vals;
			m_compassAdjustment->actual_compass_vals = nullptr;
		}
		if (m_compassAdjustment->reported_compass_vals) {
			delete []m_compassAdjustment->reported_compass_vals;
			m_compassAdjustment->reported_compass_vals = nullptr;
		}
		delete m_compassAdjustment;
		m_compassAdjustment = nullptr;
	}
}

/**
 * @brief send most recent GPS data out over network socket or serial port connection
 * 
 * @param nHandle the network socket handle (if bUseSerial is false) or the serial port file descriptor (if bUseSerial is true)
 * @param bUseSerial set to true when using a wireless serial port connection, otherwise set to false for a network connection
 * @return true if the GPS data was sent successfully
 * @return false if there was a problem sending the GPS data
 */
bool Navigation::SendGPSData(int nHandle, bool bUseSerial) {
	GPS_DATA gpsData;
	gpsData.dLatitude = m_dLatitude;
	gpsData.dLongitude = m_dLongitude;
	if (m_gpsTime!=nullptr) {
		gpsData.gps_time = *m_gpsTime;
	}
	else gpsData.gps_time=0;
	BOAT_DATA *pBoatData = BoatCommand::CreateBoatData(GPS_DATA_PACKET);
	memcpy(pBoatData->dataBytes,&gpsData.dLatitude,sizeof(double));
	memcpy(&pBoatData->dataBytes[sizeof(double)],&gpsData.dLongitude,sizeof(double));
	memcpy(&pBoatData->dataBytes[2*sizeof(double)],&gpsData.gps_time,sizeof(time_t));
	pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
	return BoatCommand::SendBoatData(nHandle, bUseSerial, pBoatData, nullptr);
}

//CollectGPSData: collects a sample of GPS data
//pShipLog = void pointer to ship's log for logging errors, occasional data, etc.
//bKeepGoing = boolean pointer to flag that indicates when to give up (in case the program is closing), flag is false if program is closing and we need to exit the function right away
//returns true if GPS data could be acquired successfully, otherwise false
bool Navigation::CollectGPSData(void *pShipLog, bool *bKeepGoing) {//collects a sample of GPS data
	ShipLog *pLog = (ShipLog *)pShipLog;
	struct gps_data_t *gpsd_data;
	if (!*bKeepGoing) return false;
	if (!m_gps_rec->waiting(TIMEOUT_TIME_US)) {
		//pLog->LogEntry((char *)"GPS timeout.\n",true);
		return false;
	}
	if (!*bKeepGoing) return false;
	if ((gpsd_data = m_gps_rec->read())==NULL) {
		pLog->LogEntry((char *)"GPSD read error.\n",true);
		return false;
	}
	else {
		unsigned int uiStart = millis();
		while (((gpsd_data = m_gps_rec->read())==NULL) ||
			(gpsd_data->fix.mode<MODE_2D)) {
				if (!*bKeepGoing) return false;
				//Do nothing until fix
				//output something to log file every 10 seconds without fix
				unsigned int uiTimeNow = millis();
				if ((uiTimeNow-uiStart)>10000) {
					uiStart = uiTimeNow;
					pLog->LogEntry((char *)"Waiting for GPS fix.\n",true);
				}
		}
		timestamp_t ts = gpsd_data->fix.time;
		m_dLatitude = gpsd_data->fix.latitude;
		m_dLongitude = gpsd_data->fix.longitude;
		m_dSpeed = gpsd_data->fix.speed;
		m_dTrack = gpsd_data->fix.track;
		//convert GPSD's timestamp_t into time_t
		if (!m_gpsTime) {
			m_gpsTime = new time_t;
		}
		*m_gpsTime = (time_t)ts;
		if (m_currentGPSData==nullptr) {
			m_currentGPSData = new GPS_DATA;
		}
		//copy to m_currentGPSData structure
		m_currentGPSData->dLatitude = m_dLatitude;
		m_currentGPSData->dLongitude = m_dLongitude;
		m_currentGPSData->gps_time = *m_gpsTime;
	}
	m_bGotValidGPSData=true;
	return true;
}

/**
 * @brief send most recent compass data out network socket or serial port connection.
 * 
 * @param nHandle the network socket handle or the serial port file descriptor, depending on whether bUseSerial is false or true respectively.
 * @param bUseSerial set to true when a wireless serial link is being used, otherwise set to false for a network connection.
 * @return true if the compass data could be successfully sent.
 * @return false if there was a problem sending the compass data.
 */
bool Navigation::SendCompassData(int nHandle, bool bUseSerial) {
	BOAT_DATA *pBoatData = BoatCommand::CreateBoatData(COMPASS_DATA_PACKET);
	memcpy(pBoatData->dataBytes,&this->m_imuData,sizeof(IMU_DATASAMPLE));
	pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
	return BoatCommand::SendBoatData(nHandle, bUseSerial, pBoatData, nullptr);
}

//CollectCompassData collects a sample of compass data
//pShipLog = void pointer to ship's log for logging errors, occasional data, etc.
//returns true if successful
bool Navigation::CollectCompassData(void *pShipLog) {//collects a sample of compass data
	const int NUM_TO_AVG = 1;//number of individual samples to average for each call to IMU::GetSample
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (!m_imu) {
		m_imu = Navigation::CreateCompass();
	}
	if (!m_imu) {
		ShowCompassError("Error, unable to create compass.\n");
		return false;
	}
	if (!m_imu->GetSample(&this->m_imuData,NUM_TO_AVG)) {
		ShowCompassError("Error getting compass data.\n");
		return false;
	}
	//adjust compass data for magnetic declination
	m_imuData.heading+=m_dMagDeclination;
	if (m_imuData.heading>=360) {
		m_imuData.heading-=360;
	}
	else if (m_imuData.heading<0) {
		m_imuData.heading+=360;
	}
	m_imuData.heading = AdjustUsingCompassCal(m_imuData.heading);
	//compute rate of change of heading (i.e. m_dYawRate and m_dFilteredYawRate)
	unsigned int uiCurrentSampleTime = millis();
	//double dHeadingDif = ComputeHeadingDif(m_dLastHeadingValue,m_imuData.heading);//difference between current heading and previous heading in degrees
	//double dTimeDifSeconds = (uiCurrentSampleTime - m_uiLastHeadingSampleTime)/1000.0;
	BufferCompassData(m_imuData.heading, uiCurrentSampleTime);
	m_dYawRate = ComputeYawRate(false);
	m_dFilteredYawRate = ComputeYawRate(true);
	
	//test
	//char sMsg[256];
	//sprintf(sMsg,"heading = %.1f, yaw = %.2f\n",m_imuData.heading,m_dYawRate);
	//pLog->LogEntry(sMsg,false);
	//end test
	return true;
}

void Navigation::ShowCompassError(const char *szErrorText) {//show error text pertaining to the compass module... just display one error message per program session
	if (m_bCompassErrorShown) {
		return;//already shown compass error
	}
	printf(szErrorText);
	m_bCompassErrorShown=true;
}

IMU * Navigation::CreateCompass() {//creates a compass object for collecting compass and orientation
	IMU *pCompass = new IMU(m_i2c_mutex);
	if (!pCompass->m_bOpenedI2C_OK) {
		printf("Error trying to open I2C port.\n");
		delete pCompass;
		pCompass=nullptr;
	}
	else if (!pCompass->m_bMagInitialized_OK) {
		printf("Error trying to initialize magnetometer.\n");
		delete pCompass;
		pCompass=nullptr;
	}
	else if (!pCompass->m_bAccGyroInitialized_OK) {
		printf("Error trying to initialize accelerometer / gyro.\n");
		delete pCompass;
		pCompass=nullptr;	
	}
	return pCompass;
}

//ComputeHeadingDif: compute difference between 2 heading angles in degrees, Heading2 - Heading1
//dHeading1Deg: the first heading value in degrees (0 to 360)
//dHeading2Deg: the second heading value in degrees (0 to 360)
double Navigation::ComputeHeadingDif(double dHeading1Deg,double dHeading2Deg) {//compute difference between 2 heading angles in degrees
	double dDif = dHeading2Deg - dHeading1Deg;
	if (dDif<-180) {
		dDif+=360.0;
	}
	else if (dDif>180) {
		dDif-=360.0;
	}
	return dDif;
}

//TurnToCompassHeading: 
//fHeading = the desired heading to turn to in degrees
//pThrusters = void pointer to previously created Thruster object for driving the propeller(s)
//command_mutex = mutex controlling access to propellers
//lastNetworkCommandTimeMS = pointer to time in milliseconds since program started that the last network command was issued
//pShipLog = pointer to ShipLog object that is used for recording program errors and some data for the current program session
//bCancel = pointer to boolean flag that is "true" whenever this function should be exited as soon as possible (ex: if the program is ending)
/**
 * @brief turn boat to desired compass heading (fHeading)
 * 
 * @param fHeading the desired heading to turn to in degrees.
 * @param pThrusters void pointer to previously created Thruster object for driving the propeller(s).
 * @param command_mutex mutex controlling access to propellers.
 * @param lastNetworkCommandTimeMS pointer to time in milliseconds since program started that the last network command was issued.
 * @param pShipLog pointer to ShipLog object that is used for recording program errors and some data for the current program session.
 * @param bCancel pointer to boolean flag that is "true" whenever this function should be exited as soon as possible (ex: if the program is ending).
 * @param nPriority the priority level of the calling thread that requested this command
 */
 void Navigation::TurnToCompassHeading(float fHeading,void *pThrusters,pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, int nPriority) {//turn boat to desired heading
	float MAX_TURN_SPEED = 5;
	const float CLOSE_ENOUGH_DEG = 10;//try to get heading to match up within this amount
	float fSpeedLeft=0, fSpeedRight=0;//left and right thruster speeds (start at slow speed)
	char sMsg[256];
	Thruster *pThrust = (Thruster *)pThrusters;
	bool bAirBoat = false;
	if (pThrust->isAirBoat()) {
		MAX_TURN_SPEED = MAX_RECOMMENDED_AIRPROP_SPEED;
		bAirBoat = true;
	}
	MAX_TURN_SPEED = min(MAX_TURN_SPEED,(float)this->m_dMaxSpeed);
	if (MAX_TURN_SPEED<=0) {
		return;//must  be in lower power mode, just exit without doing anything
	}
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (!m_imu) {
		pLog->LogEntry((char *)"No compass data yet.\n",true);
		return;//no compass data yet
	}
	if (fHeading>360) fHeading-=360;
	else if (fHeading<0) fHeading+=360;

	CheckPriority(nPriority);//check to see if there are no other higher priority threads trying to execute a navigation command at the same time
	
	//test
	sprintf(sMsg,"Turning to heading = %.1f\n",fHeading);
	pLog->LogEntry(sMsg,true);
	//end test

	
	//initially stop before doing any turning
	
	while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
		usleep(1000000);//delay executing thruster actions after recent network commands
	}
	while (m_imuData.sample_time_sec==0.0&&!*bCancel) {
		//wait for a valid sample of heading data
		usleep(10000);
	}
	CheckPriority(nPriority);//check to see if there are no other higher priority threads trying to execute a navigation command at the same time
	pthread_mutex_lock(command_mutex);
	pThrust->Stop();
	pthread_mutex_unlock(command_mutex);
	double dHeadingDif = ComputeHeadingDif(m_imuData.heading,(double)fHeading);
	double dEstimatedTimeLeft = dHeadingDif / m_dFilteredYawRate;//estimated time left in seconds to achieve desired heading (could be negative if we're yawing in the wrong direction)
	while (fabs(dHeadingDif)>CLOSE_ENOUGH_DEG&&!*bCancel) {
		if (dEstimatedTimeLeft<0) {//rotating in wrong direction
			IncrementTurningSpeed(fSpeedLeft,fSpeedRight,MAX_TURN_SPEED,dHeadingDif>0);
		}
		else {//set speed of turn to be proportional to the expected amount of time remaining
			if (dHeadingDif>0) {//need to turn clockwise
				fSpeedLeft = (dEstimatedTimeLeft - 5) / 50 * MAX_TURN_SPEED;
				if (fSpeedLeft>MAX_TURN_SPEED) fSpeedLeft = MAX_TURN_SPEED;
				if (fSpeedLeft<0) fSpeedLeft=0;//already rotating in right direction, just turn off thruster 
				fSpeedRight = -fSpeedLeft;
				if (bAirBoat&&fSpeedRight<0) {
					fSpeedRight = 0;
				}
			}
			else {//need to turn counter_clockwise
				fSpeedRight = (dEstimatedTimeLeft - 5) / 50 * MAX_TURN_SPEED;
				if (fSpeedRight>MAX_TURN_SPEED) fSpeedRight = MAX_TURN_SPEED;
				if (fSpeedRight<0) fSpeedRight=0;//already rotating in right direction, just turn off thruster 
				fSpeedLeft = -fSpeedRight;
				if (bAirBoat&&fSpeedLeft<0) {
					fSpeedLeft = 0;
				}
			}
		}
		while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
			usleep(1000000);//delay executing thruster actions after recent network commands
		}
		CheckPriority(nPriority);//check to see if there are no other higher priority threads trying to execute a navigation command at the same time
		pthread_mutex_lock(command_mutex);
		//test
		//sprintf(sMsg,"fSpeedLeft = %.1f, fSpeedRight = %.1f\n",fSpeedLeft,fSpeedRight);
		//pLog->LogEntry(sMsg,true);
		//end test
		/*if (pThrust->isAirBoat()) {
			//minimum required turning speed for airboat (+/- 2)
			if (fSpeedLeft<0&&fSpeedLeft>-2) {
				fSpeedLeft=-2;
			}
			else if (fSpeedLeft>0&&fSpeedLeft<2) {
				fSpeedLeft=2;
			}
			if (fSpeedRight<0&&fSpeedRight>-2) {
				fSpeedRight=-2;
			}
			else if (fSpeedRight>0&&fSpeedRight<2) {
				fSpeedRight=2;
			}
		}*/
		pThrust->SetLeftRightSpeed(fSpeedLeft, fSpeedRight);
		pthread_mutex_unlock(command_mutex);
		usleep(100000);//loop every 100 ms
		dHeadingDif = ComputeHeadingDif(m_imuData.heading,(double)fHeading);
		//test
		//sprintf(sMsg,"heading=%.1f, dif = %.1f\n",m_imuData.heading,dHeadingDif);
		//pLog->LogEntry(sMsg,true);
		//end test
		dEstimatedTimeLeft = dHeadingDif / m_dFilteredYawRate;
		//test
		printf("dHeadingDif = %.1f, m_dFilteredYawRate = %.1f, dEstimatedTimeLeft = %.1f\n",dHeadingDif,m_dFilteredYawRate,dEstimatedTimeLeft);
		//end test
	}
	while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
		usleep(1000000);//delay executing thruster actions after recent network commands
	}
	pthread_mutex_lock(command_mutex);
	pThrust->Stop();//stop thrusters after desired heading is achieved
	pthread_mutex_unlock(command_mutex);
	m_uiLastCompassCheckTime=millis();
}

//isDataReady: return true if compass data is available
bool Navigation::isDataReady() {//make sure at least one sample of buffered compass data is available
	if (!m_compassBuf[0]) {
		return false;
	}
	return true;
}

//IncrementTurningSpeed: function used to increment turning speed
//fSpeedLeft = speed of left thruster
//fSpeedRight = speed of right thruster
//fMaxTurnSpeed = maximum allowed turning speed
//bClockWise = true if we are incrementing turning speed in a clockwise direction
void Navigation::IncrementTurningSpeed(float &fSpeedLeft,float &fSpeedRight,float fMaxTurnSpeed,bool bClockWise) {
	if (bClockWise) {
		if (fSpeedLeft==0) {
			fSpeedLeft=1;
		}
		else fSpeedLeft+=.1;
		if (fSpeedLeft>fMaxTurnSpeed) {
			fSpeedLeft=fMaxTurnSpeed;
		}
		fSpeedRight=-fSpeedLeft;
	}
	else {//going counter-clockwise
		if (fSpeedRight==0) {
			fSpeedRight=1;
		}
		else fSpeedRight+=.1;
		if (fSpeedRight>fMaxTurnSpeed) {
			fSpeedRight=fMaxTurnSpeed;
		}
		fSpeedLeft=-fSpeedRight;
	}
}


/**
 * @brief drive boat forward for the specified time in seconds
 * 
 * @param nTotalTimeSeconds the total length of time in seconds that the boat will be driven forward
 * @param fMaxSpeed the desired maximum speed for the thrusters
 * @param fHeadingDirection the target direction in which we are to move forward (corresponds to pointing direction of boat, actual track covered by vary depending on wind and current)
 * @param pThrusters void pointer to previously created Thruster object for driving the propeller(s)
 * @param command_mutex mutex controlling access to propellers
 * @param lastNetworkCommandTimeMS time in milliseconds since program started that the last network command was issued
 * @param pShipLog pointer to ShipLog object that is used for recording program errors and some data for the current program session
 * @param bCancel pointer to boolean variable that is "true" if this function should be exited as soon as possible
 * @param bStopWhenDone true if the thrusters should be stopped after driving has completed (i.e. after nTotalTimeSeconds has elapsed)
 * @param nPriority the priority level of the calling thread that requested this command
 */
 void Navigation::DriveForwardForTime(int nTotalTimeSeconds, float fMaxSpeed, float fHeadingDirection, void *pThrusters, pthread_mutex_t *command_mutex, 
									 unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, bool bStopWhenDone, int nPriority) {
	Thruster *pThrust = (Thruster *)pThrusters;
	if (m_dMaxSpeed<=0) {
		//lower power mode, don't do anything
		return;
	}
	if (pThrust->isAirBoat()) {
		DriverAirboatForwardForTime(nTotalTimeSeconds,fMaxSpeed,fHeadingDirection,pThrusters,command_mutex,
			lastNetworkCommandTimeMS, pShipLog, bCancel, bStopWhenDone, nPriority);
		return;
	}
	ShipLog *pLog = (ShipLog *)pShipLog;
	unsigned int ui_startTime = millis();
	unsigned int ui_endTime = ui_startTime + nTotalTimeSeconds*1000;
	char sMsg[256];
	if (pLog!=nullptr) {
		//test
		sprintf(sMsg,"Driving forward for %d seconds.\n",nTotalTimeSeconds);
		pLog->LogEntry((char *)sMsg,true);
		//end test
	}
	if (lastNetworkCommandTimeMS!=nullptr) {
		while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
			usleep(1000000);//delay executing thruster actions after recent network commands
		}
	}
	CheckPriority(nPriority);
	if (command_mutex!=nullptr) {
		pthread_mutex_lock(command_mutex);
	}
	//pThrust->SetSpeed(fSpeed);
	float fLSpeed = pThrust->GetLSpeed();//speed of left propeller
	float fRSpeed = pThrust->GetRSpeed();//speed of right propeller
	if (command_mutex!=nullptr) {
		pthread_mutex_unlock(command_mutex);
	}
	double dMilliSecondsRemaining = ((double)ui_endTime) - ((double)millis());
	while (dMilliSecondsRemaining>0&&!*bCancel) {
		usleep(100000);//pause thread for 0.1 seconds
		dMilliSecondsRemaining = ((double)ui_endTime) - ((double)millis());
		if (dMilliSecondsRemaining>0) {
			fLSpeed+=0.1;
			fRSpeed+=0.1;
			if (fLSpeed>fMaxSpeed) {//check maximum speed specified in function call
				fLSpeed=fMaxSpeed;
			}
			else if (fLSpeed>m_dMaxSpeed) {//check maximum speed dictated by program settings & battery voltage
				fLSpeed = (float)m_dMaxSpeed;
			}
			if (fRSpeed>fMaxSpeed) {//check maximum speed specified in function call
				fRSpeed=fMaxSpeed;
			}
			else if (fRSpeed>m_dMaxSpeed) {//check maximum speed dictated by program settings & battery voltage
				fRSpeed=(float)m_dMaxSpeed;
			}
			float fHeadingError = m_imuData.heading - fHeadingDirection;
			TrimSpeeds(fLSpeed,fRSpeed,fHeadingError,fHeadingDirection,pShipLog);
			if (lastNetworkCommandTimeMS!=nullptr) {
				while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
					usleep(1000000);//delay executing thruster actions after recent network commands
				}
			}
			CheckPriority(nPriority);
			if (command_mutex!=nullptr) {
				pthread_mutex_lock(command_mutex);
			}
			pThrust->SetLeftRightSpeed(fLSpeed, fRSpeed);
			if (command_mutex!=nullptr) {
				pthread_mutex_unlock(command_mutex);
			}
		}
	}
	if (lastNetworkCommandTimeMS!=nullptr) {
		while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
			usleep(1000000);//delay executing thruster actions after recent network commands
		}
	}
	if (bStopWhenDone) {
		CheckPriority(nPriority);
		if (command_mutex!=nullptr) {
			pthread_mutex_lock(command_mutex);
		}
		pThrust->Stop();
		if (command_mutex!=nullptr) {
			pthread_mutex_unlock(command_mutex);
		}
	}
}

//GetStatusLogText: returns text corresponding to the ship's current navigation status, calling application is responsible for deleting the returned text (if non-null)
char *Navigation::GetStatusLogText() {
	char tempBuf[256];
	if (m_imuData.heading==0&&m_imuData.mag_temperature==0) {//compass data not ready yet, use N.A. for these values
		sprintf(tempBuf,"pos = %.6f, %.6f; speed = %.1f kts; track = %.1f deg; heading = N.A. deg; temp = N.A. deg C",
		m_dLatitude, m_dLongitude, m_dSpeed*METERS_PER_SEC_TO_KTS, m_dTrack);
	}
	else {
		double dInteriorTemp = (m_imuData.mag_temperature + m_imuData.acc_gyro_temperature)/2;//average of mag and acc/gyro temperatures
		sprintf(tempBuf,"pos = %.6f, %.6f; speed = %.1f kts; track = %.1f deg; heading = %.1f deg; temp = %.1f deg C",
			m_dLatitude, m_dLongitude, m_dSpeed*METERS_PER_SEC_TO_KTS, m_dTrack, m_imuData.heading, dInteriorTemp);
	}
	int nStrLength = strlen(tempBuf);
	char *retval = new char[nStrLength+1];
	strcpy(retval,tempBuf);
	return retval;
}

/**
 * @brief drive boat to the specified GPS location.
 * 
 * @param dLatitude the latitude in degrees of the GPS destination.
 * @param dLongitude the longitude in degrees of the GPS destination.
 * @param pThrusters void pointer to previously created Thruster object for driving the propeller(s).
 * @param command_mutex mutex controlling access to propellers.
 * @param lastNetworkCommandTimeMS time in milliseconds since program started that the last network command was issued.
 * @param pShipLog pointer to ShipLog object that is used for recording program errors and some data for the current program session.
 * @param nInterpAmount amount to interpolate (straight-line) between current point and desired destination (use 1 for no interpolation).
 * @param bCancel pointer to boolean flag that controls when to cancel drivign to the location.
 * @param nPriority the priority level of the calling thread that requested this command.
 */
 void Navigation::DriveToLocation(double dLatitude, double dLongitude, void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, int nInterpAmount, bool *bCancel, int nPriority) {
	const float MAX_ALLOWED_HEADING_ERROR = 20;//need to do a full stop turn when the course of the GPS track deviates more than this from the desired track
	const unsigned int COMPASS_REORIENT_TIME = 60000;//minimum number of ms to go without doing a full magnetic compass check 
	double dMaxSpeed = m_dMaxSpeed;//normally the maximum speed will be MAX_THRUSTER_SPEED, except when the boat gets close to its destination, or when battery charge is getting low, then the speed is reduced
	if (dMaxSpeed<=0.0) {
		//must be in low power mode, not enough power to drive anywhere
		return;
	}
	char sMsg[256];
	ShipLog *pLog = (ShipLog *)pShipLog;
	//test
	sprintf(sMsg,"driving to location: %.6f, %.6f\n",dLatitude,dLongitude);
	pLog->LogEntry(sMsg,true);
	//end test

	while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
		usleep(1000000);//delay start of driving procedure after recent network commands
	}
	while (!m_bGotValidGPSData&&!*bCancel) {//don't have valid GPS data yet...
		pLog->LogEntry((char *)"waiting for valid GPS data...\n",true);
		usleep(1000000);
	}
	if (*bCancel==true) return;
	
	double dDistToDestination = Navigation::ComputeDistBetweenPts(dLatitude, dLongitude, m_dLatitude, m_dLongitude);//use GPS locations of 2 points to get the distance between 2 points 
	CheckPriority(nPriority);
	if (nInterpAmount>1) {
		ResetNavSamples();//reset the buffer that keeps track of historical gps locations & compass headings
		if (dDistToDestination<500) {//make sure that we are not interpolating to sub-destinations that are less than 50 m from one another
			nInterpAmount = (int)(dDistToDestination/50);
			if (nInterpAmount<1) nInterpAmount=1;
		}
		double dLatitudeIncrement = (dLatitude - this->m_dLatitude) / nInterpAmount;
		double dLongitudeIncrement = (dLongitude - this->m_dLongitude) / nInterpAmount;
		double dStartLatitude = this->m_dLatitude;
		double dStartLongitude = this->m_dLongitude;
		for (int i=0;i<nInterpAmount;i++) {
			if (*bCancel) {
				break;
			}
			CheckPriority(nPriority);
			DriveToLocation(dStartLatitude + (i+1)*dLatitudeIncrement, dStartLongitude + (i+1)*dLongitudeIncrement,
				pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, 1, bCancel, nPriority);
		}
		return;
	}

	Thruster *pThrust = (Thruster *)pThrusters;
	double dDistToDest=0.0;
	
	double dInitialHeading = ComputeHeadingAndDistToDestination(dLatitude,dLongitude,dDistToDest);//use current GPS location to get initial heading and distance to destination
	TurnToCompassHeading((float)dInitialHeading, pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, bCancel, nPriority);//turn boat to desired heading
	unsigned int uiStartTime = millis();
	float fLSpeed=max(pThrust->GetLSpeed(),(float)2);//get the current speed of the left propeller 
	float fRSpeed=max(pThrust->GetRSpeed(),(float)2);//get the current speed of the right propeller
	float fAirSpeed = max(pThrust->GetAirSpeed(),(float)2);//get speed of air propeller (if used)
	float fAirRudderAngle = 0;//the angle of the air rudder (if used)
	while (dDistToDest>CLOSE_ENOUGH_M&&!*bCancel) {
		dMaxSpeed = m_dMaxSpeed;
		/*if (dDistToDest<25) {//getting close to destination, make sure the boat is not going too fast
			if (m_dMaxSpeed>(MAX_THRUSTER_SPEED/2)) {
				m_dMaxSpeed = MAX_THRUSTER_SPEED/2;
			}
		}*/
		if (pThrust->isAirBoat()) {
			fAirSpeed+=.1;
			if (fAirSpeed>dMaxSpeed) {
				fAirSpeed = dMaxSpeed;
			}
		}
		else {
			fLSpeed+=.1;
			if (fLSpeed>dMaxSpeed) {
				fLSpeed=dMaxSpeed;
			}
			fRSpeed+=.1;
			if (fRSpeed>dMaxSpeed) {
				fRSpeed=dMaxSpeed;
			}
		}
		
		
		unsigned int uiTimeNow = millis();
		AddNavSample(uiTimeNow);//add current navigation data to the buffer of samples
		if (isBoatStuck()&&dMaxSpeed>0) {
			if (pThrust->isAirBoat()) {
				float fRandomAngle = TurnToRandomAngle(pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, bCancel, nPriority);
				sprintf(sMsg,"Boat is stuck, turning to random angle of %.1f deg.\n",fRandomAngle);
				pLog->LogEntry(sMsg,true);
				this->DriveForwardForTime(10,MAX_RECOMMENDED_AIRPROP_SPEED, fRandomAngle, pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, bCancel, true, nPriority);
			}
			else {
				CheckPriority(nPriority);
				ExecuteRandomThrust(pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, bCancel);//execute a short burst of random propeller thrust to try to become un-stuck
			}
			ResetNavSamples();//reset the buffer that keeps track of historical gps locations & compass headings
		}
		double dDesiredHeading = ComputeHeadingAndDistToDestination(dLatitude,dLongitude,dDistToDest);//use current GPS location to get heading and distance to destination
		float fHeadingError = GetHeadingError(dDesiredHeading, pShipLog);//use mix of current compass reading and historical compass and GPS data to determine the current heading error of the boat
		if (fabs(fHeadingError)>MAX_ALLOWED_HEADING_ERROR&&((uiTimeNow - m_uiLastCompassCheckTime)>COMPASS_REORIENT_TIME)) {
			float fDesiredCompassHeading = m_imuData.heading - fHeadingError;//note desired compass heading for the boat can in general differ from the desired GPS track due to wind and water currents
			TurnToCompassHeading(fDesiredCompassHeading, pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, bCancel,nPriority);//turn boat to desired heading to correct GPS track
			fLSpeed=2;
			fRSpeed=2;
			fAirSpeed=2;
		}
		else {//heading is close enough so that just a minor trimming of direction can be used
			CheckPriority(nPriority);
			if (pThrust->isAirBoat()) {//air boat
				TrimAirRudder(fAirRudderAngle,fHeadingError,(float)dDesiredHeading,pShipLog);	
			}
			else {
				TrimSpeeds(fLSpeed,fRSpeed,fHeadingError,(float)dDesiredHeading,pShipLog);
			}
		}
		while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
			usleep(1000000);//delay executing thruster actions after recent network commands
		}
		CheckPriority(nPriority);
		pthread_mutex_lock(command_mutex);
		if (pThrust->isAirBoat()) {
			pThrust->SetAirPropSpeedAndRudderAngle(fAirSpeed,fAirRudderAngle);
		}
		else {
			pThrust->SetLeftRightSpeed(fLSpeed,fRSpeed);
		}
		pthread_mutex_unlock(command_mutex);
		usleep(100000);//pause thread for 100 ms
		dDesiredHeading = ComputeHeadingAndDistToDestination(dLatitude,dLongitude,dDistToDest);//use current GPS location to get heading and distance to destination
	}
	//must have gotten to destination (or within CLOSE_ENOUGH_M)
	while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
		usleep(1000000);//delay executing thruster actions after recent network commands
	}
	CheckPriority(nPriority);
	pthread_mutex_lock(command_mutex);
	pThrust->Stop();
	pthread_mutex_unlock(command_mutex);
}

//ComputeHeadingAndDistToDestination: use current GPS location to get initial heading to destination (assumes distance to destination is small in relation to the radius of the earth)
//dLatitudeDeg = latitude of the destination in degrees
//dLongitudeDeg = longitude of the destination in degrees
//dDistToDest = return value of distance to destination in m
//returns directional heading in degrees (0 to 360) to get to destination
double Navigation::ComputeHeadingAndDistToDestination(double dLatitudeDeg, double dLongitudeDeg, double &dDistToDest) {
	//#define EARTH_RADIUS_EQUATOR_M 6378150.0//radius of Earth at the equator in m
	//#define EARTH_RADIUS_POLE_M 6356890.0//radius of Earth at the poles in m
	double dLatitudeRad = dLatitudeDeg*M_PI / 180.0;//latitude in radians
	double dLongitudeRad = dLongitudeDeg*M_PI / 180.0;//longitude in radians
	double dNorthSouthRadius = cos(dLatitudeRad)*EARTH_RADIUS_EQUATOR_M + sin(fabs(dLatitudeRad))*EARTH_RADIUS_POLE_M;
	double dEastWestRadius = cos(dLatitudeRad)*dNorthSouthRadius;
	double dLatitudeDif = dLatitudeDeg - m_dLatitude;//difference between desired latitude and current latitude
	double dLongitudeDif = dLongitudeDeg - m_dLongitude;//difference between desired longitude and current longitude in radians
	//following might be necessary near the International Date Line (i.e. +/- 180 deg longitude)
	if (dLongitudeDif>180) {
		dLongitudeDif-=360;
	}
	else if (dLongitudeDif<-180) {
		dLongitudeDif+=360;
	}
	double dLatitudeDifRad = dLatitudeDif * M_PI / 180.0;//latitude difference in radians
	double dLongitudeDifRad = dLongitudeDif * M_PI / 180.0;//longitude difference in radians
	double dNorthSouthComponent = dNorthSouthRadius * dLatitudeDifRad;//north-south component of distance to destination in m
	double dEastWestComponent = dEastWestRadius * dLongitudeDifRad;//east-west component of distance to destination in m
	double dHeadingDirectionDeg = atan2(dEastWestComponent,dNorthSouthComponent)*180/M_PI;//directional heading to destination in degrees
	if (dHeadingDirectionDeg<0) dHeadingDirectionDeg+=360.0;
	dDistToDest = sqrt(dNorthSouthComponent*dNorthSouthComponent + dEastWestComponent*dEastWestComponent);
	//test
	//printf("Heading to dest = %.1f deg, dist left = %.0f m\n",dHeadingDirectionDeg,dDistToDest);
	//end test
	return dHeadingDirectionDeg;
}

//TrimSpeeds: adjust speeds of propellers in order to fine-tune the direction that the boat is moving while at speed in the forward direction
//fLSpeed: speed of left propeller that will be adjusted
//fRSpeed: speed of right propeller that will be adjusted
//fHeadingError: the amount (in degrees) that the current heading of the boat differs from the ideal heading
//fTargetHeading: the desired heading direction in degrees (0 to 360)
//pShipLog = pointer to ShipLog object that is used for recording program errors and some data for the current program session
void Navigation::TrimSpeeds(float &fLSpeed, float &fRSpeed, float fHeadingError, float fTargetHeading, void *pShipLog) {
	//make sure we don't have more than 180 degrees of heading error
	const float YAW_FACTOR = 5;//relative factor that takes into consideration the importance of the estimated yaw rate in determining thruster power
	const float MAX_TRIM = 1;//maximum amount to trim thruster in each iteration of this function
	unsigned int uiCurrentTime = millis();
	//test
	//printf("heading = %.1f, error = %.1f\n",m_imuData.heading,fHeadingError);
	//end test
	if ((uiCurrentTime - m_uiPreviousTrimTime)>2000) {//more than 2 seconds has elapsed since last trimming of thruster power, safer to reset speeds to zero
		fLSpeed=0;
		fRSpeed=0;
		m_uiPreviousTrimTime = uiCurrentTime;
		return;
	}
	
	m_uiPreviousTrimTime = uiCurrentTime;
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (fHeadingError>180) fHeadingError-=360;
	else if (fHeadingError<-180) fHeadingError+=360;
	//adjust fHeadingError for the current yawing motion of the boat and the integrated heading error over the last COMPASS_BUFSIZE samples (~ 2 seconds)
	double dIntegratedHeadingError = 0;//0.5*CalculateIntegratedHeadingError((double)fTargetHeading, pShipLog);//get integral of heading error over the last COMPASS_BUFSIZE samples (~ 2 seconds)
	//test	
	//char sMsg[256];
	//sprintf(sMsg,"heading error = %.1f\n, yaw rate = %.2f\n",fHeadingError,m_dYawRate);
	//pLog->LogEntry(sMsg,false);
	//end test
	fHeadingError+=((float)(m_dYawRate*YAW_FACTOR));
	fHeadingError+=(float)dIntegratedHeadingError;
	//printf("fHeadingError = %.1f\n",fHeadingError);
	float fMaxThrusterSpeed = max(fLSpeed,fRSpeed);//speed of fastest spinning thruster
	float fTrimAmount = 0;
	//set trim amount based on heading error, > 0 means turn boat CCW, < 0 means turn boat CW
	if (fHeadingError>0) {//need to turn the boat CCW, so decrease speed of left propeller or increase speed of right propeller
		fTrimAmount = fMaxThrusterSpeed * fHeadingError / 45;
		if (fTrimAmount>MAX_TRIM) {
			fTrimAmount=MAX_TRIM;
		}
		if (fLSpeed<=fRSpeed) {//decrease speed of left propeller to turn boat CCW
			fLSpeed = fLSpeed - fTrimAmount;
			if (fLSpeed<0) {
				fLSpeed=0;
			}
		}
		else {//fLSpeed>fRSpeed --> increase speed of right propeller to turn boat CCW
			fRSpeed = fRSpeed + fTrimAmount;
			if (fRSpeed>m_dMaxSpeed) {
				fRSpeed = (float)m_dMaxSpeed;
			}
		}
	}
	else if (fHeadingError<0) {//need to turn the boat CW, so decrease speed of right propeller or increase speed of left propeller
		fTrimAmount = -fMaxThrusterSpeed * fHeadingError / 45;
		if (fTrimAmount>MAX_TRIM) {
			fTrimAmount=MAX_TRIM;
		}
		if (fRSpeed<=fLSpeed) {//decrease speed of right propeller to turn boat CW
			fRSpeed = fRSpeed - fTrimAmount;
			if (fRSpeed<0) {
				fRSpeed=0;
			}
		}
		else {//fRSpeed>fLSpeed --> incrase speed of left propeller to turn boat CW
			fLSpeed = fLSpeed + fTrimAmount;
			if (fLSpeed>m_dMaxSpeed) {
				fLSpeed = (float)m_dMaxSpeed;
			}
		}
	}
}

/**
 * @brief hold the current GPS location for the specified period of time (in seconds).
 * 
 * @param nTotalTimeSeconds length of time to hold the current GPS location in seconds.
 * @param pThrusters void pointer to previously created Thruster object for driving the propeller(s).
 * @param command_mutex mutex controlling access to propellers.
 * @param lastNetworkCommandTimeMS time in milliseconds since program started that the last network command was issued.
 * @param pShipLog pointer to ShipLog object that is used for recording program errors and some data for the current program session.
 * @param bCancel pointer to boolean variable that is "true" whenever this function should be canceled as soon as possible.
 * @param nPriority the priority level of the calling thread that requested this command.
 */
 void Navigation::HoldCurrentPosition(int nTotalTimeSeconds, void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, int nPriority) {
	//holding cuurent position 
	double dInitialLatitude = m_dLatitude;
	double dInitialLongitude = m_dLongitude;
	double dDistFromInitial =0.0;//distance from initial location in m
	unsigned int uiInitialTime = millis();//initial time in ms
	while ((millis() - uiInitialTime)<(nTotalTimeSeconds*1000)&&!*bCancel) {
		double dHeadingToStart = ComputeHeadingAndDistToDestination(dInitialLatitude, dInitialLongitude, dDistFromInitial);
		if (dDistFromInitial>CLOSE_ENOUGH_M) {
			DriveToLocation(dInitialLatitude, dInitialLongitude, pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, 1, bCancel, nPriority);
		}
		usleep(1000000);
	}
}

void Navigation::SetPosition(double dLatitude, double dLongitude) {//set latitude and longitude for unit testing purposes
	m_dLatitude = dLatitude;
	m_dLongitude = dLongitude;
}

void Navigation::ExecuteRandomThrust(void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel) {//execute a short burst of random propeller thrust to try to become un-stuck
	char sMsg[256];
	ShipLog *pLog = (ShipLog *)pShipLog;
	float fLSpeed=(float)(-m_dMaxSpeed/2 + ((float)rand()) / RAND_MAX * 2 * m_dMaxSpeed);//assign random speed to left thruster
	float fRSpeed=(float)(-m_dMaxSpeed/2 + ((float)rand()) / RAND_MAX * 2 * m_dMaxSpeed);//assign random speed to right thruster
	Thruster *pThrust = (Thruster *)pThrusters;
	pthread_mutex_lock(command_mutex);
	//test
	sprintf(sMsg,"random extraction: left speed = %.1f, right speed = %.1f\n",fLSpeed,fRSpeed);
	pLog->LogEntry(sMsg,true);
	//end test
	pThrust->SetLeftRightSpeed(fLSpeed, fRSpeed);
	pthread_mutex_unlock(command_mutex);
	//allow the random thrust to occur for 10 seconds
	for (int i=0;i<10;i++) {
		if (*bCancel) break;
		usleep(1000000);
	}
}

void Navigation::ResetNavSamples() {//reset the buffer that keeps track of historical gps locations & compass headings
	m_nSampleIndex=0;
	m_nNumSamples=0;
	m_fEstimatedCompassError=0;
}

void Navigation::AddNavSample(unsigned int uiTimeMS) {//add current navigation data to the buffer of samples
	if (!m_currentGPSData) return;	
	if (!m_imu) return;
	if (!m_navSamples[m_nSampleIndex]) {
		m_navSamples[m_nSampleIndex] = new NAV_DATA;
	}
	m_navSamples[m_nSampleIndex]->dLatitude = m_currentGPSData->dLatitude;//latitude
	m_navSamples[m_nSampleIndex]->dLongitude = m_currentGPSData->dLongitude;//longitude
	m_navSamples[m_nSampleIndex]->dCompassHeading = m_imuData.heading;//compass reading
	m_navSamples[m_nSampleIndex]->uiTimeMS = uiTimeMS;//not the exact time when gps and compass readings were performed, but good enough for purposes of this buffer
	m_nSampleIndex = (m_nSampleIndex+1)%SAMPLE_BUFSIZE;
	if (m_nNumSamples<SAMPLE_BUFSIZE) {
		m_nNumSamples++;
	}
}

bool Navigation::isBoatStuck() {//return true if the nav buffer is full of samples all for the same GPS location
	const double CLOSE_ENOUGH = .000001;//consider latitude & longitude values to be identical if they are within this amount of one another
	if (m_nNumSamples<SAMPLE_BUFSIZE) {
		return false;//not enough samples available in sample buffer
	}
	double dLat = m_navSamples[0]->dLatitude;
	double dLong = m_navSamples[0]->dLongitude;
	for (int i=1;i<SAMPLE_BUFSIZE;i++) {
		if (fabs(m_navSamples[i]->dLatitude-dLat)>CLOSE_ENOUGH) {
			return false;//different latitude, not stuck
		}
		else if (fabs(m_navSamples[i]->dLongitude-dLong)>CLOSE_ENOUGH) {
			return false;//different longitude, not stuck
		}
	}
	//all latitude and longitude values must have been the same
	return true;//must be stuck
}

float Navigation::GetHeadingError(double dDesiredHeading, void *pShipLog) {//use mix of current compass reading and historical compass and GPS data to determine the current heading error of the boat
	float fEstimatedCompassError = 0.0;//(does not work(?)) FindEstimatedCompassError(pShipLog);//look at compass and GPS data for the last SAMPLE_BUFSIZE samples to estimate the amount of compass error; provides an estimate of how far off the compass is from the GPS track
	float fCurrentCompass = m_imuData.heading;//our current uncorrected compass reading
	float fPredictedGPSTrack = fCurrentCompass + fEstimatedCompassError;//the predicted GPS track that we will follow if we stick with the current compass heading
	float fHeadingError = fPredictedGPSTrack - (float)dDesiredHeading;//an estimate of our current error in heading, relative to the desired heading direction (dDesiredHeading)
	if (fHeadingError>180) {
		fHeadingError-=360;
	}
	else if (fHeadingError<-180) {
		fHeadingError+=360;
	}
	return fHeadingError;
}

float Navigation::FindEstimatedCompassError(void *pShipLog) {//look at compass and GPS data for the last SAMPLE_BUFSIZE samples to estimate the amount of compass error; provides an estimate of how far off the compass is from the GPS track
	const double MIN_GPS_DIST = 25;//need to have moved at least this far (in m) in order to estimate compass error
	char sMsg[256];
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (m_nNumSamples<SAMPLE_BUFSIZE) {//not enough GPS samples stored in buffer yet to accurately determine what the compass heading error is
		//test
		sprintf(sMsg, "m_nNumSamples = %d\n",m_nNumSamples);
		pLog->LogEntry(sMsg,false);
		//end test
		return 0;
	}
	int nMostRecentSampleIndex = m_nSampleIndex-1;//index of most recent GPS / compass sample
	if (nMostRecentSampleIndex<0) {
		nMostRecentSampleIndex = SAMPLE_BUFSIZE-1;
	}
	int nFirstSampleIndex = m_nSampleIndex;//index of first GPS / compass sample
	//getting average GPS direction over SAMPLE_BUFSIZE samples is easy, just use starting and ending GPS coordinates
	double dStartLatitude = m_navSamples[nFirstSampleIndex]->dLatitude;
	double dStartLongitude = m_navSamples[nFirstSampleIndex]->dLongitude;
	double dEndLatitude = m_navSamples[nMostRecentSampleIndex]->dLatitude;
	double dEndLongitude = m_navSamples[nMostRecentSampleIndex]->dLongitude;
	double dDist = ComputeDistBetweenPts(dStartLatitude,dStartLongitude,dEndLatitude,dEndLongitude);
	if (dDist<MIN_GPS_DIST) {
		sprintf(sMsg,"dDist = %.1f m\n",dDist);
		pLog->LogEntry(sMsg,false);
		return m_fEstimatedCompassError;//haven't moved far enough to get a good estimate of the compass error, just use the last known value of compass error
	}
	double dNorthSouthDist = dEndLatitude - dStartLatitude;
	double dEastWestDist = dEndLongitude - dStartLongitude;
	double dAvgGPSHeading = atan2(dEastWestDist,dNorthSouthDist)*180.0/M_PI;
	if (dAvgGPSHeading<0) {//convert from -180 to 180 output of atan2 to 0 to 360 output
		dAvgGPSHeading+=360.0;
	}
	//getting average compass direction over SAMPLE_BUFSIZE is trickier, we need to be careful about averaging headings close to north... 
	//see https://en.wikipedia.org/wiki/Mean_of_circular_quantities for a description of this problem:
	//Convert all angles to corresponding points on the unit circle, e.g., {\displaystyle \alpha } \alpha  to {\displaystyle (\cos \alpha ,\sin \alpha )} (\cos \alpha ,\sin \alpha ). That is, convert polar coordinates to Cartesian coordinates. Then compute the arithmetic mean of these points. The resulting point will lie within the unit disk. Convert that point back to polar coordinates. The angle is a reasonable mean of the input angles.
	double dSumCosines = 0.0;//sum of the cosines of all of the headings
	double dSumSines = 0.0;//sum of the sines of all of the headings
	for (int i=0;i<SAMPLE_BUFSIZE;i++) {
		double dCosVal = cos(m_navSamples[i]->dCompassHeading*M_PI/180.0);
		double dSinVal = sin(m_navSamples[i]->dCompassHeading*M_PI/180.0);
		dSumCosines+=dCosVal;
		dSumSines+=dSinVal;
	}
	double dAvgCompassHeading = atan2(dSumSines, dSumCosines)*180.0/M_PI;
	if (dAvgCompassHeading<0) {
		dAvgCompassHeading+=360.0;
	}
	//estimated compass error is just the average gps heading minus the average compass headingh
	float fAvgCompassError = (float)(dAvgGPSHeading - dAvgCompassHeading);
	if (fAvgCompassError<-180) {
		fAvgCompassError+=360;
	}
	else if (fAvgCompassError>180) {
		fAvgCompassError-=360;
	}
	m_fEstimatedCompassError = fAvgCompassError;
	sprintf(sMsg,"compass_err = %.1f\n",m_fEstimatedCompassError);
	pLog->LogEntry(sMsg,false);
	return fAvgCompassError;
}

//ExecuteLeakResponse: special emergency function for getting to shore as quickly as possible
//nLeakResponseType: the type of emergency response to perform
//pShipLog = void pointer to ship's log for logging errors, occasional data, etc.
void Navigation::ExecuteLeakResponse(int nLeakResponseType, void *pShipLog) {
	return;//not implemented yet...
}

//driftedFromEmergencyStop: return true if we have drifted away from the emergency stopping point
bool Navigation::driftedFromEmergencyStop() {
	//not implemented yet
	return false;
}

//ComputeDistBetweenPts: use GPS locations of 2 points to get the distance between those 2 points
//dLatitudeDeg1 = latitude (in degrees) of the 1st point
//dLongitudeDeg1 = longitude (in degrees) of the 2nd point
//dLatitudeDeg2 = latitude (in degrees) of the 2nd point
//dLongitudeDeg2 = longitude (in degrees) of the 2nd point
//returns distance (in m) between the 2 points
double Navigation::ComputeDistBetweenPts(double dLatitudeDeg1, double dLongitudeDeg1, 
										 double dLatitudeDeg2, double dLongitudeDeg2) {
	//#define EARTH_RADIUS_EQUATOR_M 6378150.0//radius of Earth at the equator in m
	//#define EARTH_RADIUS_POLE_M 6356890.0//radius of Earth at the poles in m
	double dLatitudeRad1 = dLatitudeDeg1*M_PI / 180.0;//latitude of 1st point in radians
	double dLongitudeRad1 = dLongitudeDeg1*M_PI / 180.0;//longitude of 1st point in radians
	double dLatitudeRad2 = dLatitudeDeg2*M_PI / 180.0;//latitude of 2nd point in radians
	double dLongitudeRad2 = dLongitudeDeg2*M_PI / 180.0;//longitude of 2nd point in radians
	double dAvgLatitude = (dLatitudeRad1 + dLatitudeRad2)/2;
	double dNorthSouthRadius = cos(dAvgLatitude)*EARTH_RADIUS_EQUATOR_M + sin(fabs(dAvgLatitude))*EARTH_RADIUS_POLE_M;
	double dEastWestRadius = cos(dAvgLatitude)*dNorthSouthRadius;
	double dLatitudeDif = dLatitudeDeg2- dLatitudeDeg1;//difference between latitudes in degrees
	double dLongitudeDif = dLongitudeDeg2 - dLongitudeDeg1;//difference between longitudes in degrees
	//following might be necessary near the International Date Line (i.e. +/- 180 deg longitude)
	if (dLongitudeDif>180) {
		dLongitudeDif-=360;
	}
	else if (dLongitudeDif<-180) {
		dLongitudeDif+=360;
	}
	double dLatitudeDifRad = dLatitudeDif * M_PI / 180.0;//latitude difference in radians
	double dLongitudeDifRad = dLongitudeDif * M_PI / 180.0;//longitude difference in radians
	double dNorthSouthComponent = dNorthSouthRadius * dLatitudeDifRad;//north-south component of distance to destination in m
	double dEastWestComponent = dEastWestRadius * dLongitudeDifRad;//east-west component of distance to destination in m
	double dDist = sqrt(dNorthSouthComponent*dNorthSouthComponent + dEastWestComponent*dEastWestComponent);
	return dDist;
}

COMPASS_ADJUSTMENT * Navigation::LoadCompassAdjustments() {//get compass adjustments (if any) from compass_cal.txt file
	if (filedata::getFileLength((char *)"compass_cal.txt")<=0) {//unable to open compass calibration file, or file does not have any info in it
		return nullptr;
	}
	filedata calFile((char *)"compass_cal.txt");
	int nNumCalPts = calFile.getInteger((char *)"[heading_adjustments]",(char *)"num_angles");
	if (nNumCalPts<=0) {
		return nullptr;
	}
	COMPASS_ADJUSTMENT *pAdjust = new COMPASS_ADJUSTMENT;
	pAdjust->nNumVals = nNumCalPts;
	pAdjust->actual_compass_vals = new double[nNumCalPts];
	pAdjust->reported_compass_vals = new double[nNumCalPts];
	calFile.getDouble((char *)"[heading_adjustments]",(char *)"reported_angles",nNumCalPts,pAdjust->reported_compass_vals);
	calFile.getDouble((char *)"[heading_adjustments]",(char *)"actual_angles",nNumCalPts,pAdjust->actual_compass_vals);
	return pAdjust;
}

double Navigation::AdjustUsingCompassCal(double dUnadjustedHeading) {//ajust the compass heading using calibration data stored in m_compassAdjustment (if available)
	if (m_compassAdjustment==nullptr) {
		return dUnadjustedHeading;
	}
	if (m_compassAdjustment->nNumVals<1) {
		return dUnadjustedHeading;
	}
	//make sure input is between 0 and 360
	if (dUnadjustedHeading<0) {
		dUnadjustedHeading+=360;
	}
	else if (dUnadjustedHeading>=360) {
		dUnadjustedHeading-=360;
	}
	double dAdjustedHeading=0.0;
	if (m_compassAdjustment->nNumVals==1) {//special case of just one calibration point, apply simple offset to output
		double dOffset = m_compassAdjustment->actual_compass_vals[0] - m_compassAdjustment->reported_compass_vals[0];
		dAdjustedHeading = dUnadjustedHeading + dOffset;
	}
	else {//need to interpolate between closest 2 calibration points
		int nLowerIndex = GetClosestHeadingCalIndexBelow(dUnadjustedHeading);
		int nUpperIndex = GetClosestHeadingCalIndexAbove(dUnadjustedHeading);
		double dReported1 = m_compassAdjustment->reported_compass_vals[nLowerIndex];
		double dReported2 = m_compassAdjustment->reported_compass_vals[nUpperIndex];
		double dReportedRange = ComputeHeadingDif(dReported1, dReported2);
		double dActual1 = m_compassAdjustment->actual_compass_vals[nLowerIndex];
		double dActual2 = m_compassAdjustment->actual_compass_vals[nUpperIndex];
		double dActualRange = ComputeHeadingDif(dActual1, dActual2);
		double dCalFactor = ComputeHeadingDif(dReported1, dUnadjustedHeading) / dReportedRange;//amount (relative to 1.0 that input angle is within the particular calibration region)
		double dOffset1 = ComputeHeadingDif(dReported1, dActual1);
		double dOffset2 = ComputeHeadingDif(dReported2, dActual2);
		double dOffset = dCalFactor * dOffset2 + (1.0 - dCalFactor) * dOffset1;
		dAdjustedHeading = dUnadjustedHeading + dOffset;
	}
	//make sure that adjusted heading is within 0 to 360 range
	if (dAdjustedHeading<0) {
		dAdjustedHeading+=360;
	}
	else if (dAdjustedHeading>=360) {
		dAdjustedHeading-=360;
	}
	return dAdjustedHeading;
}

int Navigation::GetClosestHeadingCalIndexBelow(double dHeading) {//get closest calibration heading that is <= dHeading
	if (m_compassAdjustment==nullptr) {
		return -1;
	}
	else if (m_compassAdjustment->nNumVals<=0) {
		return -1;
	}
	int nMinIndex=-1;
	double dMinDist = 360.0;//initialize to large value
	for (int i=0;i<m_compassAdjustment->nNumVals;i++) {
		double dSep = ComputeHeadingDif(m_compassAdjustment->reported_compass_vals[i], dHeading);
		if (dSep>0&&dSep<dMinDist) {
			dMinDist = dSep;
			nMinIndex = i;
		}
	}
	return nMinIndex;
}

int Navigation::GetClosestHeadingCalIndexAbove(double dHeading) {//get closest calibration heading that is >= dHeading
	if (m_compassAdjustment==nullptr) {
		return -1;
	}
	else if (m_compassAdjustment->nNumVals<=0) {
		return -1;
	}
	int nMinIndex=-1;
	double dMinDist = 360.0;//initialize to large value
	for (int i=0;i<m_compassAdjustment->nNumVals;i++) {
		double dSep = ComputeHeadingDif(dHeading, m_compassAdjustment->reported_compass_vals[i]);
		if (dSep>0&&dSep<dMinDist) {
			dMinDist = dSep;
			nMinIndex = i;
		}
	}
	return nMinIndex;
}

void Navigation::BufferCompassData(double dHeading, unsigned int uiSampleTime) {//buffer compass data so that it can be used later for estimating the rate of yaw
	if (!m_compassBuf[m_nCompassBufIndex]) {
		m_compassBuf[m_nCompassBufIndex] = new COMPASS_SAMPLE;
	}
	m_compassBuf[m_nCompassBufIndex]->dHeading = dHeading;
	m_compassBuf[m_nCompassBufIndex]->uiSampleTime = uiSampleTime;
	m_nCompassBufIndex = (m_nCompassBufIndex+1)%COMPASS_BUFSIZE;
}

double Navigation::ComputeYawRate(bool bApplyFilter) {//function looks at buffered heading data to determine yaw rate
	//look back over the last second of compass data to estimate yaw rate
	int nBufIndex = m_nCompassBufIndex - 1;
	if (nBufIndex<0) nBufIndex+=COMPASS_BUFSIZE;
	if (!m_compassBuf[nBufIndex]) {
		return 0.0;
	}
	unsigned int uiLastSampleTime = m_compassBuf[nBufIndex]->uiSampleTime;
	if (uiLastSampleTime<1000) {
		return 0.0;//not enough samples yet
	}
	unsigned int uiLookBackTime = uiLastSampleTime - 50;//1000;
	if (bApplyFilter) {//look back one second or (COMPASS_BUFSIZE-1) samples (whichever comes first) to get average change in yaw over last second
		uiLookBackTime = uiLastSampleTime - 1000;
	}
	COMPASS_SAMPLE *lastSample = m_compassBuf[nBufIndex];
	unsigned int uiSampleTime = uiLastSampleTime;
	int nLookBackCount=0;
	do {
		nBufIndex = nBufIndex-1;
		nLookBackCount++;
		if (nBufIndex<0) nBufIndex+=COMPASS_BUFSIZE;
		if (!m_compassBuf[nBufIndex]) {
			return 0.0;
		}
		uiSampleTime = m_compassBuf[nBufIndex]->uiSampleTime;
	} while (uiSampleTime>uiLookBackTime&&nLookBackCount<(COMPASS_BUFSIZE-1));
	double dTimeElapsed = (lastSample->uiSampleTime - uiSampleTime) / 1000.0;//number of seconds elapsed between samples in buffer, should be a bit more than one second
	double dHeadingDif = Navigation::ComputeHeadingDif(m_compassBuf[nBufIndex]->dHeading, lastSample->dHeading);
	if (dTimeElapsed>0.0) {
		double dYawRate = dHeadingDif / dTimeElapsed;
		return dYawRate;
	}
	return 0.0;
}

double Navigation::CalculateIntegratedHeadingError(double dTargetHeading, void *pShipLog) {//get integral of heading error over the last COMPASS_BUFSIZE samples (~ 2 seconds)
	//if (!m_compassBuf[m_nCompassBufIndex]) {
	//	m_compassBuf[m_nCompassBufIndex] = new COMPASS_SAMPLE;
	//}
	//m_compassBuf[m_nCompassBufIndex]->dHeading = dHeading;
	//m_compassBuf[m_nCompassBufIndex]->uiSampleTime = uiSampleTime;
	//m_nCompassBufIndex = (m_nCompassBufIndex+1)%COMPASS_BUFSIZE;
	ShipLog *pLog = (ShipLog *)pShipLog;
	char sMsg[256];
	double dIntegration=0.0;//the result of the integration
	int nMostRecentIndex = m_nCompassBufIndex-1;
	if (nMostRecentIndex<0) {
		nMostRecentIndex+=COMPASS_BUFSIZE;
	}
	int i=nMostRecentIndex;
	int j=i-1;
	if (j<0) {
		j+=COMPASS_BUFSIZE;
	}
	do {
		COMPASS_SAMPLE *c1 = m_compassBuf[i];
		COMPASS_SAMPLE *c2 = m_compassBuf[j];
		if (!c1||!c2) {
			break;
		}
		double dAvgHeading = Navigation::ComputeAvgHeading(c1->dHeading,c2->dHeading);//average value of heading between 2 consecutive samples
		double dTimeDifSec = (c1->uiSampleTime - c2->uiSampleTime)/1000.0;
		double dHeadingError = Navigation::ComputeHeadingDif(dTargetHeading, dAvgHeading);
		dIntegration+=(dHeadingError*dTimeDifSec);
		i = j;
		j = i-1;
		if (j<0) {
			j+=COMPASS_BUFSIZE;
		}
	} while (j!=nMostRecentIndex);
	return dIntegration;	
}

double Navigation::ComputeAvgHeading(double dHeading1Deg, double dHeading2Deg) {//return average value of 2 headings in degrees
	double dHeadingDif = Navigation::ComputeHeadingDif(dHeading1Deg, dHeading2Deg);
	double dAvgHeading = dHeading1Deg + dHeadingDif/2;
	if (dAvgHeading>360) {
		dAvgHeading-=360;
	}
	else if (dAvgHeading<0) {
		dAvgHeading+=360;
	}
	return dAvgHeading;
}

/**
 * @brief sets the mode of operation and maximum thruster speed possible
 * 
 * @param nPowerMode one of the following values: LOW_POWER_MODE: in this mode of operation the thrusters are totally shutoff in order to conserve power until the battery can be charged. HALF_POWER_MODE: in this mode of operation the maximum thruster speed is reduced by half in order to conserver power. FULL_POWER_MODE: normal mode of operation, full thruster speed is possible
 * @param dVoltage the current battery voltage, gets saved to the ship's log with a brief message
 * @param pShipLog pointer to ShipLog object that is used for recording program errors and some data for the current program session
 */
void Navigation::SetPowerMode(int nPowerMode,double dVoltage,void *pShipLog) {
	//comment out the following 3 lines to enable switching of power mode (based on battery voltge level)
	//if (1==1) {
	//	return;
	//}
	ShipLog *pLog = (ShipLog *)pShipLog;
	char sMsg[256];
	if (nPowerMode==LOW_POWER_MODE) {
		sprintf(sMsg,"Setting low power mode, voltage = %.2f V\n",dVoltage);
		m_dMaxSpeed=0.0;
		pLog->LogEntry(sMsg,true);
	}
	else if (nPowerMode==HALF_POWER_MODE) {
		sprintf(sMsg,"Setting half power mode, voltage = %.2f V\n",dVoltage);
		m_dMaxSpeed = MAX_THRUSTER_SPEED/2;
		pLog->LogEntry(sMsg,true);
	}
	else if (nPowerMode==FULL_POWER_MODE) {
		sprintf(sMsg,"Setting full power mode, voltage = %.2f V\n",dVoltage);
		m_dMaxSpeed = MAX_THRUSTER_SPEED;
		pLog->LogEntry(sMsg,true);
	}
}

//GetBoatTemp: return the temperature of the interior of the boat (comes from average of magnetometer chip and acc/gyro chip temperature sensors)
double Navigation::GetBoatTemp() {
	return (m_imuData.mag_temperature + m_imuData.acc_gyro_temperature)/2;
}

//GetLatitude: return the current latitude of the boat (as determined by GPS) in degrees
double Navigation::GetLatitude() {
	return m_dLatitude;
}

//GetLongitude: return the current longitude of the boat (as determined by GPS) in degrees 
double Navigation::GetLongitude() {
	return m_dLongitude;
}

/**
 * @brief drive airboat forward for the specified time in seconds
 * 
 * @param nTotalTimeSeconds the total length of time in seconds that the boat will be driven forward.
 * @param fMaxSpeed the desired maximum speed for the thrusters.
 * @param fHeadingDirection the target direction in which we are to move forward (corresponds to pointing direction of boat, actual track covered by vary depending on wind and current).
 * @param pThrusters void pointer to previously created Thruster object for driving the propeller(s).
 * @param command_mutex mutex controlling access to propellers.
 * @param lastNetworkCommandTimeMS time in milliseconds since program started that the last network command was issued.
 * @param pShipLog pointer to ShipLog object that is used for recording program errors and some data for the current program session.
 * @param bCancel pointer to boolean variable that is "true" if this function should be exited as soon as possible.
 * @param bStopWhenDone true if the thrusters should be stopped after driving has completed (i.e. after nTotalTimeSeconds has elapsed).
 * @param nPriority the priority level of the calling thread that requested this command.
 */
void Navigation::DriverAirboatForwardForTime(int nTotalTimeSeconds, float fMaxSpeed, float fHeadingDirection, void *pThrusters, pthread_mutex_t *command_mutex, 
									 unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, bool bStopWhenDone, int nPriority) {
	Thruster *pThrust = (Thruster *)pThrusters;
	ShipLog *pLog = (ShipLog *)pShipLog;
	unsigned int ui_startTime = millis();
	unsigned int ui_endTime = ui_startTime + nTotalTimeSeconds*1000;
	char sMsg[256];
	if (pLog!=nullptr) {
		//test
		sprintf(sMsg,"Driving forward for %d seconds.\n",nTotalTimeSeconds);
		pLog->LogEntry((char *)sMsg,true);
		//end test
	}
	if (lastNetworkCommandTimeMS!=nullptr) {
		while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
			usleep(1000000);//delay executing thruster actions after recent network commands
		}
	}
	CheckPriority(nPriority);
	if (command_mutex!=nullptr) {
		pthread_mutex_lock(command_mutex);
	}
	//pThrust->SetSpeed(fSpeed);
	float fAirSpeed = pThrust->GetAirSpeed();
	float fAirRudderAngle = pThrust->GetAirRudderAngle();//the angle of the air rudder (if used)
	if (command_mutex!=nullptr) {
		pthread_mutex_unlock(command_mutex);
	}
	double dMilliSecondsRemaining = ((double)ui_endTime) - ((double)millis());
	while (dMilliSecondsRemaining>0&&!*bCancel) {
		usleep(100000);//pause thread for 0.1 seconds
		dMilliSecondsRemaining = ((double)ui_endTime) - ((double)millis());
		if (dMilliSecondsRemaining>0) {
			fAirSpeed+=0.1;
			if (fAirSpeed>fMaxSpeed) {//check maximum speed specified in function call
				fAirSpeed=fMaxSpeed;
			}
			float fHeadingError = m_imuData.heading - fHeadingDirection;

			TrimAirRudder(fAirRudderAngle,fHeadingError,fHeadingDirection,pShipLog);	
			if (lastNetworkCommandTimeMS!=nullptr) {
				while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
					usleep(1000000);//delay executing thruster actions after recent network commands
				}
			}
			CheckPriority(nPriority);
			if (command_mutex!=nullptr) {
				pthread_mutex_lock(command_mutex);
			}
			pThrust->SetAirPropSpeedAndRudderAngle(fAirSpeed,fAirRudderAngle);
		
			if (command_mutex!=nullptr) {
				pthread_mutex_unlock(command_mutex);
			}
		}
	}
	if (lastNetworkCommandTimeMS!=nullptr) {
		while ((millis() - *lastNetworkCommandTimeMS)<10000&&!*bCancel) {
			usleep(1000000);//delay executing thruster actions after recent network commands
		}
	}
	if (bStopWhenDone) {
		CheckPriority(nPriority);
		if (command_mutex!=nullptr) {
			pthread_mutex_lock(command_mutex);
		}
		pThrust->Stop();
		if (command_mutex!=nullptr) {
			pthread_mutex_unlock(command_mutex);
		}
	}
}

float Navigation::TurnToRandomAngle(void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, int nPriority) {//turns boat to a random angle
	char sMsg[256];
	ShipLog *pLog = (ShipLog *)pShipLog;
	float fRandomAngle=(float)(360*((float)rand()) / RAND_MAX);//get a random angle between 0 and 360
	Thruster *pThrust = (Thruster *)pThrusters;
	pthread_mutex_lock(command_mutex);
	//test
	sprintf(sMsg,"setting random heading angle: %.1f deg\n",fRandomAngle);
	pLog->LogEntry(sMsg,true);
	//end test
	this->TurnToCompassHeading(fRandomAngle,pThrusters, command_mutex, lastNetworkCommandTimeMS, pShipLog, bCancel, nPriority);
	pthread_mutex_unlock(command_mutex);
}

void Navigation::TrimAirRudder(float &fAirRudderAngle,float fHeadingError,float fDesiredHeading,void *pShipLog) {//fine-tune air rudder angle in order to correct any heading error
	//make sure we don't have more than 180 degrees of heading error
	const float YAW_FACTOR = 5;//relative factor that takes into consideration the importance of the estimated yaw rate in determining thruster power
	const float MAX_TRIM = 2;//maximum amount to trim thruster (in degrees) in each iteration of this function
	const float MAX_TRIM_RUDDER_ANGLE = 20;//maximum amount to move rudder (from straight) when trimming the boat's direction
	unsigned int uiCurrentTime = millis();
	
	m_uiPreviousTrimTime = uiCurrentTime;
	ShipLog *pLog = (ShipLog *)pShipLog;
	if (fHeadingError>180) fHeadingError-=360;
	else if (fHeadingError<-180) fHeadingError+=360;
	//adjust fHeadingError for the current yawing motion of the boat and the integrated heading error over the last COMPASS_BUFSIZE samples (~ 2 seconds)
	double dIntegratedHeadingError = 0;//0.5*CalculateIntegratedHeadingError((double)fTargetHeading, pShipLog);//get integral of heading error over the last COMPASS_BUFSIZE samples (~ 2 seconds)
	//test	
	//char sMsg[256];
	//sprintf(sMsg,"heading error = %.1f\n, yaw rate = %.2f\n",fHeadingError,m_dYawRate);
	//pLog->LogEntry(sMsg,false);
	//end test
	fHeadingError+=((float)(m_dYawRate*YAW_FACTOR));
	fHeadingError+=(float)dIntegratedHeadingError;
	//printf("fHeadingError = %.1f\n",fHeadingError);
	
	float fTrimAmount = 0;
	//set trim amount based on heading error, > 0 means turn boat CCW, < 0 means turn boat CW
	if (fHeadingError>0) {//need to turn the boat CCW, so decrease trim angle
		fTrimAmount = -fHeadingError / 2;
		if (fTrimAmount< (-MAX_TRIM)) {
			fTrimAmount=-MAX_TRIM;
		}
	}
	else if (fHeadingError<0) {//need to turn the boat CW, so decrease speed of right propeller or increase speed of left propeller
		fTrimAmount = -fHeadingError / 2;
		if (fTrimAmount>MAX_TRIM) {
			fTrimAmount=MAX_TRIM;
		}
	}
	fAirRudderAngle+=fTrimAmount;
	if (fAirRudderAngle<-MAX_TRIM_RUDDER_ANGLE) {
		fAirRudderAngle = -MAX_TRIM_RUDDER_ANGLE;
	}
	else if (fAirRudderAngle>MAX_TRIM_RUDDER_ANGLE) {
		fAirRudderAngle = MAX_TRIM_RUDDER_ANGLE;
	}
	//test
	//printf("fHeadingError = %.1f, fAirRudderAngle = %.1f\n",fHeadingError,fAirRudderAngle);
	//end test
}	

/**
 * @brief return true if we have obtained at least one sample of valid GPS data
 * 
 * @return true if we have obtained at least one sample of valid GPS data
 * @return false if no valid GPS data has been obtained yet.
 */
bool Navigation::HaveValidGPS() {//return true if we have obtained at least one sample of valid GPS data
	return this->m_bGotValidGPSData;
}

void Navigation::CheckPriority(int nPriority) {//check to see if there are no other higher priority threads trying to execute a navigation command at the same time
	//just wait until priority level is low enough to continue
	while (nPriority < m_nMaxPriority) {
		usleep(100000);//pause for 100 ms
	}
	if (nPriority > m_nMaxPriority) {
		m_nMaxPriority = nPriority;
	}
}

/**
 * @brief call this function after high priority navigation command(s) are completed
 * 
 */
 void Navigation::HighPriorityCommandFinished() {
	m_nMaxPriority = LOW_PRIORITY;
}

