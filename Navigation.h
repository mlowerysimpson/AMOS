//Navigation.h
//Code for obtaining navigation data, plotting a course, navigating around obstacles, etc.
#include "CommandList.h"
#include "IMU.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <memory>

#include <libgpsmm.h>
#include <wiringPi.h>
#include <pthread.h>

#define SAMPLE_BUFSIZE 500 //number of historical NAV_DATA samples to keep in memory for keeping track of the boat's course
#define COMPASS_BUFSIZE 20 //buffer for holding historical compass heading values
#define TRIM_BUFSIZE 10 //number of samples to use for filtering trimmed thruster speeds

#define LOW_POWER_MODE 1//in this mode of operation the thrusters are totally shutoff in order to conserve power until the battery can be charged 
#define HALF_POWER_MODE 2//in this mode of operation the maximum thruster speed is reduced by half in order to conserver power
#define FULL_POWER_MODE 3//normal mode of operation, full thruster speed is possible

#define MAX_NUM_OBSTACLES 60 //maximum # of obstacles that the Navigation object can track

//priority levels for navigation commands
#define LOW_PRIORITY 0
#define HIGH_PRIORITY 1

struct NAV_DATA {//structure used for keeping track of historical GPS and compass readings
	double dLatitude;//latitude of gps reading
	double dLongitude;//longitude of gps reading
	double dCompassHeading;//heading reading using compass
	unsigned int uiTimeMS;//time of reading in ms 
};

struct COMPASS_ADJUSTMENT {//structure used for applying adjustments to reported heading angles from the compass. May be necessary to improve compass heading accuracy
	int nNumVals;//the number of points that were used for determining the compass adjustment
	double *actual_compass_vals;//the actual compass values (obtained by using a real compass, and adjusted for magnetic declination)
	double *reported_compass_vals;//the values reported by the electronic compass (and adjusted for magnetic declination)
};

struct COMPASS_SAMPLE {//structure for holding info about a compass sample, i.e. heading in degrees, and time of sample (in ms since program started)
	double dHeading;
	unsigned int uiSampleTime;
};

struct OBSTACLE_FOUND {//structure for describing the prescence of an obstacle at a particular heading angle
	unsigned int uiFoundTime;//time in ms when the obstacle was found
	int nHeading;//heading of the found obstacle (heading expressed as integer)
};

class Navigation {
public:
	Navigation(double dMagDeclination, pthread_mutex_t *i2c_mutex);//constructor
	~Navigation();//destructor

	IMU_DATASAMPLE m_imuData;//structure for holding compass, inertial data
	bool m_bExitNavFunction;//flag can be used to exit out of some navigation functions
	//functions
	static double ComputeDistBetweenPts(double dLatitudeDeg1, double dLongitudeDeg1, double dLatitudeDeg2, double dLongitudeDeg2);//use GPS locations of 2 points to get the distance between 2 points 
	void SetDriveTimeoutSeconds(unsigned int uiTimeoutSec);//timeout on drive to location function after this many seconds
	void AddObstacleAtCurrentHeading(void *pShipLog);//inform this Navigation object that there is an obstacle at the current heading
	bool HaveValidGPS();//return true if we have obtained at least one sample of valid GPS data
	double GetLatitude();//return the current latitude of the boat (as determined by GPS) in degrees
	double GetLongitude();//return the current longitude of the boat (as determined by GPS) in degrees 
	double GetGPSAccuracyM();//return the accuracy (uncertainty) of most recent GPS reading in m
	int GetNumVisibleSatellites();//return the number of currently visible GPS satellites
	int GetNumSatellitesUsed();//return the number of satellites that are used for the position calculation
	double GetBoatTemp();//return the temperature of the interior of the boat (comes from magnetometer chip temperature sensor)
	void SetPowerMode(int nPowerMode,double dVoltage,void *pShipLog);//set the mode of operation and maximum thruster speed possible
	bool driftedFromEmergencyStop();//return true if we have drifted away from the emergency stopping point
	void SetPosition(double dLatitude, double dLongitude);//set latitude and longitude for unit testing purposes
	bool CollectGPSData(void *pShipLog, bool *bKeepGoing);//collects a sample of GPS data
	bool SendGPSData(int nHandle, bool bUseSerial);//send most recent GPS data out network socket or serial port connection
	bool CollectCompassData(void *pShipLog);//collects a sample of compass data
	bool SendCompassData(int nHandle, bool bUseSerial);//send most recent compass data out network socket or serial port connection
	void ShowCompassError(const char *szErrorText);//show error text pertaining to the compass module... just display one error message per program session
	void TurnToCompassHeading(float fHeading,void *pThrusters,pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, int nPriority);//turn boat to desired compass heading
	void DriveForwardForTime(int nTotalTimeSeconds, float fMaxSpeed, float fHeadingDirection, void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, bool bStopWhenDone, int nPriority);//drive boat forward for the specified time in seconds
	void DriveToLocation(double dLatitude, double dLongitude, void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, int nInterpAmount, bool *bCancel, int nPriority);//drive boat to the specified GPS location
	void HoldCurrentPosition(int nTotalTimeSeconds, void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, int nPriority);//hold the current GPS location
	bool isDataReady();//return true if compass data is available
	char *GetStatusLogText();//returns text corresponding to the ship's current navigation status, calling application is responsible for deleting the returned text (if non-null)
	static double ComputeHeadingDif(double dHeading1Deg,double dHeading2Deg);//compute difference between 2 heading angles in degrees
	void ExecuteLeakResponse(int nLeakResponseType, void *pShipLog);//special emergency function for getting to shore as quickly as possible
	void TrimSpeeds(float &fLSpeed, float &fRSpeed, float fHeadingError, float fTargetHeading, void *pShipLog);//adjust speeds of propellers in order to fine-tune the direction that the boat is moving while at speed in the forward direction
	void HighPriorityCommandFinished();//call this function after high priority navigation command(s) are completed

private:
	//data
	unsigned int m_uiDriveToLocationTimeout;//timeout in seconds for the DriveToLocation function (set to zero to disable timeouts)
	int m_nNumObstacles;//the number of obstacles that have been found
	float m_fObstacleHeadingOffset;//a temporary heading offset (in degrees) which is used to help avoiding obstacles
	int m_nMaxPriority;//the maximum priority for a navigation instruction that is currently being executed, if a thread issues a navigation command with less priority than this, it will pause until the higher priority thread is finished and m_nMaxPriority becomes a lower value
	pthread_mutex_t *m_i2c_mutex;//mutex controlling access to the i2c bus
	bool m_bGotValidGPSData;//flag indicates whether or not valid GPS data has ever been received
	float m_fEstimatedCompassError;//estimate of compass error, relative to GPS track
	double m_dMaxSpeed;//the maximum possible thruster speed
	unsigned int m_uiPreviousTrimTime;//the time in ms when the thruster speed was last trimmed
	COMPASS_ADJUSTMENT *m_compassAdjustment;//structure used for applying adjustments to compass heading angle measurements
	NAV_DATA *m_navSamples[SAMPLE_BUFSIZE];
	int m_nSampleIndex;//index of the current sample
	int m_nNumSamples;//number of samples available in the sample buffer
	unsigned int m_uiLastCompassCheckTime;//the time in ms when the compass was last used to orient the boat
	double m_dMagDeclination;//the magnetic declination in degrees for the current area (set to zero to express headings relative to magnetic north)
	GPS_DATA *m_currentGPSData;//the most recently obtained GPS data
	gpsmm *m_gps_rec;//gps receiver
	bool m_bInitializedGPS;//flag is true if GPS receiver was initialized successfully
	double m_dLatitude;//latitude of most recent GPS reading
	double m_dLongitude;//longitude of most recent GPS reading
	double m_dGPSAccuracyM;//accuracy (uncertainty) of most recent GPS reading
	int m_nNumGPSSatellitesInView;//number of GPS satellites in view
	int m_nNumGPSSatellitesUsed;//number of GPS satellites that are used in the position calculation
	double m_dSpeed;//speed of boat (as measured by GPS) in m/s
	double m_dTrack;//track that the boat is following (in degrees relative to true north, as determined by GPS)
	time_t *m_gpsTime;//the time in seconds since Jan 1, 1970 when the most recent GPS sample was received
	IMU *m_imu;//object for getting compass and inertial data from the AltIMU-10 v5 Gyro, Accelerometer, Compass, and Altimeter from Pololu Electronics (www.pololu.com)
	bool m_bCompassErrorShown;//flag is true after an error pertaining to the compass module has been shown
	double m_dYawRate;//instantaneous rate at which heading of boat is changing (in degrees per second)
	double m_dFilteredYawRate;//rate at which heading of boat is changing (in degrees per second, filtered as an average of the last second of data)

	//functions
	float GetObstacleAvoidanceHeadingOffset(float fDesiredHeading);//get the neccessary heading offset for avoiding obstacles (if any)
	void CheckPriority(int nPriority);//check to see if there are no other higher priority threads trying to execute a navigation command at the same time
	void TrimAirRudder(float &fAirRudderAngle,float fHeadingError,float fDesiredHeading,void *pShipLog);//fine-tune air rudder angle in order to correct any heading error
	float TurnToRandomAngle(void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, int nPriority);//turns boat to a random angle
	static double ComputeAvgHeading(double dHeading1Deg, double dHeading2Deg);//return average value of 2 headings in degrees
	void DriverAirboatForwardForTime(int nTotalTimeSeconds, float fMaxSpeed, float fHeadingDirection, void *pThrusters, pthread_mutex_t *command_mutex, 
									 unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel, bool bStopWhenDone, int nPriority);
	double CalculateIntegratedHeadingError(double dTargetHeading, void *pShipLog);//get integral of heading error over the last COMPASS_BUFSIZE samples (~ 2 seconds)
	float FindEstimatedCompassError(void *pShipLog);//look at compass and GPS data for the last SAMPLE_BUFSIZE samples to estimate the amount of compass error; provides an estimate of how far off the compass is from the GPS track
	float GetHeadingError(double dDesiredHeading, void *pShipLog);//use mix of current compass reading and historical compass and GPS data to determine the current heading error of the boat
	bool isBoatStuck();//return true if the nav buffer is full of samples all for the same GPS location
	IMU * CreateCompass();//creates a compass object for collecting compass and orientation
	void IncrementTurningSpeed(float &fSpeedLeft,float &fSpeedRight,float fMaxTurnSpeed,bool bClockWise);//function used to increment turning speed
	double ComputeHeadingAndDistToDestination(double dLatitudeDeg,double dLongitudeDeg,double &dDistToDestM);//use current GPS location to get initial heading to destination, also returns distance to destination in m
	void ExecuteRandomThrust(void *pThrusters, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog, bool *bCancel);//execute a short burst of random propeller thrust to try to become un-stuck
	void ResetNavSamples();//reset the buffer that keeps track of historical gps locations & compass headings
	void AddNavSample(unsigned int uiTimeMS);//add current navigation data to the buffer of samples
	COMPASS_ADJUSTMENT * LoadCompassAdjustments();//get compass adjustments (if any) from compass_cal.txt file
	double AdjustUsingCompassCal(double dUnadjustedHeading);//ajust the compass heading using calibration data stored in m_compassAdjustment (if available)
	int GetClosestHeadingCalIndexBelow(double dHeading);//get closest calibration heading that is <= dHeading
	int GetClosestHeadingCalIndexAbove(double dHeading);//get closest calibration heading that is >= dHeading
	void BufferCompassData(double dHeading, unsigned int uiSampleTime);//buffer compass data so that it can be used later for estimating the rate of yaw
	double ComputeYawRate(bool bApplyFilter);//function looks at buffered heading data over previous second to determine yaw rate
	COMPASS_SAMPLE *m_compassBuf[COMPASS_BUFSIZE];
	int m_nCompassBufIndex;//index into m_compassBuf where data is about to be written
	OBSTACLE_FOUND *m_obstacles[MAX_NUM_OBSTACLES];
};
