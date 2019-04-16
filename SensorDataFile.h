//SensorDataFile.h  defines the SensorDataFile class for saving the boat's sensor data to file 
#pragma once 
#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <unistd.h>
#include <wiringPi.h>
#endif
#include "PHSensor.h"
#include "TurbiditySensor.h"
#include "TempSensor.h"
#include "LeakSensor.h"
#include "DiagnosticsSensor.h"
#include <fcntl.h>
#include <vector>

using namespace std;

#define MAX_SENSORS 64 //maximum number of sensors that can be attached to the boat

//define sensor types 
#define WATER_TEMP_DATA 1 //water temperature 
#define BOAT_INTERIOR_TEMP_DATA 2 //boat interior temperature
#define PH_DATA 3 //water pH value
#define WATER_TURBIDITY 4 //turbidity value for water
#define GPS_LATITUDE 5 //GPS latitude (in degrees)
#define GPS_LONGITUDE 6  //GPS longitude (in degrees)
#define LEAK_DATA 7//leak sensor (boolean indication of whether or not a leak has occurred)
#define DIAGNOSTICS_DATA 8 //general diagnostics values (eg: current draw)

#define SENSOR_GRID_PAUSETIME_SEC 5 //number of seconds to pause with thrusters off before collecting a sample when doing sensor grid data collection

class SensorDataFile {
public:
	SensorDataFile(int *sensorTypes, int nNumSensors, void *sensorObjects[], char *szDataFilePath, int nLoggingIntervalSec);//constructor
	~SensorDataFile();//destructor
	DiagnosticsSensor *GetDiagnosticsSensor();//returns the object used for boat diagnostics and control routines
	void SaveData(struct tm *sampleTime, int nLoggingIntervalSec);//save the current sample of data to file
	void SaveDataAtLocation(double dLatitude, double dLongitude);//save data from all sensors at a particular GPS location
	void SetData(int nDataType, double dData);//sets a particular data value, data will get saved to file in the next call to SaveData
	bool isTimeToLogSensorData(struct tm *currentTime);//returns true if it is time to log sensor data to file
	bool SendNumSensors(int nHandle, bool bUseSerial);//send # of sensors over socket connection or wireless serial link
	bool SendSensorTypes(int nHandle, bool bUseSerial);//send sensor types over socket connection or wireless serial link
	bool SendSensorData(int nSensorType, int nHandle, bool bUseSerial);//send sensor data over socket connection or wireless serial link
	bool m_bFileOpened;//true if the file could be successfully opened for appending
	void SetFilename(char * pszFilename);//change the name of the file used for saving data
	void CollectAndSaveDataNow();//collect and save data from all available sensors
	void CollectData();//collect data from all available sensors
	
private:
	//data
	void *m_sensorObjects[MAX_SENSORS];//void pointers to sensor objects used for collecting sensor data
	time_t m_nextLogTime;//time of next sample log in seconds since midnight, Jan. 01, 1970
	int m_nLastLogTime;//time of last sample log in seconds since midnight
	float m_fWaterTemp;//water temperature in degrees C
	float m_fInteriorTemp;//the temperature of the interior of the boat in degrees C
	float m_fWaterPH;//the measured pH value of the water
	float m_fTurbidity;//the turbidity value of the water
	float m_fCurrentDraw12V;//the amount of current (in A) coming from the +12V supply
	float m_fBatteryVoltage;//the measured voltage of the +12V supply
	float m_fHumidity;//the relative humidity inside AMOS (expressed as a percentage from 0 to 100)
	float m_fHumidityTemp;//the temperature of the humidity sensor in AMOS (in deg C)
	float m_fWirelessRXPower;//received wireless power (for serial wireless link, in dBm)
	bool m_bLeak;//true if a leak has been detected, otherwise false
	bool m_bSolarCharging;//true if solar panel is being charged through solar inputs, otherwise false
	std::ofstream *m_dataFile;//the output data file

	int *m_sensorTypes;//array of sensor types (defined above)
	int m_nNumSensors;//the number of sensors for which data is being logged
	double *m_sensorData;//data for each sensor
	bool *m_bDataAvailable;//array of flags for each sensor indicating if data has been set for the current sample or not
	char *m_szDataFilePath;//the full path of the file where data will be saved

	//functions
	TempSensor *GetWaterTempSensor();//returns the water temperature sensor (if available) or nullptr if not
	PHSensor *GetPHSensor();//returns the PH sensor (if available) or nullptr if not
	TurbiditySensor *GetTurbiditySensor();//returns the turbidity sensor (if available) or nullptr if not
	LeakSensor *GetLeakSensor();//returns the leak sensor (if available) or nullptr if not

	time_t GetNextLogTime(int nLoggingIntervalSec);//get the time of the next sample (in seconds since midnight, Jan, 1, 1970)
	void WriteDataFileHeader();
	bool CreateDataFile();//open data file for appending data or create data file if it doesn't exist already
	char * GetFormattedData(struct tm *sampleTime);//formats the currently available data
};
