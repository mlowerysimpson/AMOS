//AToD.h class for getting A to D meausurements from the ADS1115 A to D converter chip (using I2C on Raspberry Pi)
//uses this A to D board: https://shop.controleverything.com/products/4-channel-0-20v-analog-to-digital-converter
#include "ShipLog.h"
#include "BatteryCharge.h"

#ifndef _WIN32
#include <pthread.h>
#else
typedef int pthread_mutex_t;
#endif


#define NUM_ATOD_CHANNELS 4 //number of A to D channels available
#define VREF 4.096//the voltage reference of the A to D
#define MAX_COUNTS 32767//maximum number of counts for full-scale
#define ATOD_NUM_TO_AVG 16//number of A to D samples to average for each measurement
#pragma once

#define RECENT_THRESHOLD_SEC 10//threshold in seconds for what is considered to be "recent" voltage data

class AToD {//class used for getting A to D meausurements from the MCP3428 A to D converter chip
public:
	AToD(char *i2c_filename, unsigned char ucAddress);
	AToD(char *i2c_filename, unsigned char ucAddress, ShipLog *pShipLog, pthread_mutex_t *i2c_mutex);//constructor
	~AToD();//destructor

	bool GetMeasurement(int nChannel, int nPGAGain, double dMeasurementGain, double &dResult);//get a measurement from a particular A to D 
	bool GetBatteryVoltage(double& dBattVoltage, BatteryCharge* pBattCharge);//gets the battery voltage for AMOS, assumes various resistor divider and channel settings for the A to D measurement
	bool SendBatteryVoltage(int nHandle, bool bUseSerial);


private:
	//data
	pthread_mutex_t *m_i2c_mutex;//control access to i2c bus
	unsigned int m_uiLastMeasurementTime;//the time of the last battery voltage measurement in ms
	double m_dMostRecentBattVoltage;//the most recent battery voltage
	int m_file_i2c;//handle to I2C port
	ShipLog *m_pShipLog;//used for logging any errors that might occur
	unsigned char m_ucAddress;//the I2C address of the A to D chip
	bool m_bOpenedI2C_OK;//true if the I2C port was opened properly
	bool m_bInitialized;//true if the MCP3428 has been found and configured properly
	int m_nCurrentChannel;//the channel (1 to 4) of the A to D that we are getting data from
	int m_nPGAVal;//programmable gain amplifier value (defaults to 1)

	//functions
	bool InitializeAtoD(int nPGAVal, int nChannel);//configure A to D for 16-bit operation
	void LogEntry(char *szLogMsg, bool bPrintMsg);//output text to log file (and optionally print to screen)
};