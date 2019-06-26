//AToD.cpp  Implementation file for AToD class
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <wiringPi.h>
#include <math.h>
#include "AToD.h"
#include "CommandList.h"
#include <wiringPi.h>

/**
 * @brief Construct a new AToD::AToD object for collecting data from the ADS1115 chip
 * 
 * @param i2c_filename the name of the i2c port to use, ex: "/dev/i2c-1"
 * @param ucAddress the 7-bit i2c address to use for the ADS1115.
 * @param pShipLog pointer to a ShipLog object used for logging error messages, etc.
 * @param i2c_mutex used for controlling access to the i2c bus
 */
AToD::AToD(char *i2c_filename, unsigned char ucAddress, ShipLog *pShipLog, pthread_mutex_t *i2c_mutex) {//constructor
	m_ucAddress = ucAddress;
	m_nPGAVal = 1;
	m_i2c_mutex = i2c_mutex;
	m_nCurrentChannel = 1;
	m_bOpenedI2C_OK = true;
	m_bInitialized = false;
	m_uiLastMeasurementTime = 0;
	m_pShipLog = pShipLog;
	pthread_mutex_lock(m_i2c_mutex);
	m_file_i2c = open(i2c_filename, O_RDWR);
	pthread_mutex_unlock(m_i2c_mutex);
	m_dMostRecentBattVoltage=0.0;
	if (m_file_i2c<0) {
		//error opening I2C
		//test
		//char sMsg[256];
		//sprintf(sMsg,"Error: %s opening I2C.\n",strerror(errno));
		//m_pShipLog->LogEntry(sMsg,true);
		//end test
		m_bOpenedI2C_OK=false;
	}
}

AToD::~AToD() {
	

}

bool AToD::InitializeAtoD(int nPGAVal, int nChannel) {//initialize ADS1115 A to D converter for 16-bit continuous mode operation
	//nPGAVal = programmable gain amplifier gain, can be 0, 1, 2, 4, 8, or 16 corresponding to gain levels of 2/3, 1, 2, 4, 8, or 16
	//nChannel = the channel used for A to D measurements, 1 to 4
	unsigned char out_buf[1];
	char sMsg[256];
	unsigned char config_msb=0x81, config_lsb=0xE3;//most significant and least significant bytes of 16-bit configuration register (MSB is set for single-shot mode while initializing a conversion, LSB is set for 860 samples per sec, and comparator disabled)
	unsigned char sendBytes[3];//bytes to send to ADS1115
	sendBytes[0] = 0x01;//corresponds to the configuration register
	if (!m_bOpenedI2C_OK) {
		m_pShipLog->LogEntry((char *)"Error, was unable to open I2C port.\n",true);
		return false;
	}
	if (nChannel==1) {
		config_msb|=0x40;
	}
	else if (nChannel==2) {
		config_msb|=0x50;
	}
	else if (nChannel==3) {
		config_msb|=0x60;
	}
	else if (nChannel==4) {
		config_msb|=0x70;
	}
	else {//invalid channel
		sprintf(sMsg, "Invalid channel: %d. Must be between 1 and 4.\n",nChannel);
		m_pShipLog->LogEntry(sMsg,true);
		m_bInitialized = false;
		return false;
	}
	if (nPGAVal==1) {
		config_msb|=0x02;
	}
	else if (nPGAVal==2) {
		config_msb|=0x04;
	}
	else if (nPGAVal==4) {
		config_msb|=0x06;
	}
	else if (nPGAVal==8) {
		config_msb|=0x08;
	}
	else if (nPGAVal==16) {
		config_msb|=0x0a;
	}
	//else assume PGA of 2/3
	sendBytes[1] = config_msb;
	sendBytes[2] = config_lsb;

	if (!m_bOpenedI2C_OK) return false;

	pthread_mutex_lock(m_i2c_mutex);
	if (ioctl(m_file_i2c, I2C_SLAVE, m_ucAddress)<0) {
		sprintf(sMsg,"Failed (error = %s) to acquire bus access and/or talk to slave A to D chip at %d.\n",
			strerror(errno),(int)m_ucAddress);
		m_pShipLog->LogEntry(sMsg,true);
		m_bInitialized = false;
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}	
	if (write(m_file_i2c, sendBytes, 3)!=3) {
		//error, I2C transaction failed
		sprintf(sMsg,"Failed to write the configuration bytes (error = %s) to the I2C bus.\n",
			strerror(errno));
		m_pShipLog->LogEntry(sMsg,true);
		m_bInitialized = false;
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//pthread_mutex_unlock(m_i2c_mutex); --> need to leave mutex locked and then unlock it later in the calling function
	m_nPGAVal = nPGAVal;
	m_nCurrentChannel = nChannel;
	m_bInitialized = true;
	return true;
}

//GetMeasurement: Gets an A To D measurement on a particular channel
//nChannel: the A to D channel (1 to 4) that we want the measuremet from
//nPGAGain: the gain of the "pre-gain-amplifier", must be 1, 2, 4, or 8
//dMeasurementGain: the gain value to multiply by the returned voltage
//dResult: the result of the measurement (voltage from A to D, multiplied by dMeasurementGain)
bool AToD::GetMeasurement(int nChannel, int nPGAGain, double dMeasurementGain, double &dResult) {
	const int TIMEOUT_MS = 1000;//length of timeout period in ms
	unsigned char inBuf[2];//input buffer needs to be large enough to read 2 data bytes
	unsigned char sendByte[1];//single byte to send to control whether to read the conversion register or the configuration register
	char sMsg[256];
	dResult = 0.0;
	if (!m_bOpenedI2C_OK) {
		return false;
	}
	//if (nChannel!=m_nCurrentChannel||nPGAGain!=m_nPGAVal) {
	//need to re-configure A to D
	
	double dSumCountVals = 0.0;//the summation of all the A to D count values (divided later by ATOD_NUM_TO_AVG to get average value)
	
	for (int i=0;i<ATOD_NUM_TO_AVG;i++) {
		unsigned int uiStartTime = millis();
		unsigned int uiTimeoutTime = uiStartTime + TIMEOUT_MS;
		bool bDataReady = false;
		if (!InitializeAtoD(nPGAGain, nChannel)) {
			m_pShipLog->LogEntry((char *)"Error trying to configure A to D.\n",true);
			return false;
		}
		while (millis()<uiTimeoutTime&&!bDataReady) {
			//read in configuration bytes
			if (read(m_file_i2c,inBuf,2)!=2) {
				//ERROR HANDLING: i2c transaction failed
				m_pShipLog->LogEntry((char *)"Failed to read the configuration bytes from the I2C bus.\n",true);
				pthread_mutex_unlock(m_i2c_mutex);
				return false;
			}
			if ((inBuf[0]&0x80)>0) {
				//data is now ready (no longer performing a conversion)
				bDataReady=true;
				break;
			}
		}
		if (!bDataReady) {
			m_pShipLog->LogEntry((char *)"Timeout waiting for ADC data to be ready.\n",true);
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		//read in conversion register
		sendByte[0] = 0x00;//code for configuration register
		if (write(m_file_i2c, sendByte, 1)!=1) {
			//error, I2C transaction failed
			sprintf(sMsg,"Failed to write to the I2C bus for conversion register (error = %s).\n", strerror(errno));
			m_pShipLog->LogEntry(sMsg,true);
			m_bInitialized = false;
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		//read in conversion register
		if (read(m_file_i2c,inBuf,2)!=2) {
			//ERROR HANDLING: i2c transaction failed
			m_pShipLog->LogEntry((char *)"Failed to read the conversion bytes from the I2C bus.\n",true);
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		pthread_mutex_unlock(m_i2c_mutex);
		int nCountVal = (inBuf[0]<<8) + inBuf[1];
		bool bNeg = false;
		if ((inBuf[0]&0x80)>0) {
			bNeg = true;
		}
		if (bNeg) {
			nCountVal = - (65536 - nCountVal);
		}
		dSumCountVals+=nCountVal;			
	}
	dResult = dMeasurementGain *  VREF * dSumCountVals / ATOD_NUM_TO_AVG / MAX_COUNTS;
	return true;
}

//GetBatteryVoltage: gets the battery voltage for AMOS, assumes various resistor divider and channel settings for the A to D measurement (see function definition below for details)
//dBattVoltage = returns the voltage of the battery in volts
bool AToD::GetBatteryVoltage(double &dBattVoltage) {//gets the battery voltage for AMOS, assumes various resistor divider and channel settings for the A to D measurement
	//assumes use of voltage divider of two 10K resistors, which effectively halves the battery voltage at the CH1 input
	const double R1 = 51.0;//51k resistor
	const double R2 = 10.0;//10k resistor
	const int BATT_VOLTAGE_CHANNEL = 1;
	char sMsg[256];
    double dVoltageGain = (R1 + R2) / R2;
    dBattVoltage=0.0;
	bool bGotResult = GetMeasurement(BATT_VOLTAGE_CHANNEL,1,dVoltageGain,dBattVoltage);
	if (bGotResult) {
		m_uiLastMeasurementTime = millis();
		sprintf(sMsg,"Battery Voltage: %.3f V\n",dBattVoltage);
		m_pShipLog->LogEntry(sMsg,true);
	}
	else {
		m_pShipLog->LogEntry((char *)"Error getting battery voltage.\n",true);
		return false;
	}
	m_dMostRecentBattVoltage = dBattVoltage;
	return true;
}

/**
 * @brief send most recent battery voltage out network socket or serial port connection (if no battery voltage available yet, or if the last battery voltage measurement was old (more than RECENT_THRESHOLD_SEC seconds ago) then do an A to D conversion to get it)
 * 
 * @param nHandle either the network socket handle (if network communications are being used) or the serial port file descriptor if a wireless serial port connection is being used (i.e. if bUseSerial is true)
 * @param bUseSerial set to true if a wireless serial link is being used, otherwise if network communications are being used, set to false
 * @return true if the battery voltage was successfully sent
 * @return false if there was a problem sending the battery voltage
 */
bool AToD::SendBatteryVoltage(int nHandle, bool bUseSerial) {
	BOAT_DATA *pBoatData = BoatCommand::CreateBoatData(BATTVOLTAGE_DATA_PACKET);
	if (m_dMostRecentBattVoltage==0.0) {//battery might be switched off, or perhaps we haven't tried getting battery voltage yet, so try now
		GetBatteryVoltage(m_dMostRecentBattVoltage);
	}
	else if ((millis() - m_uiLastMeasurementTime)>(1000*RECENT_THRESHOLD_SEC)) {
		GetBatteryVoltage(m_dMostRecentBattVoltage);
	}
	float fBattVoltage = (float)m_dMostRecentBattVoltage;
	memcpy(pBoatData->dataBytes,&fBattVoltage,sizeof(float));
	pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
	return BoatCommand::SendBoatData(nHandle, bUseSerial, pBoatData, nullptr);
}

