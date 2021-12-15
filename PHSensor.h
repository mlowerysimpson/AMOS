//PHSensor.h -- interface class for getting pH data from pH probe sensors
#pragma once
#ifndef _WIN32
#include <pthread.h>
#else
typedef int pthread_mutex_t;
#endif
#include "Sensor.h"


//default pH calibration and channel info
#define DEFAULT_PH_LOWCALVOLTAGE 0.655 //default acidic calibration voltage (for pH of 2.4)
#define DEFAULT_PH_LOWCAL 2.4 //default low pH calibration value (corresponds to test in vinegar)
#define DEFAULT_PH_MIDCALVOLTAGE 1.82 //default neutral calibration voltage (for pH of 7)
#define DEFAULT_PH_MIDCAL 7.0 //default neutral pH calibration value
#define DEFAULT_PH_CHANNEL 3 //default A to D channel for pH probe
#define DEFAULT_PH_I2C_CHANNEL 0x63//default I2C channel for Atlas Scientific pH probe

struct PH_CALIBRATION {//pH probe calibration data for analog pH sensor (connected to A to D board)
	double dLowCalVoltage;//the voltage of the "low" (acidic) pH measurement
	double dLowCalPH;//the pH value of the "low" (acidic) pH measurement
	double dMidCalVoltage;//the voltage of the neutral (pure water) measurement
	double dMidCalPH;//the pH value of the neutral (pure water) measurement, typically set to 7
};

class PHSensor : public Sensor {//class used for getting pH data for pH probe connected to A To D device
public:
	PHSensor(int nAToDChannel, AToD *pAToD, PH_CALIBRATION *pPHCal);//constructor for analog pH sensor connected to A to D board
	PHSensor(unsigned char i2c_channel, pthread_mutex_t* i2c_mutex);//constructor for Atlas scientific pH sensor on I2C
	~PHSensor();//destructor
	bool GetPHSensorPH(double &dPHVal);//gets the pH probe pH value for AMOS (uses currently available pH probe calibration parameters)
	bool GetPHSensorPH(double& dPHVal, double dWaterTempDegC);//gets the pH probe pH value for AMOS, with temperature compensation applied (I2C probe only)
	bool CalibrateMidpoint(double dMidPHVal, double dWaterTempDegC);//perform a single-point, midpoint calibration of the pH probe (I2C devices only, removes all previous calibrations)
	bool CalibrateLowpoint(double dLowPHVal, double dWaterTempDegC);//perform the lowpoint calibration of the pH probe (I2C devices only)
	bool CalibrateHighpoint(double dHighPHVal, double dWaterTempDegC);//perform the highpoint calibration of the pH probe (I2C devices only)

private:
	//data
	//analog sensor - related variables
	int m_nAToDChannel;//the channel of the A to D device that this pH probe is connected to (1 to 4)
	PH_CALIBRATION m_phcal;//the pH calibration data for this pH probe (used only for analog pH probes)

	//I2C sensor - related variables and functions
	pthread_mutex_t* m_i2c_mutex;
	bool m_bOpenedI2C_OK;
	bool GetI2CPH(double& dPHMeasurement);//gets the pH measurement from an I2C probe
	int m_file_i2c;//handle to I2C port for the LiDAR connection
	bool write_i2c(unsigned char* ucData, int nNumBytes);//writes bytes to I2C address
	void lockmutex();//lock mutex for access to the I2C bus
	void unlockmutex();//unlock mutex for access to the I2C bus
};