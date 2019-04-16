//TempSensor.h
//class for getting temperature data from the DS18B20 water-proof temperature sensor
#pragma once
#include "Sensor.h"
//define ERROR_CODES
#define ERROR_COULD_NOT_OPEN_W1_DIRECTORY 1
#define ERROR_COULD_NOT_OPEN_W1_DEVICE 2

class TempSensor : public Sensor {//class used for communicating with and getting tilt and magnetic data from the LM303D compass module (by ST Microelectronics)
//functions are also provided for computing heading angle based on tilt and magnetic data
public:
	TempSensor();//constructor
	~TempSensor();//destructor

	bool GetTemperature(float &fTemperature);//gets a sample of temperature in degrees celsius from the DS18B20, returns false if temperature could not be obtained
	int m_nErrorCode;//code for the last encountered error (0 if no error has occurred)	

private:
	bool Initialize();//perform initialization routine for the DS18B20 temperature sensor
	bool m_bInitialized;//true if the DS18B20 temperature sensor could be successfully found and initialized

	char m_devPath[128]; // Path to device
};