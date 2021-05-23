//TempSensor.cpp
//Implementation file for TempSensor class 
#include "TempSensor.h"

#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

TempSensor::TempSensor() : Sensor(nullptr) {//constructor
	m_nErrorCode=0;
	memset(m_devPath,0,128); // Path to device
	m_tempConductivty = nullptr;
	m_bInitialized = Initialize();
}

TempSensor::TempSensor(AMLTempConductivity* pTempConductivitySensor) : Sensor(nullptr) {//constructor for use when getting water temperature from temperature / conductivity sensor
	m_nErrorCode = 0;
	m_bInitialized = true;
	memset(m_devPath, 0, 128);
	m_tempConductivty = pTempConductivitySensor;
}

TempSensor::~TempSensor() {//destructor

}

//GetTemperature: gets a sample of temperature in degrees celsius from the DS18B20, returns false if temperature could not be obtained
//fTemperature = the temperature in degrees celsius that gets returned by the function (if successful). It is left unchanged if the function is not successful
//return value: true if the temperature sample could be successfully acquired, otherwise false
bool TempSensor::GetTemperature(float &fTemperature) {
	if (m_tempConductivty != nullptr) {
		fTemperature = (float)m_tempConductivty->GetTemperature();
		return true;
	}
	char tmpData[256];   // Temp C * 1000 reported by device 
	char buf[256];     // Data from device
	ssize_t numRead;
	int fd = open(m_devPath, O_RDONLY);

	if (!m_bInitialized) {
		return false;
	}

	if(fd == -1)
	{
		perror ("Couldn't open the w1 device.");
		m_nErrorCode = ERROR_COULD_NOT_OPEN_W1_DEVICE;
		return 1;   
	}
	numRead = read(fd, buf, 256);
	if (numRead>0) {
		strcpy(tmpData, strstr(buf, "t=") + 2); 
		fTemperature = strtof(tmpData, NULL) / 1000;
	}
	close(fd);
	this->UpdateCollectionTime();
	return true;
}

bool TempSensor::Initialize() {//perform initialization routine for the DS18B20 temperature sensor
	DIR *dir;
	struct dirent *dirent;
	char dev[16];      // Dev ID
	char path[] = "/sys/bus/w1/devices"; 

	dir = opendir (path);
	if (dir != NULL)
	{
		while ((dirent = readdir (dir)))
			// 1-wire devices are links beginning with 28-
			if (dirent->d_type == DT_LNK && 
				strstr(dirent->d_name, "28-") != NULL) { 
					strcpy(dev, dirent->d_name);
					//printf("\nDevice: %s\n", dev);
			}
			(void) closedir (dir);
		}
	else
	{
		perror ("Couldn't open the w1 devices directory");
		m_nErrorCode = ERROR_COULD_NOT_OPEN_W1_DIRECTORY;
		return false;
	}
	// Assemble path to OneWire device
	sprintf(m_devPath, "%s/%s/w1_slave", path, dev);
	return true;
}