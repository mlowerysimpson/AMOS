/**
 * @file AtlasDO2Sensor.cpp
 * @author Murray Lowery-Simpson (murray@innaturerobotics.com)
 * @brief Implementation file for AtlasDO2Sensor class
 * @version 0.1
 * @date 2020-07-14
 * 
 * @copyright Copyright (c) 2020 In Nature Robotics Ltd.
 * 
 */

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <iostream>
#include <wiringPi.h>
#include <cstring>
#include "AtlasDO2Sensor.h"




/**
 * @brief Construct a new AtlasDO2Sensor::AtlasDO2Sensor object
 * 
 * @param i2c_mutex mutex controlling access to the i2c bus
 */
AtlasDO2Sensor::AtlasDO2Sensor(pthread_mutex_t *i2c_mutex) : Sensor(nullptr) {//constructor
    m_i2c_mutex = i2c_mutex;
    //open I2C port for AtlasDO2Sensor device
	char *i2c_filename = (char*)"/dev/i2c-1";
	m_bOpenedI2C_OK = true;
    m_bDO2Initialized_OK = true;
	lockmutex();
	m_file_i2c = open(i2c_filename, O_RDWR);
	unlockmutex();
	if (m_file_i2c<0) {
		//error opening I2C
		printf("Error: %s opening I2C.\n",strerror(errno));
		m_bOpenedI2C_OK=false;
	}
}

bool AtlasDO2Sensor::write_i2c(unsigned char* ucData, int nNumBytes) {//writes bytes to I2C address
    if (write(m_file_i2c, ucData, nNumBytes)!=nNumBytes) {
        return false;
    }
    return true;
}


/**
 * @brief get the dissolved oxygen concentration in mg/L. The current temperature and salinity compensations (if any) is used for the measurement. 
 * @param dDO2Measurement the returned value of the concentration of dissolved oxygen, expressed in mg/L
 * 
 * @return true if the measurement was completed successfully, false otherwise
 */
bool AtlasDO2Sensor::GetDO2(double& dDO2Measurement) {
    unsigned char outBytes[1];
    unsigned char inBytes[32];
    memset(inBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;
    
    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DO2SENSOR_ADDR_DEFAULT)<0) {
	    printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas EZO-DO device.\n",strerror(errno));
	    unlockmutex();
	    return -1;
	}
    outBytes[0] = (unsigned char)'R';
    if (!write_i2c(outBytes,1)) {
        unlockmutex();
        return -2;
    }
    //need to pause for 600 ms (see do_ezo_datasheet-1.pdf document)
    usleep(600000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 32);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from DO2 sensor.\n");
        unlockmutex();
        return -3;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from DO2 sensor.\n");
        unlockmutex();
        return -4;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, DO2 sensor not ready.\n");
        unlockmutex();
        return -5;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to get DO2 data.\n");
        unlockmutex();
        return -6;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from DO2 sensor.\n");
        unlockmutex();
        return -7;
    }
    //parse dissolved oxygen measurement
    if (sscanf((char*)&inBytes[1], "%lf", &dDO2Measurement) < 1) {
        printf("Error trying to parse DO2 data.\n");
        unlockmutex();
        return -8;
    }
    unlockmutex();
    UpdateCollectionTime();
    return true;//reading was obtained successfully
}

/**
 * @brief get the dissolved oxygen concentration in ml/L, given that the water is at a certain temperature (i.e. apply temperature compensation for the measurement)
 * @param dDO2Measurement the returned value of the concentration of dissolved oxygen, expressed in mg/L.
 * @param dTempDegC the current temperature of the water in degrees C.
 * @return true if the measurement was completed successfully, false otherwise
 */
bool AtlasDO2Sensor::GetDO2(double& dDO2Measurement, double dTempDegC) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(outBytes, 0, 32);
    memset(inBytes, 0, 32);
    sprintf((char*)outBytes, "RT,%.1f", dTempDegC);
    int nNumToSend = strlen((char*)outBytes);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DO2SENSOR_ADDR_DEFAULT) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas EZO-DO device.\n", strerror(errno));
        unlockmutex();
        return -1;
    }
    if (!write_i2c(outBytes, nNumToSend)) {
        unlockmutex();
        return -2;
    }
    //need to pause for 900 ms to set temperature compensation and do measurement (see do_ezo_datasheet-1.pdf document)
    usleep(900000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 32);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from DO2 sensor.\n");
        unlockmutex();
        return -3;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from DO2 sensor.\n");
        unlockmutex();
        return -4;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, DO2 sensor not ready.\n");
        unlockmutex();
        return -5;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to get DO2 data.\n");
        unlockmutex();
        return -6;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from DO2 sensor.\n");
        unlockmutex();
        return -7;
    }
    //parse dissolved oxygen measurement
    if (sscanf((char*)&inBytes[1], "%lf", &dDO2Measurement) < 1) {
        printf("Error trying to parse DO2 data.\n");
        unlockmutex();
        return -8;
    }
    unlockmutex();
    UpdateCollectionTime();
    return true;//reading was obtained successfully
}


void AtlasDO2Sensor::lockmutex() {//lock mutex for access to the I2C bus
    if (m_i2c_mutex!=nullptr) {
        pthread_mutex_lock(m_i2c_mutex);
    }
}


void AtlasDO2Sensor::unlockmutex() {//unlock mutex for access to the I2C bus
    if (m_i2c_mutex!=nullptr) {
        pthread_mutex_unlock(m_i2c_mutex);
    }
}

/**
 * @brief perform a single-point calibration of the dissolved O2 probe to atmospheric oxygen levels
 *
 * @return true if the calibration was performed successfully, false otherwise
 */
bool AtlasDO2Sensor::Calibrate() {
    unsigned char outBytes[3];
    unsigned char inBytes[32];
    memset(inBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DO2SENSOR_ADDR_DEFAULT) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas EZO-DO device.\n", strerror(errno));
        unlockmutex();
        return false;
    }
    outBytes[0] = (unsigned char)'C';
    outBytes[1] = (unsigned char)'a';
    outBytes[2] = (unsigned char)'l';

    if (!write_i2c(outBytes, 3)) {
        unlockmutex();
        return false;
    }
    //need to pause for 1300 ms (see do_ezo_datasheet-1.pdf document)
    usleep(1300000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 2);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from DO2 sensor.\n");
        unlockmutex();
        return false;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from DO2 sensor.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, DO2 sensor not ready.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to do DO2 calibration.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from DO2 sensor.\n");
        unlockmutex();
        return false;
    }
    unlockmutex();
    return true;//calibration was completed successfully
}

/**
 * @brief go into sleep mode to conserve a bit of power
 *
 * 
 */
void AtlasDO2Sensor::EnterSleepMode() {
    unsigned char outBytes[32];
    memset(outBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DO2SENSOR_ADDR_DEFAULT) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas EZO-DO device.\n", strerror(errno));
        unlockmutex();
        return;
    }
    outBytes[0] = (unsigned char)'S';
    outBytes[1] = (unsigned char)'l';
    outBytes[2] = (unsigned char)'e';
    outBytes[3] = (unsigned char)'e';
    outBytes[4] = (unsigned char)'p';

    if (!write_i2c(outBytes, 5)) {
        unlockmutex();
        return;
    }
    //should now be in sleep mode
    unlockmutex();
 }

/**
 * @brief come out of sleep mode to be able to get more measurements
 *
 *
 */
void AtlasDO2Sensor::ExitSleepMode() {
    double dDummyDO2Measurement = 0.0;
    this->GetDO2(dDummyDO2Measurement);//send command for DO2 measurement, may not actually get it(?). Any character or command sent to DO2 sensor should wake it up.
}


/**
 * @brief lock or unlock i2c communications for the device
 * @param nLock 0 or 1. Use 1 to lock the protocol to I2C, or 0 to unlock it and allow it to change to serial communications.
 * @return true if the protocol was locked or unlocked successfully, false otherwise
 */
bool AtlasDO2Sensor::ProtocolLock(int nLock) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(outBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DO2SENSOR_ADDR_DEFAULT) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas EZO-DO device.\n", strerror(errno));
        unlockmutex();
        return false;
    }
    outBytes[0] = (unsigned char)'P';
    outBytes[1] = (unsigned char)'l';
    outBytes[2] = (unsigned char)'o';
    outBytes[3] = (unsigned char)'c';
    outBytes[4] = (unsigned char)'k';
    outBytes[5] = (unsigned char)',';
    outBytes[6] = (unsigned char)'1';
    if (nLock <= 0) {
        outBytes[6] = (unsigned char)'0';
    }

    if (!write_i2c(outBytes, 7)) {
        unlockmutex();
        return false;
    }

    //need to pause for 300 ms (see do_ezo_datasheet-1.pdf document)
    usleep(300000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 32);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from DO2 sensor.\n");
        unlockmutex();
        return false;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from DO2 sensor.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, DO2 sensor not ready.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to get DO2 data.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from DO2 sensor.\n");
        unlockmutex();
        return false;
    }
    unlockmutex();
    return true;
}