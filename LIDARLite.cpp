/**
 * @file LIDARLite.cpp
 * @author Murray Lowery-Simpson (murray@innaturerobotics.com)
 * @brief Implementation file for LIDARLite class
 * @version 0.1
 * @date 2019-08-12
 * 
 * @copyright Copyright (c) 2019 In Nature Robotics Ltd.
 * 
 */

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <iostream>
#include <wiringPi.h>
#include <cstring>
#include "LIDARLite.h"




/**
 * @brief Construct a new LIDARLite::LIDARLite object
 * 
 * @param i2c_mutex mutex controlling access to the i2c bus
 */
LIDARLite::LIDARLite(pthread_mutex_t *i2c_mutex) {
    //make sure LiDAR is on after this object is contructed
    m_bStopped = false;
    TurnOn(true);
    m_i2c_mutex = i2c_mutex;
    //open I2C port for LiDAR device
	char *i2c_filename = (char*)"/dev/i2c-1";
	m_bOpenedI2C_OK = true;
	m_bLidarInitialized_OK = false;
	m_bInitError=false;
	lockmutex();
	m_file_i2c = open(i2c_filename, O_RDWR);
	unlockmutex();
	if (m_file_i2c<0) {
		//error opening I2C
		printf("Error: %s opening I2C.\n",strerror(errno));
		m_bOpenedI2C_OK=false;
		m_bInitError=true;
	}
	else {
		m_bLidarInitialized_OK = configure(MAXIMUM_RANGE);//initialize LiDAR for default mode
		if (!m_bLidarInitialized_OK) {
            printf("Error trying to configure LiDAR device.\n");
			m_bInitError=true;
		}
	}
}

/**
 * @brief Select one of 6 possible preset configurations for the LiDAR device
 * 
 * @param nOperationMode the operating mode of the device, must be one of the following: DEFAULT_MODE, SHORTRANGE_HIGHSPEED, HIGHSPEED_SHORTRANGE, MAXIMUM_RANGE, HIGH_SENSITIVITY, or LOW_SENSITIVITY
 * 
 * @return true if command was successfully sent to configure LiDAR
 * @return false if there was a problem sending the  command to configure the LiDAR.
 */
bool LIDARLite::configure(int nOperationMode) {
    if (!m_bOpenedI2C_OK) return false;
    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, LIDARLITE_ADDR_DEFAULT)<0) {
	    printf("Failed (error = %s) to acquire bus access and/or talk to slave LiDAR device.\n",strerror(errno));
	    unlockmutex();
	    return false;
	}
    if (nOperationMode==DEFAULT_MODE) {
        if (!write_itc(0x02,0x80)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x04,0x08)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x1c,0x00)) {
            unlockmutex();
            return false;
        }
    }
    else if (nOperationMode==SHORTRANGE_HIGHSPEED) {
        if (!write_itc(0x02,0x1d)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x04,0x08)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x1c,0x00)) {
            unlockmutex();
            return false;
        }
    }
    else if (nOperationMode==HIGHERSPEED_SHORTRANGE) {
        if (!write_itc(0x02,0x80)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x04,0x00)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x1c,0x00)) {
            unlockmutex();
            return false;
        }
    }
    else if (nOperationMode==MAXIMUM_RANGE) {
        if (!write_itc(0x02,0xff)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x04,0x08)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x1c,0x00)) {
            unlockmutex();
            return false;
        }
    }
    else if (nOperationMode==HIGH_SENSITIVITY) {
        if (!write_itc(0x02,0x80)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x04,0x08)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x1c,0x80)) {
            unlockmutex();
            return false;
        }
    }
    else if (nOperationMode==LOW_SENSITIVITY) {
        if (!write_itc(0x02,0x80)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x04,0x08)) {
            unlockmutex();
            return false;
        }
        if (!write_itc(0x1c,0xb0)) {
            unlockmutex();
            return false;
        }
    }
    unlockmutex();
    return true;
}

bool LIDARLite::write_itc(unsigned char ucRegister,unsigned char ucData) {//writes a single byte to a register using I2C
    unsigned char out_buf[2];
    out_buf[0] = ucRegister;
    out_buf[1] = ucData;
    if (write(m_file_i2c, out_buf, 2)!=2) {
        return false;
    }
    return true;
}


/**
 * @brief reset the device with default settings, takes approx. 22 ms.
 * 
 * @return true if command was sent successfully to reset the device
 * @return false if there was a problem sending the command to reset the device
 */
bool LIDARLite::reset() {//reset the device with default settings, takes approx. 22 ms.
    if (!m_bOpenedI2C_OK) return false;
    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, LIDARLITE_ADDR_DEFAULT)<0) {
	    printf("Failed (error = %s) to acquire bus access and/or talk to slave LiDAR device.\n",strerror(errno));
	    unlockmutex();
	    return false;
	}
    if (!write_itc(0x00,0x00)) {
        printf("Error sending command to reset device.\n");
        unlockmutex();
	    return false;
    }
    unlockmutex();
    return true;
}

/**
 * @brief take a distance measurement and read the result (in cm)
 * 
 * @return unsigned short distance to object in cm (returns distance of 1 cm if object cannot be found or < 0 if an error occurred)
 */
unsigned short LIDARLite::getDistance() {
    if (!m_bOpenedI2C_OK) return false;
    if (m_bStopped) {
        return 0;
    }
    unsigned char inBuf[2];
    unsigned char outBuf[1];
    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, LIDARLITE_ADDR_DEFAULT)<0) {
	    printf("Failed (error = %s) to acquire bus access and/or talk to slave LiDAR device.\n",strerror(errno));
	    unlockmutex();
	    return -1;
	}
    if (!write_itc(0x00,0x04)) {
        unlockmutex();
        return -2;
    }
    //request read from register 0x8f
	outBuf[0] = 0x8f;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		unlockmutex();
		return -3;
	}
	if (read(m_file_i2c, inBuf,2)!=2) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read distance bytes.\n");
		unlockmutex();
		return -4;
	}
    unlockmutex();
    unsigned short usDistance = (inBuf[0]<<8) + inBuf[1];
    return usDistance;//distance to object in cm (will return 1 cm if object cannot be found)
}

/**
 * @brief turn on / off device (uses orange wire to enable / disable power)
 * 
 * @param bTurnOn true if LiDAR device should be turned on, otherwise false
 */
void LIDARLite::TurnOn(bool bTurnOn) {
    if (bTurnOn) {
         //set LIDAR_ON_OFF_PIN pin to input, internal pullup in LiDAR device will make it high and enable power to the LiDAR
         m_bStopped = false;
         pinMode(LIDAR_ON_OFF_PIN, INPUT);  
    }
    else {
        m_bStopped = true;
        pinMode(LIDAR_ON_OFF_PIN, OUTPUT);//set pin to output
        digitalWrite(LIDAR_ON_OFF_PIN, LOW);//set pin low to disable power to the LiDAR device
    }
}


void LIDARLite::lockmutex() {//lock mutex for access to the I2C bus
    if (m_i2c_mutex!=nullptr) {
        pthread_mutex_lock(m_i2c_mutex);
    }
}


void LIDARLite::unlockmutex() {//unlock mutex for access to the I2C bus
    if (m_i2c_mutex!=nullptr) {
        pthread_mutex_unlock(m_i2c_mutex);
    }
}