/**
 * @file IMU.cpp
 * @author Murray Lowery-Simpson (murraylowerysimpson@gmail.com)
 * @brief Implementation file for IMU class (used for getting inertial data from the AltIMU-10 v5 Gyro, Accelerometer, Compass, and Altimeter from Pololu Electronics (www.pololu.com))
 * @version 0.1
 * @date 2018-10-26
 * 
 * @copyright Copyright (c) 2018
 * 
 */

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
#include <math.h>
#include "IMU.h"
#include "Util.h"
#include "filedata.h"


/**
 * @brief Construct a new IMU::IMU object. The constructor tries to connect to the I2C channels used for the magnetometer, accelerometers / gyro, and the pressure sensor. Check the m_bMagInitialized_OK, m_bAccGyroInitialized_OK, and m_bPressureInitialized_OK variables after calling this constructor to verify that the devices were properly initialized.
 * @param i2c_mutex mutex controlling access to the i2c bus
 */
IMU::IMU(pthread_mutex_t *i2c_mutex) {//constructor
	m_i2c_mutex = i2c_mutex;
	m_quat = nullptr;
	m_dLastSampleTime=0.0;
	m_nGyroAxisOrder = 0;
	m_dBaseAccGyroTimestamp=0.0;
	m_dAccumulatedTimeSeconds=0.0;
	m_uiAccGyroSampleCount=0;
	//open I2C port for magnetometer
	char *i2c_filename = (char*)"/dev/i2c-1";
	m_bOpenedI2C_OK = true;
	m_bMagInitialized_OK = false;
	m_bAccGyroInitialized_OK = false;
	m_bPressureInitialized_OK=false;
	m_bInitError=false;
	pthread_mutex_lock(m_i2c_mutex);
	m_file_i2c = open(i2c_filename, O_RDWR);
	pthread_mutex_unlock(m_i2c_mutex);
	if (m_file_i2c<0) {
		//error opening I2C
		printf("Error: %s opening I2C.\n",strerror(errno));
		m_bOpenedI2C_OK=false;
		m_bInitError=true;
	}
	else {
		m_bMagInitialized_OK = InitializeMagDevice();//initialize LIS3MDL for sample rate, filtering, etc.
		if (!m_bMagInitialized_OK) {
			m_bInitError=true;
		}
		m_bAccGyroInitialized_OK = InitializeAccGyroDevice();//initialize LSM6DS33 for sample rate, filtering, etc.
		if (!m_bAccGyroInitialized_OK) {
			m_bInitError=true;
		}
	}
}

/**
 * @brief Destroy the IMU::IMU object
 * 
 */
IMU::~IMU() {//destructor
	if (m_quat!=nullptr) {
		delete m_quat;
		m_quat = nullptr;
	}
}

/**
 * @brief collect magnetometer data from the LIS3MDL 3-axis magnetometer device and process it to get the magnetic vector and temperature
 * 
 * @param pIMUSample pointer to a IMU_DATASAMPLE structure that holds the magnetometer and temperature data that are obtained from the LIS3MDL 3-axis magnetometer device.
 * @param nNumToAvg the number of individual samples to collect and average (use 1 for no averaging) 
 * @return true if function was successful and magnetometer and temperature data were properly collected
 * @return false if there was a problem getting either the magnetometer or temperature data
 */
bool IMU::GetMagSample(IMU_DATASAMPLE *pIMUSample, int nNumToAvg) {//collect magnetometer data from the LIS3MDL 3-axis magnetometer device and process it to get the magnetic vector and temperature
	double dTemperatureSum = 0.0;//the sum of all temperature samples received
	double mag_data_sum[3];//the sum of all magnetometer samples received
	
	memset(mag_data_sum,0,3*sizeof(double));
	
	double mag_data[3];//magnetometer data for the current reading
	double dTemperatureData=0.0;//temperature data for the current reading
	if (!m_bOpenedI2C_OK) return false;

	pthread_mutex_lock(m_i2c_mutex);

	if (ioctl(m_file_i2c, I2C_SLAVE, MAG_I2C_ADDRESS)<0) {
		printf("Failed (error = %s) to acquire bus access and/or talk to slave magnetometer.\n",strerror(errno));
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}

	if (nNumToAvg<1) {
		printf("Invalid number of samples to average.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	
	for (int i=0;i<nNumToAvg;i++) {
		if (!WaitForMagDataReady(MAG_STATUS_REG)) {
			printf("Timed out waiting for magnetometer data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!GetMagnetometerData(mag_data)) {
			printf("Error trying to get magnetometer data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!GetMagTemperatureData(dTemperatureData)) {
			printf("Error trying to get temperature data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		for (int j=0;j<3;j++) {
			mag_data_sum[j]+=mag_data[j];
		}
		dTemperatureSum+=dTemperatureData;
	}
	pthread_mutex_unlock(m_i2c_mutex);
	//divide by number of samples to get averaged results
	dTemperatureData = dTemperatureSum / nNumToAvg;
	for (int i=0;i<3;i++) {
		mag_data[i] = mag_data_sum[i] / nNumToAvg;
	}
	//copy data to IMU_DATASAMPLE structure
	pIMUSample->mag_temperature = dTemperatureData;
	memcpy(pIMUSample->mag_data,mag_data,3*sizeof(double));
	return true;
}

bool IMU::InitializeMagDevice() {//initialize LIS3MDL for sample rate, full-scale range, etc.
	unsigned char out_buf[2];
	if (!m_bOpenedI2C_OK) return false;
	pthread_mutex_lock(m_i2c_mutex);
	if (ioctl(m_file_i2c, I2C_SLAVE, MAG_I2C_ADDRESS)<0) {
		printf("Failed (error = %s) to acquire bus access and/or talk to slave magnetometer.\n",strerror(errno));
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}	
	//set MAG_CTRL_REG1 (0x20) for temperature enable, ultra-high-performance mode (for X & Y), not the highest possible data rate (80 Hz only) and disable self-test
	out_buf[0] = MAG_CTRL_REG1;
	out_buf[1] = 0xfc;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_CTRL_REG1.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set MAG_CTRL_REG2 (0x21) for full-scale range of mags of +/- 4 gauss
	out_buf[0] = MAG_CTRL_REG2;
	out_buf[1] = 0x00;
	if (write(m_file_i2c,  out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_CTRL_REG2.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set MAG_CTRL_REG3 (0x22) for continuous conversion, normal power mode 
	out_buf[0] = MAG_CTRL_REG3;
	out_buf[1] = 0x00;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_CTRL_REG3.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set MAG_CTRL_REG4 (0x23) for ultra-high-performance mode on the z-axis
	out_buf[0] = MAG_CTRL_REG4;
	out_buf[1] = 0x0C;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_CTRL_REG4.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	} 
	LoadMagCal();//load magnetometer offset calibration (if available) from mag_cal.txt file
	ReadMagOffsets();//read in and print out mag offsets stored in offset registers
	pthread_mutex_unlock(m_i2c_mutex);
	return true;
}

bool IMU::GetMagTemperatureData(double &dTemperatureData) {//get temperature data from the LIS3MDL (function assumes that temperature data is ready, and that magnetometer currently has I2C bus access)
	//dTemperatureData = the returned temperature in degrees C 
	//function returns true if successful, false otherwise
	//and least significant bytes are not in sync
	const double DEG_C_PER_BIT = .125;//output of temperature sensor in deg C per bit
	const double ROOM_TEMP = 25.0;//room temperature in deg C 
	unsigned char outBuf[1];//buffer for requesting data over I2C
	unsigned char inBuf[2];//buffer for receiving data over I2C
	//least significant byte (8 bits) of temperature
	outBuf[0] = MAG_TEMP_OUT_L;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_TEMP_OUT_L.\n");
		return false;
	}
	if (read(m_file_i2c,&inBuf[0],1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read MAG_TEMP_OUT_L from the I2C bus.\n");
		return false;
	}
	outBuf[0] = MAG_TEMP_OUT_H;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_TEMP_OUT_H.\n");
		return false;
	}
	if (read(m_file_i2c,&inBuf[1],1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read MAG_TEMP_OUT_H from the I2C bus.\n");
		return false;
	}

	int nTemperatureCounts = ((inBuf[1]&0x0f)<<8) + inBuf[0];
	
	bool bLessThanRoomTemp = false;
	if (nTemperatureCounts>=0x800) {//temperature is less than 25 deg C (room temperature)
		bLessThanRoomTemp = true;
		nTemperatureCounts = 4096 - nTemperatureCounts;
		dTemperatureData = ROOM_TEMP - ((double)nTemperatureCounts)/8;
	}
	else {//temperature is greater than (or equal to) room temp
		dTemperatureData = ROOM_TEMP + ((double)nTemperatureCounts)/8;
	}
	dTemperatureData-=MAG_SENSOR_TEMPOFFSET;//subtract magnetometer temperature sensor offset
	return true;
}

bool IMU::GetMagnetometerData(double *mag_data) {//get magnetometer data from the LIS3MDL
	if (!Get6BytesRegData(mag_data, MAG_OUTX_L)) {
		printf("Error getting magnetometer data.\n");
		return false;
	}
	//negate x and y axes to match accelerometer data
	mag_data[0] = -mag_data[0];
	mag_data[1] = -mag_data[1];

	normalize(mag_data);
	return true;	
}

bool IMU::Get6BytesRegData(double *data, int nBaseRegAddr) {//request 6 bytes of register data starting at nBaseRegAddr
	//data = pointer to the returned data, an array of 3 numeric values
	//nBaseRegAddr = the base register address where the data is located
	unsigned char outBuf[1];
	unsigned char inBuf[2];
	//byte 1
	outBuf[0] = nBaseRegAddr;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	if (read(m_file_i2c, inBuf,1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
		return false;
	}
	//byte 2
	outBuf[0] = nBaseRegAddr+1;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	if (read(m_file_i2c, &inBuf[1],1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
		return false;
	}
	//combine bytes 1 and 2
	data[0] = (double)Get16BitTwosComplement(inBuf[1], inBuf[0]);
	
	//byte 3
	outBuf[0] = nBaseRegAddr+2;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	if (read(m_file_i2c, inBuf,1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
		return false;
	}
	//byte 4
	outBuf[0] = nBaseRegAddr+3;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	if (read(m_file_i2c, &inBuf[1],1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
		return false;
	}
	//combine bytes 3 and 4
	data[1] = (double)Get16BitTwosComplement(inBuf[1], inBuf[0]);
		
	//byte 5
	outBuf[0] = nBaseRegAddr+4;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	if (read(m_file_i2c, inBuf,1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
		return false;
	}
	//byte 6
	outBuf[0] = nBaseRegAddr+5;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	if (read(m_file_i2c, &inBuf[1],1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
		return false;
	}
	//combine bytes 5 and 6
	data[2] = (double)Get16BitTwosComplement(inBuf[1], inBuf[0]);	
	return true;
}

int IMU::Get16BitTwosComplement(unsigned char highByte, unsigned char lowByte) {//convert two-byte value into a 16-bit twos-complement number (between -32767 and +32767)
	int nCountVal = (highByte<<8) + lowByte;
	if ((nCountVal&0x8000)>0) {//negative value
		nCountVal = -(65536 - nCountVal);
	}
	return nCountVal;
}

bool IMU::WaitForMagDataReady(unsigned char ucStatusReg) {//check 3 least sig bits of ucStatusReg to verify that they are all set (indicating that X, Y, Z data is read
	//ucStatusReg = the status register for this data (i.e. either STATUS_M for magnetometer data or STATUS_A for accelerometer data)
	const int TIMEOUT = 500;//length of time to wait for data (in ms) NOTE: must be less than 1 second timeout for this function
	unsigned char outBuf[1];
	unsigned char inBuf[1];
	//check status register to make sure that x-axis, y-axis, and z-axis data is ready
	bool bDataReady = false;
	long int start_time;
	long int time_difference;
	long int timeoutVal = ((long int)TIMEOUT)*1000000;//convert timeout value from ms to ns
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (!bDataReady)
	{
		outBuf[0] = ucStatusReg;
		if (write(m_file_i2c,outBuf,1)!=1) {
			//error, I2C transaction failed
			printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
			return false;
		}
		if (read(m_file_i2c, inBuf,1)!=1) {
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
			return false;
		}
		if ((inBuf[0]&0x07)==0x07) {//3 least significant bytes of status register are set, indicating that X, Y, Z data is ready
			bDataReady = true;
			return true;
		}
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0) {
			time_difference += 1000000000;//(Rolls over every 1 second)//NOTE: this code is not suitable for more than one rollover, i.e. for handling timeouts > 1 second
		}
		if (time_difference > timeoutVal) {
			return false;
		}
	}
	//should not actually get here	
	return false;
}

void IMU::normalize(double *vec) {//normalizes vec (if it is not a null vector) 
	//vec = the 3-dimensional vector to be normalized
	double dVecMag = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
	if (dVecMag==0.0) return;//null vector, don't do anything with it
	vec[0]/=dVecMag;
	vec[1]/=dVecMag;
	vec[2]/=dVecMag;
}

bool IMU::InitializeAccGyroDevice() {//initialize LSM6DS33 for sample rate, full-scale range, etc. 
	unsigned char out_buf[2];
	if (!m_bOpenedI2C_OK) return false;
	pthread_mutex_lock(m_i2c_mutex);
	if (ioctl(m_file_i2c, I2C_SLAVE, ACC_GYRO_I2C_ADDRESS)<0) {
		printf("Failed (error = %s) to acquire bus access and/or talk to slave accelerometer / gyro.\n",strerror(errno));
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}	
	//set ACC_CTRL1_XL 0x10, for output data rate (ODR) of 104 Hz, +/- 2 G full-scale,  accelerometer full-scale selection, anti-aliasing filter bandwidth of 50 Hz
	out_buf[0] = ACC_CTRL1_XL;
	out_buf[1] = 0x43;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for ACC_CTRL1_XL.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set GYRO_CTRL2_G 0x11 for ODR of 104 Hz, full-scale of 245 deg/sec
	out_buf[0] = GYRO_CTRL2_G;
	out_buf[1] = 0x40;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for GYRO_CTRL2_G.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set ACC_GYRO_CTRL3_C 0x12 for block data update (BDU) and automatic incrementing of register address when reading multiple bytes using I2C
	out_buf[0] = ACC_GYRO_CTRL3_C;
	out_buf[1] = 0x44;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for ACC_GYRO_CTRL3_C.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set ACC_GYRO_CTRL4_C 0x13 for accelerometer bandwidth setting
	out_buf[0] = ACC_GYRO_CTRL4_C;
	out_buf[1] = 0x80;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for ACC_GYRO_CTRL4_C.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set ACC_GYRO_CTRL6_C 0x15 for accelerometer high performance mode
	out_buf[0] = ACC_GYRO_CTRL6_C;
	out_buf[1] = 0x00;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for ACC_GYRO_CTRL6_C.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set GYRO_CTRL7_G, 0x16 for gyro high performance mode, enable gyro high pass filter, set gyro high pass filter for 0.0324 Hz
	out_buf[0] = GYRO_CTRL7_G;
	out_buf[1] = 0x50;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for GYRO_CTRL7_G.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set ACC_CTRL8_XL to enable low pass acc filter
	out_buf[0] = ACC_CTRL8_XL;
	out_buf[1] = 0x80;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for ACC_CTRL8_XL.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set WAKE_UP_DUR, 0x5C for timer resolution of 25 usec per bit
	out_buf[0] = WAKE_UP_DUR;
	out_buf[1] = 0x10;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for WAKE_UP_DUR.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//set TAP_CFG, 0x58 to enable timestamps
	out_buf[0] = TAP_CFG;
	out_buf[1] = 0x80;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for TAP_CFG.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	pthread_mutex_unlock(m_i2c_mutex);
	return true;
}

/**
 * @brief collect accelerometer & gyro data from the LSM6DS33 and process it to get the acceleration vector, rotation rate vector, and temperature
 * 
 * @param pIMUSample contains the returned acceleration vector (X,Y,Z) expressed in G, the angular rate vector (RX,RY,RZ) expressed in degrees per sec, the temperature (in degrees C), and the time of the sample in seconds.
 * @param nNumToAvg the number of individual samples to collect and average (use 1 for no averaging) 
 * @return true if function was successful and accelerometer, gyro, and temperature data were properly collected
 * @return false if there was a problem getting either the accelerometer, gyro, or temperature data
 */
bool IMU::GetAccGyroSample(IMU_DATASAMPLE *pIMUSample, int nNumToAvg) {
	double dTemperatureSum = 0.0;//the sum of all temperature samples received
	double acc_data_sum[3];//the sum of all accelerometer samples received
	double gyro_data_sum[3];//the sum of all gyro samples received
	double acc_data[3];//an individual sample of accelerometer data
	double gyro_data[3];//an individual sample of gyro data
	double dTemperature=0.0;//an individual temperature sample
	unsigned char outBuf[1];
	unsigned char inBuf[3];
	
	memset(acc_data_sum,0,3*sizeof(double));
	memset(gyro_data_sum,0,3*sizeof(double));
	memset(acc_data,0,3*sizeof(double));
	memset(gyro_data,0,3*sizeof(double));
	
	double dTemperatureData=0.0;//temperature data for the current reading
	if (!m_bOpenedI2C_OK) return false;
	pthread_mutex_lock(m_i2c_mutex);
	if (ioctl(m_file_i2c, I2C_SLAVE, ACC_GYRO_I2C_ADDRESS)<0) {
		printf("Failed (error = %s) to acquire bus access and/or talk to slave acc/gyro device.\n",strerror(errno));
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	if (nNumToAvg<1) {
		printf("Invalid number of samples to average.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	for (int i=0;i<nNumToAvg;i++) {
		if (!WaitForAccDataReady(ACC_GYRO_STATUS_REG)) {
			printf("Timed out waiting for accelerometer data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!GetAccData(acc_data)) {
			printf("Error trying to get accelerometer data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!WaitForGyroDataReady(ACC_GYRO_STATUS_REG)) {
			printf("Timed out waiting for gyro data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!GetGyroData(gyro_data)) {
			printf("Error trying to get gyro data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!WaitForAccTemperatureData(ACC_GYRO_STATUS_REG)) {
			printf("Timed out waiting for temperature data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		if (!GetAccTemperatureData(dTemperature)) {
			printf("Error trying to get temperature data.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		//sum results for later computation of average
		dTemperatureSum+=dTemperature;
		for (int j=0;j<3;j++) {
			acc_data_sum[j]+=acc_data[j];
			gyro_data_sum[j]+=gyro_data[j];
		}
	}
	//get sample timestamp
	outBuf[0] = TIMESTAMP0_REG;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	if (read(m_file_i2c, inBuf,3)!=3) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read timestamp bytes.\n");
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	double dTimestampCounts = (double)(inBuf[0]+(inBuf[1]<<8)+(inBuf[2]<<16));
	if (dTimestampCounts>=16000000) {//the timestamp counter will reach the end soon and needs to be manually reset since it does not automatically roll over.
		outBuf[0] = TIMESTAMP2_REG;
		outBuf[1] = 0xAA;
		if (write(m_file_i2c, outBuf,2)!=2) {
			//error, I2C transaction failed
			printf("Error, failed to send bytes to reset timer.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
		m_dAccumulatedTimeSeconds+=(dTimestampCounts*ACC_GYRO_TIMER_RESOLUTION);
		dTimestampCounts=0;
	}
	pthread_mutex_unlock(m_i2c_mutex);
	if (m_uiAccGyroSampleCount==0) { 
		m_dBaseAccGyroTimestamp = dTimestampCounts;
		m_dAccumulatedTimeSeconds=0.0;
	}
	pIMUSample->sample_time_sec = (dTimestampCounts - m_dBaseAccGyroTimestamp)*ACC_GYRO_TIMER_RESOLUTION + m_dAccumulatedTimeSeconds;
	m_uiAccGyroSampleCount++;

	//divide by number of samples to get averaged results
	pIMUSample->acc_gyro_temperature = dTemperatureSum / nNumToAvg;
	for (int i=0;i<3;i++) {
		pIMUSample->acc_data[i] = acc_data_sum[i] / nNumToAvg;
		pIMUSample->angular_rate[i] = gyro_data_sum[i] / nNumToAvg;
	}
	return true;
}

bool IMU::WaitForAccDataReady(unsigned char ucStatusReg) {//check XLDA bit of LSM6DS33 status register to see if the accelerometer data is ready
	//ucStatusReg = the status register (0x1E) for the LSM6DS33
	const int TIMEOUT = 500;//length of time to wait for data (in ms) NOTE: must be less than 1 second timeout for this function
	unsigned char outBuf[1];
	unsigned char inBuf[1];
	//check status register to make sure that gyro data is ready
	bool bDataReady = false;
	long int start_time;
	long int time_difference;
	long int timeoutVal = ((long int)TIMEOUT)*1000000;//convert timeout value from ms to ns
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (!bDataReady)
	{
		outBuf[0] = ucStatusReg;
		if (write(m_file_i2c,outBuf,1)!=1) {
			//error, I2C transaction failed
			printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
			return false;
		}
		if (read(m_file_i2c, inBuf,1)!=1) {
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
			return false;
		}
		if ((inBuf[0]&0x01)>0) {//the XLDA bit of the status register is set, indicating that a new set of accelerometer data is available
			bDataReady = true;
			return true;
		}
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0) {
			time_difference += 1000000000;//(Rolls over every 1 second)//NOTE: this code is not suitable for more than one rollover, i.e. for handling timeouts > 1 second
		}
		if (time_difference > timeoutVal) {
			return false;
		}
	}
	//should not actually get here	
	return false;
}

bool IMU::WaitForGyroDataReady(unsigned char ucStatusReg) {//check GDA bit of LSM6DS33 status register to see if the gyro data is ready
	//ucStatusReg = the status register (0x1E) for the LSM6DS33
	const int TIMEOUT = 500;//length of time to wait for data (in ms) NOTE: must be less than 1 second timeout for this function
	unsigned char outBuf[1];
	unsigned char inBuf[1];
	//check status register to make sure that gyro data is ready
	bool bDataReady = false;
	long int start_time;
	long int time_difference;
	long int timeoutVal = ((long int)TIMEOUT)*1000000;//convert timeout value from ms to ns
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (!bDataReady)
	{
		outBuf[0] = ucStatusReg;
		if (write(m_file_i2c,outBuf,1)!=1) {
			//error, I2C transaction failed
			printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
			return false;
		}
		if (read(m_file_i2c, inBuf,1)!=1) {
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
			return false;
		}
		if ((inBuf[0]&0x02)>0) {//the GDA bit of the status register is set, indicating that a new set of gyro data is available
			bDataReady = true;
			return true;
		}
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0) {
			time_difference += 1000000000;//(Rolls over every 1 second)//NOTE: this code is not suitable for more than one rollover, i.e. for handling timeouts > 1 second
		}
		if (time_difference > timeoutVal) {
			return false;
		}
	}
	//should not actually get here	
	return false;
}

bool IMU::WaitForAccTemperatureData(unsigned char ucStatusReg) {//check TDA bit of LSM6DS33 status register to see if the temperature data is ready
	//ucStatusReg = the status register (0x1E) for the LSM6DS33
	const int TIMEOUT = 500;//length of time to wait for data (in ms) NOTE: must be less than 1 second timeout for this function
	unsigned char outBuf[1];
	unsigned char inBuf[1];
	//check status register to make sure that gyro data is ready
	bool bDataReady = false;
	long int start_time;
	long int time_difference;
	long int timeoutVal = ((long int)TIMEOUT)*1000000;//convert timeout value from ms to ns
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (!bDataReady)
	{
		outBuf[0] = ucStatusReg;
		if (write(m_file_i2c,outBuf,1)!=1) {
			//error, I2C transaction failed
			printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
			return false;
		}
		if (read(m_file_i2c, inBuf,1)!=1) {
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read register %d from the I2C bus.\n",(int)outBuf[0]);
			return false;
		}
		if ((inBuf[0]&0x04)>0) {//the TDA bit of the status register is set, indicating that a new set of temperature data is available
			bDataReady = true;
			return true;
		}
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0) {
			time_difference += 1000000000;//(Rolls over every 1 second)//NOTE: this code is not suitable for more than one rollover, i.e. for handling timeouts > 1 second
		}
		if (time_difference > timeoutVal) {
			return false;
		}
	}
	//should not actually get here	
	return false;
}

bool IMU::GetAccData(double *acc_data) {//get accelerometer data from the LSM6DS33
	unsigned char outBuf[1];
	unsigned char inBuf[6];
	outBuf[0] = OUTX_L_XL;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	//read in 6 bytes from accelerometers
	int nNumRead = read(m_file_i2c,inBuf,6);
	if (nNumRead!=6) {
		//error, I2C transaction failed
		printf("Error, failed to get 6 bytes of accelerometer data.\n");
		return false;
	}
	//accelerations expressed in counts
	double acc_counts[3];
	acc_counts[0] = Get16BitTwosComplement(inBuf[1], inBuf[0]);
	acc_counts[1] = Get16BitTwosComplement(inBuf[3], inBuf[2]);
	acc_counts[2] = Get16BitTwosComplement(inBuf[5], inBuf[4]);

	//normalize counts to tilts in G
	normalize(acc_counts);
	//change sign of accZ (to match previously used LM303D compass module)
	acc_counts[2] = -acc_counts[2];
	//copy to acc_data
	memcpy(acc_data,acc_counts,3*sizeof(double));
	return true;
}

bool IMU::GetGyroData(double *gyro_data) {//get gyro data from the LSM6DS33
	unsigned char outBuf[1];
	unsigned char inBuf[6];
	outBuf[0] = OUTX_L_G;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	//read in 6 bytes from gyros
	int nNumRead = read(m_file_i2c,inBuf,6);
	if (nNumRead>0&&nNumRead<6) {
		//error, I2C transaction failed
		printf("Error, only %d of 6 bytes of gyro data read.\n",nNumRead);
		return false;
	}
	else if (nNumRead<=0) {
		printf("Error, timed out trying to read gyro data.\n");
		return false;
	}
	//angular rates expressed in counts
	double ang_rate_counts[3];
	ang_rate_counts[0] = Get16BitTwosComplement(inBuf[1], inBuf[0]);
	ang_rate_counts[1] = Get16BitTwosComplement(inBuf[3], inBuf[2]);
	ang_rate_counts[2] = Get16BitTwosComplement(inBuf[5], inBuf[4]);

	//convert counts to angular rates in deg/sec
	gyro_data[0] = ang_rate_counts[0] * GYRO_GAIN;
	gyro_data[1] = ang_rate_counts[1] * GYRO_GAIN;
	gyro_data[2] = ang_rate_counts[2] * GYRO_GAIN;
	return true;
}

bool IMU::GetAccTemperatureData(double &dTemperatureData) {//get temperature data from the LSM6DS33
	unsigned char outBuf[1];
	unsigned char inBuf[2];
	outBuf[0] = OUT_TEMP_L;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return false;
	}
	//read in 2 bytes from temperature sensor on LSM6DS33
	int nNumRead = read(m_file_i2c,inBuf,2);
	if (nNumRead!=2) {
		//error, I2C transaction failed
		printf("Error, could not read 2 bytes of temperature data.\n");
		return false;
	}
	//angular rates expressed in counts
	double dTempCounts = Get16BitTwosComplement(inBuf[1], inBuf[0]);

	//convert counts to temperature in deg C
	dTemperatureData = 25.0 + dTempCounts / 16.0;
	return true;
}

/**
 * @brief reset the timestamps for the acc/gyro measurements back to zero seconds
 * 
 */
void IMU::ResetAccGyro() {
	m_dBaseAccGyroTimestamp=0.0;
	m_uiAccGyroSampleCount=0;
	m_dAccumulatedTimeSeconds=0.0;
}

void IMU::ReadMagOffsets() {//read in and print out mag offsets stored in offset registers
	unsigned char outBuf[1];
	unsigned char inBuf[6];
	outBuf[0] = MAG_OFFSET_X_L;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus register %d.\n",(int)outBuf[0]);
		return;
	}
	//read in 6 bytes for mag offsets
	int nNumRead = read(m_file_i2c,inBuf,6);
	if (nNumRead!=6) {
		//error, I2C transaction failed
		printf("Error, only %d of 6 bytes of mag offset data read.\n",nNumRead);
		return;
	}
	//mag_offsets expressed in counts
	double mag_offsets[3];
	mag_offsets[0] = Get16BitTwosComplement(inBuf[1], inBuf[0]);
	mag_offsets[1] = Get16BitTwosComplement(inBuf[3], inBuf[2]);
	mag_offsets[2] = Get16BitTwosComplement(inBuf[5], inBuf[4]);
	printf("mag_offsets = %.0f, %.0f, %.0f\n",mag_offsets[0],mag_offsets[1],mag_offsets[2]);
}

/**
 * @brief perform a calibration procedure on the magnetometers to get the zero-field offsets for each of the sensors. Saves the results to the offset registers.
 * 
 * @return true if the calibration was completed successfully
 * @return false if there was some sort of error in completing the calibration
 */
bool IMU::DoMagCal() {//perform a calibration procedure on the magnetometers to get the zero-field offsets for each of the sensors. Saves the results to the offset registers.
	printf("Press \'q\' to quit or \'c\' to calibrate...\n");
	double mag_data[NUM_MAGCAL_AVG][3];//do running average of 50 mag samples
	double avg_mag[3];//averaged magnetometer values
	unsigned char out_buf[2];
	memset(mag_data,0,NUM_MAGCAL_AVG*3*sizeof(double));	
	memset(avg_mag,0,3*sizeof(double));//averaged magnetometer values
	//initialize min and max sensor outputs to large and small values respectively
	double dMinMagX = 32767.0, dMaxMagX = -32767.0;
	double dMinMagY = 32767.0, dMaxMagY = -32767.0;
	double dMinMagZ = 32767.0, dMaxMagZ = -32767.0;
	int nKeyPressed = Util::getch_noblock();
	if (!m_bMagInitialized_OK) {
		printf("Error, magnetometer was not properly initialized. Cannot do calibration.\n");
		return false;
	}
	pthread_mutex_lock(m_i2c_mutex);
	if (ioctl(m_file_i2c, I2C_SLAVE, MAG_I2C_ADDRESS)<0) {
		printf("Failed (error = %s) to acquire bus access and/or talk to slave magnetometer.\n",strerror(errno));
		pthread_mutex_unlock(m_i2c_mutex);
		return false;
	}
	//zero mag offset registers
	for (int i=0;i<6;i++) {
		out_buf[0] = (unsigned char)(MAG_OFFSET_X_L+i);
		out_buf[1] = 0x00;
		if (write(m_file_i2c, out_buf, 2)!=2) {
			//error, I2C transaction failed
			printf("Failed to write to the I2C bus for zeroing mag offsets.\n");
			pthread_mutex_unlock(m_i2c_mutex);
			return false;
		}
	}

	int nSampleNum=0;
	while (nKeyPressed!=113&&nKeyPressed!=99) {//keep looping, looking for new max or min values until the 'c' for calibrate or 'q' for quit keys are pressed
		if (WaitForMagDataReady(MAG_STATUS_REG)) {
			if (Get6BytesRegData(mag_data[nSampleNum%NUM_MAGCAL_AVG], MAG_OUTX_L)) {
				nSampleNum++;
				if (GetAvgMagVals(mag_data, avg_mag, nSampleNum))
				{
					//check to see if there are new min or max values
					if (avg_mag[0] < dMinMagX)
					{
						dMinMagX = avg_mag[0];
					}
					if (avg_mag[0] > dMaxMagX)
					{
						dMaxMagX = avg_mag[0];
					}
					if (avg_mag[1] < dMinMagY)
					{
						dMinMagY = avg_mag[1];
					}
					if (avg_mag[1] > dMaxMagY)
					{
						dMaxMagY = avg_mag[1];
					}
					if (avg_mag[2] < dMinMagZ)
					{
						dMinMagZ = avg_mag[2];
					}
					if (avg_mag[2] > dMaxMagZ)
					{
						dMaxMagZ = avg_mag[2];
					}
				}
				//print out new min/max values
				printf("\rMin: %.0f, %.0f, %.0f;   Max: %.0f, %.0f, %.0f; Cur: %.0f, %.0f, %.0f                ", dMinMagX, dMinMagY, dMinMagZ, dMaxMagX, dMaxMagY, dMaxMagZ, avg_mag[0], avg_mag[1], avg_mag[2]);
				fflush(stdout);
			}	
		}
		nKeyPressed = Util::getch_noblock();
	}
	printf("\n");
	if (nKeyPressed==99) {//c for calibrate, need to calibrate mag offsets based on min and max values
		double mag_offsets[3];
		mag_offsets[0] = (dMinMagX+dMaxMagX)/2;
		mag_offsets[1] = (dMinMagY+dMaxMagY)/2;
		mag_offsets[2] = (dMinMagZ+dMaxMagZ)/2;
		if (!SaveMagOffsets(mag_offsets[0],
					  		mag_offsets[1], 
					   		mag_offsets[2])) {//store magnetometer offsets to mag offset registers
			pthread_mutex_unlock(m_i2c_mutex);				   
			return false;
		}
		//save offsets to text calibration file
		filedata magCalFile((char *)"./mag_cal.txt");
		magCalFile.writeData((char *)"[calibration]",(char *)"mag_offsets",3,mag_offsets);

		printf("Calibration offsets: %.0f, %.0f, %.0f stored successfully.\n",mag_offsets[0],mag_offsets[1],mag_offsets[2]);
	}
	pthread_mutex_unlock(m_i2c_mutex);
	return true;
}

bool IMU::GetAvgMagVals(double mag_data[NUM_MAGCAL_AVG][3], double avg_mag[3], int nNumSamples) {//get averaged magnetometer values from last NUM_MAGCAL_AVG samples stored in mag_data
	int nNumToAvg = NUM_MAGCAL_AVG;
	if (nNumSamples<nNumToAvg) {
		return false;//not enough samples yet
	}
	double mag_totals[3] = {0.0, 0.0, 0.0};
	for (int i=0;i<nNumToAvg;i++) {
		mag_totals[0]+=mag_data[i][0];
		mag_totals[1]+=mag_data[i][1];
		mag_totals[2]+=mag_data[i][2];
	}
	avg_mag[0] = mag_totals[0]/nNumToAvg;
	avg_mag[1] = mag_totals[1]/nNumToAvg;
	avg_mag[2] = mag_totals[2]/nNumToAvg;
	return true;
}

void IMU::LoadMagCal() {//load magnetometer offset calibration (if available) from mag_cal.txt file
	filedata magCal((char *)"./mag_cal.txt");
	double mag_offsets[3];
	magCal.getDouble((char *)"[calibration]",(char *)"mag_offsets",3,mag_offsets);
	SaveMagOffsets(mag_offsets[0],mag_offsets[1],mag_offsets[2]);
}

bool IMU::SaveMagOffsets(double dMagOffsetX, double dMagOffsetY, double dMagOffsetZ) {//store magnetometer offsets to mag offsets registers
	unsigned char out_buf[2] = {0,0};
	int nMagOffsetX = (int)(dMagOffsetX);
	int nMagOffsetY = (int)(dMagOffsetY);
	int nMagOffsetZ = (int)(dMagOffsetZ);
	int nOriginalOffX = nMagOffsetX;
	int nOriginalOffY = nMagOffsetY;
	int nOriginalOffZ = nMagOffsetZ;

	//convert to 2's complement values
	unsigned char magOffX[2];
	unsigned char magOffY[2];
	unsigned char magOffZ[2];
	if (nMagOffsetX < 0)
	{
		nMagOffsetX = 65536 + nMagOffsetX;
	}
	if (nMagOffsetY < 0)
	{
		nMagOffsetY = 65536 + nMagOffsetY;
	}
	if (nMagOffsetZ < 0)
	{
		nMagOffsetZ = 65536 + nMagOffsetZ;
	}
	magOffX[0] = (unsigned char)(nMagOffsetX & 0x00ff);		   //x-axis low-order byte
	magOffX[1] = (unsigned char)((nMagOffsetX & 0xff00) >> 8); //x-axis high-order byte
	magOffY[0] = (unsigned char)(nMagOffsetY & 0x00ff);		   //y-axis low-order byte
	magOffY[1] = (unsigned char)((nMagOffsetY & 0xff00) >> 8); //y-axis high-order byte
	magOffZ[0] = (unsigned char)(nMagOffsetZ & 0x00ff);		   //z-axis low-order byte
	magOffZ[1] = (unsigned char)((nMagOffsetZ & 0xff00) >> 8); //z-axis high-order byte
	out_buf[0] = (unsigned char)(MAG_OFFSET_X_L);
	out_buf[1] = magOffX[0];
	if (write(m_file_i2c, out_buf, 2) != 2)
	{
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_OFFSET_X_L calibration byte.\n");
		return false;
	}
	out_buf[0] = (unsigned char)(MAG_OFFSET_X_H);
	out_buf[1] = magOffX[1];
	if (write(m_file_i2c, out_buf, 2) != 2)
	{
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_OFFSET_X_H calibration byte.\n");
		return false;
	}
	out_buf[0] = (unsigned char)(MAG_OFFSET_Y_L);
	out_buf[1] = magOffY[0];
	if (write(m_file_i2c, out_buf, 2) != 2)
	{
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_OFFSET_Y_L calibration byte.\n");
		return false;
	}
	out_buf[0] = (unsigned char)(MAG_OFFSET_Y_H);
	out_buf[1] = magOffY[1];
	if (write(m_file_i2c, out_buf, 2) != 2)
	{
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_OFFSET_Y_H calibration byte.\n");
		return false;
	}
	out_buf[0] = (unsigned char)(MAG_OFFSET_Z_L);
	out_buf[1] = magOffZ[0];
	if (write(m_file_i2c, out_buf, 2) != 2)
	{
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_OFFSET_Z_L calibration byte.\n");
		return false;
	}
	out_buf[0] = (unsigned char)(MAG_OFFSET_Z_H);
	out_buf[1] = magOffZ[1];
	if (write(m_file_i2c, out_buf, 2) != 2)
	{
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for MAG_OFFSET_Z_H calibration byte.\n");
		return false;
	}
	return true;//offsets stored successfully
}

/**
 * @brief compute orientation (pitch, roll, and heading angles) of the AltIMU-10, using acc/mag data plus gyros. The orientation angles are saved in the IMU_DATASAMPLE structure that is passed to the function.
 * 
 * @param pSample pointer to a structure that holds the computed heading, pitch, roll angles. This structure should contain valid acceleration acceleration (X,Y,Z, in G), magnetometer (X,Y,Z, normalized units), and angular rate (RX,RY,RZ, deg/s) data prior to calling this function.
 */
void IMU::ComputeOrientation(IMU_DATASAMPLE *pSample) {
	const double RAD_TO_DEG = 57.29578;
	const double DEG_TO_RAD = 0.01745329251994;
	const double SLERP_FACTOR = 0.97;
	double mag_data[3];
	double acc_data[3];
	double gyro_data[3];
	//copy data from IMU_DATASAMPLE structure
	memcpy(mag_data,pSample->mag_data,3*sizeof(double));
	memcpy(acc_data,pSample->acc_data,3*sizeof(double));
	memcpy(gyro_data,pSample->angular_rate,3*sizeof(double));
	double dPitchAngleRad = asin(-acc_data[0]);//pitch angle in radians
	double dCosFactor = cos(dPitchAngleRad);
	double dRollAngleRad = 0.0;//roll angle in radians
	if (dCosFactor!=0.0) {
		dRollAngleRad = asin(acc_data[1] / dCosFactor);
	}
	double mx2 = mag_data[0]*cos(dPitchAngleRad) - mag_data[2]*sin(dPitchAngleRad);
	double my2 = mag_data[0]*sin(dPitchAngleRad)*sin(dRollAngleRad) + mag_data[1]*cos(dRollAngleRad) + mag_data[2]*cos(dPitchAngleRad)*sin(dRollAngleRad);
	double dHeadingAngleRad = atan2(my2,mx2);//heading angle in radians
	//copy to pSample (and convert angles to degrees)
	double dPitch = dPitchAngleRad*RAD_TO_DEG;
	double dRoll = dRollAngleRad*RAD_TO_DEG;
	double dHeading = dHeadingAngleRad*RAD_TO_DEG;
	//make sure heading is between 0 and 360
	if (dHeading<0) dHeading+=360;
	else if (dHeading>360) dHeading-=360;
	quaternion2 accMagQuat(-dRoll,-dPitch,-dHeading);
	if (m_quat==nullptr) {
		m_quat = new quaternion2(accMagQuat);
	}
	else if (m_dLastSampleTime==0.0) {
		*m_quat = accMagQuat;
	}
	else {//combine gyro and acc/mag results together using spherical linear interpolation
		vector3d vx(1,0,0);
		vector3d vy(0,1,0);
		vector3d vz(0,0,1);
		double dTimeElapsedSec = pSample->sample_time_sec - this->m_dLastSampleTime;
		double dIncRoll = -gyro_data[0]*dTimeElapsedSec;//incremental roll change
		double dIncPitch = -gyro_data[1]*dTimeElapsedSec;//incremental pitch change
		double dIncYaw = gyro_data[2]*dTimeElapsedSec;//incremental yaw change
		quaternion2 qx(vx, dIncRoll * DEG_TO_RAD);
		quaternion2 qz(vz, dIncPitch * DEG_TO_RAD);
		quaternion2 qy(vy, dIncYaw * DEG_TO_RAD);
		quaternion2 incQuat;
		if (m_nGyroAxisOrder==0) {
			incQuat = (qx*qy)*qz;
		}
		else if (m_nGyroAxisOrder==1) {
			incQuat = (qy*qz)*qx;
		}
		else {
			incQuat = (qz*qz)*qy;
		}
		quaternion2 rotatedQuat = *m_quat * incQuat;
		rotatedQuat.slerp(accMagQuat,1.0-SLERP_FACTOR);
		*m_quat = rotatedQuat;
		m_nGyroAxisOrder = (m_nGyroAxisOrder+1)%3;
	}
	m_dLastSampleTime = pSample->sample_time_sec;
	double dCurrentHeading=0.0, dCurrentPitch=0.0, dCurrentRoll=0.0;
	tmatrix mat = m_quat->getRotMatrix();
	mat.getAMOSRPY(dCurrentRoll,dCurrentPitch,dCurrentHeading);
	//copy orientation angles to IMU_DATASAMPLE structure
	pSample->heading = dCurrentHeading;
	pSample->pitch = dCurrentPitch;
	pSample->roll = dCurrentRoll;
}

bool IMU::GetSample(IMU_DATASAMPLE *pIMUSample, int nNumToAvg) {//collect magnetometer, accelerometer, and gyro data, and then call ComputeOrientation to determine orientation angles
	if (!GetMagSample(pIMUSample, nNumToAvg)) {//collect magnetometer data from the LIS3MDL 3-axis magnetometer device and process it to get the magnetic vector and temperature
		return false;
	}
	if (!GetAccGyroSample(pIMUSample, nNumToAvg)) {//collect accelerometer & gyro data from the LSM6DS33 and process it to get the acceleration vector, rotation rate vector, and temperature
		return false;
	}
	this->ComputeOrientation(pIMUSample);
	return true;
}