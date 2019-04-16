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
#include "LM303D.h"

LM303D::LM303D(char *i2c_filename) {//constructor
	m_bOpenedI2C_OK = true;
	m_bInitialized_OK = false;
	m_file_i2c = open(i2c_filename, O_RDWR);
	if (m_file_i2c<0) {
		//error opening I2C
		printf("Error: %s opening I2C.\n",strerror(errno));
		m_bOpenedI2C_OK=false;
	}
	else {
		m_bInitialized_OK = InitializeDevice();//initialize LM303DLH for sample rate, filtering, etc.
	}
}


LM303D::~LM303D() {//destructor
	
}

bool LM303D::GetSample(LM303_DATASAMPLE *pSample, int nNumToAvg) {//collect raw data from the LM303DLH device and process it to get tilts, magnetic vector, temperature, and heading
	//pSample = the sample of data returned by the function, includes, accelerometer, magnetometer, and temperature data
	//nNumToAvg = the number of individual samples to collect and average (use 1 for no averaging)
	//returns true if function was successful, false otherwise
	double dTemperatureSum = 0.0;//the sum of all temperature samples received
	double mag_data_sum[3];//the sum of all magnetometer samples received
	double acc_data_sum[3];//the sum of all accelerometer samples received
	memset(mag_data_sum,0,3*sizeof(double));
	memset(acc_data_sum,0,3*sizeof(double));
	double acc_data[3];//acceleration data for the current reading
	double mag_data[3];//magnetometer data for the current reading
	double dTemperatureData=0.0;//temperature data for the current reading
	if (nNumToAvg<1) {
		printf("Invalid number of samples to average.\n");
		return false;
	}
	
	for (int i=0;i<nNumToAvg;i++) {
		if (!GetTemperatureData(dTemperatureData)) {
			printf("Error trying to get temperature data.\n");
			return false;
		}
		if (!GetMagnetometerData(mag_data)) {
			printf("Error trying to get magnetometer data.\n");
			return false;
		}
		if (!GetAccelerometerData(acc_data)) {
			printf("Error trying to get accelerometer data.\n");
			return false;
		}	
		dTemperatureSum+=dTemperatureData;
		for (int j=0;j<3;j++) {
			mag_data_sum[j]+=mag_data[j];
			acc_data_sum[j]+=acc_data[j];
		}
	}
	//divide by number of samples to get averaged results
	dTemperatureData = dTemperatureSum / nNumToAvg;
	for (int i=0;i<3;i++) {
		acc_data[i] = acc_data_sum[i] / nNumToAvg;
		mag_data[i] = mag_data_sum[i] / nNumToAvg;
	}
	//compute pitch, roll, and heading angles
	ComputeOrientation(pSample, acc_data, mag_data);
	//copy data to pSample
	pSample->temperature = dTemperatureData;
	memcpy(pSample->acc_data,acc_data,3*sizeof(double));
	memcpy(pSample->mag_data,mag_data,3*sizeof(double));
	//pSample->heading = dHeading;
	return true;
}

bool LM303D::InitializeDevice() {//initialize LM303D for sample rate, filtering, etc.
	//initialize accelerometer data collection
	unsigned char out_buf[2];
	if (!m_bOpenedI2C_OK) return false;
	if (ioctl(m_file_i2c, I2C_SLAVE, I2C_ADDRESS)<0) {
		printf("Failed (error = %s) to acquire bus access and/or talk to slave accelerometer.\n",strerror(errno));
		return false;
	}	
	//set CTRL1_REG_A(20h) to 0x57
	//corresponds to operation at 50 Hz, all acceleration axes turned on
	out_buf[0] = CTRL1_REG_A;
	out_buf[1] = 0x57;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for CTRL1_REG_A.\n");
		return false;
	}
	//set CTRL2_REG_A(21h) to 0xC0
	//corresponds to 50 Hz bandwidth filter and +/- 2 G acceleration full-scale
	out_buf[0] = CTRL2_REG_A;
	out_buf[1] = 0xC0;
	if (write(m_file_i2c,  out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for CTRL2_REG_A.\n");
		return false;
	}
	//set CTRL5_REG_M(24h) to 0xF0
	//corresponds to temperature sensor enabled, high resolution mag data, and 50 Hz output rate for mag data
	out_buf[0] = CTRL5_REG_M;
	out_buf[1] = 0xF0;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for CTRL5_REG_A.\n");
		return false;
	}
	//set CTRL6_REG_M (25h) to 0x00
	//corresponds to +/- 2 Gauss magnetic range full-scale
	out_buf[0] = CTRL6_REG_M;
	out_buf[1] = 0x00;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for CTRL6_REG_A.\n");
		return false;
	}
	//set CTRL7_REG_AM (26h) to 0x00  
	//corresponds to high pass filter for acceleration data disabled, continuous magnetometer measurements
	out_buf[0] = CTRL7_REG_AM;
	out_buf[1] = 0x00;
	if (write(m_file_i2c, out_buf, 2)!=2) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for CTRL7_REG_AM.\n");
		return false;
	}
	//ReadRegistersTest();
	return true;
}

bool LM303D::GetTemperatureData(double &dTemperatureData) {//get temperature data from the LM303D (function assumes that temperature data is ready)
	//dTemperatureData = the returned temperature in degrees C 
	//function returns true if successful, false otherwise
	const double DEG_C_PER_BIT = .125;//output of temperature sensor in deg C per bit
	const double ROOM_TEMP = 25.0;//room temperature in deg C 
	unsigned char outBuf[1];//buffer for requesting data over I2C
	unsigned char inBuf[2];//buffer for receiving data over I2C
	//least significant byte (8 bits) of temperature
	outBuf[0] = TEMP_OUT_L;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for TEMP_OUT_L.\n");
		return false;
	}
	if (read(m_file_i2c,inBuf,1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read TEMP_OUT_L from the I2C bus.\n");
		return false;
	}
	//most significant byte (actually only 4 bits) of temperature
	outBuf[0] = TEMP_OUT_H;
	if (write(m_file_i2c,outBuf,1)!=1) {
		//error, I2C transaction failed
		printf("Failed to write to the I2C bus for TEMP_OUT_H.\n");
		return false;
	}
	if (read(m_file_i2c,&inBuf[1],1)!=1) {
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read TEMP_OUT_H from the I2C bus.\n");
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
	return true;
}

bool LM303D::GetMagnetometerData(double *mag_data) {//get magnetometer data from the LM303D
	if (!WaitForDataReady(STATUS_M)) {
		printf("Timed out waiting for magnetometer data.\n");
		return false;
	}
	if (!Get6BytesRegData(mag_data, OUT_X_L_M)) {
		printf("Error getting magnetometer data.\n");
		return false;
	}
	//negate x and y axes to match accelerometer data
	mag_data[0] = -mag_data[0];
	mag_data[1] = -mag_data[1];
	normalize(mag_data);
	return true;	
}

bool LM303D::GetAccelerometerData(double *acc_data) {//get accelerometer data from the LM303D
	if (!WaitForDataReady(STATUS_A)) {
		printf("Timed out waiting for accelerometer data.\n");
		return false;
	}
	if (!Get6BytesRegData(acc_data, OUT_X_L_A)) {
		printf("Error getting accelerometer data.\n");
		return false;
	}
	//negate z-axis to make accelerations conform to right-handed rule
	acc_data[2] = -acc_data[2];
	normalize(acc_data);
	return true;
}

bool LM303D::Get6BytesRegData(double *data, int nBaseRegAddr) {//request 6 bytes of register data starting at nBaseRegAddr
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

int LM303D::Get16BitTwosComplement(unsigned char highByte, unsigned char lowByte) {//convert two-byte value into a 16-bit twos-complement number (between -32767 and +32767)
	int nCountVal = (highByte<<8) + lowByte;
	if ((nCountVal&0x8000)>0) {//negative value
		nCountVal = -(65536 - nCountVal);
	}
	return nCountVal;
}

bool LM303D::WaitForDataReady(unsigned char ucStatusReg) {//check 3 least sig bits of ucStatusReg to verify that they are all set (indicating that X, Y, Z data is read
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

void LM303D::ComputeOrientation(LM303_DATASAMPLE *pSample, double *acc_data, double *mag_data) {//compute pitch, roll, and heading angles
	//pSample = the current sample of LM303D data (pitch, roll, and heading angles get copied to this structure)
	//acc_data = current acceleration data (normalized) from the LM303D
	//mag_data = current magnetometer data (also normalized) from the LM303D
	//this function is based on the LSM303DLH-compass-app-note.pdf from ST Microelectronics
	const double RAD_TO_DEG = 57.29578;
	double dPitchAngleRad = asin(-acc_data[0]);//pitch angle in radians
	double dCosFactor = cos(dPitchAngleRad);
	double dRollAngleRad = 0.0;//roll angle in radians
	if (dCosFactor!=0.0) {
		dRollAngleRad = asin(acc_data[1] / dCosFactor);
	}
	//double mx2 = mag_data[0]*cos(dPitchAngleRad) + mag_data[2]*sin(dPitchAngleRad);
	//double my2 = mag_data[0]*sin(dRollAngleRad)*sin(dPitchAngleRad) + mag_data[1]*cos(dRollAngleRad) - mag_data[2]*sin(dRollAngleRad)*cos(dPitchAngleRad);
	//double mx2 = mag_data[0]*cos(dPitchAngleRad) + mag_data[1]*sin(dPitchAngleRad)*sin(dRollAngleRad) + mag_data[2]*sin(dPitchAngleRad)*cos(dRollAngleRad);
	//double my2 = mag_data[1]*cos(dRollAngleRad) - mag_data[2]*sin(dRollAngleRad);
	double mx2 = mag_data[0]*cos(dPitchAngleRad) - mag_data[2]*sin(dPitchAngleRad);
	double my2 = mag_data[0]*sin(dPitchAngleRad)*sin(dRollAngleRad) + mag_data[1]*cos(dRollAngleRad) + mag_data[2]*cos(dPitchAngleRad)*sin(dRollAngleRad);
	double dHeadingAngleRad = atan2(my2,mx2);//heading angle in radians
	//copy to pSample (and convert angles to degrees)
	pSample->pitch = dPitchAngleRad*RAD_TO_DEG;
	pSample->roll = dRollAngleRad*RAD_TO_DEG;
	pSample->heading = dHeadingAngleRad*RAD_TO_DEG;
	//make sure heading is between 0 and 360
	if (pSample->heading<0) pSample->heading+=360;
	else if (pSample->heading>360) pSample->heading-=360;
}

void LM303D::normalize(double *vec) {//normalizes vec (if it is not a null vector) 
	//vec = the 3-dimensional vector to be normalized
	double dVecMag = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
	if (dVecMag==0.0) return;//null vector, don't do anything with it
	vec[0]/=dVecMag;
	vec[1]/=dVecMag;
	vec[2]/=dVecMag;
}

void LM303D::ReadRegistersTest() {//reads out all of the configuration registers and prints out their values
	unsigned char outBuf[1];//buffer for requesting data over I2C
	unsigned char inBuf[1];
	for (int i=5;i<64;i++) {
		outBuf[0] = (unsigned char)i;
		if (write(m_file_i2c,outBuf,1)!=1) {
			//error, I2C transaction failed
			printf("Failed to write to the I2C bus for TEMP_OUT_L.\n");
			return;
		}
		if (read(m_file_i2c, inBuf,1)!=1) {
			//ERROR HANDLING: i2c transaction failed
			printf("Failed to read register %02x from the I2C bus.\n",i);
			return;
		}
		printf("%02x: %02x\n",i,(int)inBuf[0]);
	}
}