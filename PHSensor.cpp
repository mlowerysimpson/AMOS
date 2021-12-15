//PHSensor.cpp
//implementation file for PHSensor class
#include "PHSensor.h"
PHSensor::PHSensor(int nAToDChannel, AToD *pAToD, PH_CALIBRATION *pPHCal) : Sensor(pAToD) {//constructor for analog pH sensor connected to A to D board
	if (nAToDChannel<1||nAToDChannel>NUM_ATOD_CHANNELS) {
		m_nAToDChannel = DEFAULT_PH_CHANNEL;
	}
	else {
		m_nAToDChannel = nAToDChannel;
	}
	if (pPHCal) {
		memcpy(&m_phcal,pPHCal,sizeof(PH_CALIBRATION));
	}
	else {//use default values
		m_phcal.dLowCalVoltage = DEFAULT_PH_LOWCALVOLTAGE;
		m_phcal.dLowCalPH = DEFAULT_PH_LOWCAL;
		m_phcal.dMidCalVoltage = DEFAULT_PH_MIDCALVOLTAGE;
		m_phcal.dMidCalPH = DEFAULT_PH_MIDCAL;
	}
	m_i2c_mutex = nullptr;
    m_bOpenedI2C_OK = false;
}

PHSensor::PHSensor(unsigned char i2c_channel, pthread_mutex_t* i2c_mutex) : Sensor(nullptr) {//constructor for Atlas scientific pH sensor on I2C
	m_nAToDChannel = -1;
	memset(&m_phcal, 0, sizeof(PH_CALIBRATION));
	m_i2c_mutex = i2c_mutex;
	//open I2C port for AtlasDO2Sensor device
	char* i2c_filename = (char*)"/dev/i2c-1";
	m_bOpenedI2C_OK = true;
	lockmutex();
	m_file_i2c = open(i2c_filename, O_RDWR);
	unlockmutex();
	if (m_file_i2c < 0) {
		//error opening I2C
		printf("Error: %s opening I2C.\n", strerror(errno));
		m_bOpenedI2C_OK = false;
	}
}

PHSensor::~PHSensor() {

}

//GetPHSensorPH: gets the pH probe pH value for AMOS, with temperature compensation applied (I2C probe only)
//dPHVal: the returned pH value (0 to 14)
//dWaterTempDegC: the temperature in deg C of the water
//returns true if successful, false otherwise
bool PHSensor::GetPHSensorPH(double& dPHVal, double dWaterTempDegC) {
    return GetI2CPH(dPHVal, double dWaterTempDegC);
}

//GetPHSensorPH: gets the pH probe pH value for AMOS (uses currently available pH probe calibration parameters), assumes some resistor divider settings (see below)
//dPHVal: the returned pH value (0 to 14)
//returns true if successful, false otherwise
bool PHSensor::GetPHSensorPH(double &dPHVal) {
    if (m_i2c_mutex != nullptr) {
        //use function for I2C probe
        return GetI2CPH(dPHVal);
    }
	//calibration obtained using voltage divider of 10K in series with another 10K resistor to ground
	char sMsg[256];
	if (!m_pAToD) return false;
	const double R1 = 10.0;//resistance in kohm of first resistor in divider
	const double R2 = 10.0;//resistance in kohm of 2nd resistor in divider
	double dPHVoltage=0.0;
	double dSensorGain = (R1 + R2)/R2;
	bool bGotResult = m_pAToD->GetMeasurement(m_nAToDChannel,1,dSensorGain,dPHVoltage);
	//test
	printf("dPHVoltage = %.3f\n",dPHVoltage);
	//end test
	if (bGotResult) {
		double dCalGain = (m_phcal.dMidCalPH - m_phcal.dLowCalPH) / 
			(m_phcal.dMidCalVoltage - m_phcal.dLowCalVoltage);
		double dCalOffset = m_phcal.dMidCalPH - dCalGain*m_phcal.dMidCalVoltage;
		dPHVal = dCalGain * dPHVoltage + dCalOffset;
	}
	else {
		printf("Error getting pH voltage.\n");
		return false;
	}
	this->UpdateCollectionTime();
	return true;		
}

bool PHSensor::write_i2c(unsigned char* ucData, int nNumBytes) {//writes bytes to I2C address
    if (write(m_file_i2c, ucData, nNumBytes) != nNumBytes) {
        return false;
    }
    return true;
}


/**
 * @brief get the pH value from an I2C probe. 
 * @param dPHMeasurement the returned value of the pH (0 - 14)
 *
 * @return true if the measurement was completed successfully, false otherwise
 */
bool PHSensor::GetI2CPH(double& dPHMeasurement) {
    unsigned char outBytes[1];
    unsigned char inBytes[32];
    memset(inBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas pH probe device.\n", strerror(errno));
        unlockmutex();
        return -1;
    }
    outBytes[0] = (unsigned char)'R';
    if (!write_i2c(outBytes, 1)) {
        unlockmutex();
        return -2;
    }
    //need to pause for 900 ms (see pH_EZO_Datasheet.pdf document)
    usleep(900000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 32);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from pH probe sensor.\n");
        unlockmutex();
        return -3;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from pH probe sensor.\n");
        unlockmutex();
        return -4;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, pH probe sensor not ready.\n");
        unlockmutex();
        return -5;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to get pH probe data.\n");
        unlockmutex();
        return -6;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from pH probe sensor.\n");
        unlockmutex();
        return -7;
    }
    //parse pH probe measurement
    if (sscanf((char*)&inBytes[1], "%lf", &dPHMeasurement) < 1) {
        printf("Error trying to parse pH probe data.\n");
        unlockmutex();
        return -8;
    }
    unlockmutex();
    UpdateCollectionTime();
    return true;//reading was obtained successfully
}

/**
 * @brief get the pH measurement, given that the water is at a certain temperature (i.e. apply temperature compensation for the measurement)
 * @param dPHMeasurement the returned value of the pH (0 - 14)
 * @param dTempDegC the current temperature of the water in degrees C.
 * @return true if the measurement was completed successfully, false otherwise
 */
bool PHSensor::GetI2CPH(double& dPHMeasurement, double dTempDegC) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(outBytes, 0, 32);
    memset(inBytes, 0, 32);
    sprintf((char*)outBytes, "RT,%.1f", dTempDegC);
    int nNumToSend = strlen((char*)outBytes);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas pH probe device.\n", strerror(errno));
        unlockmutex();
        return -1;
    }
    if (!write_i2c(outBytes, nNumToSend)) {
        unlockmutex();
        return -2;
    }
    //need to pause for 900 ms to set temperature compensation and do measurement (see pH_EZO_Datasheet.pdf document)
    usleep(900000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 32);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from pH probe sensor.\n");
        unlockmutex();
        return -3;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from pH probe sensor.\n");
        unlockmutex();
        return -4;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, pH probe sensor not ready.\n");
        unlockmutex();
        return -5;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to get pH probe data.\n");
        unlockmutex();
        return -6;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from pH probe sensor.\n");
        unlockmutex();
        return -7;
    }
    //parse pH probe measurement
    if (sscanf((char*)&inBytes[1], "%lf", &dPHMeasurement) < 1) {
        printf("Error trying to parse pH data.\n");
        unlockmutex();
        return -8;
    }
    unlockmutex();
    UpdateCollectionTime();
    return true;//reading was obtained successfully
}


void PHSensor::lockmutex() {//lock mutex for access to the I2C bus
    if (m_i2c_mutex != nullptr) {
        pthread_mutex_lock(m_i2c_mutex);
    }
}


void PHSensor::unlockmutex() {//unlock mutex for access to the I2C bus
    if (m_i2c_mutex != nullptr) {
        pthread_mutex_unlock(m_i2c_mutex);
    }
}


/**
 * @brief perform a single-point, midpoint calibration of the pH probe (I2C devices only, removes all previous calibrations)
 * 
 * @param dMidPHVal the pH of the calibration solution being used for the midpoint calibration (typically 7.0 or close to it)
 * @param dWaterTempDegC optional parameter specifiying the temperature of the solution used for the midpoint calibration. If 
 * dWaterTempDegC is less than or equal to 0 deg C, a default calibration temperature of 25 deg C is assumed, and no temperature
 * compensation is applied
 *
 * @return true if the calibration was performed successfully, false otherwise
 */
bool PHSensor::CalibrateMidpoint(double dMidPHVal, double dWaterTempDegC) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(inBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas pH probe.\n", strerror(errno));
        unlockmutex();
        return false;
    }
    sprintf((char*)outBytes, "Cal,mid,%.2f", dWaterTempDegC);
    int nNumToWrite = strlen((char*)outBytes);
    
    if (!write_i2c(outBytes, nNumToWrite)) {
        unlockmutex();
        return false;
    }
    //need to pause for 900 ms (see pH_EZO_Datasheet.pdf document)
    usleep(900000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 2);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from the Atlas pH sensor.\n");
        unlockmutex();
        return false;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from the pH sensor.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, pH sensor not ready.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to do pH midpoint calibration.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from pH sensor.\n");
        unlockmutex();
        return false;
    }
    unlockmutex();
    return true;//calibration was completed successfully
}

/**
 * @brief perform the lowpoint calibration of the pH probe (I2C devices only)
 *
 * @param dLowPHVal the pH of the calibration solution being used for the lowpoint (acidic) calibration (e.g., 4.0)
 * @param dWaterTempDegC optional parameter specifiying the temperature of the solution used for the lowpoint calibration. If
 * dWaterTempDegC is less than or equal to 0 deg C, a default calibration temperature of 25 deg C is assumed, and no temperature
 * compensation is applied
 *
 * @return true if the lowpoint calibration was performed successfully, false otherwise
 */
bool PHSensor::CalibrateLowpoint(double dLowPHVal, double dWaterTempDegC) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(inBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas pH probe.\n", strerror(errno));
        unlockmutex();
        return false;
    }
    sprintf((char*)outBytes, "Cal,low,%.2f", dWaterTempDegC);
    int nNumToWrite = strlen((char*)outBytes);

    if (!write_i2c(outBytes, nNumToWrite)) {
        unlockmutex();
        return false;
    }
    //need to pause for 900 ms (see pH_EZO_Datasheet.pdf document)
    usleep(900000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 2);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from the Atlas pH sensor.\n");
        unlockmutex();
        return false;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from the pH sensor.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, pH sensor not ready.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to do pH lowpoint calibration.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from pH sensor.\n");
        unlockmutex();
        return false;
    }
    unlockmutex();
    return true;//calibration was completed successfully
}


/**
 * @brief perform the highpoint calibration of the pH probe (I2C devices only)
 *
 * @param dHighPHVal the pH of the calibration solution being used for the highpoint (basic) calibration (e.g., 10.0)
 * @param dWaterTempDegC optional parameter specifiying the temperature of the solution used for the highpoint calibration. If
 * dWaterTempDegC is less than or equal to 0 deg C, a default calibration temperature of 25 deg C is assumed, and no temperature
 * compensation is applied
 *
 * @return true if the highpoint calibration was performed successfully, false otherwise
 */
bool PHSensor::CalibrateHighpoint(double dHighPHVal, double dWaterTempDegC) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(inBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave Atlas pH probe.\n", strerror(errno));
        unlockmutex();
        return false;
    }
    sprintf((char*)outBytes, "Cal,high,%.2f", dWaterTempDegC);
    int nNumToWrite = strlen((char*)outBytes);

    if (!write_i2c(outBytes, nNumToWrite)) {
        unlockmutex();
        return false;
    }
    //need to pause for 900 ms (see pH_EZO_Datasheet.pdf document)
    usleep(900000);
    //read in response
    int nNumRead = read(m_file_i2c, inBytes, 2);
    if (nNumRead <= 0) {
        printf("Failed to read in any response bytes from the Atlas pH sensor.\n");
        unlockmutex();
        return false;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from the pH sensor.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, pH sensor not ready.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to do pH highpoint calibration.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from pH sensor.\n");
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
void PHSensor::EnterSleepMode() {
    unsigned char outBytes[32];
    memset(outBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
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
void PHSensor::ExitSleepMode() {
    double dDummyPHMeasurement = 0.0;
    this->GetPHSensorPH(dDummyPHMeasurement);//send command for pH measurement, may not actually get it(?). Any character or command sent to pH sensor should wake it up.
}


/**
 * @brief lock or unlock i2c communications for the device
 * @param nLock 0 or 1. Use 1 to lock the protocol to I2C, or 0 to unlock it and allow it to change to serial communications.
 * @return true if the protocol was locked or unlocked successfully, false otherwise
 */
bool PHSensor::ProtocolLock(int nLock) {
    unsigned char outBytes[32];
    unsigned char inBytes[32];
    memset(outBytes, 0, 32);
    if (!m_bOpenedI2C_OK) return false;

    lockmutex();
    if (ioctl(m_file_i2c, I2C_SLAVE, DEFAULT_PH_I2C_CHANNEL) < 0) {
        printf("Failed (error = %s) to acquire bus access and/or talk to slave pH probe device.\n", strerror(errno));
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
        printf("Failed to read in any response bytes from pH sensor.\n");
        unlockmutex();
        return false;
    }
    //check first byte of response
    if (inBytes[0] == NO_DATA_TO_SEND) {
        printf("Error, no data to send from pH sensor.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == NOT_READY) {
        printf("Error, pH sensor not ready.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] == SYNTAX_ERROR) {
        printf("Error, syntax error in command to get pH data.\n");
        unlockmutex();
        return false;
    }
    else if (inBytes[0] != DATA_OK) {
        printf("Error, unexpected response from pH sensor.\n");
        unlockmutex();
        return false;
    }
    unlockmutex();
    return true;
}