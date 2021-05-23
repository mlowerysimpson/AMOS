using namespace std;
#include "3DMATH.H"
#ifndef _WIN32
#include <pthread.h>
#else
typedef int pthread_mutex_t;
#endif
//class file for use with the AltIMU-10 v5 Gyro, Accelerometer, Compass, and Altimeter from Pololu Electronics (www.pololu.com)

#define MAG_I2C_ADDRESS 0x1E //I2C slave address for the 3-axis magnetometer chip on the AltIMU-10 v5
#define ACC_GYRO_I2C_ADDRESS 0x6B //I2C slave address for the 3-axis accelerometer / gyro chip on the AltIMU-10 v5
#define MAX_IMU_COUNTS 32767 //maximum # of counts for IMU data values (16-bit 2's complement)
#define NUM_MAGCAL_AVG 50 //# of samples to average for magnetometer calibration

//magnetometer control registers (see LIS3MDL datasheet)
#define MAG_OFFSET_X_L 0x05//low-order byte of x-axis mag offset
#define MAG_OFFSET_X_H 0x06//high-order byte of x-axis mag offset
#define MAG_OFFSET_Y_L 0x07//low-order byte of y-axis mag offset
#define MAG_OFFSET_Y_H 0x08//high-order byte of y-axis mag offset
#define MAG_OFFSET_Z_L 0x09//low-order byte of z-axis mag offset
#define MAG_OFFSET_Z_H 0x0a//high-order byteo of z-axis mag offset
#define MAG_WHO_AM_I 0x0f//who am I register, should be equal to 0x3d
#define MAG_CTRL_REG1 0x20//control register 1 for temperature enable, X & Y performance mode, output data rate, and self-test enable
#define MAG_CTRL_REG2 0x21//control register 2 for full-scale range of mags and hard / soft reset of memory contents
#define MAG_CTRL_REG3 0x22//control register 3 for low power mode, SPI wire selection, & setting of continuous conversion mode
#define MAG_CTRL_REG4 0x23//control register 4 for Z performance mode, Big/Little Endian selection for data
#define MAG_CTRL_REG5 0x24//control register 5 for "fast read" setting (skips low order byte) and block data update mode
#define MAG_STATUS_REG 0x27//magnetometer status register determines whether or not data is available
#define MAG_OUTX_L 0x28//low-order byte of x-axis mag data
#define MAG_OUTX_H 0x29//high-order byte of x-axis mag data
#define MAG_OUTY_L 0x2A//low-order byte of y-axis mag data
#define MAG_OUTY_H 0x2B//high-order byte of y-axis mag data
#define MAG_OUTZ_L 0x2C//low-order byte of z-axis mag data
#define MAG_OUTZ_H 0x2D//high-order byte of z-axis mag data
#define MAG_TEMP_OUT_L 0x2e//low-order byte of magnetometer temperature 
#define MAG_TEMP_OUT_H 0x2f//high-order byte of magnetometer temperature

//acc/gyro control registers (see LSM6DS33 datasheet)
#define ACC_GYRO_WHO_AM_I 0x0f//who am I register, should be equal to 0x69
#define ACC_CTRL1_XL 0x10//linear acceleration sensor control register 1, controls output data rate (ODR), accelerometer full-scale selection, and anti-aliasing filter bandwidth selection
#define GYRO_CTRL2_G 0x11//angular rate sensor control register 2, controls output data rate (ODR), and full-scale selection for gyros
#define ACC_GYRO_CTRL3_C 0x12//control register 3 controls block data update
#define ACC_GYRO_CTRL4_C 0x13//control register 4 controls accel bandwidth selection
#define ACC_GYRO_CTRL6_C 0x15//control register 6 controls high performance mode for accelerometer
#define GYRO_CTRL7_G 0x16//control register 7 controls high performance mode for gyros and high-pass filter settings for gyros
#define ACC_CTRL8_XL 0x17//control register 8 controls filter settings for accelerometers
#define ACC_CTRL9_XL 0x18//control register 9 enables individual accelerometers
#define GYRO_CTRL10_C 0x19//control register 10 enables individual gyros
#define ACC_GYRO_STATUS_REG 0x1E//status register to indicate when data acc/gyro/temperature data is ready
#define OUT_TEMP_L 0x20//low byte of temperature
#define OUT_TEMP_H 0x21//high byte of temperature
#define OUTX_L_G 0x22//low byte of x-axis angular rate
#define OUTX_H_G 0x23//high byte of x-axis angular rate
#define OUTY_L_G 0x24//low byte of y-axis angular rate
#define OUTY_H_G 0x25//high byte of y-axis angular rate
#define OUTZ_L_G 0x26//low byte of z-axis angular rate
#define OUTZ_H_G 0x27//high byte of z-axis angular rate
#define OUTX_L_XL 0x28//low byte of x-axis acceleration
#define OUTX_H_XL 0x29//high byte of x-axis acceleration
#define OUTY_L_XL 0x2A//low byte of y-axis acceleration
#define OUTY_H_XL 0x2B//high byte of y-axis acceleration
#define OUTZ_L_XL 0x2C//low byte of z-axis acceleration
#define OUTZ_H_XL 0x2D//high byte of z-axis acceleration
#define TIMESTAMP0_REG 0x40//timestamp low byte output register
#define TIMESTAMP1_REG 0x41//timestamp mid byte output register
#define TIMESTAMP2_REG 0x42//timestamp high byte output register
#define TAP_CFG 0x58//register for enabling timestamps
#define WAKE_UP_DUR 0x5C//register for setting the timestamp resolution


#define MAX_NUM_TO_AVG 1000 //maximum number of samples that can be averaged (will take ~ 12.5 seconds at 80 Hz)
#define MAG_SENSOR_TEMPOFFSET 9.0 //temperature offset (in deg C) of temperature sensor in LIS3MDL magnetometer, this value gets subtracted from the output temperature
#define ACC_SENSOR_TEMPOFFSET 0.0 //temperature offset (in deg C) of temperature sensor in LSM6DS33 acc/gyro sensor, this value gets subtracted from the output temperature

//accelerometer sensitivity (when set to +/- 2 G)
#define ACC_GAIN 0.000061 //output in G per bit

//gyro sensitivity (when set to +/- 245 deg/sec full scale)
#define GYRO_GAIN .00875 //deg / sec per bit

//acc/gyro timer resolution in seconds per bit
#define ACC_GYRO_TIMER_RESOLUTION 0.000025

#define CAL_SAMPLE_PIN 16 //GPIO pin used to toggle the collection of data for calibration or control the heater and fan for temperature calibration


struct IMU_DATASAMPLE {//full data sample from inertial measurement unit
	double sample_time_sec;//the time of the sample in seconds
	double acc_data[3];//acceleration data in G
	double mag_data[3];//magnetometer data (Gauss)
	double angular_rate[3];//angular rate (deg/sec)
	double mag_temperature;//temperature of the LIS3MDL chip
	double acc_gyro_temperature;//temperature of the LSM6DS33 chip
	double heading;//computed heading value in degrees (direction that the +X axis of the IMU is pointed) 0 to 360
	double pitch;//computed pitch angle in degrees (direction above horizontal that the +X axis of the IMU is pointed -90 to 90
	double roll;//computed roll angle in degrees (direction around +X axis that the +Y axis of the IMU is pointed -180 to +180
};

struct IMU_TEMP_CAL {
	double accx_vs_temp;//offset change in x-axis acceleration vs. temperature (counts per deg C)
	double accy_vs_temp;//offset change in y-axis acceleration vs. temperature (counts per deg C)
	double accz_vs_temp;//offset change in z-axis acceleration vs. temperature (counts per deg C)
	double magx_vs_temp;//offset change in x-axis magnetometer vs. temperature (counts per deg C)
	double magy_vs_temp;//offset change in y-axis magnetometer vs. temperature (counts per deg C)
	double magz_vs_temp;//offset change in z-axis magnetometer vs. temperature (counts per deg C)
	double acc_cal_temp;//temperature from the accelerometer temperature sensor where offsets and gains were determined
	double mag_cal_temp;//temperature from the magnetometer temperature sensor where offsets and gains were determined
};

class IMU {//class used for communicating with and getting tilt, angular rate, and magnetic data from an IMU (AltIMU-10 v5 by Polulu Robotics & Electronics)
//functions are also provided for computing heading angle based on available sensor data
public:
	IMU(pthread_mutex_t *i2c_mutex);//constructor
	~IMU();//destructor
	bool m_bInitError;//flag is true if any sort of error occurs when opening I2C ports or initializing devices
	bool m_bOpenedI2C_OK;//flag is true if I2C port was opened properly, otherwise it is false
	bool m_bMagInitialized_OK;//flag is true if the LIS3MDL 3-axis magnetometer was properly initialized for data collection, false otherwise
	bool m_bAccGyroInitialized_OK;//flag is true if the LSM6DS33 3-axis accelerometer and 3-axis gyroscope was properly initialized for data collection, false otherwise
	bool m_bPressureInitialized_OK;//flag is true if the LPS25H pressure sensor was properly initialized for data collection, false otherwise
	bool DoTempCal();//does a factory temperature calibration of the IMU sensors
	bool GetMagSample(IMU_DATASAMPLE *pIMUSample, int nNumToAvg);//collect magnetometer data from the LIS3MDL 3-axis magnetometer device and process it to get the magnetic vector and temperature
	bool GetAccGyroSample(IMU_DATASAMPLE *pIMUSample, int nNumToAvg);//collect accelerometer & gyro data from the LSM6DS33 and process it to get the acceleration vector, rotation rate vector, and temperature
	bool GetSample(IMU_DATASAMPLE *pIMUSample, int nNumToAvg);//collect magnetometer, accelerometer, and gyro data, and then call ComputeOrientation to determine orientation angles
	void ResetAccGyro();//reset the timestamps for the acc/gyro measurements back to zero seconds
	bool DoMagCal();//perform a calibration procedure on the magnetometers to get the zero-field offsets for each of the sensors. Saves the results to the offset registers.
	bool DoXYMagCal();//perform a calibration procedure on the magnetometers to get the zero-field offsets for the X and Y magnetometers. Saves the results to the offset registers.
	bool DoXYMagCalWithToggledSampling();//perform a factory XY calibration procedure on the magnetometers to get the zero-field offsets for the X and Y magnetometers. Saves the results to the offset registers.
	bool DoXZMagCalWithToggledSampling();//perform a factory XZ calibration procedure on the magnetometers to get the zero-field offsets for the X and Z magnetometers. Saves the results to the offset registers. 
	void ComputeOrientation(IMU_DATASAMPLE *pSample);//compute orientation (pitch, roll, and heading angles) of the AltIMU-10, using acc/mag data plus gyros
	bool SaveIMUDataToFile(char* szFilename, int nNumSecs);//save data from all sensors to a text data file for a period of time

		
private:
	//data
	IMU_TEMP_CAL m_tempCal;//temperature calibration object
	double m_acc_counts[3];//used for storing accelerometer raw count  values
	double m_mag_counts[3];//used for storing magnetometer raw count values
	double m_gyro_counts[3];//used for storing gyro raw count values
	bool m_bLoadedMagCal;//flag is true after magnetometer calibration has been successfully loaded
	char m_szErrMsg[256];//buffer space used for outputting error messages
	pthread_mutex_t *m_i2c_mutex;
	double m_dLastSampleTime;//time of last orientation sample (in seconds)
	int m_nGyroAxisOrder;//cycles continuously from 0, 1, 2, 0, 1, 2, etc. for each sample and defines the order used to form the orientation matrix calculated from the gyros
	quaternion2 *m_quat;//the quaternion used for determining the orientation of the IMU
	int m_file_i2c;//handle to I2C port for the IMU
	double m_dBaseAccGyroTimestamp;//the base timestamp for the first sample 
	double m_dAccumulatedTimeSeconds;//the accumulated time in seconds from previous rollovers of the timer
	unsigned int m_uiAccGyroSampleCount;//the number of acc/gyro samples successfully collected
	
	//functions
	bool GetTempCalSample(char* lineText, unsigned int baseSampleTime, double& dTempDegC);//gets raw IMU data to use for coming up with a device temperature calibration
	bool RetryOpening();//try re-opening the I2C port, return true if successful
	void GetBestCircleFit(double *xmag, double *ymag, int nNumVals, double &magXOffset, double &magYOffset);//tries to find the approximate center and radius of a circle where nNumVals points of coordinates xmag, ymag go through the circumference of the circle
	bool SaveMagOffsets(double dMagOffsetX, double dMagOffsetY, double dMagOffsetZ);//store magnetometer offsets to mag offsets registers
	bool GetAvgMagVals(double mag_data[NUM_MAGCAL_AVG][3], double avg_mag[3], int nNumSamples);//get averaged magnetometer values from last NUM_MAGCAL_AVG samples stored in mag_data
	void ReadMagOffsets();//read in and print out mag offsets stored in offset registers
	bool GetAccData(double *acc_data);//get accelerometer data from the LSM6DS33
	bool GetGyroData(double *gyro_data);//get gyro data from the LSM6DS33
	bool GetAccTemperatureData(double &dTemperatureData);//get temperature data from the LSM6DS33
	bool GetMagTemperatureData(double &dTemperatureData);//get temperature data from the LIS3MDL (function assumes that temperature data is ready, and that magnetometer currently has I2C bus access)
	bool GetMagnetometerData(double *mag_data);//get magnetometer data from the LIS3MDL
	int Get16BitTwosComplement(unsigned char highByte, unsigned char lowByte);//convert two-byte value into a 16-bit twos-complement number (between -32767 and +32767)
	bool InitializeMagDevice();//initialize LIS3MDL for sample rate, full-scale range, etc.
	bool InitializeAccGyroDevice();//initialize LSM6DS33 for sample rate, full-scale range, etc.
	bool Get6BytesRegData(double *data, int nBaseRegAddr);//request 6 bytes of register data starting at nBaseRegAddr
	bool WaitForMagDataReady(unsigned char ucStatusReg);//check 3 least sig bits of status register to verify that they are all set (indicating that X, Y, Z data is ready to read
	bool WaitForAccDataReady(unsigned char ucStatusReg);//check XLDA bit of LSM6DS33 status register to see if the accelerometer data is ready
	bool WaitForGyroDataReady(unsigned char ucStatusReg);//check GDA bit of LSM6DS33 status register to see if the gyro data is ready
	bool WaitForAccTemperatureData(unsigned char ucStatusReg);//check TDA bit of LSM6DS33 status register to see if the temperature data is ready
	bool LoadMagCal();//load magnetometer offset calibration (if available) from mag_cal.txt file
	static void normalize(double *vec);//normalizes vec (if it is not a null vector)
};
	
