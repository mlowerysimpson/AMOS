using namespace std;

#define I2C_ADDRESS 0x1D //I2C slave address for the LM303D device

//accelerometer control registers
#define CTRL1_REG_A 0x20  //ctrl1 register for Acc (sets data rate and enables acc axes)
#define CTRL2_REG_A 0x21  //ctrl2 register for Acc (sets bandwidth filter and acceleration full-scale)
//magnetometer control registers
#define CTRL5_REG_M 0x24 //ctrl5 register for Mag (enables temp sensor, defines mag resolution, sets mag sensor data rate
#define CTRL6_REG_M 0x25 //ctrl6 register for Mag (sets the full scale value for the magnetometers)
#define CTRL7_REG_AM 0x26 //ctrl7 register for Mag and Acc (enables high pass internal filter for Acc and sets Acc data to output continuously)
//temperature output registers
#define TEMP_OUT_L 0x05//lowest 8 bits of temperature
#define TEMP_OUT_H 0x06//highest 4 bits of temperature
//status registers
#define STATUS_M 0x07//status register for magnetometer and temperature sensor
#define STATUS_A 0x27//status register for accelerometer
//magnetometer output registers
#define OUT_X_L_M 0x08//low byte of x-axis magnetometer
#define OUT_X_H_M 0x09//high byte of x-axis magnetometer
#define OUT_Y_L_M 0x0A//low byte of y-axis magnetometer
#define OUT_Y_H_M 0x0B//high byte of y-axis magnetometer
#define OUT_Z_L_M 0x0C//low byte of z-axis magnetometer
#define OUT_Z_H_M 0x0D//high byte of z-axis magnetometer
//accelerometer output registers
#define OUT_X_L_A 0x28//low byte of x-axis accelerometer
#define OUT_X_H_A 0x29//high byte of x-axis accelerometer
#define OUT_Y_L_A 0x2A//low byte of y-axis accelerometer
#define OUT_Y_H_A 0x2B//high byte of y-axis accelerometer
#define OUT_Z_L_A 0x2C//low byte of z-axis accelerometer
#define OUT_Z_H_A 0x2D//high byte of z-axis accelerometer



#define MAX_NUM_TO_AVG 1000 //maximum number of samples that can be averaged (warning: will take ~ 20 seconds at 50 Hz)


struct LM303_DATASAMPLE {//full data sample of LM303D data
	double acc_data[3];//acceleration data in G
	double mag_data[3];//magnetometer data (Gauss)
	double temperature;//temperature of the LM303D chip
	double heading;//computed heading value in degrees (direction that the +X axis of the LM303D is pointed) 0 to 360
	double pitch;//computed pitch angle in degrees (direction above horizontal that the +X axis of the LM303D is pointed -90 to 90
	double roll;//computed roll angle in degrees (direction around +X axis that the +Y axis of the LM303D is pointed -180 to +180
};

class LM303D {//class used for communicating with and getting tilt and magnetic data from the LM303D compass module (by ST Microelectronics)
//functions are also provided for computing heading angle based on tilt and magnetic data
public:
	LM303D(char *i2c_filename);//constructor
	~LM303D();//destructor
	
	bool m_bOpenedI2C_OK;//flag is true if I2C port was opened properly, otherwise it is false
	bool m_bInitialized_OK;//flag is true if the LM303DLH was properly initialized for data collection, false otherwise
	bool GetSample(LM303_DATASAMPLE *pSample, int nNumToAvg);//collect raw data from the LM303D device and process it to get tilts, magnetic vector, temperature, and heading
	bool GetTemperatureData(double &dTemperatureData);//get temperature data from the LM303D (function assumes that temperature data is ready)
	bool GetMagnetometerData(double *mag_data);//get magnetometer data from the LM303D
	bool GetAccelerometerData(double *acc_data);//get accelerometer data from the LM303D
	
	
private:
	//data
	int m_file_i2c;//handle to I2C port
	
	//functions
	int Get16BitTwosComplement(unsigned char highByte, unsigned char lowByte);//convert two-byte value into a 16-bit twos-complement number (between -32767 and +32767)
	bool InitializeDevice();//initialize LM303DLH for sample rate, filtering, etc.
	bool Get6BytesRegData(double *data, int nBaseRegAddr);//request 6 bytes of register data starting at nBaseRegAddr
	bool WaitForDataReady(unsigned char ucStatusReg);//check 3 least sig bits of ucStatusReg to verify that they are all set (indicating that X, Y, Z data is read
	void ComputeOrientation(LM303_DATASAMPLE *pSample, double *acc_data, double *mag_data);//compute pitch, roll, and heading angles
	void normalize(double *vec);//normalizes vec (if it is not a null vector)
	void ReadRegistersTest();//reads out all of the configuration registers and prints out their values
};
	
