//HumiditySensor.cpp
//implementation file for HumiditySensor class
#include "HumiditySensor.h"


/**
 * @brief function runs in separate thread for collecting humidity and temperature data from the DHT22 sensor.
 * 
 * @param pParam void pointer to the HumiditySensor object.
 * @return void* always returns nullptr.
 */
void *humidityFunction(void *pParam) {//function receives commands from base station over (wireless) serial link
	HumiditySensor *pSensor = (HumiditySensor *)pParam;
    pSensor->m_bExitThread = false;
    pSensor->m_bHumidityThreadRunning = true;
    while (!pSensor->m_bExitThread) {
        pSensor->CollectHumidityData();
        unsigned int uiWaitTime = millis() + HUMIDITY_SAMPLE_INTERVAL_SEC * 1000;
        while (!pSensor->m_bExitThread&&millis() < uiWaitTime) {
            delay(100);
        }
    }
    pSensor->m_bHumidityThreadRunning=false;
    return nullptr;
}

/**
 * @brief Construct a new HumiditySensor object.
 * 
 */
HumiditySensor::HumiditySensor() : Sensor(nullptr) {//constructor
    m_bGotValidData = false;
    m_threadId = 0;
    /* Uncomment the following lines if the WiringPi initialization has not already been performed elsewhere in your code
    // GPIO Initialization
	if (wiringPiSetupGpio() == -1)
	{
		printf("[x_x] GPIO Initialization FAILED.\n");
		return -1;
	}
    */

    m_threadMutex = PTHREAD_MUTEX_INITIALIZER;
    m_bExitThread = false;//flag is used to tell humidity data collection thread when to stop.
    m_bHumidityThreadRunning = false;//flag is used to 
    m_fHumidity = 0;//humidity expressed as an RH percentage from 0 to 100
    m_fTemperature = 0;//temperature of the sensor in degrees C
    //start thread for collecting data from DHT22 sensor
    int nError = pthread_create(&m_threadId, NULL, &humidityFunction, (void *)this);
	if (nError!=0) {
		printf("Can't create thread for getting humidity data: %s", strerror(nError));
	}    
}

/**
 * @brief Destroy the Humidity Sensor:: Humidity Sensor object
 * 
 */
HumiditySensor::~HumiditySensor() {//destructor
    StopHumidityThread();//stop thread for getting humidity data
	pinMode(HUMIDITY_PIN, INPUT);//make sure one-wire interface is set back to input
}

/**
 * @brief return the most recently collected humidity value. The thread which collects humidity (and temperature) data is successful about 75% of the time, so the value returned should be current to within some small muiltiple of HUMIDITY_SAMPLE_INTERVAL_SEC seconds.
 * 
 * @param fHumidity returned value of the humidity, expressed as a relative humidity percentage from 0 to 100.
 * @return true if at least one valid humidity reading has been obtained.
 * @return false if no valid humidity readings have been obtained yet.
 */
bool HumiditySensor::GetHumidity(float &fHumidity) {
    if (!m_bGotValidData) {
        return false;
    }
    pthread_mutex_lock(&m_threadMutex);
    fHumidity = m_fHumidity;
    pthread_mutex_unlock(&m_threadMutex);
    return true;
}

/**
 * @brief return the most recently collected temperature value. The thread which collects humidity (and temperature) data is successful about 75% of the time, so the value returned should be current to within some small muiltiple of HUMIDITY_SAMPLE_INTERVAL_SEC seconds.
 * 
 * @param fTemperature returned value of the temperature (in degrees C). 
 * @return true if at least one valid temperature reading has been obtained.
 * @return false if no valid temperature readings have been obtained yet
 */
bool HumiditySensor::GetTemperature(float &fTemperature) {//
    if (!m_bGotValidData) {
        return false;
    }
    pthread_mutex_lock(&m_threadMutex);
    fTemperature = m_fTemperature;
    pthread_mutex_unlock(&m_threadMutex);
    return true;   
}


/**
 * @brief Stop the humidity thread from running and collecting data.
 * 
 */
void HumiditySensor::StopHumidityThread() {//stop thread for getting humidity data
    const int TIMEOUT_MS = 2000;//timeout period for stopping the humidity thread in ms
    m_bExitThread = true;
    unsigned int uiTimeoutTime = millis() + TIMEOUT_MS;
    while (m_bHumidityThreadRunning&&millis() < uiTimeoutTime) {
        delay(100);
    }
    if (m_bHumidityThreadRunning) {
        printf("Error, timed out waiting for humidity thread to stop.\n");
    }
}


bool HumiditySensor::CollectDHT22RawData(unsigned short *data) {
    unsigned short val = 0x00;
	unsigned short signal_length = 0;
	unsigned short val_counter = 0;
	unsigned short loop_counter = 0;
    while (1)
	{
		// Count only HIGH signal
		while (digitalRead(HUMIDITY_PIN) == HIGH)
		{
			signal_length++;
			
			// When sending data ends, signal should stay high
			// So we have to end this infinite loop.
			if (signal_length >= 200)
			{				
				return true;
			}
			delayMicroseconds(1);
		}
		// If signal is HIGH
		if (signal_length > 0)
		{
			loop_counter++;	// HIGH signal counting

			// The DHT22 sends a lot of unstable signals(?)
			// So extended the counting range.
			if (signal_length < 10)
			{
				// Unstable signal
				val <<= 1;		// 0 bit. Just shift left
			}
			else if (signal_length < 30)
			{
				// 26~28us means 0 bit
				val <<= 1;		// 0 bit. Just shift left
			}
			else if (signal_length < 85)
			{
				// 70us means 1 bit	
				// Shift left and input 0x01 using OR operator
				val <<= 1;
				val |= 1;
			}
			else
			{
				// Unstable signal
				return false;
			}
			signal_length = 0;	// Initialize signal length for next signal
			val_counter++;		// Count for 8 bit data
		}
		// The first and second signal is DHT22's start signal.
		// So ignore these data.
		if (loop_counter < 3)
		{
			val = 0x00;
			val_counter = 0;
		}
		// If 8 bit data input complete
		if (val_counter >= 8)
		{
			// 8 bit data input to the data array
			data[(loop_counter / 8) - 1] = val;

			val = 0x00;
			val_counter = 0;
		}
	}      
    return false;//should not actually get here
}

/**
 * @brief collect data from the DHT22 temperature and humidity sensor (based on code from Hyun Wook Choi: https://github.com/ccoong7/DHT22)
 * 
 * @return true if successful
 * @return false if there was a problem (typically communications error on one-wire interface) getting data from the DHT22
 */
bool HumiditySensor::CollectHumidityData() {
    unsigned short data[5] = {0, 0, 0, 0, 0};
	short checksum=0;
    pinMode(HUMIDITY_PIN, OUTPUT);//set Pi pin for output
	// Send out start signal
	digitalWrite(HUMIDITY_PIN, LOW);
	delay(20);					// Stay LOW for 5~30 milliseconds
	pinMode(HUMIDITY_PIN, INPUT);		// 'INPUT' equals 'HIGH' level. And signal read mode

    if (CollectDHT22RawData(data)) {
		// The sum is maybe over 8 bit like this: '0001 0101 1010'.
		// Remove the '9 bit' data using AND operator.
		checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
		
		// If Check-sum data is correct display humidity and temperature
		if (data[4] == checksum) {
			// * 256 is the same thing '<< 8' (shift).
			pthread_mutex_lock(&m_threadMutex);
			m_fHumidity = ((data[0] * 256) + data[1]) / 10.0;
			m_fTemperature = data[3] / 10.0;
			// If 'data[2]' data like 1000 0000, It means minus temperature
			if (data[2] == 0x80) {
				m_fTemperature *= -1;
			}
			pthread_mutex_unlock(&m_threadMutex);
		}
		else {
			//printf("[x_x] Invalid Data.\n\n");
			return false;
		}
		m_bGotValidData = true;//now have obtained at least one sample of valid data
		return true;
	}
    return false;
}