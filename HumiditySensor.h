//HumiditySensor.h 

#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <pthread.h>
#include <wiringPi.h>
#include <unistd.h>
#endif
#include <memory>
#include <vector>
#include "Sensor.h"

#define HUMIDITY_PIN 7 //GPIO pin used for one-wire interface
#define HUMIDITY_SAMPLE_INTERVAL_SEC 5 //the interval (in seconds) between samples of the humidity sensor

using namespace std;

/**
 * @brief a class for measuring humidity and temperature from the DHT22 temperature and humidity sensor. This sensor uses a one-wire data communications interface, which requires fairly tight microsecond-level timing requirements, so the data acquisition code used for getting data from the DHT22 is not always successful. For this reason, data for this sensor is collected in a thread, and functions are provided for returning the most recent temperature and humidity values.
 * 
 */
class HumiditySensor : public Sensor {//class used for measuring humidity from DHT22 temperature and humidity sensor
public:
	HumiditySensor();//constructor
	~HumiditySensor();//destructor
    bool GetHumidity(float &fHumidity);//return the most recently collected humidity value
    bool GetTemperature(float &fTemperature);//return the most recently collected temperature value
    bool CollectHumidityData();//collect data from the DHT22 temperature and humidity sensor.
    bool m_bExitThread;//flag is used to tell humidity data collection thread when to stop.
    bool m_bHumidityThreadRunning;//flag is used to 

private:
    float m_fHumidity;//humidity expressed as an RH percentage from 0 to 100
    float m_fTemperature;//temperature of the sensor in degrees C
    bool m_bGotValidData;//flag becomes true after at least one valid reading of temperature and humidity has been obtained
#ifndef _WIN32
    pthread_t m_threadId;//thread id for getting humidity / temperature data from DHT22
    pthread_mutex_t m_threadMutex;//mutex for making sure data obtained from this class is correct
#endif

    void StopHumidityThread();//stop thread for getting humidity data
    bool CollectDHT22RawData(unsigned short *data);//collect raw data from DHT22 sensor

};