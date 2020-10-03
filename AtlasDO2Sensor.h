//AtlasDO2Sensor.h --> class for getting dissolved oxygen data from Atlas Scientific's dissolved oxygen probe (EZO-DO (TM) Embedded Dissolved Oxygen Circuit)
#pragma once
#define DO2SENSOR_ADDR_DEFAULT 0x61   //the default I2C address for the EZO-DO device
#ifndef _WIN32
#include <pthread.h>
#else
typedef int pthread_mutex_t;
#endif
#include "Sensor.h"

//I2C response codes for EZO-DO
#define NO_DATA_TO_SEND 255
#define NOT_READY 254
#define SYNTAX_ERROR 2
#define DATA_OK 1


class AtlasDO2Sensor : public Sensor
{
  public:
      AtlasDO2Sensor(pthread_mutex_t *i2c_mutex);
      bool GetDO2(double& dDO2Measurement);//get the dissolved oxygen concentration in mg/L
      bool GetDO2(double& dDO2Measurement, double dTempDegC);//get the dissolved oxygen concentration in ml/L, given that the water is at a certain temperature (i.e. apply temperature compensation for the measurement)
      bool Calibrate();//perform a single-point calibration of the dissolved O2 probe 
      void EnterSleepMode();//go into sleep mode to conserve a bit of power
      void ExitSleepMode();//come out of sleep mode for making measurements
      bool ProtocolLock(int nLock);//lock or unlock i2c communications for the device
          
  private:
      pthread_mutex_t *m_i2c_mutex;
      bool m_bOpenedI2C_OK;
	  bool m_bDO2Initialized_OK;
      int m_file_i2c;//handle to I2C port for the LiDAR connection
      bool write_i2c(unsigned char *ucData, int nNumBytes);//writes bytes to I2C address
      void lockmutex();//lock mutex for access to the I2C bus
      void unlockmutex();//unlock mutex for access to the I2C bus
};