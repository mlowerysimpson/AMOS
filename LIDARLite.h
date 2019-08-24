//LIDARLite.h
#pragma once
#define LIDARLITE_ADDR_DEFAULT 0x62
#include <pthread.h>

//possible modes of operation
#define DEFAULT_MODE 0 //default mode, balanced performance
#define SHORTRANGE_HIGHSPEED 1 //short range, high speed, uses 0x1d maximum acquisition count.
#define HIGHERSPEED_SHORTRANGE 2 //higher speed, short range. Turns on quick termination detection for faster measurements at short range (with decreased accuracy)
#define MAXIMUM_RANGE 3 //maximum range, uses 0xff maximum acquisition count.
#define HIGH_SENSITIVITY 4 //High sensitivity detection. Overrides default valid measurement detection algorithm, and uses a threshold value for high sensitivity and noise.
#define LOW_SENSITIVITY 5 //low sensitivity detection. Overrides default valid measurement detection algorithm, and uses a threshold value for low sensitivity and noise.

//pin for toggling LiDAR on / off
#define LIDAR_ON_OFF_PIN 10

class LIDARLite
{
  public:
      LIDARLite(pthread_mutex_t *i2c_mutex);
      bool configure(int nOperationMode);//select one of 6 possible preset configurations for the LiDAR device
      bool reset();//reset the device with default settings, takes approx. 22 ms.
      unsigned short getDistance();//get distance to object, returns measured distance in cm
      void TurnOn(bool bTurnOn);//turn on / off device (uses orange wire to enable / disable power)
      bool m_bInitError;//true if there was an error trying to initialize the LiDAR device
    
  private:
      bool m_bStopped;//flag is true when the LiDAR device has been stopped (i.e. put into low power mode)
      pthread_mutex_t *m_i2c_mutex;
      bool m_bOpenedI2C_OK;
	  bool m_bLidarInitialized_OK;
      int m_file_i2c;//handle to I2C port for the LiDAR connection
      bool write_itc(unsigned char ucRegister,unsigned char ucData);//writes a single byte to a register using I2C
      void lockmutex();//lock mutex for access to the I2C bus
      void unlockmutex();//unlock mutex for access to the I2C bus
};