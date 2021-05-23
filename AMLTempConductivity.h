//AMLTempConductivity.h --> class for getting temperature and conductivity data from a AML-1 X2change (orange line) sensor
#pragma once
#ifndef _WIN32
#include <pthread.h>
#else
typedef int pthread_mutex_t;
typedef int pthread_t;
#endif
#include "Sensor.h"

#define CT_SER_BUFSIZE 1024  //size of input serial buffer
#define CT_AVG_DEPTH 10 //number of samples to average together in a rolling average

class AMLTempConductivity : public Sensor
{
  public:
      AMLTempConductivity(char* szSerialPort);//constructor
      ~AMLTempConductivity();//destructor
      //data
      bool m_bExitDataCollectThread;//true if it is time to exit the data collection thread
      bool m_bSerportThreadRunning;//true if the serial data collection thread is running
      //functions
      double GetTemperature();//get the most recently acquired temperature data in deg C
      double GetConductivity();//get the most recently acquired conductivity data in mS/cm

  private:
      //data
      char* m_szSerialPort;//the name of the serial port used for communicating with the temp/conductivity sensor
      double m_dTempDegC;//most recently acquired temperature in deg C
      double m_dConductivity;//most recently acquired conductivity in mS/cm
      pthread_t m_serportThreadId;//thread id for getting data over a serial link
      char m_inBuffer[CT_SER_BUFSIZE];
      int m_nBufWriteIndex;//index in input buffer where data will be written next
      int m_nNumInBuffer;//number of bytes in the input buffer that have not been processed yet
      double m_tempVals[CT_AVG_DEPTH];//last CT_AVG_DEPTH samples of temperature
      double m_conductivityVals[CT_AVG_DEPTH];//last CT_AVG_DEPTH samples of conductivity
      int m_nNumTempVals;//number of temperature values obtained (cap at CT_AVG_DEPTH)
      int m_nNumConductivityVals;//number of conductivity values obtained (cap at CT_AVG_DEPTH)
      int m_nTempValIndex;//index value to use for writing to the m_tempVals buffer
      int m_nConductivityIndex;//index value to use for writing to the m_conductivityVals buffer

      //functions
      void StartCollectionThread();
      static void* ct_serportFunction(void* pParam);//function receives depth and temperature data (in NMEA0183 format) from fish finder over serial link
      void ReadInBytes(int fd, int nNumToRead);
      void ProcessBytes();//process whatever bytes are currently in the buffer
      void StopCollectionThread();//stop the data collection thread
      int ParseBufferAt(int nBufIndex, int nNumInBuffer);//try to parse buffer at nBufIndex to see if it contains any valid data
      void AddConductivity(double dConductivity);//add conductivity value and average with previously obtained values
      void AddTemp(double dTemp);//add temperature value and average with previously obtained values
          
};