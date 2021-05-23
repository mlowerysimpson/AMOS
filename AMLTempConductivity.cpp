#include <stdio.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include <unistd.h>
#include <string> 
#include <string.h>
#include "AMLTempConductivity.h"



AMLTempConductivity::AMLTempConductivity(char *szSerialPort) : Sensor(nullptr) {
	m_szSerialPort = new char[strlen(szSerialPort) + 1];
	strcpy(m_szSerialPort, szSerialPort);
	m_dTempDegC = 0.0;//the current temperature reading in deg C
	m_dConductivity = 0.0;//the current conductivity in mS/cm
	
	m_serportThreadId = 0;
	m_bSerportThreadRunning = false;
	m_bExitDataCollectThread = false;
	
	memset(m_inBuffer,0,CT_SER_BUFSIZE);
	m_nBufWriteIndex = 0;//index in input buffer where data will be written next
	m_nNumInBuffer = 0;//number of bytes in the input buffer that have not been processed yet
	memset(m_tempVals,0,CT_AVG_DEPTH*sizeof(double));
	memset(m_conductivityVals,0,CT_AVG_DEPTH*sizeof(double));
	m_nNumTempVals=0;
	m_nNumConductivityVals=0;
	m_nTempValIndex=0;
	m_nConductivityIndex=0;
	StartCollectionThread();
}

AMLTempConductivity::~AMLTempConductivity() {
	StopCollectionThread();
	if (m_szSerialPort != nullptr) {
		delete[] m_szSerialPort;
		m_szSerialPort = nullptr;
	}
}

double AMLTempConductivity::GetTemperature() {//get the most recently acquired temperature data in deg C
	return m_dTempDegC;
}

double AMLTempConductivity::GetConductivity() {//get the most recently acquired conductivity data in mS/cm
	return m_dConductivity;
}


void AMLTempConductivity::StartCollectionThread() {//start thread for collecting fish finder depth and temperature data
	int nError = pthread_create(&m_serportThreadId, NULL, &ct_serportFunction, (void *)this);
	if (nError != 0) {
		printf("Can't create thread: %s", strerror(nError));
	}
}

void* AMLTempConductivity::ct_serportFunction(void* pParam) {//function receives conductivity and temperature data over serial link
	AMLTempConductivity* pSensorObj = (AMLTempConductivity*)pParam;
	pSensorObj->m_bSerportThreadRunning = true;
	int fd = serialOpen(pSensorObj->m_szSerialPort, 9600);
	if (fd < 0) {
		printf("Unable to open serial device: %s\n", strerror(errno));
		pSensorObj->m_bSerportThreadRunning = false;
		return nullptr;
	}
	printf("Listening for conductivity and temperature data on port %s\n", pSensorObj->m_szSerialPort);
	while (!pSensorObj->m_bExitDataCollectThread) {
		int nNumBytesAvailable = serialDataAvail(fd);
		if (nNumBytesAvailable > 0) {
			if (nNumBytesAvailable > CT_SER_BUFSIZE) {
				nNumBytesAvailable = CT_SER_BUFSIZE;
			}
			pSensorObj->ReadInBytes(fd, nNumBytesAvailable);
			pSensorObj->ProcessBytes();
		}
		delay(10);//pause for 10 ms
	}
	pSensorObj->m_bSerportThreadRunning = false;
	printf("temperature / conductivity reading thread finished\n");
	return nullptr;
}

void AMLTempConductivity::StopCollectionThread() {//stop the data collection thread
	m_bExitDataCollectThread = true;
	unsigned int uiTimeoutTime = millis() + 2000;//timeout after 2 seconds
	while (m_bSerportThreadRunning && millis() < uiTimeoutTime) {
		delay(100);
	}
}

void AMLTempConductivity::ReadInBytes(int fd, int nNumToRead) {
	for (int i = 0; i < nNumToRead; i++) {
		int nCharVal = serialGetchar(fd);//	Returns the next character available on the serial device.This call will block for up to 10 seconds if no data is available(when it will return -1)
		if (nCharVal < 0) {
			return;
		}
		m_inBuffer[m_nBufWriteIndex] = (char)nCharVal;
		m_nNumInBuffer++;
		m_nBufWriteIndex++;
		if (m_nBufWriteIndex >= CT_SER_BUFSIZE) {
			m_nBufWriteIndex = 0;
		}
	}
	if (m_nNumInBuffer > CT_SER_BUFSIZE) {
		m_nNumInBuffer = CT_SER_BUFSIZE;
	}
}

void AMLTempConductivity::ProcessBytes() {
	//process whatever bytes are currently in the buffer
	if (m_nNumInBuffer <= 0) return;
	int nNumProcessed = 0;
	int j = m_nBufWriteIndex - m_nNumInBuffer;
	if (j < 0) j += CT_SER_BUFSIZE;
	nNumProcessed = ParseBufferAt(j,m_nNumInBuffer);
	if (nNumProcessed > 0) {
		m_nNumInBuffer -= nNumProcessed;
	}
}

int AMLTempConductivity::ParseBufferAt(int nBufIndex, int nNumInBuffer) {//try to parse buffer at nBufIndex to see if it contains any valid data
	//returns the number of bytes parsed
	char readingText[128];
	if (nNumInBuffer < 6) {
		return 0;//not enough to parse anything yet
	}
	int nParsedBytes = 0;
	//look for a 0x0d 0x0a byte sequence to end a data line
	for (int i = 0; i < nNumInBuffer; i++) {
		int j = (nBufIndex + i)%CT_SER_BUFSIZE;
		int k = (j + 1) % CT_SER_BUFSIZE;
		if ((i + 1) < nNumInBuffer) {
			if (m_inBuffer[j] == 0x0d && m_inBuffer[k] == 0x0a) {
				//found termination bytes for the end of a data line
				//go back to find the 0x0a byte from the previous data line
				int k = j - 1;
				int m = 0;
				if (k < 0) {
					k += CT_SER_BUFSIZE;
				}
				while (m < 128 && m_inBuffer[k] != 0x0a) {
					m++;
					k--;
					if (k < 0) {
						k += CT_SER_BUFSIZE;
					}
				}
				if (m >= 128) {
					return 0;
				}
				int nDataStart = k + 1;
				int n = 0;
				for (n = 0; n < m; n++) {
					readingText[n] = m_inBuffer[(nDataStart + n) % CT_SER_BUFSIZE];
				}
				readingText[n] = 0;//null terminate sample text
				double dConductivity = 0.0, dTemp = 0.0;
				if (sscanf(readingText, "%lf %lf", &dConductivity, &dTemp) == 2) {
					AddConductivity(dConductivity);//add conductivity value and average with previously obtained values
					AddTemp(dTemp);//add temperature value and average with previously obtained values
				}
				return (i + 1);//scanned (i+1) bytes ending in a 0x0d, 0x0a
			}
		}
	}
	return 0;
}

void AMLTempConductivity::AddConductivity(double dConductivity) {//add conductivity value and average with previously obtained values
	m_conductivityVals[m_nConductivityIndex] = dConductivity;
	m_nNumConductivityVals++;
	m_nConductivityIndex = (m_nConductivityIndex + 1) % CT_AVG_DEPTH;
	if (m_nNumConductivityVals > CT_AVG_DEPTH) {
		m_nNumConductivityVals = CT_AVG_DEPTH;
	}
	//average conductivity values
	double dTotalVal = 0.0;
	for (int i = 0; i < m_nNumConductivityVals; i++) {
		dTotalVal += m_conductivityVals[i];
	}
	m_dConductivity = dTotalVal / m_nNumConductivityVals;
}

void AMLTempConductivity::AddTemp(double dTemp) {//add temperature value and average with previously obtained values
	m_tempVals[m_nTempValIndex] = dTemp;
	m_nNumTempVals++;
	m_nTempValIndex = (m_nTempValIndex + 1) % CT_AVG_DEPTH;
	if (m_nNumTempVals > CT_AVG_DEPTH) {
		m_nNumTempVals = CT_AVG_DEPTH;
	}
	//average temperature values
	double dTotalVal = 0.0;
	for (int i = 0; i < m_nNumTempVals; i++) {
		dTotalVal += m_tempVals[i];
	}
	m_dTempDegC = dTotalVal / m_nNumTempVals;
}
