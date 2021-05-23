#include <stdio.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include <unistd.h>
#include <string> 
#include <string.h>
#include "RTK.h"


RTK::RTK(char* szSerialPort) {
	m_szSerialPort = new char[strlen(szSerialPort) + 1];
	strcpy(m_szSerialPort, szSerialPort);
	m_nSerportHandle = serialOpen(m_szSerialPort, 115200);
}

RTK::~RTK() {
	if (m_nSerportHandle != -1) {
		serialClose(m_nSerportHandle);
	}
	if (m_szSerialPort != nullptr) {
		delete[] m_szSerialPort;
		m_szSerialPort = nullptr;
	}
}

bool RTK::SendSerialRTKData(unsigned char* dataBytes, int nNumBytes) {
	if (m_nSerportHandle == -1) {
		return false;
	}
	for (int i = 0; i < nNumBytes; i++) {
		serialPutchar(m_nSerportHandle, dataBytes[i]);
	}
	return true;
}