#pragma once
#include <wiringPi.h>
#include <pthread.h>

#define SER_BUFSIZE 1024  //size of input serial buffer
/**
 * @brief class for sending RTK correction data over serial port to GPS board
 *
 */
class RTK {
public:
    RTK(char *szSerialPort);
    ~RTK();
    //data:
    char* m_szSerialPort;//the serial port used to receive the data from the fish finder (ex: "/dev/ttyUSBPort3")
    //functions
    bool SendSerialRTKData(unsigned char* dataBytes, int nNumBytes);
    
private:
    int m_nSerportHandle;//serial port handle
};