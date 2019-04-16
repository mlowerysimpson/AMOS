#pragma once
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>
#include <ctime>

using namespace std;

/**
 * @brief general utility class 
 * 
 */
class Util {//general utility class 
public:
    Util();//constructor
    ~Util();//destructor    
    static int getch_noblock();//get a character from the keyboard without blocking
    static bool ContainsHelpFlag(int argc, const char * argv[]);//returns true if any of the arguments contain -h or -H
    static bool isSerPort(const char *szDevice);//return true if szDevice corresponds to a serial port on this computer, i.e. is of the form /dev/serial# where # is some number >=0
    time_t GetNextLogTime(int nLoggingIntervalSec);//get the time of the next sample (in seconds since midnight, Jan, 1, 1970)
	//test
	//printf("nLoggingIntervalSec = %d\n",nLoggingIntervalSec);
	//end test
	static time_t GetNextIntervalTime(int nLoggingIntervalSec);//get next time, assuming that the day is subdivided into intervals of length nLoggingIntervalSec seconds
private:
    static int _kbhit();
};