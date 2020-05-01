#pragma once
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>
#include <ctime>
#include <vector>
#include <pthread.h>

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
    static void HeadingToVec(double dHeadingDeg,double heading_vec[]);//converts a heading in degrees to a 2-element vector with components in the E-W and N-S directions
    time_t GetNextLogTime(int nLoggingIntervalSec);//get the time of the next sample (in seconds since midnight, Jan, 1, 1970)
	//test
	//printf("nLoggingIntervalSec = %d\n",nLoggingIntervalSec);
	//end test
	static time_t GetNextIntervalTime(int nLoggingIntervalSec);//get next time, assuming that the day is subdivided into intervals of length nLoggingIntervalSec seconds
    static double slope(const std::vector<double>& x, const std::vector<double>& y);//calculate the linear regression through a bunch of points
    static bool trylock(pthread_mutex_t *mut, unsigned int uiTimeoutMS);//tries for uiTimeoutMS milliseconds to lock a mutex, if it can't do it during that time, return false
private:
    static int _kbhit();
};