#include "Util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <wiringPi.h>
#include <unistd.h>
#include <stdlib.h>

Util::Util() {

}

Util::~Util() {

}

/**
 * @brief get a character from the keyboard without blocking
 * 
 * @return int the code of the key typed, or -1 if no key was typed
 */
int Util::getch_noblock() {
    if (_kbhit()) {
        return getchar();
	}
    return -1;
}

int Util::_kbhit() {
	//Linux (POSIX) implementation of _kbhit().
	//function by: Morgan McGuire, morgan@cs.brown.edu
	static const int STDIN = 0;
	static bool initialized = false;

	if (!initialized) {
		// Use termios to turn off line buffering
		termios term;
		tcgetattr(STDIN, &term);
		term.c_lflag &= ~ICANON;
		tcsetattr(STDIN, TCSANOW, &term);
		setbuf(stdin, NULL);
		initialized = true;
	}

	int bytesWaiting;
	ioctl(STDIN, FIONREAD, &bytesWaiting);
	return bytesWaiting;
}

/**
 * @brief check an array of program arguments to see if any of them contain text that looks like a request for help (i.e. -h or -H).
 * 
 * @param argc the number of program arguments to check.
 * @param argv the array of program arguments.
 * @return true if at least one of the program arguments contains the text "-h" or "-H".
 * @return false if none of the program arguments contain the text "-h" or "-H".
 */
bool Util::ContainsHelpFlag(int argc, const char * argv[]) {//returns true if any of the arguments contain -h or -H
	for (int i=0;i<argc;i++) {
		if (strstr(argv[i],(char *)"-h")) {
			return true;
		}
		else if (strstr(argv[i],(char *)"-H")) {
			return true;
		}
	}
	return false;
}

/**
 * @brief return true if szDevice corresponds to a serial port on this computer, i.e. is of the form /dev/... 
 * 
 * @param szDevice the text to check to see if it corresponds to a serial port on this device, i.e. is of the form: /dev/serial# where # is some number >=0
 */
bool Util::isSerPort(const char *szDevice) {
	int nPortNumber=0;
	if (szDevice==nullptr) return false;
	if (strstr(szDevice,(char *)"/dev")) {
		return true;
	}
	return false;
}


/**
 * @brief get next time, assuming that the day is subdivided into intervals of length nLoggingIntervalSec seconds
 * 
 * @param nLoggingIntervalSec the interval in seconds used to subdivide the 24 hour day.
 * @return time_t the time of the next interval
 */
time_t Util::GetNextIntervalTime(int nLoggingIntervalSec) {
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	int nSecondsSinceMidnight = timeinfo->tm_hour*3600 + timeinfo->tm_min*60 + timeinfo->tm_sec;
	if (nLoggingIntervalSec==0) {//special case where nLoggingIntervalSec==0, just use current time
		return rawtime;
	}
	int nNumIntervals = nSecondsSinceMidnight / nLoggingIntervalSec;
	nNumIntervals++;
	int nTotalTimeSeconds = nNumIntervals * nLoggingIntervalSec;
	int nTimeDif = nTotalTimeSeconds - nSecondsSinceMidnight;
	
	time_t sampleTime = nTimeDif + rawtime;
	return sampleTime;
}

/**
 * @brief calculate the linear regression through a bunch of points
 * 
 * @param x a vector of the x-axis coordinates for the points for which the linear regression will be returned.
 * @param y a vector of the y-axis coordinates for the points for which the linear regression will be returned.
 * @return double the slope through the set of x, y points
 */
double Util::slope(const std::vector<double>& x, const std::vector<double>& y) {
	if(x.size() != y.size()){
        return 0.0;
    }
	else if (x.size()==0) {
		return 0.0;
	}
    double n = x.size();
	double sumX = 0.0;
	double sumY = 0.0;
	for (int i=0;i<n;i++) {
		sumX+=x[i];
		sumY+=y[i];
	}
    double avgX = sumX / n;
    double avgY = sumY / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    if(denominator == 0){
        return 0.0;
    }
    return numerator / denominator;
}

/**
 * @brief tries for uiTimeoutMS milliseconds to lock a mutex, if it can't do it during that time, return false
 * 
 * @param mut pointer to the mutex we are trying to lock
 * @param uiTimeoutMS a timeout value in milliseconds, after which it is time to give up trying to lock the mutex
 * @return true if the mutex was successfully locked
 * @return false if the mutex could not be locked during the specified time period
 */
bool Util::trylock(pthread_mutex_t *mut, unsigned int uiTimeoutMS) {
	unsigned int uiTimeoutTime = millis() + uiTimeoutMS;
	while (millis()<uiTimeoutTime) {
		if (pthread_mutex_trylock(mut)==0) {
			return true;
		}
		usleep(1000);
	}
	return false;
}

/**
 * @brief converts a heading in degrees to a 2-element vector with components in the E-W and N-S directions
 * 
 * @param dHeadingDeg a heading in degrees (0 to 360)
 * @param heading_vec a two-element vector that represents the direction vector of the heading in E-W and N-S directions
 */
void Util::HeadingToVec(double dHeadingDeg,double heading_vec[]) {
	const double dPI = 3.14159;
	heading_vec[0] = sin(dHeadingDeg*dPI/180);
	heading_vec[1] = cos(dHeadingDeg*dPI/180);
}