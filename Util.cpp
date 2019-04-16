#include "Util.h"
#include <stdio.h>
#include <string.h>

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
 * @brief return true if szDevice corresponds to a serial port on this computer, i.e. is of the form /dev/serial# where # is some number >=0
 * 
 * @param szDevice the text to check to see if it corresponds to a serial port on this device, i.e. is of the form: /dev/serial# where # is some number >=0
 */
bool Util::isSerPort(const char *szDevice) {//
	int nPortNumber=0;
	if (szDevice==nullptr) return false;
	if (sscanf(szDevice,"/dev/serial%d",&nPortNumber)>0) {
		if (nPortNumber>=0) {
			return true;
		}
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