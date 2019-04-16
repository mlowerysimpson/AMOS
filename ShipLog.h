//ShipLog.h  defines the ShipLog class for saving the boat's state to a text file 
#pragma once
#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <pthread.h>
#include <unistd.h>
#include <wiringPi.h>
#endif
#include <fcntl.h>


class ShipLog {
public:
	ShipLog();//constructor
	ShipLog(char *logFilename);//alternate constructor, used for saving a log file with a specific name
	~ShipLog();//destructor
	void LogEntry(char *szLogEntry, bool bPrintToScreen);//add an entry to the log file, and optionally print to screen (standard output)
	double SecondsSinceLastLog();//return the time in seconds since the last log was made
	
private:
	//data
#ifndef _WIN32
	pthread_mutex_t m_logMutex;//control thread access to shiplog file
#endif
	char *m_szLogFilename;//the name of the log file
	std::ofstream *m_logFile;//the output log file
	unsigned int m_uiLastLogTime;//last log time in milliseconds since the program was started

	//functions
	bool GetLogFileName();//generate a name for the log file with date / time stamp embedded in it
};
