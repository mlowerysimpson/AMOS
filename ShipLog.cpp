//ShipLog.cpp  implementation file for ShipLog class
#include "ShipLog.h"
#include "Util.h"


ShipLog::ShipLog() {
	m_logFile = nullptr;
	m_szLogFilename = nullptr;
	m_uiLastLogTime=0;
	GetLogFileName();//generate a name for the log file with date / time stamp embedded in it
#ifndef _WIN32
	m_logMutex = PTHREAD_MUTEX_INITIALIZER;
#endif
}

ShipLog::ShipLog(char *logFilename) {//alternate constructor, used for saving a log file with a specific name
	m_logFile=nullptr;
	m_szLogFilename = new char[strlen(logFilename)+1];
	strcpy(m_szLogFilename,logFilename);
	m_uiLastLogTime=0;
#ifndef _WIN32
	m_logMutex = PTHREAD_MUTEX_INITIALIZER;
#endif
}

ShipLog::~ShipLog() {
	if (m_logFile) {
		if (m_logFile->is_open()) {
			m_logFile->close();
		}
		delete m_logFile;
		m_logFile=nullptr;
	}
	if (m_szLogFilename) {
		delete [] m_szLogFilename;
		m_szLogFilename=nullptr;
	}
}

//LogEntry: add a line of text to the ship's log file
//szLogEntry: the text to add (does not need to contain a carriage return / linefeed at the end, as these are added automatically by the function)
//bPrintToScreen: if true, also adds the log text to the standard output
void ShipLog::LogEntry(char *szLogEntry, bool bPrintToScreen) {//add an entry to the log file, and optionally print to screen (standard output)
#ifndef _WIN32
	//pthread_mutex_lock(&m_logMutex);
	if (!Util::trylock(&m_logMutex,1000)) {//unable to get mutex, just return without logging anything
		return;
	}
#endif
	if (!m_logFile) {
		m_logFile = new std::ofstream();
	}
	if (!m_logFile->is_open()) {
		m_logFile->open(m_szLogFilename,std::ios_base::out|std::ios_base::app);
	}
	char szDateTimeStamp[256];
	int nEntrySize = strlen(szLogEntry);
	char *szOutput = new char[nEntrySize+3];
	strcpy(szOutput,szLogEntry);
	//get date/time stamp corresponding to current date and time
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
	sprintf(szDateTimeStamp,"%d-%02d-%02d, %02d:%02d:%02d   ",
		timeinfo->tm_year+1900,timeinfo->tm_mon+1,
		timeinfo->tm_mday,timeinfo->tm_hour,
		timeinfo->tm_min,timeinfo->tm_sec);
	m_logFile->write(szDateTimeStamp,strlen(szDateTimeStamp));
	//add carriage return and linefeed
	szOutput[nEntrySize]='\r';
	szOutput[nEntrySize+1]='\n';
	szOutput[nEntrySize+2]=0;
	m_logFile->write(szOutput,nEntrySize+2);
	if (bPrintToScreen) {
		printf(szOutput);
	}
	delete []szOutput;
#ifndef _WIN32
	m_uiLastLogTime = millis();
#endif
	m_logFile->close();
#ifndef _WIN32
	pthread_mutex_unlock(&m_logMutex);
#endif
}

bool ShipLog::GetLogFileName() {//generate a name for the log file with date / time stamp embedded in it
	//m_logFile = new std::ofstream();
	//get date/time stamp corresponding to current date and time
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
	char szFilename[256];
	sprintf(szFilename,"shiplog_%d_%02d_%02d_%02d_%02d_%02d.txt",
		timeinfo->tm_year+1900,timeinfo->tm_mon+1,
		timeinfo->tm_mday,timeinfo->tm_hour,
		timeinfo->tm_min,timeinfo->tm_sec);
	//m_logFile->open(szFilename,std::ios_base::out|std::ios_base::app);
	//save name of log file
	m_szLogFilename = new char[strlen(szFilename)+1];
	strcpy(m_szLogFilename, szFilename);
	return true;
}

//SecondsSinceLastLog: return the number of seconds since the last log entry (note the first call to this function will not be accurate, as it will represent the time elapsed since the program started)
double ShipLog::SecondsSinceLastLog() {//
#ifndef _WIN32
	return ((double)(millis() - m_uiLastLogTime))/1000.0;	
#else
	return 0.0;
#endif

}