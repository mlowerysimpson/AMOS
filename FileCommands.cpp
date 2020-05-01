//FileCommands.cpp
#include "FileCommands.h"
#include "ShipLog.h"
#include "filedata.h"
#include "Util.h"
#include "SensorDeploy.h"
#include <sys/stat.h>
#include <fstream>
#include <algorithm>
#include <pthread.h>
#include <fcntl.h>

//FileCommands constructor
//szRootFolder = the root program folder (i.e. the folder where the prefs.txt (preferences) file is located
//szFilename = the name of the text file that contains all of the commands to execute
//pNavigator = pointer to navigation object used to get GPS data, compass data, compute courses, etc.
//pThrusters = pointer to object used to control power to the boat's 2 propellers
//pSensorDataFile = pointer to a SensorDataFile object that is used for storing sensor data to a text file.
//pLidar = pointer to a LIDARLite object that is used for collecting LiDAR data for obstacle avoidance.
//bCancel = pointer to boolean variable that will be "true" when the program is ending. It should be checked to end any currently executing file commands.
FileCommands::FileCommands(char *szRootFolder, char *szFilename, Navigation *pNavigator, Thruster *pThrusters, SensorDataFile *pSensorDataFile, LIDARLite *pLidar, bool *bCancel) {//constructor, takes the file path of a commands file as argument
	m_szRootFolder = szRootFolder;//the folder where the prefs.txt (preferences) file is located 
	m_bCancel = bCancel;
	m_pBatteryCharge = nullptr;
	m_pSensorDataFile = pSensorDataFile;
	m_pLiDAR = pLidar;
	m_bFileOK = false;
	m_bRestTimeMode = false;
	m_bTimeChecking = false;
	m_bSleepTime = false;
	m_bTravelToSunnySpot = false;
	m_bTraveledToSunnySpot = false;
	m_uiPictureNumber = 0;
	m_dMorningTimeHrs = 0.0;
	m_dLowVoltage = 0.0;
	m_dRestTimeHrs = 0.0;
	m_dSleepTimeHrs = 0.0;
	m_szFilename=nullptr;
	m_nWaitTimeSeconds = 0;
	
	if (doesFileExist(szFilename)) {
		int nFilenameLength =strlen(szFilename);
		m_szFilename = new char[nFilenameLength+1];
		strcpy(m_szFilename, szFilename);
		m_bFileOK = readInFile();
	}
	m_fLastHeadingCommandAngle=0;
	m_pNavigator = pNavigator;//the navigation object used for getting GPS data, compass data, plotting courses, etc.
	m_pThrusters = pThrusters;//the 2 props that are used to propel the boat around
	m_nCurrentCommandIndex=0;
	SetRoutePlan();
	CheckTime();
}

FileCommands::~FileCommands() {//destructor
	cleanup();
}

void FileCommands::cleanup() {
	if (m_szFilename) {
		delete []m_szFilename;
		m_szFilename=nullptr;
	}
	int nNumCommands = m_commandList.size();
	for (int i=0;i<nNumCommands;i++) {
		if (m_commandList[i]) {
			RemoteCommand::DeleteCommand(m_commandList[i]);
			m_commandList[i]=nullptr;
		}
	}
	m_commandList.clear();
}

bool FileCommands::doesFileExist(char *szFilename) {//return true if file exists
	FILE *testFile = fopen(szFilename,"r");
	if (testFile) {
		fclose(testFile);
		return true;
	}
	return false;
}

bool FileCommands::readInFile() {
	std::ifstream commandFile(m_szFilename, std::ifstream::in);
    std::string sLine;

	int nLineNumber=1;
    while( std::getline(commandFile, sLine ) ) {
		if (!parseLine(sLine)) {
			printf("Error trying to parse line #%d of file: %s.\n",
				nLineNumber,m_szFilename);
			return false;
		}
		nLineNumber++;
    }
	if (!CheckConsistency()) {
		printf("Error, one or more \"goto\" commands in %s point to a label that does not exist.\n",
			m_szFilename);
		return false;
	}
	return true;
}

bool FileCommands::parseLine(std::string sLine) {//parse command from line of text
	//convert sLine to lowercase:
	//first strip off any commmas in line
	int nCommentIndex = sLine.find("//");
	if (nCommentIndex==0) {//entire line is a comment, just ignore line
		return true;
	}
	if (nCommentIndex>0) {
		sLine = sLine.substr(0,nCommentIndex);
	}
	std::transform(sLine.begin(), sLine.end(), sLine.begin(), ::tolower);
	int nHeadingIndex = sLine.find("heading");
	int nPhotoIndex = sLine.find("photo");
	if (nHeadingIndex>=0&&nPhotoIndex<0) {//heading command
		int i = skipToNumericVal(sLine, nHeadingIndex+7);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		float fHeading=0;
		if (sscanf(sRemainder.c_str(),"%f",&fHeading)>0) {
			//parsed desired heading in degrees, add to list of command
			AddCommand(FC_HEADING, (double)fHeading);
			return true;
		}
		else {
			return false;//heading is not formatted correctly(?)
		}
	}
	int nLabelIndex = sLine.find("label");
	if (nLabelIndex>=0) {//label command
		int i=skipToTextVal(sLine, nLabelIndex+5);
		if (i<0) return false;
		std::string sLabel = sLine.substr(i);
		sLabel = trimEndOfText(sLabel);//get rid of carriage returns, line feeds, spaces, or tabs from end of string
		if (sLabel.length()<=0) return false;
		if (sLabel.length()>0) {
			AddCommand(FC_LABEL, sLabel);
			return true;
		}
		return false;
	}
	int nForwardIndex = sLine.find("forward");
	if (nForwardIndex>=0) {//go forwards command
		int i = skipToNumericVal(sLine, nForwardIndex+7);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		int nTimeHrs=0, nTimeMin=0, nTimeSec=0;//get time in HH:MM:SS format
		if (sscanf(sRemainder.c_str(),"%d:%d:%d",&nTimeHrs,&nTimeMin,&nTimeSec)==3) {
			//parsed desired time in HH:MM:SS format
			AddCommand(FC_FORWARD,nTimeHrs,nTimeMin,nTimeSec);
			return true;
		}
		else {
			return false;//forward time is not formatted correctly?
		}
	}
	int nGotoIndex = sLine.find("goto");
	if (nGotoIndex>=0) {//go to a particular label
		int i = skipToTextVal(sLine, nGotoIndex+4);
		if (i<0) return false;
		std::string sGotoLabel = sLine.substr(i);
		sGotoLabel = trimEndOfText(sGotoLabel);//get rid of carriage returns, line feeds, spaces, or tabs from end of string
		if (sGotoLabel.length()<=0) return false;
		AddCommand(FC_GOTO,sGotoLabel);
		return true;
	}
	int nWaypointIndex = sLine.find("waypoint");
	if (nWaypointIndex>=0) {//drive to the specified GPS waypoint
		int i = skipToNumericVal(sLine, nWaypointIndex+8);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		double dLatitude=0.0, dLongitude=0.0;
		if (sscanf(sRemainder.c_str(),"%lf,%lf",&dLatitude,&dLongitude)==2) {
			//parsed desired latitude and longitude, add to list of commands
			AddCommand(FC_GPS_WAYPOINT, dLatitude, dLongitude);
			return true;
		}
		else {
			return false;//GPS waypoint is not formatted correctly(?)
		}
	}
	if (nPhotoIndex>=0) {//take one or more photos at the current location
		int nNumPhotos = 0;
		double dStartHeading = 0.0;//heading at which we start taking photos
		double dEndHeading = 0.0;//heading at which the last photo is taken
		int nEndHeadingIndex = sLine.find("end_heading");	
		if (nEndHeadingIndex>=0) {
			if (sscanf(sLine.c_str(),"photo: num=%d, start_heading =%lf, end_heading =%lf",&nNumPhotos,&dStartHeading,&dEndHeading)!=3) {
				//problem parsing line, not formatted correctly?
				return false;
			}
			double dHeadingRange = dEndHeading-dStartHeading;
			AddCommand(FC_HEADING,dStartHeading);
			AddCommand(FC_PHOTO);
			for (int i=1;i<(nNumPhotos-1);i++) {
				double dIntermediateHeading = 0.0;
				double dHeadingDif = dEndHeading - dStartHeading;
				if (dHeadingDif>=-180&&dHeadingDif<=180) {
					dIntermediateHeading = ((double)i) / (nNumPhotos - 1) * (dEndHeading - dStartHeading) + dStartHeading;
				}
				else if (dHeadingDif<-180) {
                    double dStart = dStartHeading - 360;
                    dIntermediateHeading = ((double)i) / (nNumPhotos - 1) * (dEndHeading - dStart) + dStart;
                }
                else if (dHeadingDif>180)
                {
                    double dStop = dEndHeading - 360;
                    dIntermediateHeading = ((double)i) / (nNumPhotos - 1) * (dStop - dStartHeading) + dStartHeading;
                }
				AddCommand(FC_HEADING, dIntermediateHeading);
				AddCommand(FC_PHOTO);
			}
			AddCommand(FC_HEADING,dEndHeading);
			AddCommand(FC_PHOTO);
		}
		else {
			//no end heading specified, so just taking a single picture
			nNumPhotos = 1;
			if (sscanf(sLine.c_str(),"photo: num=1, start_heading = %lf",&dStartHeading)!=1) {
				//problem parsing line, not formatted correctly?
				return false;
			}
			AddCommand(FC_HEADING,dStartHeading);
			AddCommand(FC_PHOTO);
		}
	}
	int nSafepointIndex = sLine.find("safepoint");
	if (nSafepointIndex>=0) {//gps location of a potential "safe point", ideal for solar re-charging
		int i = skipToNumericVal(sLine, nSafepointIndex+9);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		double dSafeLat=0.0, dSafeLong=0.0;
		if (sscanf(sRemainder.c_str(),"%lf,%lf",&dSafeLat,&dSafeLong)==2) {
			//parsed safe point latitude and longitude, add to the list of safe points
			m_safeLat.push_back(dSafeLat);
			m_safeLong.push_back(dSafeLong);
			return true;
		}
		else {
			return false;//safe point is not formatted correctly(?)
		}
	}
	int nGridSampleIndex = sLine.find("grid_sample");
	if (nGridSampleIndex>=0) {//collect m x n sensor samples in a grid pattern
		int i = skipToNumericVal(sLine, nWaypointIndex+11);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		double dLatitudeCorner1=0.0, dLongitudeCorner1=0.0;//1st corner of grid that will be sampled
		double dLatitudeCorner2=0.0, dLongitudeCorner2=0.0;//oppposite corner of grid that will be sampled
		int nNumNSPts = 0;//number of points along north-south direction
		int nNumEWPts = 0;//number of points along east-west direction
		int nNumSamplesPerLocation=0;//the number of samples to collect at each grid point location
		if (sscanf(sRemainder.c_str(),"%lf,%lf,%lf,%lf,%d,%d,%d",
			&dLatitudeCorner1,&dLongitudeCorner1,&dLatitudeCorner2,&dLongitudeCorner2,
			&nNumNSPts,&nNumEWPts,&nNumSamplesPerLocation)==7) {
			//parsed GPS coordinates of area to sample, and the number of points to sample in each direction, add to list of commands
			AddCommand(FC_GRID_SAMPLE,dLatitudeCorner1,dLongitudeCorner1,dLatitudeCorner2,dLongitudeCorner2,
				nNumNSPts,nNumEWPts,nNumSamplesPerLocation);
			return true;
		}
		else {//something was not formatted correctly(?)
			return false;
		}
	}
	int nHoldIndex = sLine.find("hold");
	if (nHoldIndex>=0) {//hold the current GPS position for the specified period of time
		int i = skipToNumericVal(sLine, nHoldIndex+7);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		int nTimeHrs=0, nTimeMin=0, nTimeSec=0;//get time in HH:MM:SS format
		if (sscanf(sRemainder.c_str(),"%d:%d:%d",&nTimeHrs,&nTimeMin,&nTimeSec)==3) {
			//parsed desired time in HH:MM:SS format
			AddCommand(FC_HOLD,nTimeHrs,nTimeMin,nTimeSec);
			return true;
		}
		else {
			return false;//forward time is not formatted correctly?
		}
	}
	int nWaitForIndex = sLine.find("waitfor");
	if (nWaitForIndex>=0) {//wait a specified period of time, if this time is >= MIN_SLEEP_TIME_SEC then put AMOS into a low power state 
		int i = skipToNumericVal(sLine, nWaitForIndex+7);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		int nTimeHrs=0, nTimeMin=0, nTimeSec=0;//get time in HH:MM:SS format
		if (sscanf(sRemainder.c_str(),"%d:%d:%d",&nTimeHrs,&nTimeMin,&nTimeSec)==3) {
			//parsed desired time in HH:MM:SS format
			AddCommand(FC_WAIT,nTimeHrs,nTimeMin,nTimeSec);
			return true;
		}
		else {
			return false;//forward time is not formatted correctly?
		}
	}
	int nSampleIndex = sLine.find("sample");
	if (nSampleIndex>=0) {//collect samples from all sensors configured in prefs.txt file and save results to specified filename
		int i=skipToTextVal(sLine, nSampleIndex+6);
		if (i<0) return false;
		std::string sSampleFilename = sLine.substr(i);
		sSampleFilename = trimEndOfText(sSampleFilename);//get rid of carriage returns, line feeds, spaces, or tabs from end of string
		if (sSampleFilename.length()<=0) {
			sSampleFilename = "sensordata.txt";//default sensor data filename
		}
		if (sSampleFilename.length()>0) {
			AddCommand(FC_SAMPLE, sSampleFilename);
			return true;
		}
	}
	int nRestTimeIndex = sLine.find("rest_time");
	if (nRestTimeIndex>=0) {//define a "rest-time" for AMOS when it should power down the air prop, stop executing file commands, and let the solar panel charge up a bit before dark
		int i = skipToNumericVal(sLine, nWaitForIndex+9);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		int nTimeHrs=0, nTimeMin=0, nTimeSec=0;//get time in HH:MM:SS format
		if (sscanf(sRemainder.c_str(),"%d:%d:%d",&nTimeHrs,&nTimeMin,&nTimeSec)==3) {
			//parsed desired time in HH:MM:SS format
			this->m_dRestTimeHrs = nTimeHrs + nTimeMin/60.0 + nTimeSec/3600.0;
			this->m_bTimeChecking = true;
			return true;
		}
		else {
			return false;//time is not formatted correctly?
		}
	}
	int nSleepTimeIndex = sLine.find("sleep_time");
	if (nSleepTimeIndex>=0) {
		int i = skipToNumericVal(sLine, nWaitForIndex+10);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		int nTimeHrs=0, nTimeMin=0, nTimeSec=0;//get time in HH:MM:SS format
		if (sscanf(sRemainder.c_str(),"%d:%d:%d",&nTimeHrs,&nTimeMin,&nTimeSec)==3) {
			//parsed desired time in HH:MM:SS format
			this->m_dSleepTimeHrs = nTimeHrs + nTimeMin/60.0 + nTimeSec/3600.0;
			this->m_bTimeChecking = true;
			return true;
		}
		else {
			return false;//time is not formatted correctly?
		}
	}
	int nMorningTimeIndex = sLine.find("morning_time");
	if (nMorningTimeIndex>=0) {//define a "morning-time" for AMOS when it can power up the air prop, resume execution of file commands, and start moving around
		int i = skipToNumericVal(sLine, nWaitForIndex+12);
		if (i<0) return false;
		std::string sRemainder = sLine.substr(i);
		int nTimeHrs=0, nTimeMin=0, nTimeSec=0;//get time in HH:MM:SS format
		if (sscanf(sRemainder.c_str(),"%d:%d:%d",&nTimeHrs,&nTimeMin,&nTimeSec)==3) {
			//parsed desired time in HH:MM:SS format
			this->m_dMorningTimeHrs = nTimeHrs + nTimeMin/60.0 + nTimeSec/3600.0;
			this->m_bTimeChecking = true;
			return true;
		}
		else {
			return false;//time is not formatted correctly?
		}
	}
	
	//unknown command or text, just ignore and return true
	return true;
}

bool FileCommands::CheckConsistency() {//check to make sure that file commands are consistent, ex: labels exist for each of the goto commands
	int nNumCommands = m_commandList.size();
	for (int i=0;i<nNumCommands;i++) {
		REMOTE_COMMAND *pRC = m_commandList[i];
		if (pRC) {
			if (pRC->nCommand==FC_GOTO) {
				if (!matchingLabelFound((char *)pRC->pDataBytes)) {
					printf("Error, no matching label found for goto statement pointing to: %s\n",(char *)pRC->pDataBytes);
					return false;
				}
			}
		}
	}
	return true;
}

bool FileCommands::matchingLabelFound(char *szLabel) {//returns true if one of the file commands has been assigned the text label szLabel
	int nNumCommands = m_commandList.size();
	for (int i=0;i<nNumCommands;i++) {
		REMOTE_COMMAND *pRC = m_commandList[i];
		if (pRC) {
			if (pRC->nCommand==FC_LABEL) {
				if (strcmp((char *)pRC->pDataBytes,szLabel)==0) {
					return true;
				}
			}
		}
	}
	return false;
}

//skipToNumericVal: find the index of the first numeric digit in sLine, starting at nStartIndex
//sLine = line of text that we want to parse a numeric value from 
//nStartIndex = zero-based index of the character in sLine where we start looking for a numeric digit
int FileCommands::skipToNumericVal(std::string sLine, int nStartIndex) {
	int nLineLength = sLine.length();
	for (int i=0;i<nLineLength;i++) {
		if (sLine[i]>='0'&&sLine[i]<='9') {
			return i;
		}
		else if (sLine[i]=='.'||sLine[i]=='+'||sLine[i]=='-') {
			return i;
		}
	}
	return -1;//could not find the start of a numeric value
}

//skipToTextVal: find the index of the first non-whitespace or delimiter text in sLine, starting at nStartIndex
//sLine: the text line where we want to find the first occurrence of text (not including delimiters ":,;<space><tab>") on or after nStartIndex
//nStartIndex: the zero-based index of sLine where we start looking for text
int FileCommands::skipToTextVal(std::string sLine, int nStartIndex) {
	int nLineLength = sLine.length();
	for (int i=nStartIndex;i<nLineLength;i++) {
		if (sLine[i]!=' '&&sLine[i]!='\t'&&sLine[i]!=','&&sLine[i]!=':'&&sLine[i]!=';'&&sLine[i]!='\r'&&sLine[i]!='\n') {
			return i;
		}
	}
	return -1;//could not find any valid text on or after nStartIndex
}

//AddCommand: adds a command to the list that is associated with a single numeric value
//nCommandType: integer code for the particular command type being added (see CommandList.h for list of commands)
//dVal: the numeric value associated with the command (ex: heading)
void FileCommands::AddCommand(int nCommandType, double dVal) {
	if (nCommandType==FC_HEADING) {//command to change heading
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND();
		memset(pRC,0,sizeof(REMOTE_COMMAND));
		pRC->nCommand=FC_HEADING;
		pRC->nNumDataBytes = sizeof(float);
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		float fHeading = (float)dVal;
		memcpy(pRC->pDataBytes,&fHeading,pRC->nNumDataBytes);
		m_commandList.push_back(pRC);
	}
}

//AddCommand: adds a command to the list that is associated with two numeric values (ex: latitude and longitude)
//nCommandType: integer code for the particular command type being added (see CommandList.h for list of commands)
//dVal1: the first numeric value associated with the command (ex: latitude)
//dVal2: the second numeric value associated with the command (ex: longitude)
void FileCommands::AddCommand(int nCommandType, double dVal1, double dVal2) {
	if (nCommandType==FC_GPS_WAYPOINT) {//command to drive to a GPS location
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND();
		memset(pRC,0,sizeof(REMOTE_COMMAND));
		pRC->nCommand=FC_GPS_WAYPOINT;
		pRC->nNumDataBytes = 2*sizeof(double);
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		double dLatitude = dVal1;
		double dLongitude = dVal2;
		memcpy(pRC->pDataBytes,&dLatitude,sizeof(double));
		memcpy(&pRC->pDataBytes[sizeof(double)],&dLongitude,sizeof(double));
		m_commandList.push_back(pRC);
	}
}

//AddCommand: adds a command to the list that is associated with six numeric values, 4 floating point and 2 integer
//nCommandType: integer code for the particular command type being added (see CommandList.h for list of commands)
//dVal1: the first numeric value associated with the command (floating point value)
//dVal2: the second numeric value associated with the command (floating point value)
//dVal3: the third numeric value associated with the command (floating point value)
//dVal4: the fourth numeric value associated with the command (floating point value)
//nVal5: the fifth numeric value associated with the command (integer value)
//nVal6: the sixth numeric value associated with the command (integer value)
//nVal7: the seventh numeric value associated with the command (integer value)
void FileCommands::AddCommand(int nCommandType, double dVal1, double dVal2, double dVal3, double dVal4, int nVal5, int nVal6, int nVal7) {
	if (nCommandType==FC_GRID_SAMPLE) {//command to take a number of sensor samples in a grid pattern
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND();
		memset(pRC,0,sizeof(REMOTE_COMMAND));
		pRC->nCommand=FC_GRID_SAMPLE;
		pRC->nNumDataBytes = 4*sizeof(double)+3*sizeof(int);
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		//latitude & longitude of 1st corner
		double dLatitudeCorner1 = dVal1;
		double dLongitudeCorner1 = dVal2;
		double dLatitudeCorner2 = dVal3;
		double dLongitudeCorner2 = dVal4;
		int nNumNSPts = nVal5;//number of points to sample along north-south direction
		int nNumEWPts = nVal6;//number of points to sample along east-west direction
		int nNumSamplesPerLocation = nVal7;//the number of samples to collect at each grid point location
		memcpy(pRC->pDataBytes,&dLatitudeCorner1,sizeof(double));
		memcpy(&pRC->pDataBytes[sizeof(double)],&dLongitudeCorner1,sizeof(double));
		memcpy(&pRC->pDataBytes[2*sizeof(double)],&dLatitudeCorner2,sizeof(double));
		memcpy(&pRC->pDataBytes[3*sizeof(double)],&dLongitudeCorner2,sizeof(double));
		memcpy(&pRC->pDataBytes[4*sizeof(double)],&nNumNSPts,sizeof(int));
		memcpy(&pRC->pDataBytes[4*sizeof(double)+sizeof(int)],&nNumEWPts,sizeof(int));
		memcpy(&pRC->pDataBytes[4*sizeof(double)+2*sizeof(int)],&nNumSamplesPerLocation,sizeof(int));
		m_commandList.push_back(pRC);
	}
}

//AddCommand: adds a text-based command to the list of commands (ex: goto command or label command)
//nCommandType: the type of command being added (see CommandList.h for a list of commands)
//sText: the text associated with the command
void FileCommands::AddCommand(int nCommandType, std::string sText) {
	int nTextLength = sText.length();
	if (nCommandType==FC_LABEL||nCommandType==FC_GOTO||nCommandType==FC_SAMPLE) {
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND;
		int nTextLength = sText.length();
		pRC->nCommand = nCommandType;
		pRC->nNumDataBytes = nTextLength+1;
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		strcpy((char *)pRC->pDataBytes,sText.c_str());
		m_commandList.push_back(pRC);
	}
}

//AddCommand: adds a command to the list that is associated with a time in hrs, min, sec
//nCommandType: the type of command being added (see CommandList.h for a list of commands)
//nTimeHrs: the length of time in hours to carry out the command
//nTimeMin: the length of time in minutes to carry out the command
//nTimeSec: the length of time in seconds to carry out the command
void FileCommands::AddCommand(int nCommandType, int nTimeHrs, int nTimeMin, int nTimeSec) {
	if (nCommandType==FC_FORWARD) {
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND;
		pRC->nCommand = FC_FORWARD;
		int nTotalTimeSec = 3600*nTimeHrs+60*nTimeMin+nTimeSec;
		pRC->nNumDataBytes = sizeof(int);
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		memcpy(pRC->pDataBytes,&nTotalTimeSec,sizeof(int));
		m_commandList.push_back(pRC);
	}
	else if (nCommandType==FC_HOLD) {
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND;
		pRC->nCommand = FC_HOLD;
		int nTotalTimeSec = 3600*nTimeHrs+60*nTimeMin+nTimeSec;
		pRC->nNumDataBytes = sizeof(int);
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		memcpy(pRC->pDataBytes,&nTotalTimeSec,sizeof(int));
		m_commandList.push_back(pRC);
	}
	else if (nCommandType==FC_WAIT) {
		REMOTE_COMMAND *pRC = new REMOTE_COMMAND;
		pRC->nCommand = FC_WAIT;
		pRC->nNumDataBytes = 3;
		pRC->pDataBytes = new unsigned char[pRC->nNumDataBytes];
		pRC->pDataBytes[0] = (unsigned char)nTimeHrs;
		pRC->pDataBytes[1] = (unsigned char)nTimeMin;
		pRC->pDataBytes[2] = (unsigned char)nTimeSec;
		m_commandList.push_back(pRC);
	}
}

//PrintOutCommandList: test function for printing out list of commands
void FileCommands::PrintOutCommandList() {//test function for printing out list of commands
	if (this->m_bTimeChecking) {
		printf("Morning Time: %.2f hrs\n",m_dMorningTimeHrs);
		printf("Rest Time: %.2f hrs\n",m_dRestTimeHrs);
		printf("Sleep Time: %.2f hrs\n",m_dSleepTimeHrs);
	}
	else {
		printf("No time checking in script file.\n");
	}
	int nNumSafePoints = m_safeLat.size();
	if (nNumSafePoints>0) {
		for (int i=0;i<nNumSafePoints;i++) {
			printf("Safe point: %.6f, %.6f\n",m_safeLat[i],m_safeLong[i]);
		}
	}
	else {
		printf("No safe points defined.\n");
	}
	int nNumCommands = m_commandList.size();
	for (int i=0;i<nNumCommands;i++) {
		if (m_commandList[i]) {
			if (m_commandList[i]->nCommand==FC_LABEL) {
				printf("label: %s\n",(char *)m_commandList[i]->pDataBytes);
			}
			else if (m_commandList[i]->nCommand==FC_GOTO) {
				printf("goto: %s\n",(char *)m_commandList[i]->pDataBytes);
			}
			else if (m_commandList[i]->nCommand==FC_HEADING) {
				float fHeading=0;
				memcpy(&fHeading,m_commandList[i]->pDataBytes,sizeof(float));
				printf("heading: %.2f deg\n",fHeading);
			}
			else if (m_commandList[i]->nCommand==FC_PHOTO) {
				printf("photo\n");
			}
			else if (m_commandList[i]->nCommand==FC_FORWARD) {
				int nTotalTimeSec=0;
				memcpy(&nTotalTimeSec,m_commandList[i]->pDataBytes,sizeof(int));
				printf("forwards: %d seconds\n",nTotalTimeSec);
			}
			else if (m_commandList[i]->nCommand==FC_GPS_WAYPOINT) {
				double dLatitude=0.0, dLongitude=0.0;
				memcpy(&dLatitude,m_commandList[i]->pDataBytes,sizeof(double));
				memcpy(&dLongitude,&m_commandList[i]->pDataBytes[8],sizeof(double));
				printf("drive to location: %.6f, %.6f\n",dLatitude,dLongitude);
			}
			else if (m_commandList[i]->nCommand==FC_HOLD) {
				int nTotalTimeSec=0;
				memcpy(&nTotalTimeSec,m_commandList[i]->pDataBytes,sizeof(int));
				printf("hold current location: %d seconds\n",nTotalTimeSec);
			}
			else if (m_commandList[i]->nCommand==FC_GRID_SAMPLE) {//collect sensor samples in a grid pattern
				double dLatitudeCorner1=0.0, dLongitudeCorner1=0.0;
				double dLatitudeCorner2=0.0, dLongitudeCorner2=0.0;
				int nNumNSPts=0, nNumEWPts=0, nNumSamplesPerLocation=0;
				memcpy(&dLatitudeCorner1,m_commandList[i]->pDataBytes,sizeof(double));
				memcpy(&dLongitudeCorner1,&m_commandList[i]->pDataBytes[8],sizeof(double));
				memcpy(&dLatitudeCorner2,&m_commandList[i]->pDataBytes[16],sizeof(double));
				memcpy(&dLongitudeCorner2,&m_commandList[i]->pDataBytes[24],sizeof(double));
				memcpy(&nNumNSPts,&m_commandList[i]->pDataBytes[32],sizeof(int));
				memcpy(&nNumEWPts,&m_commandList[i]->pDataBytes[36],sizeof(int));
				memcpy(&nNumSamplesPerLocation,&m_commandList[i]->pDataBytes[40],sizeof(int));
				printf("collect %d x %d x %d samples between %.6f, %.6f, and %.6f, %.6f\n",
					nNumNSPts,nNumEWPts,nNumSamplesPerLocation,dLatitudeCorner1,dLongitudeCorner1,dLatitudeCorner2,dLongitudeCorner2);
			}
			else if (m_commandList[i]->nCommand==FC_WAIT) {//wait for the specified interval HH:MM:SS
				printf("Wait for specified interval (HH:MM:SS) = %02d:%02d:%02d\n",
					(int)m_commandList[i]->pDataBytes[0],(int)m_commandList[i]->pDataBytes[1],(int)m_commandList[i]->pDataBytes[2]);
			}
			else if (m_commandList[i]->nCommand==FC_SAMPLE) {//save samples to file
				printf("Collect and save sample data to: %s\n",(char *)m_commandList[i]->pDataBytes);
			}
		}
	}
}

std::string FileCommands::trimEndOfText(std::string sText) {//get rid of carriage returns, line feeds, spaces, or tabs from end of sText
	int nTextLength = sText.length();
	int i=nTextLength-1;
	while (i>=0) {
		if (sText[i]==' '||sText[i]=='\t'||sText[i]=='\r'||sText[i]=='\n') {
			i--;
		}
		else break;
	}
	int nNewTextLength = i+1;
	if (nNewTextLength>0) {
		sText = sText.substr(0,nNewTextLength);
	}
	else sText = nullptr;
	return sText;	
}


/**
 * @brief execute the next command in the list of fidle commands
 * 
 * @param command_mutex mutex to control access to propellers
 * @param lastNetworkCommandTimeMS pointer to the last time (in ms since program started) that a network command was issued
 * @param pShipLog void pointer to a ShipLog object that can be used to log error messages or other data to a log file.
 * @return 0 if no action is required by the calling function, otherwise return a non-zero code (see CommandList.h)
 */
int FileCommands::DoNextCommand(pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog) {
	if (m_bTraveledToSunnySpot) {
		return 0;//going to shut down soon, so don't start any new commands
	}
	if (m_bTravelToSunnySpot) {
		//need to first travel to a sunny spot to get recharged before doing anything else
		double dSunnyLatitude=0.0;//latitude of point where it is hoped there will be some sunshine for recharging
		double dSunnyLongitude=0.0;//longitude of point where it is hoped there will be some sunshine for recharging
		double dCurrentLatitude=m_pNavigator->GetLatitude();//current latitude of boat
		double dCurrentLongitude=m_pNavigator->GetLongitude();//current longitude of boat
		if (dCurrentLatitude==0.0&&dCurrentLongitude==0.0) {//GPS data is not ready yet, so just pause for a bit and then return for now
			usleep(1000000);
			return 0;
		}
		if (GetClosestSafePoint(dCurrentLatitude,dCurrentLongitude,dSunnyLatitude,dSunnyLongitude)) {
			ShipLog *pLog = (ShipLog *)pShipLog;
			char sMsg[256];
			sprintf(sMsg,"Voltage was only: %.2f V. Driving to %.6f, %.6f to look for more sun.",m_dLowVoltage,dSunnyLatitude,dSunnyLongitude);
			pLog->LogEntry(sMsg,true);
			m_pNavigator->SetDriveTimeoutSeconds(300);//timeout on DriveToLocation function after 5 minutes in order to avoid straining battery too much
			m_pNavigator->m_bExitNavFunction = false;
			m_pNavigator->DriveToLocation(dSunnyLatitude,dSunnyLongitude,(void *)m_pThrusters,command_mutex,lastNetworkCommandTimeMS,pShipLog,1,m_bCancel,LOW_PRIORITY);
			m_pNavigator->SetDriveTimeoutSeconds(0);//disable timeouts for DriveToLocation function
		}
		if (m_pBatteryCharge!=nullptr) {
			m_pBatteryCharge->SetInsufficientStartupCharge();//indicate that boat should enter a sleep state for faster recharging. Hopefully it won't drift too much from the sunny location during that time.
		}
		m_bTraveledToSunnySpot = true;
		m_bTravelToSunnySpot = false;
		return 0;
	}
	int nNumCommands = m_commandList.size();
	CheckTime();//check to see if it is time to rest, go to sleep, or morning time yet
	if (nNumCommands<=0||m_nCurrentCommandIndex>=nNumCommands||!m_pNavigator->isDataReady()||m_bRestTimeMode) {//no valid commands or we have finished all of the available commands
		usleep(1000000);//just pause in this thread, since there are no commands to execute at the moment, or the navigator object is not ready yet (ex: no compass data), or AMOS is in rest mode
		return 0;
	}
	int nRetval = 0;
	
	if (m_bTravelToSunnySpot) {
		usleep(1000000);//just pause in this thread, since there are no commands to execute at the moment, or the navigator object is not ready yet (ex: no compass data), or AMOS is in rest mode
		return 0;
	}
	
	if (m_commandList[m_nCurrentCommandIndex]) {
		SaveCurrentCommand();//save the current command index to the preferences file (prefs.txt)
		nRetval = DoCommand(m_commandList[m_nCurrentCommandIndex], command_mutex, lastNetworkCommandTimeMS, pShipLog);
	}
	m_nCurrentCommandIndex++;
	return nRetval;
}

/**
 * @brief executes a particular file command
 * 
 * @param pCommand pointer to a REMOTE_COMMAND structure that describes the command to perform
 * @param command_mutex pointer to a pthread_mutex_t object that is used to make sure that 2 or more threads do not try to get AMOS to execute 2 commands at the same time
 * @param lastNetworkCommandTimeMS the time in ms (since this program was started) when a remote command was last sent to AMOS.
 * @param pShipLog void pointer to a ShipLog object that can be used to log error messages or other data to a log file.
 * @return int 0 if no action is required by the calling function, otherwise return a non-zero code (see CommandList.h)
 */
int FileCommands::DoCommand(REMOTE_COMMAND *pCommand, pthread_mutex_t *command_mutex, 
							 unsigned int *lastNetworkCommandTimeMS, void *pShipLog) {
	if (pCommand->nCommand==FC_LABEL) {//just a label, no need to do anything
		return 0;
	}
	else if (pCommand->nCommand==FC_GOTO) {//need to jump command execution to a particular label
		int nMatchingLabelIndex = getMatchingLabelIndex((char *)pCommand->pDataBytes);
		if (nMatchingLabelIndex>=0) {
			m_nCurrentCommandIndex=nMatchingLabelIndex;
		}
		return 0;
	}
	else if (pCommand->nCommand==FC_HEADING) {
		float fHeading=0;
		memcpy(&fHeading,pCommand->pDataBytes,sizeof(float));
		m_fLastHeadingCommandAngle = fHeading;
		m_pNavigator->TurnToCompassHeading(fHeading,(void *)m_pThrusters,command_mutex,lastNetworkCommandTimeMS,pShipLog,m_bCancel,LOW_PRIORITY);
	}
	else if (pCommand->nCommand==FC_PHOTO) {
		TakePhoto(pShipLog);
	}
	else if (pCommand->nCommand==FC_FORWARD) {
		int nTotalTimeSeconds=0;
		memcpy(&nTotalTimeSeconds,pCommand->pDataBytes,sizeof(int));
		float fCurrentHeading = (float)m_pNavigator->m_imuData.heading;
		if (fCurrentHeading==0) {//may not have valid heading yet
			m_pNavigator->CollectCompassData(pShipLog);
			fCurrentHeading = (float)m_pNavigator->m_imuData.heading;
		}
		m_pNavigator->DriveForwardForTime(nTotalTimeSeconds,MAX_THRUSTER_SPEED,fCurrentHeading,(void *)m_pThrusters,command_mutex,lastNetworkCommandTimeMS,pShipLog,m_bCancel,true,LOW_PRIORITY);
	}
	else if (pCommand->nCommand==FC_GPS_WAYPOINT) {
		double dLatitude=0.0, dLongitude=0.0;
		memcpy(&dLatitude,pCommand->pDataBytes,sizeof(double));
		memcpy(&dLongitude,&pCommand->pDataBytes[8],sizeof(double));
		m_pNavigator->DriveToLocation(dLatitude,dLongitude,(void *)m_pThrusters,command_mutex,lastNetworkCommandTimeMS,pShipLog,10,m_bCancel,LOW_PRIORITY);
	}
	else if (pCommand->nCommand==FC_HOLD) {
		int nTotalTimeSeconds=0;
		memcpy(&nTotalTimeSeconds,pCommand->pDataBytes,sizeof(int));
		m_pNavigator->HoldCurrentPosition(nTotalTimeSeconds,(void *)m_pThrusters,command_mutex,lastNetworkCommandTimeMS,pShipLog,m_bCancel,LOW_PRIORITY);
	}
	else if (pCommand->nCommand==FC_GRID_SAMPLE) {//collect sensor samples in a grid pattern
		if (!m_pSensorDataFile) return 0;
		double dLatitudeCorner1=0.0, dLongitudeCorner1=0.0;//1st corner of rectangle to sample
		double dLatitudeCorner2=0.0, dLongitudeCorner2=0.0;//opposite corner of rectangle to sample
		int nNumNSPts=0;//number of sensor samples to collect along north-south direction
		int nNumEWPts=0;//number of sensor samples to collect along east-west direction
		int nNumSamplesPerLocation=0;//the number of sensor samples to collect at each grid point location
		memcpy(&dLatitudeCorner1,pCommand->pDataBytes,sizeof(double));
		memcpy(&dLongitudeCorner1,&pCommand->pDataBytes[8],sizeof(double));
		memcpy(&dLatitudeCorner2,&pCommand->pDataBytes[16],sizeof(double));
		memcpy(&dLongitudeCorner2,&pCommand->pDataBytes[24],sizeof(double));
		memcpy(&nNumNSPts,&pCommand->pDataBytes[32],sizeof(int));
		memcpy(&nNumEWPts,&pCommand->pDataBytes[36],sizeof(int));
		memcpy(&nNumSamplesPerLocation,&pCommand->pDataBytes[40],sizeof(int));
		double dLatitudeIncrement = 0.0;
		if (nNumNSPts>1) {
			dLatitudeIncrement = (dLatitudeCorner2 - dLatitudeCorner1) / (nNumNSPts-1);
		}
		double dLongitudeIncrement = 0.0;
		if (nNumEWPts>1) {
			dLongitudeIncrement = (dLongitudeCorner2 - dLongitudeCorner1) / (nNumEWPts-1);
		}
		for (int i=0;i<nNumNSPts;i++) {
			double dLatitude = dLatitudeCorner1 + i*dLatitudeIncrement;
			for (int j=0;j<nNumEWPts;j++) {
				double dLongitude = dLongitudeCorner1 + j*dLongitudeIncrement;
				if ((i%2)>0) {
					//on odd numbered rows, move in the opposite EW direction, in order to collect the sample points as efficiently as possible.
					dLongitude = dLongitudeCorner2 - j*dLongitudeIncrement;
				}
				m_pNavigator->DriveToLocation(dLatitude,dLongitude,(void *)m_pThrusters,command_mutex,lastNetworkCommandTimeMS,pShipLog,10,m_bCancel,LOW_PRIORITY);
				//turn off thrusters
				m_pThrusters->Stop();
				//pause for SENSOR_GRID_PAUSETIME_SEC seconds with thrusters off
				unsigned int uiEndPauseTime = millis() + 1000*SENSOR_GRID_PAUSETIME_SEC;
				SensorDeploy s(this->m_szRootFolder,false);
				s.Deploy();
				usleep(10000000);//wait ten seconds for sensors to stabilize in water
				for (int k=0;k<nNumSamplesPerLocation;k++) {
					while (millis()<uiEndPauseTime) {
						usleep(1000000);//sleep for a second
					}
					//get actual current location
					double dCurrentLatitude = m_pNavigator->GetLatitude();
					double dCurrentLongitude = m_pNavigator->GetLongitude();
					m_pSensorDataFile->SaveDataAtLocation(dCurrentLatitude, dCurrentLongitude);
				}
				s.Retract();
			}
		}
	}
	else if (pCommand->nCommand==FC_SAMPLE) {
		SensorDeploy s(this->m_szRootFolder,false);
		m_pSensorDataFile->SetFilename((char *)pCommand->pDataBytes);
		s.Deploy();
		usleep(10000000);//wait ten seconds for sensors to stabilize in water
		m_pSensorDataFile->CollectAndSaveDataNow();
		s.Retract();
	}
	else if (pCommand->nCommand==FC_WAIT) {
		int nIntervalHrs = (int)pCommand->pDataBytes[0];//number of hours in wait interval
		int nIntervalMin = (int)pCommand->pDataBytes[1];//number of minutes in wait interval
		int nIntervalSec = (int)pCommand->pDataBytes[2];//number of seconds in wait interval
		int nTotalIntervalSec = 3600*nIntervalHrs + 60*nIntervalMin + nIntervalSec;//total interval in seconds
		time_t timeNow;
		time(&timeNow);
		time_t nextIntervalTime = Util::GetNextIntervalTime(nTotalIntervalSec);
		this->m_nWaitTimeSeconds = (int)(nextIntervalTime - timeNow);
		return FC_WAIT;
	}
	return 0;
}

int FileCommands::getMatchingLabelIndex(char *pLabelText) {//find the index of a label that matches pLabelText
	int nNumCommands = m_commandList.size();
	for (int i=0;i<nNumCommands;i++) {
		if (m_commandList[i]) {
			if (m_commandList[i]->nCommand==FC_LABEL) {
				if (strcmp((char *)m_commandList[i]->pDataBytes,pLabelText)==0) {
					return i;
				}
			}
		}
	}
	return -1;//could not find matching label
}

void FileCommands::ContinueFromPrevious() {//continue at the last known stage of the file command (i.e. from previous program instance, useful for example in picking up where you left off if AMOS goes into sleep mode for a while to charge up its battery).
	char prefsFilename[PATH_MAX];			
	sprintf(prefsFilename,(char *)"%s//prefs.txt",m_szRootFolder);
	filedata prefsFile(prefsFilename);
	int nLastStep = prefsFile.getInteger((char *)"[file_command_list]",(char *)"last_step");
	if (nLastStep>0) {
		int nNumCommands = m_commandList.size();
		if (nLastStep>=nNumCommands) return;
		m_nCurrentCommandIndex = nLastStep-1;
	}
}

void FileCommands::SaveCurrentCommand() {//save the current command index to the preferences file (prefs.txt)
	char prefsFilename[PATH_MAX];			
	sprintf(prefsFilename,(char *)"%s//prefs.txt",m_szRootFolder);
	filedata prefsFile(prefsFilename);
	prefsFile.writeData((char *)"[file_command_list]",(char *)"last_step",m_nCurrentCommandIndex+1);
}

/**
 * @brief increment the file command index and save index to prefs.txt file
 * 
 */
void FileCommands::IncrementAndSaveCommandIndex() {
	m_nCurrentCommandIndex++;
	SaveCurrentCommand();
}

void FileCommands::CheckTime() {//check to see if it is time for a rest, time to go to sleep, or morning-time.
	if (!m_bTimeChecking) {//not bothering with time checking
		this->m_bRestTimeMode = false;
		return;
	}
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
	double dHrsToday = timeinfo->tm_hour + timeinfo->tm_min/60.0 + timeinfo->tm_sec/3600.0;//number of hours elapsed so far today
	if (this->m_dRestTimeHrs>0.0) {
		if (dHrsToday>=m_dRestTimeHrs&&!m_bRestTimeMode) {//it is now time to rest
			m_bRestTimeMode = true;
			//test
			printf("Entering rest mode.\n");
			//end test
			if (this->m_pThrusters!=nullptr) {
				this->m_pThrusters->Stop();
			}
			if (m_pLiDAR!=nullptr) {
				m_pLiDAR->TurnOn(false);//turn off LiDAR to save a bit of power
			}
		}
	}
	if (dHrsToday>m_dSleepTimeHrs&&m_dSleepTimeHrs>m_dMorningTimeHrs) {//sleep time before midnight
		m_bSleepTime = true;
	}
	else if (m_dSleepTimeHrs<m_dMorningTimeHrs) {//sleep time must be after midnight
		if (dHrsToday>m_dSleepTimeHrs&&dHrsToday<m_dMorningTimeHrs) {
			m_bSleepTime = true;
		}
	}
	if (m_bRestTimeMode) {
		if (dHrsToday<m_dRestTimeHrs) {
			if (dHrsToday>=m_dMorningTimeHrs) {
				//it is now morning time
				m_bRestTimeMode = false;
				m_bSleepTime = false;
				if (m_pLiDAR!=nullptr) {
					//make sure that LiDAR is turned on
					m_pLiDAR->TurnOn(true);
				}
			}
		}
	}
}

/**
 * @brief get length of time in hours that sleep will last
 * 
 * @return double the length of time in hours that the sleep will last
 */
double FileCommands::GetSleepTimeHrs() {
	double dSleepTimeHrs = 0.0;
	if (m_dSleepTimeHrs>=this->m_dMorningTimeHrs) {
		dSleepTimeHrs = 24 - (m_dSleepTimeHrs - m_dMorningTimeHrs);
	}
	else {
		dSleepTimeHrs = m_dMorningTimeHrs - m_dSleepTimeHrs;
	}
	return dSleepTimeHrs;
}


/**
 * @brief call this function shortly after program startup to insert a command to travel to a sunny location so that proper solar charging can occur	
 * 
 * @param dLowVoltage the voltage of the battery that is deemed too low to travel much further. 
 * @param pBatteryCharge pointer to a BatteryCharge object that will be updated after the command to travel to a sunny spot has been completed. After the BatteryCharge object has been updated, the software will enter a sleep state for a period of time so that it can charge more quickly.
 */
void FileCommands::TravelToSunnySpot(double dLowVoltage, BatteryCharge *pBatteryCharge) {
	m_bTravelToSunnySpot = true;
	m_dLowVoltage = dLowVoltage;
	m_pBatteryCharge = pBatteryCharge;
	//exit out of current navigation function
	if (m_pNavigator!=nullptr) {
		m_pNavigator->m_bExitNavFunction = true;
	}
}

bool FileCommands::GetClosestSafePoint(double dCurrentLatitude, double dCurrentLongitude, double &dSunnyLatitude, double &dSunnyLongitude) {
	//find the closest safe point (if any) that was defined in the file command script
	//optional safe points are defined with the "safepoint:" field, followed by a comma-separated latitude and longitude in degrees.
	int nNumSafePoints = m_safeLat.size();
	if (nNumSafePoints<=0) {
		return false;//no safe points defined in the file command script
	}
	int nClosestIndex = 0;//index of closest safe point
	double dClosestDist = Navigation::ComputeDistBetweenPts(dCurrentLatitude,dCurrentLongitude,m_safeLat[0],m_safeLong[0]);
	for (int i=1;i<nNumSafePoints;i++) {
		double dDist = Navigation::ComputeDistBetweenPts(dCurrentLatitude,dCurrentLongitude,m_safeLat[i],m_safeLong[i]);
		if (dDist<dClosestDist) {
			dClosestDist = dDist;
			nClosestIndex = i;
		}
	}
	dSunnyLatitude = m_safeLat[nClosestIndex];
	dSunnyLongitude = m_safeLong[nClosestIndex];
}

/**
 * @brief check to see if a FileCommands object has one or more GPS waypoints associated with it (i.e. one or more "hold" or "waypoint" entries)
 * 
 * @param pFileCommands pointer to a FileCommands object
 * @return true if pFileCommands has one or more GPS waypoints associated with it (i.e. with the "waypoint" or "hold" command)
 * @return false if no GPS waypoints are associated with pFileCommands, or if pFileCommands is null
 */
bool FileCommands::HasGPSWaypoints(FileCommands *pFileCommands) {
	if (pFileCommands==nullptr) {
		return false;
	}
	int nNumCommands = pFileCommands->GetNumCommands();
	for (int i=0;i<nNumCommands;i++) {
		REMOTE_COMMAND *pRC = pFileCommands->GetCommand(i);
		if (pRC!=nullptr) {
			if (pRC->nCommand==FC_GPS_WAYPOINT||pRC->nCommand==FC_HOLD) {
				return true;
			}
		}
	}
	return false;
}

/**
 * @brief get the number of commands associated with this FileCommands object
 * 
 * @return int the number of commands associated with this FileCommands object
 */
int FileCommands::GetNumCommands() {
	return m_commandList.size();
}

/**
 * @brief get the command at index nCommandIndex	
 * 
 * @param nCommandIndex the index of the command to get
 * @return REMOTE_COMMAND* object corresponding to the index specified, or null if no command exists for the specified index
 */
REMOTE_COMMAND * FileCommands::GetCommand(int nCommandIndex) {
	if (nCommandIndex<0||nCommandIndex>=m_commandList.size()) {
		return nullptr;
	}
	return m_commandList[nCommandIndex];
}

/**
 * @brief necessary to do this in case AMOS drifted a long distance while sleeping. Should be called after ContinueFromPrevious function and after a GPS fix has been achieved, with valid latitude and longitude
 * 
 */
void FileCommands::SetStepToClosestGPSWaypoint() {
	const double DIST_TOLERANCE_M = 1.0;//consider two distances equivalent if they are within this distance (in m) to one another
	if (m_pNavigator==nullptr) return;
	if (FileCommands::HasGPSWaypoints(this)) {
		//set current index to correspond to the closest gps location to our current position (necessary to do this in case AMOS has drifted a lot while sleeping)
		double dCurrentLatitude = m_pNavigator->GetLatitude();
		double dCurrentLongitude = m_pNavigator->GetLongitude();
		int nClosestStepIndex = m_nCurrentCommandIndex;//the index of the step whose GPS coordinates are closest to our current position
		double dMinDist = 999999999;//initialize to impossibly large value
		int nNumSteps = this->GetNumCommands();
		if (dCurrentLatitude!=0||dCurrentLongitude!=0) {
			for (int i = 0;i<nNumSteps;i++) {
				int j = (m_nCurrentCommandIndex + i)%nNumSteps;
				if (m_commandList[j]->nCommand==FC_GPS_WAYPOINT||m_commandList[j]->nCommand==FC_HOLD) {
					double dLatitude=0.0, dLongitude=0.0;
					memcpy(&dLatitude,m_commandList[j]->pDataBytes,sizeof(double));
					memcpy(&dLongitude,&m_commandList[j]->pDataBytes[8],sizeof(double));
					
					double dDist = Navigation::ComputeDistBetweenPts(dLatitude,dLongitude,dCurrentLatitude,dCurrentLongitude);
					if (dDist<(dMinDist-DIST_TOLERANCE_M)) {
						nClosestStepIndex = j;
						dMinDist = dDist;
					}
					else if (dDist<(dMinDist+DIST_TOLERANCE_M)) {
						//an equivalent distance was found, choose the one whose index is closest to m_nCurrentCommandIndex
						int nDif1 = abs(nClosestStepIndex-m_nCurrentCommandIndex); 
						int nDif2 = abs(j-m_nCurrentCommandIndex);
						if (nDif2<nDif1) {
							nClosestStepIndex = j;
							dMinDist = dDist;
						}
					}
				}
			}
			m_nCurrentCommandIndex = nClosestStepIndex;
		}
	}
}

/**
 * @brief adds a command to perform some action (no additional parameters required)
 * 
 * @param nCommandType the type of command being added (see CommandList.h for a list of commands)
 */
void FileCommands::AddCommand(int nCommandType) {
	REMOTE_COMMAND *pRC = new REMOTE_COMMAND;
	pRC->nCommand = nCommandType;
	pRC->nNumDataBytes = 0;
	pRC->pDataBytes = nullptr;
	m_commandList.push_back(pRC);
}

/**
 * @brief take a high-resolution photograph and save to file. The folder path and filename prefix used for the image file is set in the "[camera]" section of the prefs.txt configuration file.
 * 
 * @param pShipLog void pointer to a ShipLog object that can be used to log error messages or other data to a log file.
 */
void FileCommands::TakePhoto(void *pShipLog) {
	char *DEFAULT_IMAGE_FOLDER = "/home/pi/Computer_Programs/RemoteControlTest/Images";
	char *DEFAULT_IMAGE_PREFIX = "image";

	const unsigned int MAX_FILE_NUMBER = 999999;
	char prefsFilename[PATH_MAX];			
	char imageFilename[PATH_MAX];
	sprintf(prefsFilename,(char *)"%s//prefs.txt",m_szRootFolder);
	filedata prefsFile(prefsFilename);
	char *imageFolder = NULL;
	char *imageFilePrefix = NULL;
	//get image folder
	prefsFile.getString("[camera]","storage_folder",&imageFolder);
	if (imageFolder==NULL) {
		//use default image folder
		imageFolder = new char[strlen(DEFAULT_IMAGE_FOLDER)+1];
		strcpy(imageFolder,DEFAULT_IMAGE_FOLDER);
	}
	//get image filename prefix
	prefsFile.getString("[camera]","filename_prefix",&imageFilePrefix);
	if (imageFilePrefix==NULL) {
		//use default image file prefix
		imageFilePrefix = new char[strlen(DEFAULT_IMAGE_PREFIX)+1];
		strcpy(imageFilePrefix,DEFAULT_IMAGE_PREFIX);
	}

	do {
		m_uiPictureNumber++;
		//check to see if folder name already has '/' character at the end of it 
		if (imageFolder[strlen(imageFolder)-1]=='/') {
			sprintf(imageFilename,"%s%s%05u.jpg",imageFolder,imageFilePrefix,m_uiPictureNumber);
		}
		else {//insert '/' character between folder and filename
			sprintf(imageFilename,"%s/%s%05u.jpg",imageFolder,imageFilePrefix,m_uiPictureNumber);
		}
		if (FileCommands::doesFileExist(imageFilename)) {//file exists, increase # and try again
			continue;
		}
		else {
			break;//found a filename that doesn't already exist
		}
	} while (m_uiPictureNumber<MAX_FILE_NUMBER);

	char *commandStr = new char[strlen(imageFilename)+128];
	sprintf(commandStr,"raspistill -o %s",imageFilename);
	system(commandStr);
	char sMsg[256];
	sprintf(sMsg,"Took photo: %s%05u.jpg",imageFilePrefix,m_uiPictureNumber);
	((ShipLog *)pShipLog)->LogEntry(sMsg,true);
	delete []commandStr;
	delete []imageFilePrefix;
	delete []imageFolder;

}

void FileCommands::SetRoutePlan() {
	//informs the Navigation object of the route plan to assist it with achieving the desired waypoints
	if (m_pNavigator==nullptr) {
		return;
	}
	m_pNavigator->ClearRoutePlan();
	int nNumCommands = m_commandList.size();
	for (int i=0;i<nNumCommands;i++) {
		if (m_commandList[i]) {
			if (m_commandList[i]->nCommand==FC_GPS_WAYPOINT) {
				double dLatitude=0.0, dLongitude=0.0;
				memcpy(&dLatitude,m_commandList[i]->pDataBytes,sizeof(double));
				memcpy(&dLongitude,&m_commandList[i]->pDataBytes[8],sizeof(double));
				m_pNavigator->AddWaypointToPlan(dLatitude,dLongitude);
			}
		}
	}
}

