//CommandList.h
#pragma once
#include <time.h>
#include "SensorDataFile.h"
//list of possible commands and data types
#define THRUST_ON 0//turn one or both propellers on at defined speed and direction
#define THRUST_OFF 1//turn both propellers off
#define DO2_DATA_PACKET 2//dissolved oxygen data packet
#define COMPASS_DATA_PACKET 3//compass / tilt data packet
#define GPS_DATA_PACKET 4//GPS data packet
#define CONDUCTIVITY_DATA_PACKET 5//conductivity data packet
#define BATTVOLTAGE_DATA_PACKET 11//battery voltage data packet
#define SUPPORTED_SENSOR_DATA 12//find out what types of sensor data the boat is capable of collecting
#define SENSOR_TYPES_INFO 13//code that indicates that sensor type info will follow (see SensorDataFile.h for definitions of the sensor types)
#define WATER_TEMP_DATA_PACKET 14//water temperature data packet
#define WATER_PH_DATA_PACKET 15//water pH data packet
#define VIDEO_DATA_PACKET 17//screen capture from video camera data packet
#define GPS_DESTINATION_PACKET 18//GPS destination packet (used to tell the boat where to go)
#define WATER_TURBIDITY_DATA_PACKET 19//water turbidity data packet
#define LEAK_DATA_PACKET 20//leak info data packet
#define DIAGNOSTICS_DATA_PACKET 21//diagnostics info data packet
#define CANCEL_OPERATION 22//cancel the operation currently in progress
#define QUIT_PROGRAM 23//quit the currently running Pi program (RemoteControlTest) on AMOS
#define ENTER_SLEEP 24//put AMOS into sleep mode for an optional length of time (or indefinitely). AMOS wakes up when time elapses or when a command to wakeup is received over the serial wireless link.
#define SCRIPT_STATUS_PACKET 25//used to request or send the current status of the file script currently running on AMOS
#define SCRIPT_STEP_CHANGE 26//used to change the step of the currently running file script
#define LIST_REMOTE_SCRIPTS 27//command sent to AMOS to return a list of all of the available remote scripts
#define USE_REMOTE_SCRIPT 28//command sent to AMOS to use a particular remote script
#define FILE_TRANSFER 29//transfer a particular file to AMOS
#define FILE_RECEIVE 30//receive / download a particular file from AMOS
#define REFRESH_SETTINGS 31//command to refresh settings from a partiulcar part of the prefs.txt file
#define LIST_REMOTE_DATA 32//command sent to AMOS to return a list of all of the available data files
#define LIST_REMOTE_LOG 33//command sent to AMOS to return a list of all of the available log files
#define LIST_REMOTE_IMAGE 34//command sent to AMOS to return a list of all of the available image files
#define RTK_CORRECTION 35//sending RTK correction bytes to AMOS from an RTK base station
#define DELETE_FILES 36//delete one or more AMOS files
#define LAST_COMMAND 37//the highest possible command #

#define PASSCODE_TEXT "AMOS2018"

#define NUM_RESEND_SYNC_BYTES 10 //number of sync bytes to use before re-sending a large data chunk

#define REMOTE_SCRIPT_NAMELENGTH 64 //number of bytes allocated to the name of the text script file running on the boat

 //various settings types
#define AMOS_ALARM_SETTINGS 1
#define AMOS_LIDAR_SETTINGS 2
#define AMOS_SENSOR_SETTINGS 3
#define AMOS_CAMERA_SETTINGS 4
#define AMOS_BATTERY_SETTINGS 5



//file command types
#define FC_LABEL 5 //command is actually just a label in the command file 
#define FC_GOTO 6 //command to jump execution to a particular label (as previously defined with the label command)
#define FC_HEADING 7 //command to turn the boat to a particular heading (in degrees)
#define FC_FORWARD 8 //command to move forward 
#define FC_GPS_WAYPOINT 9//command to drive to a GPS location
#define FC_HOLD 10//command to hold the current GPS location for a specified period of time
#define FC_GRID_SAMPLE 11 //command to take a bunch of sensor samples in a grid pattern, defined by GPS coordinates and # of samples in each direction
#define FC_WAIT 12 //command to wait for a specified amount of time, if that time is >= MIN_SLEEP_TIME_SEC then put AMOS into a low power state
#define FC_SAMPLE 13 //command to collect sample data (from all sensors configured in prefs.txt file) and save data to specified filename
#define FC_PHOTO 14 //command to take a high-resolution photograph

//structure used for indicating how many sensors the boat has and what types of sensors they are
struct SENSOR_INFO {
	int nNumSensors;//the number of sensors available on the boat
	int *sensorTypes;//an array of size nNumSensors that describes the types of each sensor, for the definitions of the sensor types, see SensorDataFile.h
};


//structure used for sending / receiving remote commands
struct REMOTE_COMMAND {
	int nCommand;//command code sent from remote host
	int nNumDataBytes;//number of data bytes included with command
	unsigned char *pDataBytes;//remotely received data bytes, may be NULL if no data bytes were received
};

//structure used for specifying propeller speed
struct PROPELLER_STATE {
	float fRudderAngle;//the angle of the air rudder in degrees
	float fPropSpeed;//the rotational speed of the air propeller (arbitrary units)
};

//structure used for GPS data
struct GPS_DATA {
	double dLatitude;//latitude in degrees (-90 to +90)
	double dLongitude;//longitude in degrees (-180 to +180)
	time_t gps_time;//time of the GPS reading
};



struct BOAT_DATA {
	int nPacketType;//packet code describing what type of data this is
	int nDataSize;//number of dataBytes 
	unsigned char *dataBytes;
	unsigned char checkSum;//simple 8-bit checksum of everything in this structure, except this checkSum byte
};

class BoatCommand {
public:
	BoatCommand();//constructor
	~BoatCommand();//destructor
	static BOAT_DATA * CreateBoatData(int nDataType);//create an empty BOAT_DATA structure for a given data type. Calling function is responsible for deleting the created object
	static void DeleteBoatData(BOAT_DATA *boatData);//deletes the memory used by a BOAT_DATA structure
	static bool SendBoatData(int nSocket, bool bUseSerial, BOAT_DATA *boatData, void *pDiagnostics);//sends boat data out over socket or serial port connection
	static bool SendLargeSerialData(int nSocket, unsigned char *outputBuf, int nNumToSend, void *pDiagSensor);//send large amount of data out serial port, need to get confirmation after sending each chunk
	static unsigned char CalculateChecksum(BOAT_DATA *pData);//calculate simple 8-bit checksum for BOAT_DATA structure
	static int Encrypt(char *destBuf, char *sourceBuf, int nSize);
	static int Decrypt(char* destBuf, char* sourceBuf, int nSize);
	
private:
	static int fillchunk(unsigned char *chunkBuf,int nChunkID,unsigned char *outputBuf,int nBufSize,int nBufIndex,int nMaxChunkSize);

};

