//FileCommands.h
#include <stdlib.h>
#include <vector>
#include <pthread.h>
#include "RemoteCommand.h"
#include "SensorDataFile.h"
#include "BatteryCharge.h"
#include "LIDARLite.h"

class FileCommands {
public:
	FileCommands(char *rootFolder, char *szFilename, Navigation *pNavigator, Thruster *pThrusters, SensorDataFile *pSensorDataFile, LIDARLite *pLidar, bool *bCancel);//constructor takes path of commands filename as input
	~FileCommands();//destructor
	bool m_bFileOK;//true if the file could be successfully found, opened, and parsed
	bool m_bSleepTime;//true if it is time to put AMOS into sleep mode
	bool m_bTravelToSunnySpot;//flag may be set to true shortly after program startup, if it is necessary for the boat to first travel to the nearest sunny location for re-charging, prior to executing any of its required commands.
	int m_nWaitTimeSeconds;//time in seconds that execution should be paused (if a FC_WAIT file command is received)
	bool m_bTraveledToSunnySpot;//flag that is set to true after the boat has traveled to a sunny or safe spot (or after the command to drive to the sunny or safe spot has timed out).
	REMOTE_COMMAND *GetNextCommand();
	REMOTE_COMMAND *GetCommand(int nCommandIndex);//get the command at index nCommandIndex	
	double GetSleepTimeHrs();//get length of time in hours that sleep will last
	void PrintOutCommandList();//test function for printing out list of commands
	int DoNextCommand(pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog);//execute the next command in the list of file commands
	static bool doesFileExist(char *szFilename);//return true if the file exists
	static bool HasGPSWaypoints(FileCommands *pFileCommands);//return true if the pFileCommands object has one or more GPS waypoints associated with it (i.e. one or more "hold" or "waypoint" entries)
	void ContinueFromPrevious();//continue at the last known stage of the file command sequence (i.e. from previous program instance, useful for example in picking up where you left off if AMOS goes into sleep mode for a while to charge up its battery)
	void SetStepToClosestGPSWaypoint();//necessary to do this in case AMOS drifted a long distance while sleeping
	void IncrementAndSaveCommandIndex();//increment the file command index and save index to prefs.txt file
	void TravelToSunnySpot(double dLowVoltage, BatteryCharge *pBatteryCharge);//call this function shortly after program startup to insert a command to travel to a sunny location so that proper solar charging can occur	
	int GetNumCommands();//return the number of commands associated with this FileCommands object

private:
	//functions
	void SetRoutePlan();//informs the Navigation object of the route plan to assist it with achieving the desired waypoints
	void TakePhoto(void *pShipLog);//take a high-resolution photograph and save to file
	bool GetClosestSafePoint(double dCurrentLatitude, double dCurrentLongitude, double &dSunnyLatitude, double &dSunnyLongitude);//find the closest safe point (if any) that was defined in the file command script
	void CheckTime();//check to see if it is time for a rest, time to go to sleep, or morning time.
	int getMatchingLabelIndex(char *pLabelText);//find the index of a label that matches pLabelText
	int DoCommand(REMOTE_COMMAND *pCommand, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog);//execute a particular file command
	void AddCommand(int nCommandType);//adds a command to perform some action (no additional parameters required)
	void AddCommand(int nCommandType, std::string sText);//adds a text-based command to the list
	void AddCommand(int nCommandType, double dVal);//adds a command to the list that is associated with a single numeric value
	void AddCommand(int nCommandType, double dVal1, double dVal2);//adds a command to the list that is associated with two numeric values (ex: latitude and longitude)
	void AddCommand(int nCommandType, int nTimeHrs, int nTimeMin, int nTimeSec);//adds a command to the list that is associated with a time in hrs, min, sec
	void AddCommand(int nCommandType, double dVal1, double dVal2, double dVal3, double dVal4, int nVal5, int nVal6, int nVal7);//AddCommand: adds a command to the list that is associated with seven numeric values, 4 floating point and 3 integer
	int skipToTextVal(std::string sLine, int nStartIndex);//find the index of the first non-whitespace or delimiter text in sLine, starting at nStartIndex
	int skipToNumericVal(std::string sLine, int nStartIndex);//find the index of the first numeric digit in sLine, starting at nStartIndex
	std::string trimEndOfText(std::string sText);//get rid of carriage returns, line feeds, spaces, or tabs from end of sText
	bool matchingLabelFound(char *szLabel);//returns true if one of the file commands has been assigned the text label szLabel
	bool CheckConsistency();//check to make sure that file commands are consistent, ex: labels exist for each of the goto commands
	bool parseLine(std::string sLine);//parse command from line of text
	void cleanup();//delete commands from memory
	bool readInFile();//read in and parse the text commands from the file (return true if successful)
	void SaveCurrentCommand();//save the current command index to the preferences file (prefs.txt)

	unsigned int m_uiPictureNumber;//the number of the last picture taken using the "photo" file command
	vector <double>m_safeLat;//safe point latitudes (in degrees) where solar re-charging is optimal
	vector <double>m_safeLong;//safe point longitudes (in degrees) where solar re-charging is optimal
	LIDARLite *m_pLiDAR;//object used for making LiDAR measurements (can be turned off to save a bit of power)
	SensorDataFile *m_pSensorDataFile;//pointer to object used for storing sensor data to file
	char *m_szRootFolder;//the folder where the prefs.txt (preferences) file is located 
	bool *m_bCancel;//pointer to boolean variable that will be "true" when the program is ending. It should be checked to end any currently executing file commands.
	char *m_szFilename;//the name of the command file
	vector <REMOTE_COMMAND *>m_commandList;
	Navigation *m_pNavigator;//the navigation object used for getting GPS data, compass data, plotting courses, etc.
	Thruster *m_pThrusters;//the prop(s) that are used to propel the boat around
	int m_nCurrentCommandIndex;//the index of the command that is currently being executed
	float m_fLastHeadingCommandAngle;//used for storing the most recently specified heading angle
	bool m_bRestTimeMode;//true when AMOS is in "rest", mode, responsive, but not using its air propeller. It does not execute any file commands in this mode
	bool m_bTimeChecking;//true if we are checking to see when it is time for a rest, time to go to sleep, or morning time (default = false).
	double m_dMorningTimeHrs;//designation of when morning time starts (in hours)
	double m_dRestTimeHrs;//designation of when rest time starts (in hours)
	double m_dSleepTimeHrs;//designation of when AMOS should be put to sleep (in hours)
	double m_dLowVoltage;//the voltage of the boat that is deemed too low to travel much further
	BatteryCharge *m_pBatteryCharge;//pointer to the BatteryCharge object for the boat. May be necessary to use for indicating that the boat should enter a sleep state for faster re-charging prior to executing more commands.
};