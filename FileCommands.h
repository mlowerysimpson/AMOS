//FileCommands.h
#include <stdlib.h>
#include <vector>
#include <pthread.h>
#include "RemoteCommand.h"
#include "SensorDataFile.h"

class FileCommands {
public:
	FileCommands(char *rootFolder, char *szFilename, Navigation *pNavigator, Thruster *pThrusters, SensorDataFile *pSensorDataFile, bool *bCancel);//constructor takes path of commands filename as input
	~FileCommands();//destructor
	bool m_bFileOK;//true if the file could be successfully found, opened, and parsed
	int m_nWaitTimeSeconds;//time in seconds that execution should be paused (if a FC_WAIT file command is received)
	REMOTE_COMMAND *GetNextCommand();	
	void PrintOutCommandList();//test function for printing out list of commands
	int DoNextCommand(pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog);//execute the next command in the list of file commands
	static bool doesFileExist(char *szFilename);//return true if the file exists
	void ContinueFromPrevious();//continue at the last known stage of the file command (i.e. from previous program instance, useful for example in picking up where you left off if the program crashes for some mysterious reason)
	void IncrementAndSaveCommandIndex();//increment the file command index and save index to prefs.txt file

private:
	//functions
	int getMatchingLabelIndex(char *pLabelText);//find the index of a label that matches pLabelText
	int DoCommand(REMOTE_COMMAND *pCommand, pthread_mutex_t *command_mutex, unsigned int *lastNetworkCommandTimeMS, void *pShipLog);//execute a particular file command
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

	SensorDataFile *m_pSensorDataFile;//pointer to object used for storing sensor data to file
	char *m_szRootFolder;//the folder where the prefs.txt (preferences) file is located 
	bool *m_bCancel;//pointer to boolean variable that will be "true" when the program is ending. It should be checked to end any currently executing file commands.
	char *m_szFilename;//the name of the command file
	vector <REMOTE_COMMAND *>m_commandList;
	Navigation *m_pNavigator;//the navigation object used for getting GPS data, compass data, plotting courses, etc.
	Thruster *m_pThrusters;//the 2 props that are used to propel the boat around
	int m_nCurrentCommandIndex;//the index of the command that is currently being executed
	float m_fLastHeadingCommandAngle;//used for storing the most recently specified heading angle

};