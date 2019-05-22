//RemoteCommand.h
#include <stdio.h>      
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include "Thruster.h"
#include "Navigation.h"
#include "AToD.h"
#include "SensorDataFile.h"
#include "Vision.h"

//list of possible error codes 
#define ERROR_NOT_ENOUGH_DATA -1 //not enough data was provided by the remote host
#define ERROR_NO_COMMAND_RECEIVED -2//return value of GetCommand function if a command was never received
#define ERROR_NO_DATASIZE_BYTES -3//unable to receive the data size bytes
#define ERROR_NO_SOCKET -4//no network socket available (i.e. m_nSocket<=0)
#define ERROR_SENDING_DATA -5//an error occurred trying to send data over the network
#define ERROR_NOSERPORT_HANDLE -6//no serial port handle available (i.e. m_nSocket<=0)
#define ERROR_SERIAL_ERROR -7//an error occurred trying to get serial port data
#define ERROR_INVALID_COMMAND -8//an unknown or invalid remote command was sent


class RemoteCommand {
public:
	RemoteCommand(int nSocket, Navigation *pNav, Thruster *pThrusters, AToD *pAToD, SensorDataFile *pSensorDataFile, Vision *pVision, pthread_mutex_t *command_mutex, void *pShipLog);//constructor
	~RemoteCommand();
	
	void *m_shipLog;//void pointer to a ship log object
	Navigation *m_pNav;//object used for navigation
	unsigned int m_lastNavigationmmandTimeMS;//variable that holds the time in ms when a network command was last sent
	Thruster *m_thrusters; 
	pthread_mutex_t *m_command_mutex;//mutex that various command functions use to make sure that no two of them drive the thrusters at the same time
	double m_latitudeDest;//destination latitude, sent from the remote captain
	double m_longitudeDest;//destination longitude, sent from the remote captain
	bool m_bStopDriving;//flag is false if a thread for driving the boat should be active, set to true in order to halt a thread for driving the boat
	bool m_bDrivingThreadRunning;//flag is true if a thread for driving the boat is active
	int m_nLastError;//public variable that holds the last available error code, will be zero if no error has occurred
	float m_fMoveForwardDirection;//heading direction in degrees (0 to 360) that we are trying to maintain while moving forward
	float m_fMoveForwardSpeed;//the speed to try to maintain while moving forward in a fixed direction
	float m_fTurnRate;//the turn rate in degrees per second
	REMOTE_COMMAND * GetCommand();//reads in command from remote host, remote host is responsible for delting returned REMOTE_COMMAND object (if not null)
	static char * GetCommandDescription(REMOTE_COMMAND *pCommand);//return a text description of a given command
	static void DeleteCommand(REMOTE_COMMAND *pCommand);//deletes a REMOTE_COMMAND structure from memory
	bool ExecuteCommand(REMOTE_COMMAND *pCommand, bool bUseSerial);//execute command
	void SetSerportMode(bool bSerportMode);//set this remote command object to use a serial port functions for getting remote commands
	
	
private:
	//data
	bool m_bSerportMode;//flag is true when this object is getting commands over a serial port connection (otherwise false if a network (default) connection is being used)
	pthread_t m_moveForwardThreadId;
	pthread_t m_moveToLocationThreadId;
	Vision *m_vision;//object used for handling image capture requests
	SensorDataFile *m_pSensorDataFile;//object used for saving sensor data
	AToD *m_pAToD;//object used for handling analog to digital conversions and data
	int m_nSocket;//network socket identifier or serial port file descriptor if in serial port mode (i.e. m_bSerportMode==true)
	
	//functions
	bool WaitForSerialBytes(int nNumSerialBytesExpected, unsigned int uiTimeoutMS, int &nNumAvailable);//waits up to uiTimeoutMS milliseconds for nNumSerialBytesExpected to be available in the serial buffer
	void ExitMoveThreads();
	void MoveToLocationThread();//start thread that moves the boat to a particular GPS destination
	void MoveForwardInFixedDirection();//start thread that moves the boat forward in its current heading
	REMOTE_COMMAND * GetNetworkCommand();//get the next available command over a TCP/IP network from a remote client host
	REMOTE_COMMAND * GetSerialCommand();//get the next available command over a (wireless) serial port connection from a remote device
	unsigned int GetUIntFromBytes(unsigned char *dataSizeBytes);//returns an unsigned integer from 4 bytes, assumes that bytes are ordered from MSB to LSB
	bool requiresMoreData(int nCommand);//check to see if a particular command is sent with additional data bytes (in addition to the command bytes), if so return true
	bool isValidCommand(int nCommand);//checks to see if nCommand corresponds to a valid / known command or not. Returns true if it does, false if not
		
};
