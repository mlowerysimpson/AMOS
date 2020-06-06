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

//definitions for the various possible return values from the CheckPacket function
#define PACKET_OK 0 //the packet is fine without any data problems
#define REPEATED_CHUNK 1 //the packet is fine, but corresponds to an old (repeated) chunk of data that has already been received
#define BAD_CRC 2 //one or both of the CRC bytes are incorrect
#define NOT_ENOUGH_BYTES 3 //what has been received of the packet so far is ok, but there are not enough bytes present
#define NO_SYNC_BYTES 4 //no synchronization bytes could be found in the packet data
#define BAD_CHUNK_INDEX 5 //bytes for chunk index corresponded to a chunk that was too high
#define BAD_DATASIZE 6 //bytes for data size are wrong


class RemoteCommand {
public:
	RemoteCommand(char *rootFolder, int nSocket, Navigation *pNav, Thruster *pThrusters, AToD *pAToD, SensorDataFile *pSensorDataFile, Vision *pVision, pthread_mutex_t *command_mutex, void *pShipLog);//constructor
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
	char* m_szRootFolder;//the folder where the main program resides and runs from 
	bool m_bSerportMode;//flag is true when this object is getting commands over a serial port connection (otherwise false if a network (default) connection is being used)
	pthread_t m_moveForwardThreadId;
	pthread_t m_moveToLocationThreadId;
	Vision *m_vision;//object used for handling image capture requests
	SensorDataFile *m_pSensorDataFile;//object used for saving sensor data
	AToD *m_pAToD;//object used for handling analog to digital conversions and data
	int m_nSocket;//network socket identifier or serial port file descriptor if in serial port mode (i.e. m_bSerportMode==true)
	
	//functions
	void CopyPacketToBuf(unsigned char* inBuf, int nBufferStartIndex, int nBufSize, char* destBuf);//copy the data portion of inBuf to destBuf
	int CheckPacket(unsigned char* inBuf, int nBufSize, int nChunkIndex, int& nMoreBytesRequired, int& nPacketStartIndex);//check to see if a chunk of data is valid or not
	int ReceiveLargeDataChunk(int nSocket, char* rxBytes, int nNumToReceive);//tries to receive a large chunk of data over a serial port connection
	bool WaitForSerialBytes(int nNumSerialBytesExpected, unsigned int uiTimeoutMS, int &nNumAvailable);//waits up to uiTimeoutMS milliseconds for nNumSerialBytesExpected to be available in the serial buffer
	void ExitMoveThreads();
	void MoveToLocationThread();//start thread that moves the boat to a particular GPS destination
	void MoveForwardInFixedDirection();//start thread that moves the boat forward in its current heading
	REMOTE_COMMAND * GetNetworkCommand();//get the next available command over a TCP/IP network from a remote client host
	REMOTE_COMMAND * GetSerialCommand();//get the next available command over a (wireless) serial port connection from a remote device
	unsigned int GetUIntFromBytes(unsigned char *dataSizeBytes);//returns an unsigned integer from 4 bytes, assumes that bytes are ordered from MSB to LSB
	bool requiresMoreData(int nCommand);//check to see if a particular command is sent with additional data bytes (in addition to the command bytes), if so return true
	bool isValidCommand(int nCommand);//checks to see if nCommand corresponds to a valid / known command or not. Returns true if it does, false if not
	void SendConfirmationBytes(int nSocket, unsigned char byte1, unsigned char byte2);//send a couple of confirmation bytes to the sender
	void SendErrorBytes(int nSocket);//send serial port bytes that correspond to a communications error
	void SendCancelBytes(int nSocket);//send four 0x0a bytes out serial port to indicate that downloading is stopping
};
