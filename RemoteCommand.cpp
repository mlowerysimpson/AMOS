//RemoteCommand.cpp
#include "RemoteCommand.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <wiringPi.h>
#include <wiringSerial.h>

void *driveForwardThread(void *pParam) {//function drives boat forward at its current heading and speed
	RemoteCommand *pRC = (RemoteCommand *)pParam;
	pRC->m_bStopDriving = false;
	pRC->m_bDrivingThreadRunning = true;
    while (!pRC->m_bStopDriving) {//keep moving forward in direction pRC->m_fMoveForwardDirection and at speed pRC->m_fMoveForwardSpeed
		pRC->m_pNav->DriveForwardForTime(1,pRC->m_fMoveForwardSpeed,pRC->m_fMoveForwardDirection,
			(void *)pRC->m_thrusters,nullptr, nullptr, nullptr,&pRC->m_bStopDriving,false,LOW_PRIORITY); 
	}
	pRC->m_bDrivingThreadRunning=false;
    return 0;
}

void *driveToLocationThread(void *pParam) {//function drives boat to a particular GPS location
	RemoteCommand *pRC = (RemoteCommand *)pParam;
	pRC->m_bStopDriving = false;
	pRC->m_bDrivingThreadRunning = true;
	pRC->m_pNav->DriveToLocation(pRC->m_latitudeDest,pRC->m_longitudeDest,(void *)pRC->m_thrusters,pRC->m_command_mutex,&pRC->m_lastNavigationmmandTimeMS, pRC->m_shipLog, 10,&pRC->m_bStopDriving,HIGH_PRIORITY);//higher priority since command was explicity issued by user using app
	pRC->m_bDrivingThreadRunning = false;
	return 0;
}

//RemoteCommand constructor
//nSocket = network socket descriptor (socket descriptor from accepted network connection)
//pNav = pointer to navigation object used for getting GPS and compass data and plotting routes
//pThrusters = pointer to Thruster object that controls power to the propeller(s)
//pVision = pointer to a Vision object for controling access to the camera and performing obstacle avoidance routines
//command_mutex = pointer to mutex that is used to make sure that 2 commands do not try to access the thrusters at the same time
//pShipLog = void pointer to ShipLog object for logging various data and debugging info to file
RemoteCommand::RemoteCommand(int nSocket, Navigation *pNav, Thruster *pThrusters, AToD *pAToD, SensorDataFile *pSensorDataFile, Vision *pVision, pthread_mutex_t *command_mutex, void *pShipLog) {
	m_lastNavigationmmandTimeMS=0;
	m_bSerportMode = false;
	m_shipLog = pShipLog;
	m_command_mutex = command_mutex;
	m_bDrivingThreadRunning=false;
	m_bStopDriving=false;
	m_nSocket = nSocket;
	m_vision = pVision;
	m_nLastError = 0;
	m_thrusters=pThrusters;
	m_pNav = pNav;
	m_pAToD = pAToD;
	m_pSensorDataFile = pSensorDataFile;
	m_fMoveForwardDirection=0;
	m_fMoveForwardSpeed=0;
	m_fTurnRate=0;
	m_moveForwardThreadId=0;
	m_moveToLocationThreadId=0;
	m_latitudeDest=0.0;
	m_longitudeDest=0.0;
}

RemoteCommand::~RemoteCommand() {
	ExitMoveThreads();
}

//GetCommand: get the next available command from the remote host
//returns a REMOTE_COMMAND structure that corresponds to the remote command received
//the calling function is responsible for deleting any non-null structure that gets returned
REMOTE_COMMAND * RemoteCommand::GetCommand() {
	if (m_nSocket>0) {
		if (m_bSerportMode) {
			return GetSerialCommand();
		}
		else {
			return GetNetworkCommand();
		}
	}
	//no command received
	m_nLastError = ERROR_NO_COMMAND_RECEIVED;
	return nullptr;
}

//GetNetworkCommand: get the next available command over a TCP/IP network from a remote client host
//returns a REMOTE_COMMAND structure that corresponds to the command received 
//returns nullptr if an error occurs and sets m_nLastError to an error code
REMOTE_COMMAND * RemoteCommand::GetNetworkCommand() {
	unsigned char commandBuf[4];
	if (m_nSocket<=0) {
		m_nLastError = ERROR_NO_SOCKET;
		//test
		printf("ERROR_NO_SOCKET\n");
		//end test
		return nullptr;
	}
	int nNumRead = read(m_nSocket, commandBuf, 4);
	if (nNumRead<4) {//probably timed out
		m_nLastError = ERROR_NO_COMMAND_RECEIVED;
		return nullptr;
	}
	int nCommand = (int)GetUIntFromBytes(commandBuf);
	REMOTE_COMMAND *pCommandReceived = new REMOTE_COMMAND;
	memset(pCommandReceived,0,sizeof(REMOTE_COMMAND));
	pCommandReceived->nCommand = nCommand;
	if (requiresMoreData(nCommand)) {
		unsigned char dataSizeBytes[4];
		nNumRead = read(m_nSocket, dataSizeBytes, 4);
		if (nNumRead<4) {
			m_nLastError = ERROR_NO_DATASIZE_BYTES;
			//test
			printf("ERROR_NO_DATASIZE_BYTES\n");
			//end test
			delete pCommandReceived;
			return nullptr;
		}
		int nDataSize = (int)GetUIntFromBytes(dataSizeBytes);
		pCommandReceived->nNumDataBytes = nDataSize;
		if (nDataSize>0) {
			unsigned char *dataBytes = new unsigned char[nDataSize];
			nNumRead = read(m_nSocket, dataBytes, nDataSize);
			if (nNumRead<nDataSize) {
				m_nLastError = ERROR_NOT_ENOUGH_DATA;
				//test
				printf("ERROR_NOT_ENOUGH_DATA\n");
				//end test
				delete pCommandReceived;
				delete []dataBytes;
				return nullptr;
			}
			pCommandReceived->pDataBytes = dataBytes;
		}
	}
	//send confirmation back to remote client that command was successfully received
	//just echo back commandBuf bytes	
	if (send(m_nSocket, (char *)commandBuf, 4, 0)<0) {
		m_nLastError = ERROR_SENDING_DATA;
		//test
		printf("ERROR_SENDING_DATA\n");
		//end test
		if (pCommandReceived->pDataBytes!=nullptr) {
			delete []pCommandReceived->pDataBytes;
			pCommandReceived->pDataBytes=nullptr;
		}
		delete pCommandReceived;
		return nullptr;
	}
	return pCommandReceived;
}

//GetUIntFromBytes: gets an unsigned integer from a buffer of 4 bytes, assumes that dataSizeBytes contains 4 bytes that are ordered from MSB to LSB
//dataSizeBytes = pointer to 4 bytes of data that correspond to a 32-bit unsigned integer
unsigned int RemoteCommand::GetUIntFromBytes(unsigned char *dataSizeBytes) {
	unsigned int b1 = (unsigned int)dataSizeBytes[0];
	unsigned int b2 = (unsigned int)dataSizeBytes[1];
	unsigned int b3 = (unsigned int)dataSizeBytes[2];
	unsigned int b4 = (unsigned int)dataSizeBytes[3];
	unsigned int retVal = (b1<<24) + (b2<<16) + (b3<<8) + b4;
	return retVal;
	
}

//requiresMoreData: check to see if a particular command is sent with additional data bytes (in addition to the command bytes), if so return true
//nCommand = command code from remote host
bool RemoteCommand::requiresMoreData(int nCommand) {
	if (nCommand==THRUST_ON) return true;
	else if (nCommand==VIDEO_DATA_PACKET) return true;
	else if (nCommand==GPS_DESTINATION_PACKET) return true;
	return false;
}

char * RemoteCommand::GetCommandDescription(REMOTE_COMMAND *pCommand) {//return a text description of a given command
	//calling function is responsible for deleting the returned text
	if (pCommand==nullptr) {
		return nullptr;
	}
	char *szDescription = nullptr;
	if (pCommand->nCommand==THRUST_ON) {
		//command to turn one  or both thrusters on
		PROPELLER_STATE *pPropState = (PROPELLER_STATE *)pCommand->pDataBytes;
		szDescription = new char[256];
		memset(szDescription,0,256);
		sprintf(szDescription,"Set rudder angle to %.1f degrees, prop speed to %.1f\n",
			pPropState->fRudderAngle,pPropState->fPropSpeed);
	}
	else if (pCommand->nCommand==VIDEO_DATA_PACKET) {
		//command to capture and send back a video frame from the 1st camera
		szDescription = new char[256];
		strcpy(szDescription,"Capture video frame.\n");
	}
	else if (pCommand->nCommand==GPS_DESTINATION_PACKET) {
		//command to go to a particular GPS destination
		szDescription = new char[256];
		double dLatitude=0.0, dLongitude=0.0;
		memcpy(&dLatitude,pCommand->pDataBytes,sizeof(double));
		memcpy(&dLongitude,&pCommand->pDataBytes[8],sizeof(double));
		sprintf(szDescription,"GPS Destination Command: %.6f, %.6f\n",dLatitude, dLongitude);
	}
	else if (pCommand->nCommand==CANCEL_OPERATION) {
		szDescription = new char[256];
		sprintf(szDescription,"Cancel command sent.\n");
	}
	else if (pCommand->nCommand==QUIT_PROGRAM) {
		szDescription = new char[256];
		sprintf(szDescription,"Quit command sent.\n");
	}
	return szDescription;
}

/**
 * @brief execute a particular command from a remote host
 * 
 * @param pCommand a pointer to a REMOTE_COMMAND structure that represents the command to execute
 * @param bUseSerial set to true if the command was executed from a serial port link, otherwise if it is a network connection, set to false
 * @return true if the command was executed successfully
 * @return false if there was a problem executing the command
 */
bool RemoteCommand::ExecuteCommand(REMOTE_COMMAND *pCommand, bool bUseSerial) {//execute a particular command
	if (!pCommand) return false;
	if (pCommand->nCommand==THRUST_ON) {
		m_lastNavigationmmandTimeMS = millis();
		ExitMoveThreads();
		//command to turn one or both thrusters on 
		PROPELLER_STATE *pPropState = (PROPELLER_STATE *)pCommand->pDataBytes;
		m_thrusters->SetAirPropSpeedAndRudderAngle(pPropState->fPropSpeed, pPropState->fRudderAngle);
	}
	else if (pCommand->nCommand==GPS_DATA_PACKET) {
		//test
		printf("GPS command...\n");
		//end test
		if (!m_pNav->SendGPSData(m_nSocket, bUseSerial)) {
			//test
			printf("failed to send gps data.\n");
			//end test
			return false;
		}
	}
	else if (pCommand->nCommand==COMPASS_DATA_PACKET) {
		if (!m_pNav->SendCompassData(m_nSocket, bUseSerial)) {
			return false;
		}
	}
	else if (pCommand->nCommand==BATTVOLTAGE_DATA_PACKET) {
		if (!m_pAToD->SendBatteryVoltage(m_nSocket, bUseSerial)) {
			return false;
		}
	}
	else if (pCommand->nCommand==SUPPORTED_SENSOR_DATA) {
		if (!m_pSensorDataFile) return false;
		if (!m_pSensorDataFile->SendNumSensors(m_nSocket, bUseSerial)) {//send # of sensors
			return false;
		}
		if (!m_pSensorDataFile->SendSensorTypes(m_nSocket, bUseSerial)) {//send sensor types
			return false;
		}
	}
	else if (pCommand->nCommand==WATER_TEMP_DATA_PACKET) {
		if (!m_pSensorDataFile) return false;
		if (!m_pSensorDataFile->SendSensorData(WATER_TEMP_DATA,m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand==WATER_PH_DATA_PACKET) {
		if (!m_pSensorDataFile) return false;
		if (!m_pSensorDataFile->SendSensorData(PH_DATA, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand==WATER_TURBIDITY_DATA_PACKET) {
		if (!m_pSensorDataFile) return false;
		if (!m_pSensorDataFile->SendSensorData(WATER_TURBIDITY, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand==VIDEO_DATA_PACKET) {//capture frame of video and send captured image data over network connection
		if (!m_vision) {
			m_vision = new Vision();
		}
		int nThresholdInfo = 0;//value relates to image detection as follows:
		//2nd most significant byte is 0 for Canny Edge detection or 1 for fast feature detection
		//least significant byte is the low threshold for Canny Edge detection or the feature threshold for fast feature detection
		//2nd least significant byte is the high threshold for Canny Edge detection or zero for fast feature detection
		memcpy(&nThresholdInfo,pCommand->pDataBytes,sizeof(int));
		if (!m_vision->CaptureFrame(nThresholdInfo,(char *)"framecap.jpg")) {
			printf("Error capturing video frame.\n");
			return false;
		}
		void *pDiagSensor = nullptr;
		if (m_pSensorDataFile!=nullptr) {
			pDiagSensor = (void *)m_pSensorDataFile->GetDiagnosticsSensor();
		}
		if (!m_vision->SendCapturedImage(m_nSocket, bUseSerial, (char *)"framecap.jpg", pDiagSensor)) {
			printf("Error sending captured video frame.\n");
			return false;
		}
	}
	else if (pCommand->nCommand==GPS_DESTINATION_PACKET) {//command to go to a particular GPS destination
		ExitMoveThreads();
		m_lastNavigationmmandTimeMS = millis();
		m_latitudeDest=0.0, m_longitudeDest=0.0;
		memcpy(&m_latitudeDest,pCommand->pDataBytes,sizeof(double));
		memcpy(&m_longitudeDest,&pCommand->pDataBytes[8],sizeof(double));
		MoveToLocationThread();
	}
	else if (pCommand->nCommand==CANCEL_OPERATION) {//command to cancel the current operation has been sent
		//just end any of the active "move" threads for moving in a certain direction or to a certain destination
		ExitMoveThreads();
	}
	else if (pCommand->nCommand==LEAK_DATA_PACKET) {
		if (!m_pSensorDataFile) return false;
		if (!m_pSensorDataFile->SendSensorData(LEAK_DATA,m_nSocket,bUseSerial)) {//send leak sensor data
			return false;
		}
	}
	else if (pCommand->nCommand==DIAGNOSTICS_DATA_PACKET) {
		if (!m_pSensorDataFile) return false;
		if (!m_pSensorDataFile->SendSensorData(DIAGNOSTICS_DATA, m_nSocket, bUseSerial)) {//send diagnostics data
			return false;
		}
	}
	return true;
}

void RemoteCommand::DeleteCommand(REMOTE_COMMAND *pCommand) {//deletes a REMOTE_COMMAND structure from memory
	if (pCommand) {
		if (pCommand->pDataBytes) {
			delete pCommand->pDataBytes;
			pCommand->pDataBytes=0;
		}
		delete pCommand;
	}
}

void RemoteCommand::MoveForwardInFixedDirection() {//start thread that moves the boat forward in its current heading
	int nError = pthread_create(&m_moveForwardThreadId, NULL, &driveForwardThread, (void *)this);
	if (nError != 0) {
		printf("Can't create thread: %s", strerror(nError));
	}
}

void RemoteCommand::ExitMoveThreads() {
	m_bStopDriving = true;
	unsigned int uiTimeoutTime = millis() + 10000;
	while (millis()<uiTimeoutTime&&m_bDrivingThreadRunning) {
		usleep(10000);
	}
	if (m_moveForwardThreadId) {
		pthread_cancel(m_moveForwardThreadId);
		m_moveForwardThreadId=0;
	}
	if (m_moveToLocationThreadId) {
		pthread_cancel(m_moveToLocationThreadId);
		m_moveToLocationThreadId=0;
	}
	m_fMoveForwardDirection = 0.0;
}

void RemoteCommand::MoveToLocationThread() {//start thread that moves the boat to a particular GPS destination
	int nError = pthread_create(&m_moveToLocationThreadId, NULL, &driveToLocationThread, (void *)this);
	if (nError != 0) {
		printf("Can't create thread: %s", strerror(nError));
	}
}

/**
 * @brief set this remote command object to use a serial port functions for getting remote commands
 * 
 * @param bSerportMode set to true if this object is using a serial port connection to get data and commands. In that case, the first parameter of the RemoteCommand constructor should have used a serial port handle (from the serialOpen function) rather than a network socket handle.
 */
void RemoteCommand::SetSerportMode(bool bSerportMode) {
	m_bSerportMode = bSerportMode;
}

//GetSerialCommand: get the next available command over a (wireless) serial port connection from a remote device
//returns a REMOTE_COMMAND structure that corresponds to the command received 
//returns nullptr if an error occurs and sets m_nLastError to an error code
REMOTE_COMMAND * RemoteCommand::GetSerialCommand() {
	unsigned char commandBuf[4];
	memset(commandBuf,0,4);
	int nNumRead = 0;
	if (m_nSocket<=0) {
		printf("ERROR_NOSERPORT_HANDLE\n");
		m_nLastError = ERROR_NOSERPORT_HANDLE;
		return nullptr;
	}
	int nNumBytesAvailable = serialDataAvail(m_nSocket);//the number of characters available in the serial buffer
	if (nNumBytesAvailable<4) {
		//not enough bytes yet
		m_nLastError = ERROR_NO_COMMAND_RECEIVED;
		return nullptr;
	}
	//read in bytes until we get 2 consecutive 0x00 bytes (indicating a valid command)
	while (nNumRead<2&&nNumBytesAvailable>=2) {
		commandBuf[nNumRead] = (unsigned char)serialGetchar(m_nSocket);
		if (commandBuf[nNumRead]==0x00) {
			nNumRead++;
		}
		else {
			nNumRead=0;
		}
		nNumBytesAvailable = serialDataAvail(m_nSocket);//the number of characters available in the serial buffer
	}
	int nNumAvailable = 0;
	if (!WaitForSerialBytes(2,100,nNumAvailable)) {
		//not enough bytes came in
		//test
		printf("not enough bytes\n");
		//end test
		m_nLastError = ERROR_NO_COMMAND_RECEIVED;
		return nullptr;
	}

	for (int i=nNumRead;i<4;i++) {
		int nByte = serialGetchar(m_nSocket);
		if (nByte<0) {
			m_nLastError = ERROR_SERIAL_ERROR;
			return nullptr;		
		}
		commandBuf[i] = (unsigned char)nByte;
	}
	
	int nCommand = (int)GetUIntFromBytes(commandBuf);
	if (!isValidCommand(nCommand)) {
		m_nLastError = ERROR_INVALID_COMMAND;
		//test
		printf("invalid command.\n");
		serialFlush(m_nSocket);//flush serial buffer
		//end test
		return nullptr;
	}
	REMOTE_COMMAND *pCommandReceived = new REMOTE_COMMAND;
	memset(pCommandReceived,0,sizeof(REMOTE_COMMAND));
	pCommandReceived->nCommand = nCommand;

	if (requiresMoreData(nCommand)) {
		unsigned char dataSizeBytes[4];
		if (!WaitForSerialBytes(4,100,nNumAvailable)) {
			//test
			printf("Error, not enough datasize bytes arrived.\n");
			serialFlush(m_nSocket);//flush serial buffer
			m_nLastError = ERROR_NO_DATASIZE_BYTES;
			return nullptr;
			//end test
		}
		for (int i=0;i<4;i++) {
			int nByte = serialGetchar(m_nSocket);
			if (nByte<0) {
				//test
				serialFlush(m_nSocket);//flush serial buffer
				printf("ERROR_NO_DATASIZE_BYTES\n");
				//end test
				m_nLastError = ERROR_NO_DATASIZE_BYTES;
				return nullptr;
			}
			dataSizeBytes[i] = (unsigned char)nByte;
		}
		int nDataSize = (int)GetUIntFromBytes(dataSizeBytes);
		pCommandReceived->nNumDataBytes = nDataSize;
		if (nDataSize>0) {
			if (!WaitForSerialBytes(nDataSize,100,nNumAvailable)) {
				//test 
				serialFlush(m_nSocket);//flush serial buffer
				printf("Error, not enough data (%d of %d bytes available).\n",nNumAvailable,nDataSize);
				delete pCommandReceived;
				return nullptr;
			}
			unsigned char *dataBytes = new unsigned char[nDataSize];
			for (int i=0;i<nDataSize;i++) {
				int nByte = serialGetchar(m_nSocket);
				if (nByte<0) {
					printf("ERROR_NOT_ENOUGH_DATA\n");
					serialFlush(m_nSocket);//flush serial buffer
					delete pCommandReceived;
					delete []dataBytes;
					return nullptr;
				}
				dataBytes[i] = (unsigned char)nByte;
			}
			pCommandReceived->pDataBytes = dataBytes;
		}
		else {//invalid data size
			serialFlush(m_nSocket);//flush serial buffer		
			printf("Invalid data size.\n");
			delete pCommandReceived;
			return nullptr;
		}
	}
	//send confirmation back to remote device that command was successfully received
	//just echo back commandBuf bytes	
	//test
	//printf("commandBuf = %02x, %02x, %02x, %02x\n",commandBuf[0],commandBuf[1],commandBuf[2],commandBuf[3]);
	//end test
	for (int i=0;i<4;i++) {
		serialPutchar(m_nSocket,commandBuf[i]);
	}
	return pCommandReceived;
}

bool RemoteCommand::isValidCommand(int nCommand) {//checks to see if nCommand corresponds to a valid / known command or not. Returns true if it does, false if not
	if (nCommand>=THRUST_ON && nCommand<=LAST_COMMAND) {
		return true;
	}
	return false;
}

bool RemoteCommand::WaitForSerialBytes(int nNumSerialBytesExpected, unsigned int uiTimeoutMS, int &nNumAvailable) {//waits up to uiTimeoutMS milliseconds for nNumSerialBytesExpected to be available in the serial buffer
	//returns true if at least nNumSerialBytesExpected are available in the buffer, false otherwise
	int nNumAvail = serialDataAvail(m_nSocket);//the number of characters available in the serial buffer
	unsigned int uiTimeoutTime = millis() + uiTimeoutMS;
	while (nNumAvail<nNumSerialBytesExpected&&millis()<uiTimeoutTime) {
		nNumAvail = serialDataAvail(m_nSocket);
		delay(10);
	}	
	nNumAvailable = nNumAvail;
	return nNumAvail>=nNumSerialBytesExpected;
}