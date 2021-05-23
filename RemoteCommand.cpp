//RemoteCommand.cpp
#include "RemoteCommand.h"
#include "FileCommands.h"
#include "filedata.h"
#include "DiagnosticsSensor.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <wiringPi.h>
#include <wiringSerial.h>

extern FileCommands* g_fileCommands;//used for handling commands sent to boat through a text file passed on the command line of this program
extern Thruster* g_thrusters;//pointer to Thruster object for controlling power to the boat's thruster(s)
extern SensorDataFile* g_sensorDataFile;//object used for logging sensor data to file
extern LIDARLite* g_lidar;//object used for getting distance measurements to objects using LIDAR Lite
extern bool g_bCancelFunctions;//boolean flag is set to true, typically when program is ending. It is polled by various functions (ex: navigation functions) that sometimes take a long time to complete
extern void StartFileCommandsThread();//start thread for executing commands from a text file
extern bool GetDataLoggingPreferences();//get preferences from prefs.txt file.
extern DiagnosticsSensor* g_shipdiagnostics;//interface for ship monitoring, power control, etc.
extern NotifyOperator g_notifier;//used for notifying one or more people by email or text of the boat's condition
extern pthread_mutex_t g_remoteCommandsMutex;//mutex for making sure remote commands do not interfere with one another
extern Vision g_vision;//object used for taking pictures with camera(s)
extern bool g_bExitFileCommandsThread;//boolean flag used to control when the file commands thread should exit
extern bool g_bFileCommandsThreadRunning;//boolean flag is true when the file commands thread is running
extern char* g_szRTKPort;//serial port used for sending RTK correction data to GPS receiver board


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
	pRC->m_pNav->DriveToLocation(pRC->m_latitudeDest,pRC->m_longitudeDest,(void *)pRC->m_thrusters,&g_remoteCommandsMutex,&pRC->m_lastNavigationmmandTimeMS, pRC->m_shipLog, 10,&pRC->m_bStopDriving,HIGH_PRIORITY);//higher priority since command was explicity issued by user using app
	pRC->m_bDrivingThreadRunning = false;
	return 0;
}

//RemoteCommand constructor
//szRootFolder = the folder where the main program resides and is run from 
//nSocket = network socket descriptor (socket descriptor from accepted network connection)
//pNav = pointer to navigation object used for getting GPS and compass data and plotting routes
//pThrusters = pointer to Thruster object that controls power to the propeller(s)
//pAToD = pointer to object used for handling analog to digital measurements
//pShipLog = void pointer to ShipLog object for logging various data and debugging info to file
RemoteCommand::RemoteCommand(char *szRootFolder, int nSocket, Navigation *pNav, Thruster *pThrusters, AToD *pAToD, void *pShipLog) {
	m_szRootFolder = szRootFolder;
	m_lastNavigationmmandTimeMS=0;
	m_bSerportMode = false;
	m_shipLog = pShipLog;
	m_bDrivingThreadRunning=false;
	m_bStopDriving=false;
	m_rtk = nullptr;
	m_nSocket = nSocket;
	m_nLastError = 0;
	m_thrusters=pThrusters;
	m_pNav = pNav;
	m_pAToD = pAToD;
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
	if (m_rtk != nullptr) {
		delete m_rtk;
		m_rtk = nullptr;
	}
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
	//test
	printf("nCommand = %d\n", nCommand);
	//end test
	if (requiresMoreData(nCommand)) {
		//test
		printf("Requires more data...\n");
		//end test
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
	if (needsConfirmation(pCommandReceived->nCommand)) {
		//send confirmation back to remote client that command was successfully received
		//just echo back commandBuf bytes	
		//test
		printf("About to send confirmation...\n");
		//end test
		if (send(m_nSocket, (char*)commandBuf, 4, 0) < 0) {
			m_nLastError = ERROR_SENDING_DATA;
			//test
			printf("ERROR_SENDING_DATA\n");
			//end test
			if (pCommandReceived->pDataBytes != nullptr) {
				delete[]pCommandReceived->pDataBytes;
				pCommandReceived->pDataBytes = nullptr;
			}
			delete pCommandReceived;
			return nullptr;
		}
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
	if (nCommand == THRUST_ON) return true;
	else if (nCommand == VIDEO_DATA_PACKET) return true;
	else if (nCommand == GPS_DESTINATION_PACKET) return true;
	else if (nCommand == SCRIPT_STEP_CHANGE) return true;
	else if (nCommand == USE_REMOTE_SCRIPT) return true;
	else if (nCommand == FILE_TRANSFER) return true;
	else if (nCommand == FILE_RECEIVE) return true;
	else if (nCommand == USE_REMOTE_SCRIPT) return true;
	else if (nCommand == REFRESH_SETTINGS) return true;
	else if (nCommand == RTK_CORRECTION) return true;
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
	else if (pCommand->nCommand == SCRIPT_STATUS_PACKET) {
		szDescription = new char[256];
		sprintf(szDescription, "Script status query command sent.\n");
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
bool RemoteCommand::ExecuteCommand(REMOTE_COMMAND* pCommand, bool bUseSerial) {//execute a particular command
	if (!pCommand) return false;
	if (pCommand->nCommand == THRUST_ON) {
		m_lastNavigationmmandTimeMS = millis();
		ExitMoveThreads();
		//command to turn one or both thrusters on 
		PROPELLER_STATE* pPropState = (PROPELLER_STATE*)pCommand->pDataBytes;
		m_thrusters->SetAirPropSpeedAndRudderAngle(pPropState->fPropSpeed, pPropState->fRudderAngle);
	}
	else if (pCommand->nCommand == GPS_DATA_PACKET) {
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
	else if (pCommand->nCommand == COMPASS_DATA_PACKET) {
		if (!m_pNav->SendCompassData(m_nSocket, bUseSerial)) {
			return false;
		}
	}
	else if (pCommand->nCommand == BATTVOLTAGE_DATA_PACKET) {
		if (!m_pAToD->SendBatteryVoltage(m_nSocket, bUseSerial)) {
			return false;
		}
	}
	else if (pCommand->nCommand == SUPPORTED_SENSOR_DATA) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendNumSensors(m_nSocket, bUseSerial)) {//send # of sensors
			return false;
		}
		if (!g_sensorDataFile->SendSensorTypes(m_nSocket, bUseSerial)) {//send sensor types
			return false;
		}
	}
	else if (pCommand->nCommand == WATER_TEMP_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(WATER_TEMP_DATA, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand == WATER_PH_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(PH_DATA, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand == WATER_TURBIDITY_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(WATER_TURBIDITY, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand == DO2_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(DO2_DATA, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand == CONDUCTIVITY_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(CONDUCTIVITY_DATA, m_nSocket, bUseSerial)) {//send water temperature
			return false;
		}
	}
	else if (pCommand->nCommand == VIDEO_DATA_PACKET) {//capture frame of video and send captured image data over network connection
		int nThresholdInfo = 0;//value relates to image detection as follows:
		//2nd most significant byte is 0 for Canny Edge detection or 1 for fast feature detection
		//least significant byte is the low threshold for Canny Edge detection or the feature threshold for fast feature detection
		//2nd least significant byte is the high threshold for Canny Edge detection or zero for fast feature detection
		memcpy(&nThresholdInfo, pCommand->pDataBytes, sizeof(int));
		if (!g_vision.CaptureFrame(nThresholdInfo, (char*)"framecap.jpg")) {
			printf("Error capturing video frame.\n");
			return false;
		}
		void* pDiagSensor = nullptr;
		if (g_sensorDataFile != nullptr) {
			pDiagSensor = (void*)g_sensorDataFile->GetDiagnosticsSensor();
		}
		if (!g_vision.SendCapturedImage(m_nSocket, bUseSerial, (char*)"framecap.jpg", pDiagSensor)) {
			printf("Error sending captured video frame.\n");
			return false;
		}
	}
	else if (pCommand->nCommand == GPS_DESTINATION_PACKET) {//command to go to a particular GPS destination
		ExitMoveThreads();
		m_lastNavigationmmandTimeMS = millis();
		m_latitudeDest = 0.0, m_longitudeDest = 0.0;
		memcpy(&m_latitudeDest, pCommand->pDataBytes, sizeof(double));
		memcpy(&m_longitudeDest, &pCommand->pDataBytes[8], sizeof(double));
		MoveToLocationThread();
	}
	else if (pCommand->nCommand == CANCEL_OPERATION) {//command to cancel the current operation has been sent
		//just end any of the active "move" threads for moving in a certain direction or to a certain destination
		ExitMoveThreads();
	}
	else if (pCommand->nCommand == LEAK_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(LEAK_DATA, m_nSocket, bUseSerial)) {//send leak sensor data
			return false;
		}
	}
	else if (pCommand->nCommand == DIAGNOSTICS_DATA_PACKET) {
		if (!g_sensorDataFile) return false;
		if (!g_sensorDataFile->SendSensorData(DIAGNOSTICS_DATA, m_nSocket, bUseSerial)) {//send diagnostics data
			return false;
		}
	}
	else if (pCommand->nCommand == SCRIPT_STATUS_PACKET) {
		return FileCommands::SendRemoteScriptInfo(g_fileCommands, m_nSocket, bUseSerial);//send info about the current file script (if any) that is running
	}
	else if (pCommand->nCommand == SCRIPT_STEP_CHANGE) {
		int nStepChange = 0;//the change in the step # of the currently running script
		memcpy(&nStepChange, pCommand->pDataBytes, sizeof(int));
		if (g_fileCommands != nullptr) {
			if (nStepChange != 0) {
				g_fileCommands->ChangeCurrentStep(nStepChange);
			}
		}
	}
	else if (pCommand->nCommand == LIST_REMOTE_SCRIPTS) {
		return FileCommands::ListRemoteScriptsAvailable(m_szRootFolder, m_nSocket, bUseSerial);//return a list of all of the file scripts available in the root program folder
	}
	else if (pCommand->nCommand == LIST_REMOTE_DATA) {
		return FileCommands::ListRemoteDataAvailable(m_szRootFolder, m_nSocket, bUseSerial);//return a list of all of the data files available in the root program folder
	}
	else if (pCommand->nCommand == LIST_REMOTE_LOG) {
		return FileCommands::ListRemoteLogsAvailable(m_szRootFolder, m_nSocket, bUseSerial);//return a list of all of the log files available in the root program folder
	}
	else if (pCommand->nCommand == LIST_REMOTE_IMAGE) {
		char imageFolder[PATH_MAX];//folder where images are stored
		sprintf(imageFolder, "%s/Images", m_szRootFolder);
		return FileCommands::ListRemoteImageAvailable(imageFolder, m_nSocket, bUseSerial);//return a list of all of the image files available in the Images subfolder
	}
	else if (pCommand->nCommand == USE_REMOTE_SCRIPT) {//instruction to change the current file script
		int MAX_NAME_LENGTH = 256;//maximum length of remote script filename
		int nRemoteScriptNameLength = 0;//the length (in characters) of the script that AMOS should switch to
		memcpy(&nRemoteScriptNameLength, pCommand->pDataBytes, sizeof(int));
		if (nRemoteScriptNameLength > MAX_NAME_LENGTH) {
			nRemoteScriptNameLength = MAX_NAME_LENGTH;
		}
		if (g_fileCommands != nullptr) {
			g_fileCommands->CancelCurrentOperation();//cancel the current step (if there is one in progress)
		}
		char* szScriptName = new char[nRemoteScriptNameLength + 1];
		memcpy(szScriptName, &pCommand->pDataBytes[sizeof(int)], nRemoteScriptNameLength);
		szScriptName[nRemoteScriptNameLength] = 0;//null terminate filename string
		if (nRemoteScriptNameLength == 0) {
			//need to stop the script that is currently running
			g_bExitFileCommandsThread = true;
			unsigned int uiTimeoutTime = millis() + 3000;
			while (g_bFileCommandsThreadRunning&&millis()<uiTimeoutTime) {
				usleep(100000);//pause for 100 ms
			}
			if (g_fileCommands != nullptr) {
				delete g_fileCommands;
				g_fileCommands = nullptr;
			}
		}
		bool bRetval = true;
		if (g_fileCommands != nullptr) {
			//test
			printf("Changing to script: %s\n", szScriptName);
			//end test
			bRetval = g_fileCommands->ChangeToFile(szScriptName);
		}
		else {
			g_fileCommands = new FileCommands(m_szRootFolder, szScriptName, this->m_pNav, g_thrusters, &g_bCancelFunctions);
			StartFileCommandsThread();
		}
		delete[] szScriptName;
		return bRetval;
	}
	else if (pCommand->nCommand == FILE_TRANSFER) {
		int nSizeToDownload = (int)GetUIntFromBytes(pCommand->pDataBytes);
		//test
		printf("nSizeToDownload = %d\n", nSizeToDownload);
		//end test
		if (nSizeToDownload > 100000000) {
			//don't try to receive anything > 100 MBytes!
			printf("file size is too large!");
			return false;
		}
		unsigned char* downloadedBytes = new unsigned char[nSizeToDownload];
		if (this->m_bSerportMode) {
			if (ReceiveLargeDataChunk(m_nSocket, (char*)downloadedBytes, nSizeToDownload)) {
				int nOutputPathSize = (int)GetUIntFromBytes(downloadedBytes);
				int nFileSize = nSizeToDownload - nOutputPathSize - sizeof(int);
				if (nFileSize > 0 && nOutputPathSize > 0) {
					char* destPath = new char[nOutputPathSize + 1];
					memcpy(destPath, &downloadedBytes[sizeof(int) + nFileSize], nOutputPathSize);
					destPath[nOutputPathSize] = 0;//null terminate path string
					bool bRetval = false;
					FILE* outputFile = fopen(destPath, "w");//open for writing (or overwriting if it already exists)
					if (outputFile != NULL) {
						int nNumWritten = fwrite(&downloadedBytes[sizeof(int)], 1, nFileSize, outputFile);
						bRetval = (nNumWritten == nFileSize);
						fclose(outputFile);
					}
					delete[]destPath;
					delete[]downloadedBytes;
					return bRetval;
				}
			}
			delete[]downloadedBytes;
		}
		else {//TODO: network mode impementation
			const int MAX_ALLOWED_TIMEOUTS = 5;
			int nNumRemaining = nSizeToDownload;
			int nNumReceived = 0;
			int nTimeoutCount = 0;
			do {
				int nRX = read(m_nSocket, &downloadedBytes[nNumReceived], nNumRemaining);
				if (nRX > 0) {
					nNumRemaining -= nRX;
					nNumReceived += nRX;
					nTimeoutCount = 0;
				}
				else {//timeout or error occurred
					nTimeoutCount++;
				}
			} while (nNumRemaining > 0 && nTimeoutCount < MAX_ALLOWED_TIMEOUTS);
			if (nNumReceived < nSizeToDownload) {
				printf("timed out, read %d of %d bytes.\n", nNumReceived, nSizeToDownload);
				delete[]downloadedBytes;
				return false;
			}
			int nOutputPathSize = (int)GetUIntFromBytes(downloadedBytes);
			int nFileSize = nSizeToDownload - nOutputPathSize - sizeof(int);
			if (nFileSize > 0 && nOutputPathSize > 0) {
				char* destPath = new char[nOutputPathSize + 1];
				memcpy(destPath, &downloadedBytes[sizeof(int) + nFileSize], nOutputPathSize);
				destPath[nOutputPathSize] = 0;//null terminate path string
				bool bRetval = false;
				FILE* outputFile = fopen(destPath, "w");//open for writing (or overwriting if it already exists)
				if (outputFile != NULL) {
					int nNumWritten = fwrite(&downloadedBytes[sizeof(int)], 1, nFileSize, outputFile);
					bRetval = (nNumWritten == nFileSize);
					fclose(outputFile);
				}
				delete[]destPath;
				delete[]downloadedBytes;
				return bRetval;
			}
		}
		return false;
	}
	else if (pCommand->nCommand == FILE_RECEIVE) {
		int nFilenameLength = (int)GetUIntFromBytes(pCommand->pDataBytes);
		//test
		printf("nFilenameLength = %d\n", nFilenameLength);
		//end test
		if (nFilenameLength > 256) {
			//filename is too long!
			printf("file name is too long!");
			return false;
		}
		char* fileNameBytes = new char[nFilenameLength+1];
		if (this->m_bSerportMode) {
			unsigned int uiTimeoutTime = millis() + 5000;//allow for a 5 second timeout (shouldn't take that long!)
			int nNumBytesAvail = serialDataAvail(m_nSocket);
			int nNumRead = 0;
			while (nNumRead < nFilenameLength && millis() < uiTimeoutTime) {
				if (nNumBytesAvail > 0) {
					for (int i = 0; i < nNumBytesAvail; i++) {
						fileNameBytes[nNumRead] = serialGetchar(m_nSocket);
						nNumRead++;
					}
				}
				nNumBytesAvail = serialDataAvail(m_nSocket);
			}
			if (nNumRead < nFilenameLength) {
				printf("Timed out trying to read in filename. %d of %d bytes read.\n", nNumRead, nFilenameLength);
				delete[]fileNameBytes;
				return false;
			}
		}
		else {
			//receive filename over network socket
			int nFileLength = filedata::getFileLength(fileNameBytes);//Finds the length of an opened file stream in bytes
			int nNumRead = read(m_nSocket, fileNameBytes, nFilenameLength);
			if (nNumRead < nFilenameLength) {
				printf("Timed out trying to read in filename. %d of %d bytes read.\n", nNumRead, nFilenameLength);
				delete[]fileNameBytes;
				return false;
			}
		}
		fileNameBytes[nFilenameLength] = 0;//null terminate filename
		int nFileSize = filedata::getFileLength(fileNameBytes);
		if (nFileSize<=0) {
			printf("Error, unable to open file: %s\n", fileNameBytes);
			delete[]fileNameBytes;
			return false;
		}
		FILE* localFile = fopen(fileNameBytes, "rb");
		delete[]fileNameBytes;
		unsigned char* fileBytes = new unsigned char[nFileSize];
		fread(fileBytes, 1, nFileSize, localFile);
		fclose(localFile);
		//create and send BOAT_DATA structure for sending the file
		BOAT_DATA* pBoatData = BoatCommand::CreateBoatData(FILE_RECEIVE);
		memcpy(pBoatData->dataBytes, &nFileSize, sizeof(int));
		pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
		if (!BoatCommand::SendBoatData(m_nSocket, bUseSerial, pBoatData, nullptr)) {
			delete pBoatData;
			delete[] fileBytes;
			return false;
		}
		//send out actual file bytes
		if (bUseSerial) {//using serial port link
			if (!BoatCommand::SendLargeSerialData(m_nSocket, (unsigned char*)fileBytes, nFileSize, nullptr)) {//send large amount of data out serial port, need to get confirmation after sending each chunk
				delete pBoatData;
				delete[] fileBytes;
				return false;
			}
		}
		else {//using network link
			if (send(m_nSocket, fileBytes, nFileSize, 0) < 0) {
				//error sending data
				delete pBoatData;
				delete[] fileBytes;
				return false;
			}
		}
		delete pBoatData;
		delete[] fileBytes;
		return true;
	}
	else if (pCommand->nCommand == REFRESH_SETTINGS) {
		int nSettingsType = (int)GetUIntFromBytes(pCommand->pDataBytes);
		return RefreshSettings(nSettingsType);
	}
	else if (pCommand->nCommand == RTK_CORRECTION) {
		//packet of incoming RTK correction data
		if (g_szRTKPort != nullptr) {
			if (m_rtk == nullptr) {
				m_rtk = new RTK(g_szRTKPort);
			}
			m_rtk->SendSerialRTKData(pCommand->pDataBytes, pCommand->nNumDataBytes);
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
	//test
	printf("nCommand = %d\n", nCommand);
	//end test
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
			if (!WaitForSerialBytes(nDataSize, 100, nNumAvailable)) {
				serialFlush(m_nSocket);//flush serial buffer
				printf("Error, not enough data (%d of %d bytes available).\n", nNumAvailable, nDataSize);
				delete pCommandReceived;
				return nullptr;
			}
			unsigned char* dataBytes = new unsigned char[nDataSize];
			for (int i = 0; i < nDataSize; i++) {
				int nByte = serialGetchar(m_nSocket);
				if (nByte < 0) {
					printf("ERROR_NOT_ENOUGH_DATA\n");
					serialFlush(m_nSocket);//flush serial buffer
					delete pCommandReceived;
					delete[]dataBytes;
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

/**
 * @brief tries to receive a large chunk of data over serial port connection.
 *
 * @param nSocket the serial port descriptor id
 * @param rxBytes buffer used to store the received data.
 * @param nNumToReceive the number of bytes to receive over the serial port connection.
 * @return int the number of bytes successfully read (and stored in rxBytes) over the serial port connection.
 */
int RemoteCommand::ReceiveLargeDataChunk(int nSocket, char* rxBytes, int nNumToReceive) {
	const int MAX_ALLOWED_TIMEOUTS = 5;
	const int MAX_RECEIVE_ATTEMPT = 128;//maximum number of bytes to attempt receiving in a single ReadFile attempt
	const int NUM_PACKET_OVERHEAD_BYTES = 11;//the number of packet overhead bytes, sync bytes, id bytes, data size bytes, crc bytes, etc.
	unsigned char outBuf[3] = { 0,0,0 };//output buffer for sending confirmation bytes over serial port link
	unsigned char inBuf[1024];//temporary buffer for collecting incoming data
	int nNumRemaining = nNumToReceive;
	int nNumReceived = 0;
	int nTimeoutCount = 0;
	int nChunkIndex = 0;
	//test
	printf("About to receive %d data bytes.\n", nNumToReceive);
	//end test
	do {
		int nNumToReceive = min(nNumRemaining + NUM_PACKET_OVERHEAD_BYTES, MAX_RECEIVE_ATTEMPT);
		unsigned int uiTimeoutTime = millis() + 1000;
		int nNumAvail = serialDataAvail(nSocket);
		while (nNumAvail < nNumToReceive && millis() < uiTimeoutTime) {
			nNumAvail = serialDataAvail(nSocket);
		}
		if (nNumAvail<nNumToReceive) {
			//timeout or error occurred
			nTimeoutCount++;
		}
		else {
			for (int i = 0; i < nNumToReceive; i++) {
				inBuf[i] = (unsigned char)serialGetchar(nSocket);
			}
			int  nMoreBytesRequired = 0;//will be non-zero if we need to receive more bytes in order to make a valid packet of data
			int nPacketStartIndex = 0;
			int nValidPacket = CheckPacket(inBuf, nNumToReceive, nChunkIndex, nMoreBytesRequired, nPacketStartIndex);
			if (nValidPacket == PACKET_OK) {
				CopyPacketToBuf(inBuf, nPacketStartIndex, nNumToReceive, &rxBytes[nNumReceived]);
				nNumRemaining -= ((int)(nNumToReceive - NUM_PACKET_OVERHEAD_BYTES));
				nNumReceived += ((int)(nNumToReceive - NUM_PACKET_OVERHEAD_BYTES));
				//send confirmation (CRC) bytes back to sender
				printf("data received ok, sending confirmation.\n");
				nChunkIndex++;
				SendConfirmationBytes(nSocket, inBuf[nNumToReceive - 2], inBuf[nNumToReceive - 1]);
				nTimeoutCount = 0;
			}
			else if (nValidPacket == REPEATED_CHUNK) {
				//just send confirmation (CRC) bytes back to boat
				//send confirmation (CRC) bytes back to boat
				printf("received repeated chunk.\n");
				SendConfirmationBytes(nSocket, inBuf[nNumToReceive - 2], inBuf[nNumToReceive - 1]);
				nTimeoutCount = 0;
			}
			else if (nValidPacket == NOT_ENOUGH_BYTES) {//need to receive more bytes of data
				//nMoreBytesRequired is non-zero
				//try reading in more bytes
				unsigned int uiTimeoutTime = millis() + 1000;
				int nNumAvail = serialDataAvail(nSocket);
				while (nNumAvail < nMoreBytesRequired&&millis()<uiTimeoutTime) {
					nNumAvail = serialDataAvail(nSocket);
				}
				if (nNumAvail > 0) {
					for (int i = 0; i < nNumAvail; i++) {
						inBuf[i+nNumToReceive] = (unsigned char)serialGetchar(nSocket);
					}
				}
				if (nNumAvail == 0) {
					printf("not enough bytes received\n");
					SendErrorBytes(nSocket);
				}
				else {//got at least some more bytes, check again to see if packet is valid
					nNumToReceive += nNumAvail;
					nValidPacket = CheckPacket(inBuf, nNumToReceive, nChunkIndex, nMoreBytesRequired, nPacketStartIndex);
					if (nValidPacket == PACKET_OK) {
						CopyPacketToBuf(inBuf, nPacketStartIndex, nNumReceived, &rxBytes[nNumReceived]);
						nNumRemaining -= (nNumToReceive - NUM_PACKET_OVERHEAD_BYTES);
						nNumReceived += (nNumToReceive - NUM_PACKET_OVERHEAD_BYTES);
						//send confirmation (CRC) bytes back to boat
						printf("data received ok, sending confirmation.\n");
						nChunkIndex++;
						SendConfirmationBytes(nSocket, inBuf[nNumToReceive - 2], inBuf[nNumToReceive - 1]);
						nTimeoutCount = 0;
					}
					else if (nValidPacket == REPEATED_CHUNK) {
						//just send confirmation (CRC) bytes back to sender
						//send confirmation (CRC) bytes back to sender
						printf("received repeated chunk.\n");
						SendConfirmationBytes(nSocket, inBuf[nNumToReceive - 2], inBuf[nNumToReceive - 1]);
						nTimeoutCount = 0;
					}
					else {//packet is not valid
						printf("invalid packet.\n");
						SendErrorBytes(nSocket);
					}
				}
			}
			else {
				printf("data got garbled\n");
				SendErrorBytes(nSocket);
			}
			printf("nNumReceived = %d\n", nNumReceived);
		}
		if (g_shipdiagnostics != nullptr) {
			g_shipdiagnostics->ActivityPulse();//send activity pulse to RF220SU to keep AMOS from getting shut down
		}
	} while (nNumRemaining > 0 && nTimeoutCount < MAX_ALLOWED_TIMEOUTS);
	return nNumReceived;
}

int RemoteCommand::CheckPacket(unsigned char* inBuf, int nBufSize, int nChunkIndex, int& nMoreBytesRequired, int& nPacketStartIndex) {//check to see if a chunk of data is valid or not
	//returns one of the following:
	//PACKET_OK 0 //the packet is fine without any data problems
	//REPEATED_CHUNK 1 //the packet is fine, but corresponds to an old (repeated) chunk of data that has already been received
	//BAD_CRC 2 //one or both of the CRC bytes are incorrect
	//NOT_ENOUGH_BYTES 3 //what has been received of the packet so far is ok, but there are not enough bytes present
	//NO_SYNC_BYTES 4 //no synchronization bytes could be found in the packet data
	//BAD_CHUNK_INDEX 5 //bytes for chunk index corresponded to a chunk that was too high
	//BAD_DATASIZE 6 //bytes for data size are wrong
	//test
	const int MAX_DATA_CHUNKSIZE = 117;//the maximum number of allowed data bytes in the packet
	nMoreBytesRequired = 0;
	nPacketStartIndex = -1;
	printf("checking %d bytes at %d\n", nBufSize, millis());
	//end test
	if (nBufSize <= 11) {
		printf("not enough data present.\n");
		return NOT_ENOUGH_BYTES;//not enough data present
	}
	//look for sync bytes (i.e. "AMOS" text)
	int nSyncIndex = -1;
	for (int i = 0; i < (nBufSize - 4); i++) {
		if (inBuf[i] == 'A' && inBuf[i + 1] == 'M' && inBuf[i + 2] == 'O' && inBuf[i + 3] == 'S') {
			//found sync bytes
			nSyncIndex = i;
			break;
		}
	}
	if (nSyncIndex < 0) {
		printf("no sync bytes could be found\n");
		return NO_SYNC_BYTES;
	}
	//check chunk value
	int nChunkVal = (inBuf[nSyncIndex + 4] << 16) + (inBuf[nSyncIndex + 5] << 8) + inBuf[nSyncIndex + 6];
	if (nChunkVal > nChunkIndex) {//receiving an unexpected chunk of data, data for chunk index must have gotten garbled
		printf("bytes for chunk index are wrong.\n");
		return BAD_CHUNK_INDEX;
	}
	int nDataSize = (inBuf[nSyncIndex + 7] << 8) + inBuf[nSyncIndex + 8];
	if (nDataSize > MAX_DATA_CHUNKSIZE) {
		printf("bytes for data size are wrong.\n");
		return BAD_DATASIZE;
	}
	if ((nDataSize + 11) > nBufSize) {//not enough data available
		printf("not enough data available.\n");
		return NOT_ENOUGH_BYTES;
	}
	//add up checksum
	int nChecksum = 0;
	for (int i = 0; i < nDataSize; i++) {
		nChecksum += ((int)(inBuf[nSyncIndex + 9 + i]));
	}
	int nTestChecksum = ((int)inBuf[nSyncIndex + 9 + nDataSize]) * 256 + inBuf[nSyncIndex + 9 + nDataSize + 1];
	bool bValidChecksum = (nChecksum == nTestChecksum);
	if (!bValidChecksum) {
		printf("invalid checksum, crc_byte1 = %d, crc_byte2 = %d, nChecksum = %d, nTestChecksum = %d.\n", (int)inBuf[nSyncIndex + 9 + nDataSize], (int)inBuf[nSyncIndex + 9 + nDataSize + 1],
			nChecksum, nTestChecksum);
		return BAD_CRC;
	}
	nPacketStartIndex = nSyncIndex;
	if (nChunkVal < nChunkIndex) {
		//everything was ok, but it is for old data that has already been received
		printf("repeated chunk of data.\n");
		return REPEATED_CHUNK;
	}
	return PACKET_OK;
}

void RemoteCommand::CopyPacketToBuf(unsigned char* inBuf, int nBufferStartIndex, int nBufSize, char* destBuf) {//copy the data portion of inBuf to destBuf
	int nDataSize = (inBuf[nBufferStartIndex + 7] << 8) + inBuf[nBufferStartIndex + 8];
	if (nDataSize > nBufSize) {
		return;//data size too big? should not happen
	}
	memcpy(destBuf, &inBuf[9 + nBufferStartIndex], nDataSize);
}

void RemoteCommand::SendConfirmationBytes(int nSocket, unsigned char byte1, unsigned char byte2) {//send a couple of confirmation bytes to the sender
	unsigned char outBuf[4];
	printf("sending confirmation bytes %d and %d\n", (int)byte1, (int)byte2);
	outBuf[0] = 0;
	outBuf[1] = 0;
	outBuf[2] = byte1;
	outBuf[3] = byte2;
	for (int i = 0; i < 4; i++) {
		serialPutchar(nSocket, outBuf[i]);
	}
}

void RemoteCommand::SendErrorBytes(int nSocket) {//send serial port bytes that correspond to a communications error
	unsigned char outBuf[4] = { 0xff, 0xff, 0xff, 0xff };
	serialFlush(nSocket);
	for (int i = 0; i < 4; i++) {
		serialPutchar(nSocket, outBuf[i]);
	}
}

void RemoteCommand::SendCancelBytes(int nSocket) {//send four 0x0a bytes out serial port to indicate that downloading is stopping
	unsigned char outBuf[4] = { 0x0a, 0x0a, 0x0a, 0x0a };
	for (int i = 0; i < 4; i++) {
		serialPutchar(nSocket, outBuf[i]);
	}
}

bool RemoteCommand::RefreshSettings(int nSettingsType) {//refresh settings of a particular type (see CommandList.h) from the current contents of the prefs.txt file
	//open prefs.txt file to get settings from it
	//#define AMOS_ALARM_SETTINGS 1
	//#define AMOS_LIDAR_SETTINGS 2
	//#define AMOS_SENSOR_SETTINGS 3
	//#define AMOS_CAMERA_SETTINGS 4
	//#define AMOS_BATTERY_SETTINGS 5
	
	if (nSettingsType == AMOS_ALARM_SETTINGS) {
		return g_notifier.ReloadConfigInfo(m_szRootFolder);
	}
	else if (nSettingsType == AMOS_SENSOR_SETTINGS||nSettingsType==AMOS_CAMERA_SETTINGS) {
		return GetDataLoggingPreferences();
	}
	return false;
}

bool RemoteCommand::needsConfirmation(int nCommandType) {//return true if an nCommandType command requires some form of confirmation back to the host
	//currently only RTK_CORRECTION commands do not require any sort of confirmation
	if (nCommandType == RTK_CORRECTION) {
		return false;
	}
	return true;
}