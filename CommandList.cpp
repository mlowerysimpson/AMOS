//comment out the next line for non-Windows builds, or include a dummy stdafx.h file
#include "pch.h"
#include "stdafx.h"
#include "CommandList.h"
#include "IMU.h"
#include <stdio.h>      
#ifndef _WIN32
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#endif
#include <string.h>

#define AMOSEN_VAL "tyo7-rDeI-3OOEfz"

BoatCommand::BoatCommand() {

}

BoatCommand::~BoatCommand() {

}

BOAT_DATA * BoatCommand::CreateBoatData(int nDataType) {//create an empty BOAT_DATA structure for a given data type. Calling function is responsible for deleting the created object
	BOAT_DATA *pAMOSData = new BOAT_DATA;
	memset(pAMOSData,0,sizeof(BOAT_DATA));
	if (nDataType==GPS_DATA_PACKET) {
		pAMOSData->nPacketType = GPS_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(GPS_DATA);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==COMPASS_DATA_PACKET) {
		pAMOSData->nPacketType = COMPASS_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(IMU_DATASAMPLE);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==BATTVOLTAGE_DATA_PACKET) {
		pAMOSData->nPacketType = BATTVOLTAGE_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(float);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==SUPPORTED_SENSOR_DATA) {
		pAMOSData->nPacketType = SUPPORTED_SENSOR_DATA;
		pAMOSData->nDataSize = sizeof(int);//just getting the number of sensors first, will then get a second packet of info later that corresponds to the actual sensor types
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==SENSOR_TYPES_INFO) {
		pAMOSData->nPacketType = SENSOR_TYPES_INFO;
		pAMOSData->nDataSize = 0;//needs to be assigned later
		pAMOSData->dataBytes = NULL;//needs to be assigned later
		pAMOSData->checkSum = 0;//needs to be assigned later
	}
	else if (nDataType==WATER_TEMP_DATA_PACKET) {
		pAMOSData->nPacketType = WATER_TEMP_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(float);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==WATER_PH_DATA_PACKET) {
		pAMOSData->nPacketType = WATER_PH_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(float);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==WATER_TURBIDITY_DATA_PACKET) {
		pAMOSData->nPacketType = WATER_TURBIDITY_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(float);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType == DO2_DATA_PACKET) {
		pAMOSData->nPacketType = DO2_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(float);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes, 0, pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}		
	else if (nDataType == CONDUCTIVITY_DATA_PACKET) {
		pAMOSData->nPacketType = CONDUCTIVITY_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(float);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes, 0, pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==VIDEO_DATA_PACKET) {
		pAMOSData->nPacketType = VIDEO_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		memset(pAMOSData->dataBytes,0,pAMOSData->nDataSize);
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated whenever pAMOSData changes
	}
	else if (nDataType==LEAK_DATA_PACKET) {
		pAMOSData->nPacketType = LEAK_DATA_PACKET;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType==DIAGNOSTICS_DATA_PACKET) {
		pAMOSData->nPacketType = DIAGNOSTICS_DATA_PACKET;
		pAMOSData->nDataSize = 7*sizeof(float) + sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == SCRIPT_STATUS_PACKET) {
		pAMOSData->nPacketType = SCRIPT_STATUS_PACKET;
		pAMOSData->nDataSize = REMOTE_SCRIPT_NAMELENGTH + 2 * sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == LIST_REMOTE_SCRIPTS) {
		pAMOSData->nPacketType = LIST_REMOTE_SCRIPTS;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == LIST_REMOTE_DATA) {
		pAMOSData->nPacketType = LIST_REMOTE_DATA;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == LIST_REMOTE_LOG) {
		pAMOSData->nPacketType = LIST_REMOTE_LOG;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == LIST_REMOTE_IMAGE) {
		pAMOSData->nPacketType = LIST_REMOTE_IMAGE;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == FILE_TRANSFER) {
		pAMOSData->nPacketType = FILE_TRANSFER;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == FILE_RECEIVE) {
		pAMOSData->nPacketType = FILE_RECEIVE;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == REFRESH_SETTINGS) {
		pAMOSData->nPacketType = REFRESH_SETTINGS;
		pAMOSData->nDataSize = sizeof(int);
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else if (nDataType == DELETE_FILES) {
		pAMOSData->nPacketType = DELETE_FILES;
		pAMOSData->nDataSize = sizeof(int) + 1;
		pAMOSData->dataBytes = new unsigned char[pAMOSData->nDataSize];
		pAMOSData->checkSum = CalculateChecksum(pAMOSData);//checksum needs to be recalculated
	}
	else {//unknown data type, return null
		delete pAMOSData;
#ifndef _WIN32
		return nullptr;
#else
		return NULL;
#endif
	}
	return pAMOSData;
}

unsigned char BoatCommand::CalculateChecksum(BOAT_DATA *pData) {//calculate simple 8-bit checksum for BOAT_DATA structure
	unsigned char ucChecksum = 0;
	unsigned char *pBytes = (unsigned char *)pData;
	int nNumToCheck = 2*sizeof(int) + pData->nDataSize;
	for (int i=0;i<nNumToCheck;i++) {
		ucChecksum+=pBytes[i];
	}
	return ucChecksum;
}

//DeleteBoatData: deletes a BOAT_DATA structure that had been previously created by the CreateBoatData function
//boatData = pointer to BOAT_DATA structure that is to be deleted.
void BoatCommand::DeleteBoatData(BOAT_DATA *boatData) {//deletes the memory used by a BOAT_DATA structure
	if (!boatData) return;
	if (boatData->dataBytes) {
		delete boatData->dataBytes;
		boatData->dataBytes=NULL;
	}
	delete boatData;
}

/**
 * @brief send boat data out over a network socket or serial port connection
 * 
 * @param nHandle the network socket handle or the serial port file descriptor used for communications
 * @param bUseSerial set to true if a wireless serial port link is being used, otherwise set to false
 * @param boatData pointer to structure containing the data that is being sent
 * @param pDiagnostics void pointer to a DiagnosticsSensor object that is used in some cases to signal that the program is still running correctly (for example when sending large chunks of data at a time). It can be set to nullptr when sending small amounts of data (< 1 kbyte). 
 * @return true 
 * @return false 
 */
bool BoatCommand::SendBoatData(int nSocket, bool bUseSerial, BOAT_DATA *boatData, void *pDiagnostics) {//sends boat data out over socket connection
	//first send the data packet type and data size so the remote server knows what type of data (and how much) to receive
	int nNumToSend = 2*sizeof(int);
	char *outputBuf = new char[nNumToSend];
	memcpy(outputBuf,&boatData->nPacketType,nNumToSend);
	if (bUseSerial) {//using wireless serial port link
#ifndef _WIN32
		if (write(nSocket,outputBuf,nNumToSend)!=nNumToSend) {
			printf("serial write error\n");
		}
		//for (int i=0;i<nNumToSend;i++) {
		//	serialPutchar(nSocket,outputBuf[i]);
		//}
#endif
	}
	else {//using network link
		if (send(nSocket, outputBuf, nNumToSend, 0) < 0)
		{
			//error sending data
			return false;
		}
	}
	delete []outputBuf;
	//now send the actual data and the checksum byte
	nNumToSend = boatData->nDataSize + 1;
	outputBuf = new char[nNumToSend];
	memcpy(outputBuf,boatData->dataBytes,boatData->nDataSize);
	outputBuf[nNumToSend-1] = boatData->checkSum;
	if (bUseSerial) {//using wireless serial port link
#ifndef _WIN32
		if (nNumToSend>=512) {
			if (!SendLargeSerialData(nSocket,(unsigned char *)outputBuf,nNumToSend,pDiagnostics)) {//send large amount of data out serial port, need to get confirmation after sending each chunk
				//error occurred sending large amount of serial data
				return false;
			}
		}
		else
		{
			if (write(nSocket,outputBuf,nNumToSend)!=nNumToSend) {
				printf("serial write error\n");
			}
			//for (int i = 0; i < nNumToSend; i++)
			//{
			//	serialPutchar(nSocket, outputBuf[i]);
			//}
		}
#endif
	}
	else {//using network link
		if (send(nSocket,outputBuf,nNumToSend,0)<0) {
			//error sending data
			return false;
		}
	}
	delete []outputBuf;
	return true;
}


/**
 * @brief send a large amount of data out the serial port. Function requires the receiver to send a one byte confirmation (0x0d) after receiving each chunk (512 bytes)
 * of data. The receiver can cancel the sending of serial data by sending a 0x0a byte.
 * 
 * @param nSocket The serial port file descriptor used for communications.
 * @param outputBuf The buffer of data to send over the serial port connection.
 * @param nNumToSend The total number of bytes to send.
 * @param pDiagSensor void pointer to a DiagnosticsSensor object that is used in some cases to signal that the program is still running correctly (for example when sending large chunks of data at a time). It can be set to nullptr when sending small amounts of data (< 1 kbyte). 
 * @return true if the data was sent successfully.
 * @return false if there was a problem sending the data.
 */
bool BoatCommand::SendLargeSerialData(int nSocket, unsigned char *outputBuf, int nNumToSend, void *pDiagSensor) {//send large amount of data out serial port, need to get confirmation after sending each chunk
	const int CHUNK_SIZE = 128;//maximum amount of data to send out serial port before getting a confirmation in return that it was received
	const int MAX_NUM_FAILURES = 20;//maximum # of consecutive failed confirmations before giving up
	int nNumSent = 0;
	int nNumFailures = 0;
	int nNumRemaining = nNumToSend;
	int nChunkIndex = 0;//number each chunk that is sent out, starting at zero
	unsigned char chunkBuf[128];//temporary buffer to use for storing each chunk, bytes in chunkBuf are organized as follows:
	//chunkBuf[0] = 'A'
	//chunkBuf[1] = 'M'
	//chunkBuf[2] = 'O'
	//chunkBuf[3] = 'S'
	//chunkBuf[4] = <chunk index, most sig byte>
	//chunkBuf[5] = <chunk index, middle sig byte>
	//chunkBuf[6] = <chunk index, least sig byte>
	//chunkBuf[7] = <chunk data portion size (CRC not included), most sig byte>
	//chunkBuf[8] = <chunk data portion size (CRC not included), least sig byte>
	//chunkBuf[9] = <first data byte>
	//chunkBuf[10] = <2nd data byte>
	//...
	//chunkBuf[9+#data bytes] = <CRC, most sig byte>
	//chunkBuf[10+#data bytes] = <CRC, least sig byte>
#ifndef _WIN32
	DiagnosticsSensor *pDiag = (DiagnosticsSensor *)pDiagSensor;
	while (nNumRemaining>0 && nNumFailures<MAX_NUM_FAILURES) {
		//test
		printf("nNumRemaining = %d, nNumFailures = %d\n",nNumRemaining,nNumFailures);
		//end test
		int nNumInChunk = fillchunk(chunkBuf,nChunkIndex,outputBuf,nNumToSend,nNumSent,CHUNK_SIZE);
		//test
		printf("nNumInChunk = %d\n",nNumInChunk);
		if (nNumInChunk<CHUNK_SIZE) {
			printf("sending last chunk: %d bytes\n",nNumInChunk);	
		}
		//end test
		if (write(nSocket,chunkBuf,nNumInChunk)!=nNumInChunk) {
			printf("serial write error\n");
		}
		//now wait for response (should get 4 bytes back indicating successful reception or an error)
		unsigned int uiStartTime = millis();
		unsigned int uiTimeoutTime = uiStartTime + 1000;
		int nNumAvail = serialDataAvail(nSocket);
		while (nNumAvail<4&&millis()<uiTimeoutTime) {
			nNumAvail = serialDataAvail(nSocket);
			delay(10);
		}
		bool bError = false;
		bool bCancel = false;
		if  (nNumAvail<4) {
			bError = true;
			printf("Timeout waiting for response, %d bytes received.\n",nNumAvail);
			for (int i=0;i<nNumAvail;i++) {
				printf("%d, ",serialGetchar(nSocket));
			}
			printf("\n");
		}
		else {
			int nByte1 = serialGetchar(nSocket);
			int nByte2 = serialGetchar(nSocket);
			int nByte3 = serialGetchar(nSocket);
			int nByte4 = serialGetchar(nSocket);
			printf("bytes received: %d, %d, %d, %d\n",nByte1,nByte2,nByte3,nByte4);
			if (nByte1==0xff||nByte2==0xff) {
				//error occurred
				bError = true;
			}
			else if (nByte1==0xaa||nByte2==0xaa) {
				bCancel = true;
				//test
				printf("command to stop stending stuff.\n");
				//end test
				return false;
			}
			else if (nByte3!=((int)chunkBuf[nNumInChunk-2])) {
				bError = true;
				printf("1st returned checksum byte is wrong.\n");
			}
			else if (nByte4!=((int)chunkBuf[nNumInChunk-1])) {
				bError = true;
				printf("2nd returned checksum byte is wrong.\n");
			}
		}
		if (!bError) {
			nNumFailures = 0;
			nNumSent+=(nNumInChunk-11);
			nNumRemaining-=(nNumInChunk-11);
			nChunkIndex++;
		}
		else {//some problem occurred getting confirmation
			nNumFailures++;
			if (nNumFailures>=MAX_NUM_FAILURES) {
				return false;
			}
		}
		//send signal to indicate that program is still running
		if (pDiag!=nullptr) {
			pDiag->ActivityPulse();
		}
	}
#else 
	//windows implementation
	unsigned char inBuf[4];
	while (nNumRemaining > 0 && nNumFailures < MAX_NUM_FAILURES) {
		int nNumInChunk = fillchunk(chunkBuf, nChunkIndex, outputBuf, nNumToSend, nNumSent, CHUNK_SIZE);
		DWORD dwNumWritten = 0;
		WriteFile((HANDLE)nSocket, chunkBuf, (DWORD)nNumInChunk, &dwNumWritten, NULL);
		if (((int)dwNumWritten) != nNumInChunk) {
			TRACE("serial write error\n");
			return false;
		}
		//now wait for response (should get 4 bytes back indicating successful reception or an error)
		HANDLE hPort = (HANDLE)nSocket;
		DWORD dwNumRead = 0;
		ReadFile(hPort, inBuf, 4, &dwNumRead, NULL);
		bool bError = false;
		bool bCancel = false;
		if (dwNumRead < 4) {
			bError = true;
			TRACE("Timeout waiting for response, %u bytes received.\n", dwNumRead);
		}
		else {
			TRACE("bytes received: %d, %d, %d, %d\n", (int)inBuf[0], (int)inBuf[1], (int)inBuf[2], (int)inBuf[3]);
			if (inBuf[0] == 0xff || inBuf[1] == 0xff) {
				//error occurred
				bError = true;
			}
			else if (inBuf[0] == 0xaa || inBuf[1] == 0xaa) {
				bCancel = true;
				//test
				TRACE("command to stop stending stuff.\n");
				//end test
				return false;
			}
			else if (inBuf[2] != (chunkBuf[nNumInChunk - 2])) {
				bError = true;
				TRACE("1st returned checksum byte is wrong.\n");
			}
			else if (inBuf[3] != (chunkBuf[nNumInChunk - 1])) {
				bError = true;
				TRACE("2nd returned checksum byte is wrong.\n");
			}
		}
		if (!bError) {
			nNumFailures = 0;
			nNumSent += (nNumInChunk - 11);
			nNumRemaining -= (nNumInChunk - 11);
			nChunkIndex++;
		}
		else {//some problem occurred getting confirmation
			nNumFailures++;
			if (nNumFailures >= MAX_NUM_FAILURES) {
				return false;
			}
		}
	}
#endif
	return (nNumFailures<MAX_NUM_FAILURES);
}

 
/**
 * @brief fill a chunk buffer with synchronization and ID bytes at the beginning, a data payload in the middle, and a couple of CRC bytes at the end
 * 
 * @param chunkBuf the returned chunk buffer that is filled from data contained in inputBuf
 * @param nChunkID an ID number that is inserted into chunkBuf and is useful to positively identify the incoming chunk at the receiving end of the connection.
 * @param inputBuf the input buffer from which data is taken to form chunkBuf
 * @param nBufSize the size of the input buffer 
 * @param nBufIndex the index of inputBuf from which data is to be taken and inserted into chunkBuf
 * @param nMaxChunkSize the maximum possible size of chunkBuf
 * @return the # of bytes in chunkBuf, will typically be equal to nMaxChunkSize, except for the last packet of a transmission, which will typically be less
 */
int BoatCommand::fillchunk(unsigned char *chunkBuf,int nChunkID,unsigned char *inputBuf,int nBufSize,int nBufIndex,int nMaxChunkSize) {
	chunkBuf[0] = 'A';
	chunkBuf[1] = 'M';
	chunkBuf[2] = 'O';
	chunkBuf[3] = 'S';
	//chunkBuf[4] = <chunk index, most sig byte>
	chunkBuf[4] = (unsigned char)((nChunkID&0x00ff0000)>>16);
	//chunkBuf[5] = <chunk index, middle sig byte>
	chunkBuf[5] = (unsigned char)((nChunkID&0x0000ff00)>>8);
	//chunkBuf[6] = <chunk index, least sig byte>
	chunkBuf[6] = (unsigned char)(nChunkID&0x000000ff);
	//test
	//printf("nChunkID = %d, chunkBuf[5] = %d, chunkBuf[6] = %d\n",nChunkID, (int)chunkBuf[5],(int)chunkBuf[6]);
	//end ntest
	nMaxChunkSize-=11;//subtract 9 bytes for pre-amble stuff and 2 bytes for CRC bytes
	int nDataPortionSize = min((nBufSize-nBufIndex),nMaxChunkSize);
	
	//chunkBuf[7] = <chunk data portion size (CRC not included), most sig byte>
	chunkBuf[7] = (unsigned char)((nDataPortionSize&0x0000ff00)>8);
	//chunkBuf[8] = <chunk data portion size (CRC not included), least sig byte>
	chunkBuf[8] = (unsigned char)(nDataPortionSize&0x000000ff);
	//chunkBuf[9] = <first data byte>
	//chunkBuf[10] = <2nd data byte>
	//...
	int nChecksum=0;
	for (int i=0;i<nDataPortionSize;i++) {
		chunkBuf[9+i] = inputBuf[nBufIndex+i];
		nChecksum+=(int)(inputBuf[nBufIndex+i]);
	}
	//chunkBuf[9+#data bytes] = <CRC, most sig byte>
	int nTest1 = (nChecksum&0x0000ff00)/256;
	chunkBuf[9+nDataPortionSize] = (unsigned char)nTest1;
	//chunkBuf[10+#data bytes] = <CRC, least sig byte>
	chunkBuf[10+nDataPortionSize] = (unsigned char)(nChecksum&0x000000ff);
	//test
	//printf("chunkBuf[%d] = %d\n",10+nDataPortionSize,(int)chunkBuf[10+nDataPortionSize]);
	//end test
	return 11 + nDataPortionSize;
}

int BoatCommand::Encrypt(char* destBuf, char* sourceBuf, int nSize) {
	const char MIN_CHAR = 33;
	const char MAX_CHAR = 126;
	int nCharRange = MAX_CHAR - MIN_CHAR;
	char keyval[128];
	memset(keyval, 0, 128);
	strcpy(keyval, (char*)AMOSEN_VAL);
	int nKeyLength = strlen(keyval);
	for (int i = 0; i < nSize; i++) {
		char c = sourceBuf[i];
		if (sourceBuf[i] >= MIN_CHAR && sourceBuf[i] <= MAX_CHAR) {
			int nDif = c - MIN_CHAR;
			int j = i % nKeyLength;
			int nKeyDif = keyval[j] - MIN_CHAR;
			int nNewVal = MIN_CHAR + ((nKeyDif + nDif) % nCharRange);
			destBuf[i] = (char)nNewVal;
		}
		else {
			destBuf[i] = sourceBuf[i];//shouldn't get these sorts of characters, but just in case
		}
	}
	//null terminate
	destBuf[nSize] = 0;
	return 0;
}

int BoatCommand::Decrypt(char* destBuf, char* sourceBuf, int nSize) {
	const char MIN_CHAR = 33;
	const char MAX_CHAR = 126;
	int nCharRange = MAX_CHAR - MIN_CHAR;
	char keyval[128];
	memset(keyval, 0, 128);
	strcpy(keyval, (char*)AMOSEN_VAL);
	int nKeyLength = strlen(keyval);
	for (int i = 0; i < nSize; i++) {
		char c = sourceBuf[i];
		if (sourceBuf[i] >= MIN_CHAR && sourceBuf[i] <= MAX_CHAR) {
			int nDif = c - MIN_CHAR;
			int j = i % nKeyLength;
			int nKeyDif = keyval[j] - MIN_CHAR;
			int nNewDif = nDif - nKeyDif;
			if (nNewDif < 0) {
				nNewDif += nCharRange;
			}
			destBuf[i] = (char)(MIN_CHAR + nNewDif);
		}
		else {
			destBuf[i] = sourceBuf[i];//shouldn't get these sorts of characters, but just in case
		}
	}
	//null terminate
	destBuf[nSize] = 0;
	return 0;
}