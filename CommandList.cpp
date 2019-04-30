//comment out the next line for non-Windows builds, or include a dummy stdafx.h file
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
		pAMOSData->nDataSize = 5*sizeof(float) + sizeof(int);
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
		for (int i=0;i<nNumToSend;i++) {
			serialPutchar(nSocket,outputBuf[i]);
		}
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
			for (int i = 0; i < nNumToSend; i++)
			{
				serialPutchar(nSocket, outputBuf[i]);
			}
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
	const int MAX_NUM_FAILURES = 5;//maximum # of consecutive failed confirmations before giving up
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
		if (nNumInChunk<CHUNK_SIZE) {
			printf("sending last chunk: %d bytes\n",nNumInChunk);	
		}
		//end test
		for (int i=0;i<nNumInChunk;i++) {
			//test
			//printf("outputBuf[%d] = %d\n",nNumSent+i,(int)outputBuf[nNumSent+i]);
			//end test
			serialPutchar(nSocket, chunkBuf[i]);
			//test
			delay(1);
			//end test
		}
		//now wait for response (should get 2 bytes back that are equal to the checksum bytes that were sent)
		int nResponse1=0, nResponse2=0;
		nResponse1 = serialGetchar(nSocket);
		//test 
		printf("nResponse1 = %d\n",nResponse1);
		//end test
		bool bResponseOK = false;
		bool bGot2Bytes = false;
		if  (nResponse1<0) {
			printf("Timeout waiting for 1st byte of response.\n");
		}
		else if (nResponse1!=((int)chunkBuf[nNumInChunk-2])) {
			printf("1st byte of response is incorrect.\n");
		}
		else {
			//1st byte of response was ok, now read in 2nd byte
			nResponse2 = serialGetchar(nSocket);
			if (nResponse2<0) {
				printf("Tiemout waiting for 2nd byte of response.\n");
			}
			else if (nResponse2!=((int)chunkBuf[nNumInChunk-1])) {
				bGot2Bytes = true;
				printf("2nd byte of response is incorrect.\n");
			}
			else {
				bGot2Bytes = true;
				//both response bytes were ok
				bResponseOK = true;
			}
		}
		if (bGot2Bytes&&nResponse1==0x0a&&nResponse2==0x0a) {
			//if we get a 3rd 0x0a byte, then this is the command to stop sending stuff
			int nResponse3 = serialGetchar(nSocket);
			if (nResponse3==0x0a) {//download has been aborted, so stop sending stuff
				//test
				printf("command to stop stending stuff.\n");
				//end test
				return false;
			}
		}
		if (bResponseOK) {
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
		pDiag->ActivityPulse();
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
	printf("nChunkID = %d, chunkBuf[5] = %d, chunkBuf[6] = %d\n",nChunkID, (int)chunkBuf[5],(int)chunkBuf[6]);
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
	unsigned int uiChecksum=0;
	for (int i=0;i<nDataPortionSize;i++) {
		chunkBuf[9+i] = inputBuf[nBufIndex+i];
		uiChecksum+=(unsigned int)(inputBuf[nBufIndex+i]);
	}
	//chunkBuf[9+#data bytes] = <CRC, most sig byte>
	unsigned int uiTest1 = uiChecksum&0x0000ff00;
	unsigned int uiTest2 = uiTest1>>8;
	chunkBuf[9+nDataPortionSize] = (unsigned char)uiTest2;
	//chunkBuf[10+#data bytes] = <CRC, least sig byte>
	chunkBuf[10+nDataPortionSize] = (unsigned char)(uiChecksum&0x000000ff);
	//test
	printf("chunkBuf[%d] = %d\n",10+nDataPortionSize,(int)chunkBuf[10+nDataPortionSize]);
	//end test
	return 11 + nDataPortionSize;
}