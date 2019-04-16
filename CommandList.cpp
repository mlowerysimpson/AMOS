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
#ifndef _WIN32
	const int CHUNK_SIZE = 512;//maximum amount of data to send out serial port before getting a confirmation in return that it was received
	const int MAX_NUM_FAILURES = 3;//maximum # of failed confirmations before giving up
	int nNumSent = 0;
	int nNumFailures = 0;
	int nNumRemaining = nNumToSend;
	DiagnosticsSensor *pDiag = (DiagnosticsSensor *)pDiagSensor;
	while (nNumRemaining>0 && nNumFailures<MAX_NUM_FAILURES) {
		int nNumInChunk = min(nNumRemaining, CHUNK_SIZE);
		for (int i=0;i<nNumInChunk;i++) {
			serialPutchar(nSocket, outputBuf[nNumSent+i]);
			//test
			delay(1);
			//end test
		}
		//now wait for response (0x0d)
		int nResponse = serialGetchar(nSocket);
		if (nResponse==0) {//receiver did not receive the expected number of bytes, need to re-send chunk that was just sent
			//but first need to send sync bytes to make sure that receiver is synced up properly
			//test
			printf("Receiver did not get all bytes, sending sync bytes.\n");
			//end test
			for (int i=0;i<NUM_RESEND_SYNC_BYTES;i++) {
				serialPutchar(nSocket, (unsigned char)i);

			}	
			continue;
		}
		else if (nResponse==0x0a) {//command to stop sending stuff
			//echo back 0x0a character
			//test
			printf("command to stop stending stuff.\n");
			//end test
			serialPutchar(nSocket, (unsigned char)0x0a);
			return false;
		}
		else if (nResponse!=0x0d) {//got invalid response, or no response at all
			printf("nResponse = %d",nResponse);
			nNumFailures++;
			if (nNumFailures>=MAX_NUM_FAILURES) {
				return false;
			}
			//send sync bytes and then try again
			for (int i=0;i<NUM_RESEND_SYNC_BYTES;i++) {
				serialPutchar(nSocket, (unsigned char)i);
			}	
			continue;
		}
		nNumSent+=nNumInChunk;
		nNumRemaining-=nNumInChunk;
		//send signal to indicate that program is still running
		pDiag->ActivityPulse();
		//test
		//printf("nNumSent = %d\n",nNumSent);
		//end test
	}
#endif
	return true;
}