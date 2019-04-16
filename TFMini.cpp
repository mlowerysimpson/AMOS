/*
port of Peter Jansen's Arduino driver for Benewake TFMini time-of-flight distance sensor to Raspberry Pi
by Murray Lowery-Simpson (August 18, 2018)
This code is open source software in the public domain.

THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

The names of the contributors may not be used to endorse or promote products
derived from this software without specific prior written permission.
*/

#include "TFMini.h"
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>


// Constructor
TFMini::TFMini() { 
	m_fd=0;
}

TFMini::~TFMini() {//destructor
 
}


bool TFMini::begin(int fd) {
  // Store file descriptor for serial port (returned by wiringPi's serialOpen function)
  m_fd = fd;;

  // Clear state
  distance = -1;
  strength = -1;
  state = READY;

  /*
  //Set configuration mode, may need to repeat to get out of streaming mode
  if (!setConfigMode()) {
	  if (!setConfigMode()) {//try 2nd time
		  if (!setConfigMode()) {//try 3rd time
			return false;
		  }
	  }
  }
  //Set standard output mode
  if (!setStandardOutputMode()) {
	  return false;
  }
  //Exit configuration mode
  if (!exitConfigMode()) {
	  return false;
  }*/

  return true;
}


// Public: The main function to measure distance. 
unsigned short TFMini::getDistance() {
  int numMeasurementAttempts = 0;
  while (takeMeasurement() != 0) {
    numMeasurementAttempts += 1;
    if (numMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS) {
      printf("TF Mini error: too many measurement attempts.\n");
      printf("Last error: ");
      if (state == ERROR_SERIAL_NOHEADER)     printf("ERROR_SERIAL_NOHEADER\n");
      if (state == ERROR_SERIAL_BADCHECKSUM)  printf("ERROR_SERIAL_BADCHECKSUM\n");
      if (state == ERROR_SERIAL_TOOMANYTRIES) printf("ERROR_SERIAL_TOOMANYTRIES\n");      
      
      state = ERROR_SERIAL_TOOMANYTRIES;
      distance = -1;
      strength = -1;      
      return -1;      
    }
  }

  if (state == MEASUREMENT_OK) {
	  return distance;
  } 
  return -1;
}

// Public: Return the most recent signal strength measuremenet from the TF Mini
unsigned short TFMini::getRecentSignalStrength() {
  return strength;
}



bool TFMini::setStandardOutputMode() {//set to use the standard output mode
  //flush I/O buffer
  serialFlush(m_fd); 
  serialPutchar(m_fd, (unsigned char)0x42);
  serialPutchar(m_fd, (unsigned char)0x57);
  serialPutchar(m_fd, (unsigned char)0x02);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x01);
  serialPutchar(m_fd, (unsigned char)0x06);
  //check for echoed back characters, indicating success
  unsigned int timeoutVal = millis() + 3000;//timeout after 3 seconds
  int nNumAvail = serialDataAvail(m_fd);//number of bytes available in serial RX buffer
  while (nNumAvail<8&&millis()<timeoutVal) {
	nNumAvail = serialDataAvail(m_fd);
  }
  if (nNumAvail<8) {
	  printf("Timeout waiting for echo response for standard output mode, nNumAvail = %d\n",nNumAvail);
	  return false;
  }
  unsigned char rxBuf[8];
  for (int i=0;i<8;i++) {
	  rxBuf[i] = (unsigned char )serialGetchar(m_fd);
  }
  printf("bytes = %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n",rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7]); 
  //should be the following echoed back to indicate that the device has been properly set to standard output mode: 42 57 02 01 00 00 01 06
  if (rxBuf[0]!=0x42) {
	printf("Invalid byte 1 of setStandardOutputMode echo response.\n");
	return false;
  }
  if (rxBuf[1]!=0x57) {
	printf("Invalid byte 2 of setStandardOutputMode echo response.\n");
	return false;
  }
  if (rxBuf[2]!=0x02) {
	printf("Invalid byte 3 of setStandardOutputMode echo response.\n");
	return false;
  }	
  if (rxBuf[3]!=0x01) {
	printf("Invalid byte 4 of setStandardOutputMode echo response.\n");
	return false;
  }
  if (rxBuf[4]!=0x00) {
	printf("Invalid byte 5 of setStandardOutputMode echo response.\n");
	return false;
  }
  if (rxBuf[5]!=0x00) {
	printf("Invalid byte 6 of setStandardOutputMode echo response.\n");
	return false;
  }
  if (rxBuf[6]!=0x01) {
	printf("Invalid byte 7 of setStandardOutputMode echo response.\n");
	return false;
  }
  if (rxBuf[7]!=0x06) {
	printf("Invalid byte 8 of setStandardOutputMode echo response.\n");
	return false;
  }
  return true;
}


bool TFMini::setConfigMode() {//go into configuration mode
  //flush I/O buffer
  serialFlush(m_fd);
  // advanced parameter configuration mode
  serialPutchar(m_fd, (unsigned char)0x42);
  serialPutchar(m_fd, (unsigned char)0x57);
  serialPutchar(m_fd, (unsigned char)0x02);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x01);
  serialPutchar(m_fd, (unsigned char)0x02);  
  //check for echoed back characters, indicating success
  unsigned int timeoutVal = millis() + 3000;//timeout after 3 seconds
  int nNumAvail = serialDataAvail(m_fd);//number of bytes available in serial RX buffer
  while (nNumAvail<8&&millis()<timeoutVal) {
	nNumAvail = serialDataAvail(m_fd);
  }
  if (nNumAvail<8) {
	  printf("Timeout waiting for echo response, nNumAvail = %d\n",nNumAvail);
	  return false;
  }
  unsigned char rxBuf[8];
  for (int i=0;i<8;i++) {
	  rxBuf[i] = (unsigned char )serialGetchar(m_fd);
  }
  printf("bytes = %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n",rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7]); 
  //should be the following echoed back to indicate that the device has been properly set to configuration mode: 42 57 02 01 00 00 01 02
  if (rxBuf[0]!=0x42) {
	printf("Invalid byte 1 of setConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[1]!=0x57) {
	printf("Invalid byte 2 of setConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[2]!=0x02) {
	printf("Invalid byte 3 of setConfigMode echo response.\n");
	return false;
  }	
  if (rxBuf[3]!=0x01) {
	printf("Invalid byte 4 of setConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[4]!=0x00) {
	printf("Invalid byte 5 of setConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[5]!=0x00) {
	printf("Invalid byte 6 of setConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[6]!=0x01) {
	printf("Invalid byte 7 of setConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[7]!=0x02) {
	printf("Invalid byte 8 of setConfigMode echo response.\n");
	return false;
  }
  return true;
}

// Set single scan mode (external trigger)
void TFMini::setSingleScanMode() {
  setConfigMode();
  // setting trigger source to external
  serialPutchar(m_fd, (unsigned char)0x42);
  serialPutchar(m_fd, (unsigned char)0x57);
  serialPutchar(m_fd, (unsigned char)0x02);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x40);
}

// Send external trigger
void TFMini::externalTrigger() {
  setConfigMode();      
  // send trigger
  serialPutchar(m_fd, (unsigned char)0x42);
  serialPutchar(m_fd, (unsigned char)0x57);
  serialPutchar(m_fd, (unsigned char)0x02);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x41);
}

// Private: Handles the low-level bits of communicating with the TFMini, and detecting some communication errors.
int TFMini::takeMeasurement() {
  //flush I/O buffer
  serialFlush(m_fd); 
  int numCharsRead = 0;
  unsigned char lastChar = 0x00;  
  
  // Step 1: Read the serial stream until we see the beginning of the TF Mini header, or we timeout reading too many characters.
  while (1) {
    if (serialDataAvail(m_fd)>0) {      
      unsigned char curChar = (unsigned char)serialGetchar(m_fd);
      if ((lastChar == 0x59) && (curChar == 0x59)) {
        // Break to begin frame
        break;
      }
	  else {
        // We have not seen two 0x59's in a row -- store the current character and continue reading.         
        lastChar = curChar;
        numCharsRead += 1; 
      }           
    }
    // Error detection:  If we read more than X characters without finding a frame header, then it's likely there is an issue with 
    // the Serial connection, and we should timeout and throw an error. 
    if (numCharsRead > TFMINI_MAXBYTESBEFOREHEADER) {
      state = ERROR_SERIAL_NOHEADER;
      distance = -1;
      strength = -1;     
      if (TFMINI_DEBUGMODE == 1) printf("ERROR: no header, %d chars read.\n",numCharsRead);
      return -1;      
    }
  }
  // Step 2: Read one frame from the TFMini
  unsigned char frame[TFMINI_FRAME_SIZE];

  unsigned char checksum = 0x59 + 0x59;
  for (int i=0; i<TFMINI_FRAME_SIZE; i++) {
    // Read one character
    while (!serialDataAvail(m_fd)) {
      // wait for a character to become available
    }    
    frame[i] = (unsigned char)serialGetchar(m_fd);

    // Store running checksum
    if (i < TFMINI_FRAME_SIZE-2) {
      checksum += frame[i];
    }
  }

  // Step 2A: Compare checksum
  // Last byte in the frame is an 8-bit checksum 
  unsigned char checksumByte = frame[TFMINI_FRAME_SIZE-1];
  if (checksum != checksumByte) {
    state = ERROR_SERIAL_BADCHECKSUM;
    distance = -1;
    strength = -1;
    if (TFMINI_DEBUGMODE == 1) printf("ERROR: bad checksum.\n");
    return -1;
  }


  // Step 3: Interpret frame
  unsigned short dist = (frame[1] << 8) + frame[0];
  unsigned short st = (frame[3] << 8) + frame[2];
  unsigned char reserved = frame[4];
  unsigned char originalSignalQuality = frame[5];


  // Step 4: Store values
  distance = dist;
  strength = st;
  state = MEASUREMENT_OK;

  // Return success
  return 0;  
}

bool TFMini::exitConfigMode() {//exit the configuration mode
	//flush I/O buffer
  serialFlush(m_fd);
  // exit configuration mode
  serialPutchar(m_fd, (unsigned char)0x42);
  serialPutchar(m_fd, (unsigned char)0x57);
  serialPutchar(m_fd, (unsigned char)0x02);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x00);
  serialPutchar(m_fd, (unsigned char)0x02);  
  //check for echoed back characters, indicating success
  unsigned int timeoutVal = millis() + 3000;//timeout after 3 seconds
  int nNumAvail = serialDataAvail(m_fd);//number of bytes available in serial RX buffer
  while (nNumAvail<8&&millis()<timeoutVal) {
	nNumAvail = serialDataAvail(m_fd);
  }
  if (nNumAvail<8) {
	  printf("Timeout waiting for echo response, nNumAvail = %d\n",nNumAvail);
	  return false;
  }
  unsigned char rxBuf[8];
  for (int i=0;i<8;i++) {
	  rxBuf[i] = (unsigned char )serialGetchar(m_fd);
  }
  printf("bytes = %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n",rxBuf[0],rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5],rxBuf[6],rxBuf[7]); 
  //should be the following echoed back to indicate that the device has been properly set to configuration mode: 42 57 02 01 00 00 00 02
  if (rxBuf[0]!=0x42) {
	printf("Invalid byte 1 of exitConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[1]!=0x57) {
	printf("Invalid byte 2 of exitConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[2]!=0x02) {
	printf("Invalid byte 3 of exitConfigMode echo response.\n");
	return false;
  }	
  if (rxBuf[3]!=0x01) {
	printf("Invalid byte 4 of exitConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[4]!=0x00) {
	printf("Invalid byte 5 of exitConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[5]!=0x00) {
	printf("Invalid byte 6 of exitConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[6]!=0x00) {
	printf("Invalid byte 7 of exitConfigMode echo response.\n");
	return false;
  }
  if (rxBuf[7]!=0x02) {
	printf("Invalid byte 8 of exitConfigMode echo response.\n");
	return false;
  }
  return true;
}