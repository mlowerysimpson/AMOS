//DiagnosticsSensor.cpp -- implementation file for DiagnosticsSensor class
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "AToD.h"
#include "DiagnosticsSensor.h"

/**
 * @brief Construct a new Diagnostics Sensor:: Diagnostics Sensor object
 * 
 * @param pAToD void pointer to AToD object used for getting high precision A to D measurements.
 */
DiagnosticsSensor::DiagnosticsSensor(AToD *pAToD) : Sensor(pAToD) {
    m_fd = 0;
    m_bOpenedPort = false;
    m_nActivityState = 0;
    m_uiLastActivityPulseTime = 0;
}

DiagnosticsSensor::~DiagnosticsSensor() {

}

/**
 * @brief specify a serial port file descriptor to use for communications with the diagnostics hardware
 * 
 * @param fd the file descriptor to the serial port (as returned from the serialOpen function) that is being used for communications
 */
void DiagnosticsSensor::SetSerportHandle(int fd) {
    if (m_bOpenedPort) {
        serialClose(m_fd);
        m_bOpenedPort = false;
    }
    m_fd = fd;
}

/**
 * @brief return an estimate of the amount of current being consumed from the +12 V power supply
 * 
 * @param fCurrent an estimate of the amount of current being consumed (in Amps) is returned in this variable if the function was successful (i.e. the return value is true). If the function was unsuccessful, the return value is false, and the returned value is zero.
 * @return true if an estimate of the current in Amps was successfully returned.
 * @return false if some sort of error occurred trying to get the current.
 */
bool DiagnosticsSensor::GetCurrentDraw(float &fCurrent) {
    fCurrent=0;
    if (m_fd==0) {
        m_fd = OpenDefaultPort();
        if (m_fd==0) {
            printf("Error trying to open default serial port.\n");
            return false;
        }
        m_bOpenedPort = true;
    }
    //flush serial port
    serialFlush(m_fd);
    //send carriage return, followed by "current" followed by carriage return out serial port to get estimate of current
    //should receive a response of the form: "Current: ###" where ### is a value in counts (10-bit)
    serialPutchar(m_fd, (unsigned char)0x0d);
    serialPutchar(m_fd, (unsigned char)'c');
    serialPutchar(m_fd, (unsigned char)'u');
    serialPutchar(m_fd, (unsigned char)'r');
    serialPutchar(m_fd, (unsigned char)'r');
    serialPutchar(m_fd, (unsigned char)'e');
    serialPutchar(m_fd, (unsigned char)'n');
    serialPutchar(m_fd, (unsigned char)'t');
    serialPutchar(m_fd, (unsigned char)0x0d);
    char response[128];
    memset(response,0,128);
    unsigned int uiTimeoutTime = millis() + 2000;//wait up to 2 seconds for a response
    int i=0;
    bool bGotResponse = false;
    while (millis() < uiTimeoutTime) {
        int nNumAvail = serialDataAvail(m_fd);
        if (nNumAvail>0) {
            response[i] = (char)serialGetchar(m_fd);
            if (response[i]==0x0a) {
                response[i] = 0;//change to null terminator
                bGotResponse = true;
                break;//exit while loop
            }
            i++;
        }
    }
    if (!bGotResponse) {//timed out, did not get any response
        //test
        printf("Timed out, did not get any response.\n");
        //end test
        return false;
    }
    //parse response 
    int nNumCounts = 0;

    if (sscanf(response,"Current = %d",&nNumCounts)<1) {//error parsing response
        return false;
    }
    //convert counts to amps
    fCurrent = ConvertCountsToAmps(nNumCounts);
    return true;
}

int DiagnosticsSensor::OpenDefaultPort() {//tries to open the default serial port, and if successful returns a file descriptor to that serial port
    const int BAUD_RATE = 9600;
    m_bOpenedPort = false;
    int fd = serialOpen((char *)DEFAULT_DIAG_PORT, BAUD_RATE);
    return fd;
}

float DiagnosticsSensor::ConvertCountsToAmps(int nNumCounts) {
    //convert output of current sensor (ACS712) from counts to Amps
    //the output of the current sensor (ACS712) goes through a resistive divider (10k and 22k resistor) before being sampled by the 10-bit A to D on the RFU220.
    const float ZERO_AMPS_OUTPUT = 415;//value in A to D counts corresponding to zero current (@20 deg C)
    const float ERR_TOLERANCE = 6;//error tolerance in counts (outputs within plus / minus this amount of ZERO_AMPS_OUTPUT can't be trusted, need to assume some values)
    const float RFU220_VREF = 1.6;//reference voltage (V) on the RFU220
    const float NOMINAL_AMOS_CURRENT = 0.55;//nominal current required by AMOS when just the Pi and RF220SU are running, with +12 V bus power applied, but propeller and rudder not moving
    float COUNTS_PER_V = 1024 / RFU220_VREF;
    float CURRENT_PER_V = 10 * (10.0 + 22.0) / 10.0;//10 amps output per volt, with the 10k and 22k resistor divider 
    float CURRENT_PER_COUNT = CURRENT_PER_V / COUNTS_PER_V;
    
    float fLowToleranceLimit = ZERO_AMPS_OUTPUT - ERR_TOLERANCE;
    float fHighToleranceLimit = ZERO_AMPS_OUTPUT + ERR_TOLERANCE;
    //test
    printf("current counts = %d\n",nNumCounts);
    //end test

    float fCurrent = NOMINAL_AMOS_CURRENT;
    if (nNumCounts<fLowToleranceLimit) {//must have at least some charging from solar panel
        fCurrent -= ((fLowToleranceLimit - nNumCounts)*CURRENT_PER_COUNT);
    }
    else if (nNumCounts > fHighToleranceLimit) {
        fCurrent += ((nNumCounts - fHighToleranceLimit)*CURRENT_PER_COUNT);
    }
    return fCurrent;
}

/**
 * @brief return true if there is sufficient sunlight (as measured by the differential voltage at the + and - solar input terminals of the charge controller) to charge the battery.
 * 
 * @return true if the battery is being charged by sunlight.
 * @return false if there is insufficient sunlight to charge the battery
 */
bool DiagnosticsSensor::IsSolarCharging() {
    if (m_fd==0) {
        m_fd = OpenDefaultPort();
        if (m_fd==0) {
            printf("Error trying to open default serial port.\n");
            return false;
        }
        m_bOpenedPort = true;
    }
    //flush serial port
    serialFlush(m_fd);
    //send carriage return, followed by 's' followed by carriage return out serial port to get measured values in counts for the voltages at the + and - solar charger inputs
    //note that these voltage are stepped down by resistor dividers, i.e. 5K in series with 100K, so voltage level is ~ 21x less than actual. The default voltage reference on the RFU220SU is 1.6 volts.
    //There is also resistance associated with the solar charge controller itself, so this makes the voltage measurement somewhat uncertain. 
    //This function simply uses the voltage at the negative terminal, and returns true if it is equal to zero counts.
    serialPutchar(m_fd, (unsigned char)0x0d);
    serialPutchar(m_fd, (unsigned char)'s');
    serialPutchar(m_fd, (unsigned char)'o');
    serialPutchar(m_fd, (unsigned char)'l');
    serialPutchar(m_fd, (unsigned char)'a');
    serialPutchar(m_fd, (unsigned char)'r');

    serialPutchar(m_fd, (unsigned char)0x0d);
    char line1[128];//first line of response
    char line2[128];//second line of response
    memset(line1,0,128);
    memset(line2,0,128);
    unsigned int uiTimeoutTime = millis() + 2000;//wait up to 2 seconds for a response
    int i=0;
    bool bGotLine1=false;
    bool bGotLine2=false;
    while (millis() < uiTimeoutTime&&!bGotLine2&&i<128) {
        int nNumAvail = serialDataAvail(m_fd);
        if (nNumAvail>0) {
            if (!bGotLine1) {
                line1[i] = (char)serialGetchar(m_fd);
                if (line1[i]==0x0a) {
                   line1[i] = 0;//change to null terminator
                   bGotLine1 = true;
                   i=0;
                }
            }
            else {
                line2[i]=(char)serialGetchar(m_fd);
                if (line2[i]==0x0a) {
                    line2[i]=0;//change to null terminator
                    bGotLine2=true;
                    break;
                }
            }
            i++;
        }
    }
    if (!bGotLine2) {//timed out, did not get a valid response
        //test
        printf("Timed out, did not get solar charge data.\n");
        //end test
        return false;
    }
    //parse response of voltage at neg terminal
    int nNumCountsNegTerminal = -1;
    if (sscanf(line1,"Solarneg = %d",&nNumCountsNegTerminal)<1) {//error parsing response
        //test
        printf("Error parsing negative terminal response.\n");
        //end test
        return false;
    }
    return (nNumCountsNegTerminal==0);
}


/**
 * @brief send command to RFU220SU to power down this computer (i.e. the Raspberry Pi module of AMOS)
 * 
 * @param nSleepTimeSec the length of time to power down AMOS in seconds
 * @return true if the command is successfully sent to the RFU220SU to power down this computer, false otherwise.
 */
bool DiagnosticsSensor::EnterSleepMode(int nSleepTimeSec) {
    if (m_fd==0) {
        m_fd = OpenDefaultPort();
        if (m_fd==0) {
            printf("Error trying to open default serial port.\n");
            return false;
        }
        m_bOpenedPort = true;
    }
    int nSleepTimeMinutes = nSleepTimeSec / 60 - 1;//send the length of time in minutes to the RFU220SU (since it can only resolve integers up to 32767)
    int nSecondsRemainder = 60 + nSleepTimeSec % 60;//remainder portion (in seconds), just pause for this time
    //test
    printf("delay for %d seconds...\n",nSecondsRemainder);
    //end test
    delay((unsigned int)(nSecondsRemainder*1000));//sleep for the seconds portion
    if (nSleepTimeMinutes>0) {//Raspberry Pi (i.e. this computer) will be powered down, need to send command to RFU220SU to tell it when to wake Pi back up
        //test
        printf("About to go to sleep for %d minutes.\n",nSleepTimeMinutes);
        //end test
        char szMinutes[16];
        memset(szMinutes,0,16);
        sprintf(szMinutes,"%d",nSleepTimeMinutes);
        int nStrLength = strlen(szMinutes); 
        //send carriage return, followed by 'd' followed by characters representing the length of time in minutes to go to sleep, followed by a carriage return out serial port
        serialPutchar(m_fd, (unsigned char)0x0d);
        serialPutchar(m_fd, (unsigned char)'d');
        serialPutchar(m_fd, (unsigned char)'o');
        serialPutchar(m_fd, (unsigned char)'w');
        serialPutchar(m_fd, (unsigned char)'n');
        for (int i=0;i<nStrLength;i++) {
            serialPutchar(m_fd, (unsigned char)szMinutes[i]);
        }
        serialPutchar(m_fd, (unsigned char)0x0d);
    }
    return true;
}

/**
 * @brief return the text header that should be used when saving diagnostics data to a sensor data file, calling function is responsible for deleting returned text
 * 
 * @return char* header text to be used when saving diagnostics info to a sensor data file. The calling function should delete the returned pointer after it is finished with it.
 */
char * DiagnosticsSensor::GetSensorFileHeader() {
    char *szHeader = "Voltage(V), Current(A), RH(%%), BoatTemp2(degC), WirelessRX(dBm), Solar, ";
    int nHeaderLength = strlen(szHeader);
    char *szRetval = new char[nHeaderLength+1];
    strcpy(szRetval,szHeader);
    return szRetval;
}

/**
 * @brief do an A to D measurement of the battery voltage and return the result
 * 
 * @param fBatteryVoltage the returned battery voltage (if successfully obtained from the A to D).
 * @return true if the battery voltage was successfully obtained from the A to D.
 * @return false if there was an error trying to get the battery voltage from the A to D.
 */
bool DiagnosticsSensor::GetBatteryVoltage(float &fBatteryVoltage) {
    if (!this->m_pAToD) {
        return false;
    }
    double dBatteryVoltage=0.0;
    if (m_pAToD->GetBatteryVoltage(dBatteryVoltage)) {
        fBatteryVoltage = (float)dBatteryVoltage;
        return true;
    }
    return false;
}

/**
 * @brief Get the relative humidity and temperature from the DHT22 sensor module inside the main Raspberry Pi compartment of AMOS. 
 * 
 * @param fHumidity the returned relative humidity measured by the DHT22 sensor module, expressed as a percentage (between 0 and 100).
 * @param fTemp the returned temperature measured by the DHT22 sensor module, expressed in degrees C.
 * @return true if the relative humidity and temperature were obtained successfully from the DHT22.
 * @return false if there was a problem getting the relative humidity and / or temperature from the DHT22. This may occur on initial calls to this function if valid data has not yet been received from the DHT22.
 */
bool DiagnosticsSensor::GetHumidityAndTemp(float &fHumidity, float &fTemp) {
    if (!m_humiditySensor.GetHumidity(fHumidity)) {
        return false;
    }
    if (!m_humiditySensor.GetTemperature(fTemp)) {
        return false;
    }
    return true;
}

/**
 * @brief Get the received power level of the serial radio in dBm
 * 
 * @param fRXPower the received power level of the serial wireless radio on AMOS, measured in dBm.
 * @return true if the received power level was successfully obtained
 * @return false if there was a problem getting the received power level from the serial radio
 */
bool DiagnosticsSensor::GetWirelessRXPower(float &fRXPower) {//
    fRXPower=0;
    if (m_fd==0) {
        m_fd = OpenDefaultPort();
        if (m_fd==0) {
            printf("Error trying to open default serial port.\n");
            return false;
        }
        m_bOpenedPort = true;
    }
    //flush serial port
    serialFlush(m_fd);
    //send carriage return, followed by "power" followed by carriage return out serial port to get estimate of received radio power
    //should receive a response of the form: "Link: ### dBm" where ### is a value in dBm. 
    //but if there is no wireless link, or the link quality is very poor, might get a response of: "Link: n.A."
    serialPutchar(m_fd, (unsigned char)0x0d);
    serialPutchar(m_fd, (unsigned char)'p');
    serialPutchar(m_fd, (unsigned char)'o');
    serialPutchar(m_fd, (unsigned char)'w');
    serialPutchar(m_fd, (unsigned char)'e');
    serialPutchar(m_fd, (unsigned char)'r');
    serialPutchar(m_fd, (unsigned char)0x0d);
    char response[128];
    memset(response,0,128);
    unsigned int uiTimeoutTime = millis() + 2000;//wait up to 2 seconds for a response
    int i=0;
    bool bGotResponse = false;
    while (millis() < uiTimeoutTime) {
        int nNumAvail = serialDataAvail(m_fd);
        if (nNumAvail>0) {
            response[i] = (char)serialGetchar(m_fd);
            if (response[i]==0x0a) {
                response[i] = 0;//change to null terminator
                bGotResponse = true;
                break;//exit while loop
            }
            i++;
        }
    }
    if (!bGotResponse) {//timed out, did not get any response
        //test
        printf("Timed out, did not get any response.\n");
        //end test
        return false;
    }
    //parse response 
    int nReceivedPower = 0;
    if (sscanf(response,"Link: %d",&nReceivedPower)<1) {//error parsing response
        return false;
    }
    fRXPower = nReceivedPower;
    return true;
}

/**
 * @brief send activity pulse out on activity pin to indicate that program is running
 * 
 */
void DiagnosticsSensor::ActivityPulse() {
	//change state of activity pin no more often than once per second
	unsigned int uiCurrentTime = millis();
	if ((uiCurrentTime - m_uiLastActivityPulseTime)<1000) {
		return;//already changed state of activity pin within the last second
	}
	m_nActivityState = (m_nActivityState+1)%2;
	digitalWrite(ACTIVITY_PIN,m_nActivityState);
	m_uiLastActivityPulseTime = uiCurrentTime;
}


/**
 * @brief send short burst of pulses out activity pin to indicate that program was closed normally
 * 
 */
void DiagnosticsSensor::ActivityBurst() {
	const int NUM_PULSES = 10;
	for (int i=0;i<NUM_PULSES;i++) {
		m_nActivityState = (m_nActivityState+1)%2;
		digitalWrite(ACTIVITY_PIN,m_nActivityState);	
		delay(25);//pause for 25 ms		
	}
}

