#include <stdio.h>
#include <wiringPi.h>
#include "SensorDeploy.h"
#include "filedata.h"


/**
 * @brief Construct a new SensorDeploy object for controlling the servo motor that is used to deploy and retract sensors into and from the surrounding water
 * 
 */
SensorDeploy::SensorDeploy(char *rootFolder, bool bInitializeWiringPi) {//SensorDeploy constructor
    m_rootFolder = nullptr;
    m_bDeployed = false;
	if (rootFolder!=nullptr) {
        m_rootFolder = new char[strlen(rootFolder)+1];
        strcpy(m_rootFolder,rootFolder);
        GetSettings();
    }
    if (bInitializeWiringPi) {
        OneTimeSetup();
    }
}

/**
 * @brief destructor
 * 
 */
SensorDeploy::~SensorDeploy() {//destructor
	if (m_rootFolder!=nullptr) {
        delete m_rootFolder;
        m_rootFolder = nullptr;
    }
}


/**
 * @brief static method that should be called prior to creating and using a Rudder object
 * 
 */
void SensorDeploy::OneTimeSetup() {//setup procedure to be performed once per power-cycle of the deployment servo motor
	wiringPiSetupGpio();//need to call this before doing anything with GPIO
    pinMode(DEPLOYMENT_PULSE_PIN,OUTPUT);
    digitalWrite(DEPLOYMENT_PULSE_PIN,0);//set initial output of pin to low
}

/**
 * @brief get sensor arm deployment status (if available) from the prefs.txt file found in the root program folder
 * 
 */
void SensorDeploy::GetSettings() {
    if (m_rootFolder==nullptr) return;
    char prefsFilename[FILENAME_MAX];
    sprintf(prefsFilename,"%s/prefs.txt",m_rootFolder);
    filedata prefsFile(prefsFilename);
    if (prefsFile.FindString("[sensors]","deployed")<=0) {
        return;//deployment state is not available in file
    } 
    int nDeployed = prefsFile.getInteger("[sensors]","deployed");
    if (nDeployed>0) {
        m_bDeployed = true;
    }
    else {
        m_bDeployed = false;
    }
}

void SensorDeploy::SaveSettings() {
    if (m_rootFolder==nullptr) return;
    char prefsFilename[FILENAME_MAX];
    sprintf(prefsFilename,"%s/prefs.txt",m_rootFolder);
    filedata prefsFile(prefsFilename);    
    prefsFile.writeData("[sensors]","deployed",(int)m_bDeployed);
}

void SensorDeploy::Deploy() {//move the servo motor to deploy the sensors into the water
    pinMode(DEPLOYMENT_PULSE_PIN,OUTPUT);//make sure pin is set to output
    if (m_bDeployed) {
        //already deployed, just make sure that servo is at the correct angle
        Pulse(MIN_SD_PULSE_TIME, 5000);//send pulses corresponding to deployed angle for 5 seconds
    }
    else {//gradually move through increasing angles to swing the sensor armature into the water
        for (int i=MAX_SD_PULSE_TIME;i>=MIN_SD_PULSE_TIME;i-=5) {
            Pulse(i,10);//send pulses corresponding to a specific angle for 10 ms
        }
    }
    m_bDeployed = true;
    //save setting to prefs.txt file
    SaveSettings();
    pinMode(DEPLOYMENT_PULSE_PIN,INPUT);//set pin to input to save power
}

void SensorDeploy::Retract() {//move the servo motor to retract the sensor from the water back onto the boat deck
    pinMode(DEPLOYMENT_PULSE_PIN,OUTPUT);//make sure pin is set to output
    if (!m_bDeployed) {
        //already retracted, just make sure that servo is at the correct angle
        Pulse(MAX_SD_PULSE_TIME, 5000);//send pulses corresponding to retracted angle for 5 seconds
    }
    else {//gradually move through decreasing angles to swing the sensor armature out of the water and back onto the boat deck
        for (int i=MIN_SD_PULSE_TIME;i<=MAX_SD_PULSE_TIME;i+=5) {
            Pulse(i,10);//send pulses corresponding to a specific angle for 10 ms
        }
    }
    m_bDeployed = false;
    //save setting to prefs.txt file
    SaveSettings();
    pinMode(DEPLOYMENT_PULSE_PIN,INPUT);//set pin to input to save power
}

void SensorDeploy::Pulse(int nPulseTimeMicroseconds, int nDurationMilliseconds) {//send pulses of width nPulseTimeMicroseconds for a duration of nDurationMilliseconds
    unsigned int uiMoveEndTime = millis() + nDurationMilliseconds;
    while (millis()<uiMoveEndTime) {
        digitalWrite(DEPLOYMENT_PULSE_PIN,1);
        usleep(nPulseTimeMicroseconds);
        digitalWrite(DEPLOYMENT_PULSE_PIN,0);
        unsigned int uiWaitTime = SD_PULSE_INTERVAL - nPulseTimeMicroseconds;
        usleep(uiWaitTime);
    }
}
