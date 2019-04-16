//LeakSensor.cpp
//Impementation file for LeakSensor class

#include "LeakSensor.h"
#include "ShipLog.h"
#include <wiringPi.h>



LeakSensor::LeakSensor() : Sensor(nullptr) {//constructor
	m_bSentLeakNotification = false;
	m_bRespondedToLeak = false;
	m_uiLastLeakLogTime = 0;
}

LeakSensor::~LeakSensor() {//destructor
	
}

//CheckLeakSensors: //poll the digital leak sensors and return true if at least one of them has detected a leak
//pShipLog: void pointer to the ship's log, used for recording leak sensor state (if a leak has been detected)
//return true if at least one of the leak sensors is showing a leak, otherwise return false
bool LeakSensor::CheckLeakSensors(void *pShipLog) {
	const int LOG_EVERY_MS = 60000;//log messages about a leak this often (in ms)
	int nLeak = digitalRead(LEAK_PIN);
	if (nLeak>0) {
		unsigned int uiCurrentTime = millis();
		if (m_uiLastLeakLogTime==0||(uiCurrentTime - m_uiLastLeakLogTime)>LOG_EVERY_MS) {
			m_uiLastLeakLogTime = uiCurrentTime;
			if (pShipLog!=nullptr) {
				ShipLog *pLog = (ShipLog *)pShipLog;
				pLog->LogEntry((char *)"LEAK DETECTED!!!\n",true);
			}
		}
		this->UpdateCollectionTime();
		return true;
	}
	this->UpdateCollectionTime();
	return false;//no leak detected
}

//HasSentLeakNotification: return true if a notification text or email has been sent already, informing of a detected leak
bool LeakSensor::HasSentLeakNotification() {
	return m_bSentLeakNotification;
}

//SendLeakNotifcation: send a notification by email and / or text to recipients specified in prefs.txt file
//pShipLog: void pointer to the ship's log file, used to record that a notification was sent
//pNotifier: NotifyOperator object used to send out text and / or emails to configured recipients that a leak has occurred
//returns true if notification(s) could be sent successfully
bool LeakSensor::SendLeakNotification(void *pShipLog, NotifyOperator *pNotifier) {
	ShipLog *pLog = (ShipLog *)pShipLog;
	char *sLeakMessageText = (char *)"WARNING ALERT: A leak has been detected by one or more sensors!";
	m_bSentLeakNotification = pNotifier->IssueNotification(sLeakMessageText, pShipLog);
	return m_bSentLeakNotification;
}

//HasRespondedToLeak: return true if we have already responded to the leak condition
//note that "responding to the leak" does not refer to sending a leak notification, but
//rather refers to having executed some navigational routine in order to get to shore as 
//quickly as possible. The "responded" state should be set accordingly somewhere by calling 
//the "RespondedToLeak" function of this class from one of the main program threads.
bool LeakSensor::HasRespondedToLeak() {
	return m_bRespondedToLeak;
}

//RespondedToLeak: set the "responded" state as to whether or not we have responded to the leak
//pShipLog: void pointer to the ship's log file, used to record that a notification was sent
//bResponded: set to true to indicate that some navigational response has been initiated to get the boat to shore as quickly as possible
void LeakSensor::RespondedToLeak(void *pShipLog, bool bResponded) {
	m_bRespondedToLeak = bResponded;
	if (bResponded) {
		ShipLog *pLog = (ShipLog *)pShipLog;
		pLog->LogEntry((char *)"Navigation command has been issued in response to an emergency condition.\n",true);
	}
}

//GetLeakResponse: return the type of response that should be executed if a leak occurs
int LeakSensor::GetLeakResponse() {
	//not implemented yet...
	return 0;
}

