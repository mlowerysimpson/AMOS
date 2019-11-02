//LeakSensor.h 
#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#include "NotifyOperator.h"
#endif
#include <memory>
#include <vector>
#include "Sensor.h"

#define LEAK_PIN 16

using namespace std;

class LeakSensor : public Sensor {//class used for polling the leak sensors
public:
	LeakSensor();//constructor
	~LeakSensor();//destructor

	bool CheckLeakSensors(void *pShipLog);//poll the digital leak sensors and return true if at least one of them has detected a leak
	bool HasSentLeakNotification();//return true if a notification text or email has been sent already, informing of a detected leak
#ifndef _WIN32
	bool SendLeakNotification(void *pShipLog, NotifyOperator *pNotify);//send a notification by email and / or text to recipients specified in prefs.txt file
#endif
	bool HasRespondedToLeak();//return true if we have already responded to the leak condition
	void RespondedToLeak(void *pShipLog, bool bResponded);//set the "responded" state as to whether or not we have responded to the leak
	int GetLeakResponse();//return the type of response that should be executed if a leak occurs

private:
	unsigned int m_uiLastLeakLogTime;//time in ms when something about a leak was last saved to the log file
	bool m_bSentLeakNotification;//flag is true if a leak notification has been issued already
	bool m_bRespondedToLeak;//flag is true if some navigational action has been taken to get to shore as quickly as possible (to repair the leak)
};