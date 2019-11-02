//SwitchRelay.h 
#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <memory>
#include <vector>

#define SOLAR_SWITCH 22 //GPIO pin used for switching on / off the +ve solar output to the solar charge controller
#define DEFAULT_MAX_ALLOWED_VOLTAGE 13.6 //the default maximum voltage that we want to see on the battery. If the voltage of the battery is measured above this level, then the solar input to the charge controller should be switched off.
#define DEFAULT_FULLY_CHARGED_VOLTAGE 13.0 //the default voltage that corresponds to the fully charged state of the battery. 

using namespace std;

class SwitchRelay  {//class used for switching various devices on / off (controls relay device inside the battery box)
public:
	SwitchRelay(char *szPrefsFilename);
	~SwitchRelay();//destructor

    bool isOverVoltage(double dVoltage);//return true if dVoltage is too high 
    bool isFullyCharged(double dVoltage);//return true if dVoltage is sufficiently high to consider the battery fully charged, or close enough
    void TurnOnSolarPower(bool bTurnOn);//call this function to switch on / off the +ve solar output to the charge controller 
    bool isSwitchedOn();//return the state of the solar power switch 

private:
    bool m_bSolarSwitchedOn;//the state of the solar power input to the charge controller
    void LoadPreferences(char *szPrefsFilename);//load preferences from the prefs.txt file (if it exists, otherwise use default preferences)
    double m_dMaxAllowedVoltage;//the maximum allowed voltage for the battery being used.
    double m_dFullyChargedVoltage;//the minimum voltage of the battery for which it could be considered fully charged

};