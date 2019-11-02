//Copyright 2019 In Nature Robotics Ltd.
//BatteryCharge.h
#pragma once

#define FULL_VOLTAGE_LEVEL 13.0 //if battery voltage level is above this amount, then reset the count of low voltage samples
#define LOW_VOLTAGE_LEVEL 11.8 //if battery voltage level is below this amount, then it is considered a "low" voltage, and is analyzed over time to determine if it is starting to drop too fast
#define OFF_VOLTAGE_LEVEL 5.0 //if the battery voltage level is below this amount, then it can be assumed that the main power switch must be in the "off" position.
#define WARMUP_TIME_MIN 5 //warmup time in minutes, during which low battery readings (if any) are not considered
#define CHECK_WINDOW_MIN 15 //width of window in time (in minutes) during which low voltage readings (if any) are considered for monitoring the battery's charge level
#define MIN_VOLT_LOSSRATE -1.0 //the minimum voltage loss rate allowed, in volts per hour (if voltage rate decreaese becomes less than this it is a sign that the battery is getting low and should be charged).

#ifndef _WIN32
    #include <wiringPi.h>
#endif
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <vector>
#include "ShipLog.h"

#define MIN_STARTUP_VOLTAGE 11.5 //minimum voltage required for starting up program

using namespace std;

struct LOW_VOLTAGE_SAMPLE {//structure used to store voltages and times of samples where the voltage is considered low (i.e. < LOW_VOLTAGE_LEVEL)
    double dVoltage;//voltage of battery (in V)
    unsigned int timeMS;//time of battery voltage sample in ms
};

class BatteryCharge {//class used for estimating whether or not the charge on the battery is
//class requires the wiringPi library, so a call to the wiringPiSetup function should be made before calling any of the method functions of this class
public:
	BatteryCharge(char *szPrefsFilename, ShipLog *pShipLog);//constructor
	~BatteryCharge();//destructor
    void AddBattVoltageMeasurement(double dVoltage);//add a battery voltage measurement (need to analyze battery voltage measurements over time to determine if charge is getting low)
    bool isChargeOK();//return true if the battery has sufficient charge
    bool hasEnoughChargeToStart(double dVoltage);//return true if dVoltage >= MIN_STARTUP_VOLTAGE
    double GetMinStartupVoltage();//return the minimum voltage required for startup
    void SetInsufficientStartupCharge();//call this function on startup if the battery charge isn't really high enough to last very long (i.e. if voltage < MIN_STARTUP_VOLTAGE)
    void IgnoreInsufficentStartupCharge();//call this function to override a previous call to SetInsufficientStartupCharge();

private:
    vector <LOW_VOLTAGE_SAMPLE *>m_lowVoltageSamples;
    ShipLog *m_pShipLog;
    bool m_bLeadAcidBattery;//true if a lead acid battery is being used, false if a different type (ex: Lithium Polymer) is being used.
    bool m_bInsufficientStartupVoltage;//flag is true if the voltage on startup is too low, otherwise it is false
    vector <LOW_VOLTAGE_SAMPLE *> GetRecentLowVoltReadings();//get the 4 most recent low voltage readings (if they are within the last CHECK_WINDOW_MIN minutes)
    double CalculateLowVoltRegression(vector <LOW_VOLTAGE_SAMPLE *>samples);//calculate the regression slope of the most recent 4 low voltage readings
    void DeleteVoltageSamples();//get rid of all of the collected voltage samples
    void LoadPreferences(char *szPrefsFilename);//load preferences from the prefs.txt file
};
