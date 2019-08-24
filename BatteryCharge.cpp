//Copyright 2019 In Nature Robotics Ltd.
//BatteryCharge.cpp
//Implementation file for BatteryCharge class
#include "BatteryCharge.h"
#include "filedata.h"
#include "Util.h"

#define LEAD_ACID_VOLTAGE_LIMIT 11.2 //limit below which a lead acid battery is deemed to require re-charging

/**
 * @brief Construct a new Battery Charge:: Battery Charge object
 * 
 * @param szPrefsFilename, the path to the preferences file which contains info about the type of battery being used.
 * @param pShipLog pointer to the ship's log, used for recorded debugging and diagnostic information.
 */
BatteryCharge::BatteryCharge(char *szPrefsFilename, ShipLog *pShipLog) {//constructor
    m_bLeadAcidBattery = false;
    m_bInsufficientStartupVoltage = false;
	m_pShipLog = pShipLog;
    LoadPreferences(szPrefsFilename);
}

/**
 * @brief Destroy the Battery Charge:: Battery Charge object
 * 
 */
BatteryCharge::~BatteryCharge() {//destructor
	DeleteVoltageSamples();
}


/**
 * @brief add a battery voltage measurement (need to analyze battery voltage measurements over time to determine if charge is getting low)
 * 
 * @param dVoltage the battery voltage measurement being added to help determine if charge is getting low
 */
void BatteryCharge::AddBattVoltageMeasurement(double dVoltage) {//add a battery voltage measurement (need to analyze battery voltage measurements over time to determine if charge is getting low)
    if (dVoltage>=FULL_VOLTAGE_LEVEL) {
        //reset count of low voltage values
        DeleteVoltageSamples();
    }
    if (dVoltage<LOW_VOLTAGE_LEVEL) {
        unsigned int uiCurrentTime = millis();
        if (uiCurrentTime<(WARMUP_TIME_MIN*60*1000)) {
            return;//do not consider any low voltage readings during the first WARMUP_TIME_MIN minutes of operation
        }
        LOW_VOLTAGE_SAMPLE *pLowVoltSample = new LOW_VOLTAGE_SAMPLE;
        pLowVoltSample->dVoltage = dVoltage;
        pLowVoltSample->timeMS = millis();
        m_lowVoltageSamples.push_back(pLowVoltSample);
    }
}

/**
 * @brief Check to see if the battery charge level is OK. It does this by using the following algorithm:
 * 1. 4 or more readings of less than LOW_VOLTAGE_LEVEL volts are required in the last CHECK_WINDOW_MIN minutes of operation
 * 2. Any voltage >= FULL_VOLTAGE_LEVEL resets the count of low voltage level readings.
 * 3. If the slope of the regression line through the last 4 low voltage readings is less than MIN_VOLT_LOSSRATE (volts per hour) then the battery needs to be charged (i.e. current charge level is NOT ok).
 * 4. Ignore any low voltage readings received within the first WARMUP_TIME_MIN minutes of operation. 
 * @return true 
 * @return false 
 */
bool BatteryCharge::isChargeOK() {
    char sMsg[128];
    if (m_bInsufficientStartupVoltage) {
        return false;
    }
    vector <LOW_VOLTAGE_SAMPLE *>recentLowVoltReadings = GetRecentLowVoltReadings();
    int nNumRecentLowVoltReadings = recentLowVoltReadings.size();
    if (nNumRecentLowVoltReadings<4) return true;//not enough recent low voltage readings, so assume battery charge is ok.
    if (this->m_bLeadAcidBattery) {
        //look at average of last 4 readings
        int m = nNumRecentLowVoltReadings;
        double dAvgVoltage = (recentLowVoltReadings[m-1]->dVoltage + recentLowVoltReadings[m-2]->dVoltage + 
            recentLowVoltReadings[m-3]->dVoltage + recentLowVoltReadings[m-4]->dVoltage) / 4;
        if (dAvgVoltage<=LEAD_ACID_VOLTAGE_LIMIT) {
            return false;//voltage is pretty low, time to recharge
        }
        return true;//voltage is still ok
    }
    //need to look at slope of voltage vs. time for Lithium Polymer battery
    double dVoltageSlope = CalculateLowVoltRegression(recentLowVoltReadings);
    sprintf(sMsg,"Log voltage slope = %.3f V/hr\n",dVoltageSlope);
    m_pShipLog->LogEntry(sMsg,true);
    if (dVoltageSlope>=MIN_VOLT_LOSSRATE) {
        return true;//voltage has been holding relatively level
    }
    return false;//voltage must be dropping too fast, so charge is not ok
}

vector <LOW_VOLTAGE_SAMPLE *> BatteryCharge::GetRecentLowVoltReadings() {//get the 4 most recent low voltage readings (if they are within the last CHECK_WINDOW_MIN minutes)
    unsigned int uiCurrentTime = millis();
    vector <LOW_VOLTAGE_SAMPLE *>retval;
    int nNumReadings = m_lowVoltageSamples.size();
    int nMinReading = nNumReadings - 4;
    if (nMinReading<0) nMinReading = 0;
    for (int i=nMinReading;i<nNumReadings;i++) {
        unsigned int uiSampleTime = m_lowVoltageSamples[i]->timeMS;
        if ((uiCurrentTime-uiSampleTime)<(CHECK_WINDOW_MIN*60*1000)) {//sample is recent enough
            retval.push_back(m_lowVoltageSamples[i]);
        }        
    }
    return retval;
}

double BatteryCharge::CalculateLowVoltRegression(vector <LOW_VOLTAGE_SAMPLE *>samples) {//calculate the regression slope of the most recent 4 low voltage readings
    vector <double>time_hrs;//time of each voltage sample in hours
    vector <double>voltage;//voltage of each sample
    int nNumSamples = samples.size();
    for (int i=0;i<nNumSamples;i++) {
        double dTimeHrs = samples[i]->timeMS / 1000.0 / 60.0;
        time_hrs.push_back(dTimeHrs);
        voltage.push_back(samples[i]->dVoltage);
    }
    return Util::slope(time_hrs,voltage);
}

void BatteryCharge::DeleteVoltageSamples() {//get rid of all of the collected voltage samples
    int nNumSamples = m_lowVoltageSamples.size();
    for (int i=0;i<nNumSamples;i++) {
        if (m_lowVoltageSamples[i]) {
            delete m_lowVoltageSamples[i];
            m_lowVoltageSamples[i] = NULL;
        }
    }
    m_lowVoltageSamples.clear();
}

void BatteryCharge::LoadPreferences(char *szPrefsFilename) {//load preferences from the prefs.txt file
    if (szPrefsFilename==nullptr) {
        //use defaults
        m_bLeadAcidBattery = false;
        return;
    }
    filedata prefsFile(szPrefsFilename);
    m_bLeadAcidBattery = (bool)prefsFile.getInteger("[battery]","leadacid");
    if (m_bLeadAcidBattery) {
        m_pShipLog->LogEntry((char *)"Lead acid battery specified in prefs.txt.\n",true);
    }
    else {
        m_pShipLog->LogEntry((char *)"Lithium polymer battery assumed (leadacid variable set to zero).\n",true);
    }
}

/**
 * @brief return true if dVoltage >= MIN_STARTUP_VOLTAGE
 * 
 * @param dVoltage the voltage to check to see if it is high enough.
 * @return true if the last received voltage is >= MIN_STARTUP_VOLTAGE
 * @return false otherwise
 */
bool BatteryCharge::hasEnoughChargeToStart(double dVoltage) {
    return (dVoltage>=MIN_STARTUP_VOLTAGE);
}

/**
 * @brief return the minimum voltage required for startup
 * 
 * @return double the minimum required voltage for starting up RemoteControlTest (MIN_STARTUP_VOLTAGE)
 */
double BatteryCharge::GetMinStartupVoltage() {
    return MIN_STARTUP_VOLTAGE;
}

/**
 * @brief call this function on startup if the battery charge isn't really high enough to last very long (i.e. if voltage < MIN_STARTUP_VOLTAGE)
 * 
 */
void BatteryCharge::SetInsufficientStartupCharge() {
    m_bInsufficientStartupVoltage = true;
}