//SwitchRelay.cpp -- implementation file for SwitchRelay class
#include "SwitchRelay.h"
#include "filedata.h"
#include <wiringPi.h>

/**
 * @brief Construct a new Switch Relay:: Switch Relay object
 * 
 */
SwitchRelay::SwitchRelay(char *szPrefsFilename) {
    m_dMaxAllowedVoltage = DEFAULT_MAX_ALLOWED_VOLTAGE;
    m_dFullyChargedVoltage = DEFAULT_FULLY_CHARGED_VOLTAGE;
    m_bSolarSwitchedOn = true;
    pinMode(SOLAR_SWITCH, OUTPUT);//set pin for controlling solar power switch to output
    digitalWrite(SOLAR_SWITCH, HIGH);//default output to high state (solar power swtich is "on")
    LoadPreferences(szPrefsFilename);
}

/**
 * @brief Destroy the Switch Relay:: Switch Relay object
 * 
 */
SwitchRelay::~SwitchRelay() {

}

/**
 * @brief check to see if a particular voltage is too high for the battery or not
 * 
 * @param dVoltage the voltage to check to see if it is too high.
 * @return true if the voltage is too high.
 * @return false if the voltage is not too high.
 */
bool SwitchRelay::isOverVoltage(double dVoltage) {
    return (dVoltage>m_dMaxAllowedVoltage);
}

/**
 * @brief call this function to switch on / off the +ve solar output to the charge controller
 * 
 * @param bTurnOn set to true to switch on the +ve solar output to the charge controller, or set to false to turn off the +ve solar output to the charge controller.
 */
void SwitchRelay::TurnOnSolarPower(bool bTurnOn) {
    if (bTurnOn) {
        digitalWrite(SOLAR_SWITCH, HIGH);//high output turns solar power swtich "on"
        m_bSolarSwitchedOn = true;
    }
    else {
        digitalWrite(SOLAR_SWITCH, LOW);//low output turns solar power switch "off"
        m_bSolarSwitchedOn = false;
    }
}

void SwitchRelay::LoadPreferences(char *szPrefsFilename) {
    if (szPrefsFilename==nullptr) {
        //use defaults
        m_dMaxAllowedVoltage = DEFAULT_MAX_ALLOWED_VOLTAGE;
        return;
    }
    filedata prefsFile(szPrefsFilename);
    m_dMaxAllowedVoltage = prefsFile.getDouble("[switchprefs]","max_allowed_voltage");
    m_dFullyChargedVoltage = prefsFile.getDouble("[switchprefs]","fully_charged_voltage");
    if (m_dMaxAllowedVoltage<=0) {
        m_dMaxAllowedVoltage = DEFAULT_MAX_ALLOWED_VOLTAGE;
    }
    if (m_dFullyChargedVoltage<=0) {
        m_dFullyChargedVoltage = DEFAULT_FULLY_CHARGED_VOLTAGE;
    }
}

/**
 * @brief check to see if a particular voltage is sufficiently high to consider the battery fully charged, or close enough
 * 
 * @param dVoltage the voltage to check to see if it corresponds to a fully charged state or not
 * @return true if dVoltage represents a fully charged (or nearly fully charged) battery
 * @return false if dVoltage is lower than what would correspond to a fully charged battery.
 */
bool SwitchRelay::isFullyCharged(double dVoltage) {//return true if dVoltage is sufficiently high to consider the battery fully charged, or close enough
    return (dVoltage>=m_dFullyChargedVoltage);
}

/**
 * @brief return the state of the solar power switch
 * 
 * @return true if the solar power switch is turned on
 * @return false if the solar power switch is turned off 
 */
bool SwitchRelay::isSwitchedOn() {
    return m_bSolarSwitchedOn;
}