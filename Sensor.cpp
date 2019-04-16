//Sensor.cpp Implementation file for Sensor class
#include "Sensor.h"
#include <wiringPi.h>

/**
 * @brief Construct a new Sensor:: Sensor object
 * 
 * @param pAToD pointer to AToD object that handles high precision A to D measurements.
 */
Sensor::Sensor(AToD *pAToD) {
    m_uiLastCollectionTime = 0;
    m_pAToD = pAToD;
}

/**
 * @brief Destroy the Sensor:: Sensor object
 * 
 */
Sensor::~Sensor() {

}

/**
 * @brief returns true if data has not been collected for a while (i.e. if sensor data has not been collected within the last RECENT_THRESHOLD_SEC seconds)
 * 
 * @return true if data for the sensor has been collected in the last RECENT_THRESHOLD_SEC seconds.
 * @return false if no data for the sensor has been collected in the last RECENT_THRESHOLD_SEC seconds.
 */
bool Sensor::isOldData() {
    if (m_uiLastCollectionTime==0) return true;//no data has been collected yet
    unsigned int uiCurrentTime = millis();
    unsigned int uiTimeSinceLastReading = uiCurrentTime - m_uiLastCollectionTime;
    if (uiTimeSinceLastReading <= (RECENT_THRESHOLD_SEC*1000)) {
        return true;
    }
    return false;
}

/**
 * @brief call this function whenever data is collected to update the time in ms when data was collected
 * 
 */
void Sensor::UpdateCollectionTime() {
    m_uiLastCollectionTime = millis();
}