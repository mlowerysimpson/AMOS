//sensor class (base class for all types of sensors)
#pragma once

#include "AToD.h"

#define RECENT_THRESHOLD_SEC 10

/**
 * @brief sensor class (base class for all types of sensors)
 * 
 */
class Sensor {
    public:
        Sensor(AToD *pAToD);
        ~Sensor();

        void UpdateCollectionTime();//call this function whenever data is collected
        bool isOldData();//returns true if data has not been collected for a while

    protected:
        unsigned int m_uiLastCollectionTime;//last time that sensor data was collected in ms
        AToD *m_pAToD;//A to D object that handles high precision A to D conversions

};