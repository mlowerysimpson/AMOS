//sensor class (base class for all types of sensors)
#pragma once

#include "AToD.h"

#define RECENT_THRESHOLD_SEC 10

//I2C response codes for Atlas Scientific probes in I2C mode
#define NO_DATA_TO_SEND 255
#define NOT_READY 254
#define SYNTAX_ERROR 2
#define DATA_OK 1

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