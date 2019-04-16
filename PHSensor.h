//PHSensor.h -- interface class for getting pH data from pH probe sensors
#pragma once
#include "Sensor.h"

//default pH calibration and channel info
#define DEFAULT_PH_LOWCALVOLTAGE 0.655 //default acidic calibration voltage (for pH of 2.4)
#define DEFAULT_PH_LOWCAL 2.4 //default low pH calibration value (corresponds to test in vinegar)
#define DEFAULT_PH_MIDCALVOLTAGE 1.82 //default neutral calibration voltage (for pH of 7)
#define DEFAULT_PH_MIDCAL 7.0 //default neutral pH calibration value
#define DEFAULT_PH_CHANNEL 3 //default A to D channel for pH probe

struct PH_CALIBRATION {//pH probe calibration data
	double dLowCalVoltage;//the voltage of the "low" (acidic) pH measurement
	double dLowCalPH;//the pH value of the "low" (acidic) pH measurement
	double dMidCalVoltage;//the voltage of the neutral (pure water) measurement
	double dMidCalPH;//the pH value of the neutral (pure water) measurement, typically set to 7
};

class PHSensor : public Sensor {//class used for getting pH data for pH probe connected to A To D device
public:
	PHSensor(int nAToDChannel, AToD *pAToD, PH_CALIBRATION *pPHCal);//constructor
	~PHSensor();//destructor
	bool GetPHSensorPH(double &dPHVal);//gets the pH probe pH value for AMOS (uses currently available pH probe calibration parameters)

private:
	//data
	int m_nAToDChannel;//the channel of the A to D device that this pH probe is connected to (1 to 4)
	PH_CALIBRATION m_phcal;//the pH calibration data for this pH probe
};