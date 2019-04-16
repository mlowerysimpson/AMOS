//TurbiditySensor.h -- interface class for getting water turbidity measurements from turbidity sensors
#pragma once
#include "AToD.h"//requires connection of pH probe to AToD device
#include "Sensor.h"

//default turbidity A To D channel info
#define DEFAULT_TURBIDITY_CHANNEL 2 //default A to D channel for pH probe

class TurbiditySensor : public Sensor {//class used for getting turbidity data 
public:
	TurbiditySensor(int nAToDChannel, AToD *pAToD);//constructor
	~TurbiditySensor();//destructor
	bool GetTurbidity(double &dTurbidity);//gets the turbidity value from the turbidity sensor

private:
	//data
	int m_nAToDChannel;//the channel of the A to D device that this pH probe is connected to (1 to 4)
};