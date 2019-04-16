//TurbiditySensor.cpp
//implementation file for TurbiditySensor class
#include "TurbiditySensor.h"
TurbiditySensor::TurbiditySensor(int nAToDChannel, AToD *pAToD) : Sensor(pAToD) {
	if (nAToDChannel<1||nAToDChannel>NUM_ATOD_CHANNELS) {
		m_nAToDChannel = DEFAULT_TURBIDITY_CHANNEL;
	}
	else {
		m_nAToDChannel = nAToDChannel;
	}
}

TurbiditySensor::~TurbiditySensor() {

}

//GetTurbidity: gets the turbidity value for AMOS, for A to D conversions, assumes some resistor divider settings (see below)
//dTurbidity: the returned turbidity value (voltage from 0 to 5)
//returns true if successful, false otherwise
bool TurbiditySensor::GetTurbidity(double &dTurbidity) {
	//assumes :voltage divider of 20K in series with 10K resistor, so voltage gain is as follows:
	const double R1 = 20.0;//20k resistor
	const double R2 = 10.0;//10k resistor
	double dTurbidityGain = VREF / MAX_COUNTS * (R1 + R2) / R2;
	double dTurbidityVoltage=0.0;
	bool bGotResult = m_pAToD->GetMeasurement(m_nAToDChannel,1,dTurbidityGain,dTurbidityVoltage);
	if (bGotResult) {
		dTurbidity = dTurbidityVoltage;
		this->UpdateCollectionTime();
	}
	return bGotResult;
}
	
