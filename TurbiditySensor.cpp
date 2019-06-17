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


/**
 * @brief Gets the turbidity value for AMOS, for A to D conversions, assumes some resistor divider settings (see below)
 * @param dTurbidity the returned turbidity value, expressed as a voltage from 0 to 5 volts.
 * @return true if the turbidity was obtained correctly, otherwise returns false.
 */
 bool TurbiditySensor::GetTurbidity(double &dTurbidity) {
	//assumes :voltage divider of 20K in series with 10K resistor, so voltage gain is as follows:
	const double R1 = 20.0;//20k resistor
	const double R2 = 10.0;//10k resistor
	double dTurbidityGain = (R1 + R2) / R2;
	double dTurbidityVoltage=0.0;
	bool bGotResult = m_pAToD->GetMeasurement(m_nAToDChannel,1,dTurbidityGain,dTurbidityVoltage);
	if (bGotResult) {
		dTurbidity = dTurbidityVoltage;
		//test
		printf("dTurbidity = %.3f\n",dTurbidity);
		//end test
		this->UpdateCollectionTime();
	}
	else {
		printf("Failed to get turbidity.\n");
	}
	return bGotResult;
}
	
