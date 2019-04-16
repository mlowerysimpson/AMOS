//PHSensor.cpp
//implementation file for PHSensor class
#include "PHSensor.h"
PHSensor::PHSensor(int nAToDChannel, AToD *pAToD, PH_CALIBRATION *pPHCal) : Sensor(pAToD) {
	if (nAToDChannel<1||nAToDChannel>NUM_ATOD_CHANNELS) {
		m_nAToDChannel = DEFAULT_PH_CHANNEL;
	}
	else {
		m_nAToDChannel = nAToDChannel;
	}
	if (pPHCal) {
		memcpy(&m_phcal,pPHCal,sizeof(PH_CALIBRATION));
	}
	else {//use default values
		m_phcal.dLowCalVoltage = DEFAULT_PH_LOWCALVOLTAGE;
		m_phcal.dLowCalPH = DEFAULT_PH_LOWCAL;
		m_phcal.dMidCalVoltage = DEFAULT_PH_MIDCALVOLTAGE;
		m_phcal.dMidCalPH = DEFAULT_PH_MIDCAL;
	}
}

PHSensor::~PHSensor() {

}

//GetPHSensorPH: gets the pH probe pH value for AMOS (uses currently available pH probe calibration parameters), assumes some resistor divider settings (see below)
//dPHVal: the returned pH value (0 to 14)
//returns true if successful, false otherwise
bool PHSensor::GetPHSensorPH(double &dPHVal) {
	//assumes use of voltage divider of 10K in series with another 10K resistor to ground, so voltage gain is as follows:
	const double R1 = 10.0;//10k resistor
	const double R2 = 10.0;//10k resistor
	char sMsg[256];
	if (!m_pAToD) return false;
    double dPHVoltage=0.0;
	bool bGotResult = m_pAToD->GetMeasurement(m_nAToDChannel,1,1.0,dPHVoltage);
	if (bGotResult) {
		double dCalGain = (m_phcal.dMidCalPH - m_phcal.dLowCalPH) / 
			(m_phcal.dMidCalVoltage - m_phcal.dLowCalVoltage);
		double dCalOffset = m_phcal.dMidCalPH - dCalGain*m_phcal.dMidCalVoltage;
		dPHVal = dCalGain * dPHVoltage + dCalOffset;
	}
	else {
		printf("Error getting pH voltage.\n");
		return false;
	}
	this->UpdateCollectionTime();
	return true;		
}