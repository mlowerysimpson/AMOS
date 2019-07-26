//SensorDataFile.cpp
//implementation file for SensorDataFile class
#include "SensorDataFile.h"
#include "CommandList.h"
#include "Navigation.h"
#include "Util.h"

//SesnorDataFile constructor
//sensorTypes = array of sensor types, ex: WATER_TEMP_DATA, PH_DATA, etc.
//nNumSensors = the number of sensor types in the sensorTypes array
//sensorObjects = an array of void pointers to sensor objects, can be used for acquiring and configuring boat sensors
//szDataFilePath = the full path of the file where data will be saved
//nLoggingIntervalSec = the interval between samples in seconds
SensorDataFile::SensorDataFile(int *sensorTypes, int nNumSensors, void *sensorObjects[], char *szDataFilePath, int nLoggingIntervalSec) {//constructor
	m_sensorTypes = nullptr;
	m_sensorData = nullptr;
	m_bDataAvailable = nullptr;
	m_nNumSensors = nNumSensors;
	m_nLastLogTime=0;
	m_fWaterTemp=0;
	m_fInteriorTemp=0;
	m_fWaterPH=0;
	m_fTurbidity=0;
	m_fWirelessRXPower=0;
	m_fCurrentDraw12V=0;
	m_fHumidityCPU=0;
	m_fHumidityBattery=0;
	m_fHumidityTempCPU=0;
	m_fHumidityTempBattery=0;
	m_fBatteryVoltage=0;
	m_bSolarCharging = false;
	m_bLeak = false;
	
	m_nextLogTime=GetNextLogTime(nLoggingIntervalSec);//get the next sample time in seconds since midnight, Jan. 01, 1970
	
	if (m_nNumSensors>0) {
		m_sensorTypes = new int[m_nNumSensors];
		m_sensorData = new double[m_nNumSensors];
		m_bDataAvailable = new bool[m_nNumSensors];
		memcpy(m_sensorTypes,sensorTypes,m_nNumSensors*sizeof(int));
		memset(m_sensorData,0,m_nNumSensors*sizeof(double));
		memset(m_bDataAvailable,0,m_nNumSensors*sizeof(bool));
		for (int i=0;i<nNumSensors;i++) {
			m_sensorObjects[i] = sensorObjects[i];
		}
		for (int i=nNumSensors;i<MAX_SENSORS;i++) {
			m_sensorObjects[i] = NULL;
		}
	}
	m_dataFile = nullptr;
	m_szDataFilePath = new char[strlen(szDataFilePath)+1];
	strcpy(m_szDataFilePath, szDataFilePath);
	CreateDataFile();
}

SensorDataFile::~SensorDataFile() {//destructor
	if (m_sensorTypes) {
		delete []m_sensorTypes;
		m_sensorTypes = nullptr;
	}
	if (m_sensorData) {
		delete []m_sensorData;
		m_sensorData = nullptr;
	}
	if (m_bDataAvailable) {
		delete []m_bDataAvailable;
		m_bDataAvailable=nullptr;
	}
	if (m_szDataFilePath) {
		delete []m_szDataFilePath;
		m_szDataFilePath=nullptr;
	}
	if (m_dataFile) {
		if (m_dataFile->is_open()) {
			m_dataFile->close();
		}
		delete m_dataFile;
		m_dataFile=nullptr;
	}
}

//SaveData: save the current sample of data to file
//sampleTime = structure defining the time and date of the sample
//nLoggingIntervalSec = the logging interval in seconds
void SensorDataFile::SaveData(struct tm *sampleTime, int nLoggingIntervalSec) {
	char * szFormattedData = GetFormattedData(sampleTime);
	m_dataFile->write(szFormattedData, strlen(szFormattedData));
	delete []szFormattedData;
	//reset data flags
	for (int i=0;i<m_nNumSensors;i++) {
		m_bDataAvailable[i] = false;
	}
	if (nLoggingIntervalSec>0) {
		m_nextLogTime = GetNextLogTime(nLoggingIntervalSec);
	}
}

void SensorDataFile::SetData(int nDataType, double dData) {//sets a particular data value, data will get saved to file in the next call to SaveData
	for (int i=0;i<m_nNumSensors;i++) {
		if (nDataType==m_sensorTypes[i]) {
			m_sensorData[i] = dData;
			m_bDataAvailable[i] = true;
		}
		if (nDataType==WATER_TEMP_DATA) {
			m_fWaterTemp = (float)dData;
		}
		else if (nDataType==BOAT_INTERIOR_TEMP_DATA) {
			m_fInteriorTemp = (float)dData;
		}
		else if (nDataType==PH_DATA) {
			m_fWaterPH = (float)dData;
		}
		else if (nDataType==WATER_TURBIDITY) {
			m_fTurbidity = (float)dData;
		}
		else if (nDataType==DIAGNOSTICS_DATA) {
			m_fCurrentDraw12V = (float)dData;
		}
	}
}

bool SensorDataFile::CreateDataFile() {//open data file for appending data or create data file if it doesn't exist already
	m_dataFile = new std::ofstream();
	try {
		m_dataFile->open(m_szDataFilePath,std::ios_base::out|std::ios_base::app);//open file for appending
	}
	catch (const std::exception& e) {
		printf("Error (%s) trying to open %s for appending.\n",e.what(),m_szDataFilePath);
		return false;
	}
	long lFilePos = m_dataFile->tellp();
	if (lFilePos==0) {
		WriteDataFileHeader();
	}
	return true;
}

char * SensorDataFile::GetFormattedData(struct tm *sampleTime) {//formats the currently available data
	//get conservative estimate of length of text required
	int nEstimateLength = 20 + 16 * m_nNumSensors;
	char *retVal = new char[nEstimateLength];//the formatted text that this function returns (calling function responsible for deleting it)
	char szDateTime[20];//date and time, format as YYYY-MM-DD HH:MM:SS, 
	sprintf(szDateTime, "%d-%02d-%02d %02d:%02d:%02d, ",sampleTime->tm_year+1900, sampleTime->tm_mon+1, sampleTime->tm_mday, 
		sampleTime->tm_hour, sampleTime->tm_min, sampleTime->tm_sec);
	strcpy(retVal, szDateTime);
	char szTmp[32];
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_bDataAvailable[i]) {
			if (m_sensorTypes[i]==WATER_TEMP_DATA||m_sensorTypes[i]==BOAT_INTERIOR_TEMP_DATA) {
				sprintf(szTmp, "%.2f, ", m_sensorData[i]);
			}
			else if (m_sensorTypes[i]==PH_DATA||m_sensorTypes[i]==WATER_TURBIDITY) {
				sprintf(szTmp, "%.3f, ", m_sensorData[i]);
			}
			else if (m_sensorTypes[i]==GPS_LATITUDE||m_sensorTypes[i]==GPS_LONGITUDE) {
				sprintf(szTmp, "%.6f, ",m_sensorData[i]);
			}
			else if (m_sensorTypes[i]==LEAK_DATA) {
				if (m_bLeak) {
					strcpy(szTmp, (char *)"Yes, ");
				}
				else {
					strcpy(szTmp, (char *)"No, ");
				}
			}
			else if (m_sensorTypes[i]==DIAGNOSTICS_DATA) {//output current in mA and whether or not solar power is currently available
				char szWirelessRX[32];//received serial wireless power level, expressed in dBm
				if (this->m_fWirelessRXPower!=0) {
					sprintf(szWirelessRX,"%.0f",m_fWirelessRXPower);
				}
				else sprintf(szWirelessRX,"N.A.");
				if (this->m_bSolarCharging) {//solar panel is charging
					sprintf(szTmp, "%.3f, %.2f, %.2f, %.2f, %s, Yes, ", this->m_fBatteryVoltage, this->m_fCurrentDraw12V, this->m_fHumidityCPU, this->m_fHumidityTempCPU, this->m_fHumidityBattery, this->m_fHumidityTempBattery, szWirelessRX);
				}
				else {//solar panel is not charging
					sprintf(szTmp, "%.3f, %.2f, %.2f, %.2f, %s, No, ", this->m_fBatteryVoltage, this->m_fCurrentDraw12V, this->m_fHumidityCPU, this->m_fHumidityTempCPU, this->m_fHumidityBattery, this->m_fHumidityTempBattery, szWirelessRX);
				}
			}
		}
		else {//data is not available for this particular sensor, use "N.A., " instead
			strcpy(szTmp, (char *)"N.A., ");
		}
		strcat(retVal, szTmp);
	}	
	//take off last trailing comma and space and replace with carriage return, linefeed
	int nRetValLength = strlen(retVal);
	retVal[nRetValLength-2]=(char)0x0d;
	retVal[nRetValLength-1]=(char)0x0a;
	return retVal;
}

void SensorDataFile::WriteDataFileHeader() {
	int nEstimateLength = 20 + 128 * m_nNumSensors;//estimate the length of the header line required
	char *szHeaderLine = new char[nEstimateLength];
	memset(szHeaderLine,0,nEstimateLength);
	strcpy(szHeaderLine, "AMOS Data File\n");
	m_dataFile->write(szHeaderLine,strlen(szHeaderLine));
	char szTmp[256];
	strcpy(szHeaderLine,"Date/Time, ");
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==WATER_TEMP_DATA) {
			strcpy(szTmp, "WaterTemp(degC), ");
		}
		else if (m_sensorTypes[i]==BOAT_INTERIOR_TEMP_DATA) {
			strcpy(szTmp, "BoatTemp(degC), ");
		}
		else if (m_sensorTypes[i]==PH_DATA) {
			strcpy(szTmp, "WaterPH, ");
		}
		else if (m_sensorTypes[i]==WATER_TURBIDITY) {
			strcpy(szTmp, "Turbidity, ");
		}
		else if (m_sensorTypes[i]==GPS_LATITUDE) {
			strcpy(szTmp, "Latitude(deg), ");
		}
		else if (m_sensorTypes[i]==GPS_LONGITUDE) {
			strcpy(szTmp, "Longitude(deg), ");
		}
		else if (m_sensorTypes[i]==LEAK_DATA) {
			strcpy(szTmp, "Leak, ");
		}
		else if (m_sensorTypes[i]==DIAGNOSTICS_DATA) {
			char *szDiagnosticsHeader = DiagnosticsSensor::GetSensorFileHeader();
			if (szDiagnosticsHeader) {
				strcpy(szTmp, szDiagnosticsHeader);
				delete []szDiagnosticsHeader;
			}
			else {
				strcpy(szTmp, "UnknownDiagnostic(???), ");
			}
		}
		else {//unknown sensor type
			strcpy(szTmp, "???, ");
		}
		strcat(szHeaderLine, szTmp);
	}
	//take off last trailing comma and space and replace with carriage return, linefeed
	int nRetValLength = strlen(szHeaderLine);
	szHeaderLine[nRetValLength-2]=(char)0x0d;
	szHeaderLine[nRetValLength-1]=(char)0x0a;
	m_dataFile->write(szHeaderLine,strlen(szHeaderLine));
	delete []szHeaderLine;
}

//isTimeToLogSensorData: 
bool SensorDataFile::isTimeToLogSensorData(struct tm *currentTime) {//returns true if it is time to log sensor data to file, based on the logging interval (nLoggingIntervalSec)
	if (!currentTime) return false;
	time_t timeNow = mktime(currentTime);
	if (timeNow>=m_nextLogTime) {
		return true;
	}
	return false;
}

time_t SensorDataFile::GetNextLogTime(int nLoggingIntervalSec) {//get the time of the next sample (in seconds since midnight, Jan, 1, 1970)
	//test
	//printf("nLoggingIntervalSec = %d\n",nLoggingIntervalSec);
	//end test
	return Util::GetNextIntervalTime(nLoggingIntervalSec);
}


/**
 * @brief send # of sensors over network socket connection or a wireless serial port connection.
 * 
 * @param nHandle the network socket handle (if bUseSerial is false) or the serial port file descriptor (if bUseSerial is true).
 * @param bUseSerial set to true when using a wireless serial port link, otherwise set to false for a network link.
 * @return true if the number of sensors info could be successfully sent.
 * @return false if there was a problem sending the number of sensors.
 */
bool SensorDataFile::SendNumSensors(int nHandle, bool bUseSerial) {
	BOAT_DATA *pBoatData = BoatCommand::CreateBoatData(SUPPORTED_SENSOR_DATA);
	memcpy(pBoatData->dataBytes,&this->m_nNumSensors,sizeof(int));
	pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
	return BoatCommand::SendBoatData(nHandle, bUseSerial, pBoatData, nullptr);
}


/**
 * @brief send sensor types over socket connection
 * 
 * @param nHandle the network socket handle (if bUseSerial is false) or the serial port file descriptor (if bUseSerial is true).
 * @param bUseSerial set to true when using a wireless serial port link, otherwise set to false for a network link.
 * @return true if the sensor types could be successfully sent, otherwise return false
 * @return false if there was a problem sending the sensor types
 */
bool SensorDataFile::SendSensorTypes(int nHandle, bool bUseSerial) {
	//struct BOAT_DATA {
	//int nPacketType;//packet code describing what type of data this is
	//int nDataSize;//number of dataBytes 
	//unsigned char *dataBytes;
	//unsigned char checkSum;//simple 8-bit checksum of everything in this structure, except this checkSum byte	
	//};
	BOAT_DATA boatData;
	boatData.nPacketType = SENSOR_TYPES_INFO;
	boatData.nDataSize = m_nNumSensors*sizeof(int);
	boatData.dataBytes = new unsigned char[boatData.nDataSize];
	for (int i=0;i<m_nNumSensors;i++) {
		memcpy(&boatData.dataBytes[4*i],&m_sensorTypes[i],sizeof(int));
	}
	boatData.checkSum = BoatCommand::CalculateChecksum(&boatData);//calculate 8-bit checksum for BOAT_DATA structure
	return BoatCommand::SendBoatData(nHandle, bUseSerial, &boatData, nullptr);
}
	
/**
 * @brief send sensor data over socket connection or wireless serial link.
 * 
 * @param nSensorType the type of data to send over the socket connection or wireless serial link, see SensorDataFile.h for supported sensor types.
 * @param nHandle the network socket handle (if bUseSerial is false) or the serial port file descriptor (if bUseSerial is true).
 * @param bUseSerial set to true when using a wireless serial port link, otherwise set to false for a network link.
 * @return true if the sensor data could be successfully sent.
 * @return false if there was a problem sending the sensor data.
 */
bool SensorDataFile::SendSensorData(int nSensorType, int nHandle, bool bUseSerial) {
	//BOAT_DATA *pBoatData = BoatCommand::CreateBoatData(SUPPORTED_SENSOR_DATA);
	BOAT_DATA *pBoatData = nullptr;
	if (nSensorType==WATER_TEMP_DATA) {
		TempSensor *pWaterSensor = GetWaterTempSensor();
		//test
		if (pWaterSensor!=nullptr) {
			if (pWaterSensor->isOldData()) {
				if (!pWaterSensor->GetTemperature(m_fWaterTemp)) {
					printf("error getting water temp.\n");
				}
			}
		}
		pBoatData = BoatCommand::CreateBoatData(WATER_TEMP_DATA_PACKET);
		memcpy(pBoatData->dataBytes,&m_fWaterTemp,sizeof(float));
	}
	else if (nSensorType==PH_DATA) {
		PHSensor *pPHSensor = GetPHSensor();
		if (pPHSensor&&pPHSensor->isOldData()) {
			double dPHVal=0.0;
			if (pPHSensor->GetPHSensorPH(dPHVal)) {
				m_fWaterPH = (float)dPHVal;
			}
		}
		pBoatData = BoatCommand::CreateBoatData(WATER_PH_DATA_PACKET);
		memcpy(pBoatData->dataBytes,&m_fWaterPH,sizeof(float));
	}
	else if (nSensorType==WATER_TURBIDITY) {
		TurbiditySensor *pTurbiditySensor = GetTurbiditySensor();
		if (pTurbiditySensor&&pTurbiditySensor->isOldData()) {
			double dTurbidityVal = 0.0;
			if (pTurbiditySensor->GetTurbidity(dTurbidityVal)) {
				m_fTurbidity = (float)dTurbidityVal;
			}
		}
		pBoatData = BoatCommand::CreateBoatData(WATER_TURBIDITY_DATA_PACKET);
		memcpy(pBoatData->dataBytes,&m_fTurbidity,sizeof(float));
	}
	else if (nSensorType==LEAK_DATA) {
		LeakSensor *pLeakSensor = GetLeakSensor();
		if (pLeakSensor&&pLeakSensor->isOldData()) {
			m_bLeak = pLeakSensor->CheckLeakSensors(nullptr);
		}
		int nLeak = (int)m_bLeak;
		pBoatData = BoatCommand::CreateBoatData(LEAK_DATA_PACKET);
		memcpy(pBoatData->dataBytes,&nLeak,sizeof(int));
	}
	else if (nSensorType==DIAGNOSTICS_DATA) {
		DiagnosticsSensor *pDiagnosticsSensor = GetDiagnosticsSensor();
		int nSolarCharging = 0;
		if (pDiagnosticsSensor&&pDiagnosticsSensor->isOldData()) {
			if (bUseSerial) {
				usleep(100000);
			}
			pDiagnosticsSensor->GetBatteryVoltage(this->m_fBatteryVoltage);
			pDiagnosticsSensor->GetCurrentDraw(this->m_fCurrentDraw12V);
			pDiagnosticsSensor->GetHumidityAndTemp(this->m_fHumidityCPU,this->m_fHumidityTempCPU,CPUBOX);
			pDiagnosticsSensor->GetHumidityAndTemp(this->m_fHumidityBattery, this->m_fHumidityTempBattery,BATTERYBOX);
			pDiagnosticsSensor->GetWirelessRXPower(this->m_fWirelessRXPower);
			m_bSolarCharging = pDiagnosticsSensor->IsSolarCharging();
			if (m_bSolarCharging) {
				nSolarCharging = 1;
			}
		}
		pBoatData = BoatCommand::CreateBoatData(DIAGNOSTICS_DATA_PACKET);
		memcpy(pBoatData->dataBytes,&m_fBatteryVoltage,sizeof(float));
		memcpy(&pBoatData->dataBytes[sizeof(float)],&m_fCurrentDraw12V,sizeof(float));
		memcpy(&pBoatData->dataBytes[2*sizeof(float)],&m_fHumidityCPU,sizeof(float));
		memcpy(&pBoatData->dataBytes[3*sizeof(float)],&m_fHumidityTempCPU,sizeof(float));
		memcpy(&pBoatData->dataBytes[4*sizeof(float)],&m_fHumidityBattery,sizeof(float));
		memcpy(&pBoatData->dataBytes[5*sizeof(float)],&m_fHumidityTempBattery,sizeof(float));
		memcpy(&pBoatData->dataBytes[6*sizeof(float)],&m_fWirelessRXPower,sizeof(float));
		memcpy(&pBoatData->dataBytes[7*sizeof(float)],&nSolarCharging,sizeof(int));
	}
	pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
	return BoatCommand::SendBoatData(nHandle, bUseSerial, pBoatData, (void *)GetDiagnosticsSensor());	
}

/**
 * @brief save data from all sensors at a particular GPS location
 * 
 * @param dLatitude latitude of GPS location in degrees (-90 to +90)
 * @param dLongitude longitude of GPS location in degrees (-180 to +180)
 */
void SensorDataFile::SaveDataAtLocation(double dLatitude, double dLongitude) {
	CollectData();
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==GPS_LATITUDE) {
			SetData(GPS_LATITUDE, dLatitude);
		}
		else if (m_sensorTypes[i]==GPS_LONGITUDE) {
			SetData(GPS_LONGITUDE, dLongitude);
		}
	}
	
	//get current time and date for sample
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	SaveData(timeinfo,0);
}

/**
 * @brief returns the water temperature sensor (if available) or nulwaterlptr if not
 * 
 * @return TempSensor* pointer to the water temperature sensor (if available) or nullptr if not
 */
TempSensor * SensorDataFile::GetWaterTempSensor() {
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==WATER_TEMP_DATA) {
			TempSensor *pTempSensor = (TempSensor *)m_sensorObjects[i];
			return pTempSensor;	
		}
	}
	return nullptr;
}

/**
 * @brief returns the PH sensor (if available) or nullptr if not
 * 
 * @return PHSensor* the PHSensor object (if available) or nullptr if not 
 */
PHSensor * SensorDataFile::GetPHSensor() {
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==PH_DATA) {
			PHSensor *pPHSensor = (PHSensor *)m_sensorObjects[i];
			return pPHSensor;	
		}
	}
	return nullptr;
}


/**
 * @brief returns the turbidity sensor (if available) or nullptr if not
 * 
 * @return TurbiditySensor* returns the TurbiditySensor object (if available) or nullptr if not
 */
TurbiditySensor * SensorDataFile::GetTurbiditySensor() {
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==WATER_TURBIDITY) {
			TurbiditySensor *pTurbiditySensor = (TurbiditySensor *)m_sensorObjects[i];
			return pTurbiditySensor;	
		}
	}
	return nullptr;
}

/**
 * @brief returns the leak sensor (if available) or nullptr if not
 * 
 * @return LeakSensor* the LeakSensor object (if available) or nullptr if not
 */
LeakSensor * SensorDataFile::GetLeakSensor() {
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==LEAK_DATA) {
			LeakSensor *pLeakSensor = (LeakSensor *)m_sensorObjects[i];
			return pLeakSensor;	
		}
	}
	return nullptr;
}

/**
 * @brief Get the Diagnostics Sensor object
 * 
 * @return DiagnosticsSensor* the object used for boat diagnostics and control routines
 */
DiagnosticsSensor * SensorDataFile::GetDiagnosticsSensor() {
	for (int i=0;i<m_nNumSensors;i++) {
		if (m_sensorTypes[i]==DIAGNOSTICS_DATA) {
			DiagnosticsSensor *pDiagnosticsSensor = (DiagnosticsSensor *)m_sensorObjects[i];
			return pDiagnosticsSensor;	
		}
	}
	return nullptr;
}


/**
 * @brief change the name of the file used for saving data
 * 
 * @param pszFilename is the new name of the file to use for saving data
 */
void SensorDataFile::SetFilename(char * pszFilename) {
	if (m_szDataFilePath) {
		delete []m_szDataFilePath;
		m_szDataFilePath=nullptr;
	}
	if (m_dataFile) {
		if (m_dataFile->is_open()) {
			m_dataFile->close();
		}
		delete m_dataFile;
		m_dataFile=nullptr;
	}
	m_szDataFilePath = new char[strlen(pszFilename)+1];
	strcpy(m_szDataFilePath, pszFilename);
	CreateDataFile();
}

/**
 * @brief collect data from all available sensors (GPS not included)
 * 
 */
void SensorDataFile::CollectData() {
	for (int i=0;i<m_nNumSensors;i++) {
		if (!m_sensorObjects[i]) {
			continue;
		}
		if (m_sensorTypes[i]==BOAT_INTERIOR_TEMP_DATA) {//boat interior temperature
			Navigation *pNav = (Navigation *)m_sensorObjects[i];
			double dInteriorTemperature=pNav->GetBoatTemp();
			SetData(BOAT_INTERIOR_TEMP_DATA, dInteriorTemperature);//sets data value for interior boat temperature, data will get saved to file in the next call to SaveData
		}
		else if (m_sensorTypes[i]==WATER_TEMP_DATA) {//water temperature
			TempSensor *pTempSensor = (TempSensor *)m_sensorObjects[i];
			float fWaterTemperature=0;//current water temperature in deg C
			if (pTempSensor->GetTemperature(fWaterTemperature)) {//get water temperature
				SetData(WATER_TEMP_DATA, (double)fWaterTemperature);//sets data value for water temperature, data will get saved to file in the next call to SaveData
			}
		}
		else if (m_sensorTypes[i]==PH_DATA) {//water pH
			PHSensor *pPHSensor = (PHSensor *)m_sensorObjects[i];
			double dPHVal=0.0;//water pH
			if (pPHSensor->GetPHSensorPH(dPHVal)) {//get the pH probe pH value, using currently available pH probe calibration parameters, data will get saved to file in the next call to SaveData
				SetData(PH_DATA, dPHVal);//sets data value for water pH
			}
		}
		else if (m_sensorTypes[i]==WATER_TURBIDITY) {//water turbidity
			TurbiditySensor *pTurbiditySensor = (TurbiditySensor *)m_sensorObjects[i];
			double dTurbidityVal=0.0;//water turbidity
			if (pTurbiditySensor->GetTurbidity(dTurbidityVal)) {//get water turbidity
				SetData(WATER_TURBIDITY, dTurbidityVal);//sets data value for water turbidity
			}
		}
		else if (m_sensorTypes[i]==LEAK_DATA) {//leak sensor data
			LeakSensor *pLeakSensor = (LeakSensor *)m_sensorObjects[i];
			m_bLeak = pLeakSensor->CheckLeakSensors(nullptr);
			SetData(LEAK_DATA,(double)m_bLeak);
		}
		else if (m_sensorTypes[i]==DIAGNOSTICS_DATA) {//diagnostics data
			DiagnosticsSensor *pDiagnostics = (DiagnosticsSensor *)m_sensorObjects[i];
			m_bSolarCharging = pDiagnostics->IsSolarCharging();
			float  fCurrentDraw=0;
			pDiagnostics->GetBatteryVoltage(m_fBatteryVoltage);
			if (pDiagnostics->GetCurrentDraw(fCurrentDraw)) {
				SetData(DIAGNOSTICS_DATA,(double)fCurrentDraw);
			}
			pDiagnostics->GetHumidityAndTemp(m_fHumidityCPU,m_fHumidityTempCPU,CPUBOX);
			pDiagnostics->GetHumidityAndTemp(m_fHumidityBattery, m_fHumidityTempBattery,BATTERYBOX);
		}
	}
}

/**
 * @brief collects sensor data from all available sensors and then saves it 
 * 
 */
void SensorDataFile::CollectAndSaveDataNow() {
	CollectData();
	//get current time and date for sample
	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	SaveData(timeinfo,0);
}