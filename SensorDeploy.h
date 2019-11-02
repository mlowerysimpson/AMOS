#define ZERO_SD_PULSE_TIME 1520 //pulse time in microseconds required for sending the midway angle to the servo motor connected to the sensor deployment arm
#define MIN_SD_PULSE_TIME 800 //mininum pulse time in microseconds, corresponds to the fully deployed state of the sensor deployment arm (i.e. sensors should be in the water)
#define MAX_SD_PULSE_TIME 2300 //maximum pulse time in microseconds, corresponds to the fully retracted state of the sensor deployment arm (i.e. sensors should be retracted out of the water onto the boat deck)
#define SD_PULSE_INTERVAL 10000 //interval in microseconds between pulses
#define DEPLOYMENT_PULSE_PIN 21 //pin used for sending pulses to the servo motor that is used for deploying / retracting sensors

class SensorDeploy {
public:
	SensorDeploy(char *rootFolder, bool bInitializeWiringPi);//SensorDeploy constructor
	~SensorDeploy();
	void Deploy();//move the servo motor to deploy the sensors into the water
	void Retract();//move the servo motor to retract the sensor from the water back onto the boat deck
	static void OneTimeSetup();//setup procedure to be performed once per power-cycle of the servo deployment arm


private:
    //data
    char *m_rootFolder;//the root program folder, where the prefs.txt preferences file is located (can be accessed to get last known sensor arm location)
    bool m_bDeployed;//set to true when the sensor arm is fully deployed into the water
    
	//functions
    void SaveSettings();//save the current state of the sensor deployment arm to the preferences file
    void GetSettings();//try to get info about current state of sensor arm from preferences file
    void Pulse (int nPulseTimeMicroseconds, int nDurationMilliseconds);//send pulses of width nPulseTimeMicroseconds for a duration of nDurationMilliseconds
};
