#include <stdio.h>      
#include <unistd.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <wiringSerial.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <netdb.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <stropts.h>
#include <errno.h>
#include <vector>
#include <string> 
#include <string.h>
#include <pthread.h>
#include "filedata.h"
#include "RemoteCommand.h"
#include "ShipLog.h"
#include "FileCommands.h"
#include "LeakSensor.h"
#include "TempSensor.h"
#include "BatteryCharge.h"
#include "NotifyOperator.h"
#include "HelixDepth.h"
#include "AToD.h"
#include "PHSensor.h"
#include "TurbiditySensor.h"
#include "AtlasDO2Sensor.h"
#include "AMLTempConductivity.h"
#include "SensorDataFile.h"
#include "DiagnosticsSensor.h"
#include "HumiditySensor.h"
#include "Vision.h"
#include "SwitchRelay.h"
#include "SensorDeploy.h"
//#include "TFMini.h" // use this if using TF Mini LiDAR
#include "LIDARLite.h"
#include "Util.h"

using namespace std;

//#define IMUTEST_BUILD 1 //uncomment when building the IMUTest program or any other program that refers to this file
#define SERVER_PORT 81 //the port number that we are listening on for network connections 
#define SHIP_LOG_INTERVAL_SECONDS 60 //length of time in seconds between ship's log posts
#define ATOD_ADDRESS 0x48 //I2C address of 4-channel ADS1115 ADC converter
//#define LiDAR_SERPORT "/dev/serial0" //serial port used for communications with TFMini LiDAR device

vector <string> g_ipNames;
vector <string> g_ipAddresses;
NotifyOperator g_notifier;//used for notifying one or more people by email or text of the boat's condition
Navigation *g_navigator;//used for handling all of the boat's navigation needs
AToD *g_atod;//used for collecting A to D converter data (such as from battery voltage measurement and various analog sensor measurements
BatteryCharge *g_batteryCharge;//used for monitoring the battery charge level
FileCommands *g_fileCommands;//used for handling commands sent to boat through a text file passed on the command line of this program
Thruster *g_thrusters;//pointer to Thruster object for controlling power to the boat's thruster(s)
ShipLog g_shiplog;//used for logging data and to assist in debugging
LeakSensor g_leaksensor;//interface for leak sensor, poll it to make sure boat is not leaking.
TempSensor *g_tempsensor;//interface for waterproof temperature sensor, poll to monitor temperature of water
DiagnosticsSensor *g_shipdiagnostics;//interface for ship monitoring, power control, etc.
PHSensor *g_PHSensor;//interface object for pH probe
TurbiditySensor *g_turbiditySensor;//interface object for turbidity sensor
AtlasDO2Sensor* g_do2Sensor;//interface object for dissolved oxygen sensor
AMLTempConductivity* g_tempConductivitySensor;//interface object for temperature / conductivity sensor
SwitchRelay *g_switchRelay;//interface object for controlling switch relay (eg. for switching solar input to charge controller on / off)
HelixDepth* g_helixDepth;//interface for getting depth readings from Humminbird Helix fish finder
Vision g_vision;//object used for taking pictures with camera(s)
//TFMini m_miniLiDAR;//object used for getting distance measurements to objects using TFMini LiDAR
LIDARLite *g_lidar;//object used for getting distance measurements to objects using LIDAR Lite

//LiDAR parameters
bool g_bUseLiDAR;//true if LiDAR measurements are being used for detecting obstacles
double g_dLiDAR_IntervalSec;//interval in seconds between LiDAR measurements
bool g_bObjectPictures;//whether or not to take a picture whenever an obstacle is encountered with LiDAR measurements.	
bool g_bLiDARSafetyMode;//flag is true if thrusters should be switched off whenever a nearby obstacle is detected using liDAR
bool g_bLiDARAvoidanceMode;//flag is true if boat should alter its heading in order to try to avoid obstacles.

//other global parameters
char g_rootFolder[PATH_MAX];//root folder where this program starts from
vector <char *> g_commandLineParams;//command line parameters for this program
unsigned int g_lastNetworkCommandTime;//last network command time (for controlling thrusters) in ms
char g_remote_ipaddr[32];//the remote IP address 
bool g_bVoltageSwitchedOff;//true if the user has switched off battery power on AMOS (i.e. measured voltage has shifted from a high value to a low value)
double g_dBatteryVoltage;//the voltage of the battery, should be >= 12 volts
bool g_bEnteringSleepMode;//flag is true when AMOS is about to be put to sleep (i.e. powered down) with just the RFU220SU running
bool g_bKeepGoing;//set to true to keep listening for incoming commands from remote host(s)
bool g_bCancelFunctions;//boolean flag is set to true, typically when program is ending. It is polled by various functions (ex: navigation functions) that sometimes take a long time to complete
bool g_bNetServerThreadRunning;//boolean flag is true when the network server thread is running (monitor flag to make sure that socket gets closed correctly)
bool g_bSerportThreadRunning;//boolean flag is true when the serial port thread is running (monitor flag to make sure that thead and serial port get closed correctly)
bool g_bGPSThreadRunning;//boolean flag is true when the gps collection thread is running
bool g_bSensorThreadRunning;//boolean flag is true when the sensor collection thread is running
bool g_bExitFileCommandsThread;//boolean flag used to control when the file commands thread should exit
bool g_bFileCommandsThreadRunning;//boolean flag is true when the file commands thread is running
char *g_serPort;//the text identifier of the serial port being used for a wireless serial link (set to nullptr if not used)
char g_keyboardBuf[4];//used to keep track of the last 4 characters typed by the user, looking for the word "quit" to exit the program
int g_keyTypedIndex;//index within g_keyboardBuf for where to store the next keyboard character typed
char* g_szRTKPort;//serial port used for sending RTK correction data to GPS receiver board
int g_nSensorStabilizeTimeSec;//length of time required for stabilization before taking sensor measurements (if non-zero, thrusters are turned off prior to taking measurements)

//thread IDs
pthread_t g_networkThreadId;//thread id for thread that listens for incoming network connections
pthread_t g_gpsThreadId;//thread id for collecting GPS data
pthread_t g_sensorThreadId;//thread id for collecting sensor data
pthread_t g_threadFileCommandsId;//thread id for getting file commands
pthread_t g_serportThreadId;//thread id for getting data over a serial (wireless) link
pthread_mutex_t g_remoteCommandsMutex;//mutex for making sure remote commands do not interfere with one another
pthread_mutexattr_t g_remoteCommandsMutexAttr;//mutex attribute for remote commads mutex (allows it to be recursive)
pthread_mutex_t g_i2cMutex;//mutex for controlling access to i2c bus
SensorDataFile *g_sensorDataFile;//object used for logging sensor data to file

//variables controlling logging of data
bool g_bLogGPSData;//flag indicates whether or not to log GPS data to the sensor data file (GPS data also gets logged to the general ship log file)
bool g_bLogWaterTemp;//flag indicates whether or not to log water temperature to file
bool g_bLogBoatTemp;//flag indicates whether or not to log interior boat temperature to file
int g_nLoggingIntervalSec;//the interval in seconds between samples of data (set to zero to disable periodic logging of data)

void ShowUsage() {
	printf("RemoteControlTest version 1.0\n");
	printf("This program is used to operate and collect data from AMOS (the robotic boat).\n");
	printf("Usage: RemoteControlTest server_ip_addr|ser_port [command_file]\n");
	printf("server_ip_addr: the IP address of the server software. Either a server IP address (eg: 192.168.2.1) or the name of the serial port used for wireless communications on this computer (eg: /dev/serial1) needs to be specified.\n");
	printf("command_file: an optional text command file that AMOS should execute.\n");
	printf("-h or --help: displays this help text.\n");

}

void *serportFunction(void *pParam) {//function receives commands from base station over (wireless) serial link
	g_shiplog.LogEntry((char *)"serial port function started...\n",true);
	g_bSerportThreadRunning=true;
	char sMsg[128];
	char buffer[256];
	char *szSerportName = (char  *)pParam;
	int fd = serialOpen (szSerportName, 38400);
	if (fd<0) {
		printf ("Unable to open serial device: %s\n", strerror (errno));
		g_bSerportThreadRunning = false;
		return nullptr;
	}
	if (g_shipdiagnostics==nullptr) {
		strcpy(sMsg,"Error, ship diagnostics object not created.\n");
		g_bSerportThreadRunning = false;
		return nullptr;
	}
	//g_shipdiagnostics->SetSerportHandle(fd);
	sprintf(sMsg,"Listening for commands on port %s\n",szSerportName);
	g_shiplog.LogEntry(sMsg,true);

	RemoteCommand *pRC = new RemoteCommand((char *)g_rootFolder, fd, g_navigator, g_thrusters, g_atod, (void *)&g_shiplog);
	pRC->SetSerportMode(true);//set the RemoteCommand object to use serial port communications
	while (g_bKeepGoing) {
		REMOTE_COMMAND *pCommandReceived = pRC->GetCommand();
		if (pCommandReceived)
		{
			if (pCommandReceived->nCommand == THRUST_ON || pCommandReceived->nCommand == THRUST_OFF||pCommandReceived->nCommand== SCRIPT_STATUS_PACKET||pCommandReceived->nCommand==SCRIPT_STEP_CHANGE||pCommandReceived->nCommand==FILE_TRANSFER)
			{
				g_lastNetworkCommandTime = millis();
			}
			sprintf(sMsg,"Serial Command Received: %d\n",pCommandReceived->nCommand);
			g_shiplog.LogEntry(sMsg,true);
			pthread_mutex_lock(&g_remoteCommandsMutex);
			pRC->ExecuteCommand(pCommandReceived, true);
			if (pCommandReceived->nCommand==QUIT_PROGRAM) {
				g_bKeepGoing=false;//remote command to quit program was received
				g_bCancelFunctions = true;//cancel any navigation functions that might be in progress
			}
			RemoteCommand::DeleteCommand(pCommandReceived);
			pthread_mutex_unlock(&g_remoteCommandsMutex);
		}
	}
	delete pRC;
	g_bSerportThreadRunning = false;
	//test
	printf("serport thread finished\n");	
	//end test
	return nullptr;
}

void *serverNetFunction(void* pParam) {//function connects to the boat server and sends and receives data with it
	g_shiplog.LogEntry((char*)"server network function started...\n", true);
	int sockfd;//socket description
	int portno = SERVER_PORT;
	g_bNetServerThreadRunning = true;
	char* serverIPAddress = (char*)pParam;
	char sMsg[128];
	struct sockaddr_in serv_addr;//server address
	struct sockaddr_in client_addr;//client address
	char buffer[256];
	
	while (g_bKeepGoing) {
		//create socket
		if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			g_shiplog.LogEntry((char*)"ERROR trying to create socket.\n", true);
			g_bNetServerThreadRunning = false;
			return nullptr;
		}
		//set option to resuse sockets
		int nReuse = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&nReuse, sizeof(int));
		int new_socket, c;
		
		//Prepare the sockaddr_in structure
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(SERVER_PORT);

		//Bind
		if (bind(sockfd, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) < 0)
		{
			sprintf(sMsg, (char*)"server binding failed. Last Error = %d\n", true, errno);
			g_shiplog.LogEntry(sMsg, true);
			g_shiplog.LogEntry(strerror(errno), true);
			g_bNetServerThreadRunning = false;
			return nullptr;
		}
		//Listen
		listen(sockfd, SOMAXCONN);

		//Accept an incoming connection
		g_shiplog.LogEntry((char*)"Waiting for an incoming connection...\n", true);
		c = sizeof(struct sockaddr_in);
		new_socket = accept(sockfd, (struct sockaddr*) & client_addr, (socklen_t*)&c);
		if (new_socket < 0)
		{
			g_shiplog.LogEntry((char*)"accept failed.\n", true);
			close(sockfd);
			continue;
		}
		g_shiplog.LogEntry((char*)"Connection accepted.\n", true);
		
		//receive passcode
		char passCode[32];
		char receivedCode[32];
		memset(passCode, 0, 32);
		memset(receivedCode, 0, 32);
		strcpy(passCode, (char*)PASSCODE_TEXT);
		int nNumRead = read(new_socket, receivedCode, strlen(passCode));
		if (nNumRead < strlen(passCode)) {
			g_shiplog.LogEntry((char*)"Error, timed out trying to receive passcode.\n", true);
			close(new_socket);
			close(sockfd);
			continue;
		}
		else if (strcmp(passCode, receivedCode) != 0) {
			g_shiplog.LogEntry((char*)"Error, incorrect passcode received.\n", true);
			close(new_socket);
			close(sockfd);
			continue;
		}

		//set timeout for receives	
		struct timeval tv;
		tv.tv_sec = 1;//set for 1 second timeout
		tv.tv_usec = 0;
		setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
		

		//keep trying to receive data on the accepted socket until an error occurs
		RemoteCommand* pRC = new RemoteCommand((char*)g_rootFolder, new_socket, g_navigator, g_thrusters, g_atod, (void*)&g_shiplog);
		int nErrorCount = 0;
		bool bSocketsClosed = false;
		while (g_bKeepGoing) {
			REMOTE_COMMAND* pCommandReceived = pRC->GetCommand();
			if (pCommandReceived) {
				nErrorCount = 0;
				if (pCommandReceived->nCommand == THRUST_ON || pCommandReceived->nCommand == THRUST_OFF || pCommandReceived->nCommand == SCRIPT_STATUS_PACKET || pCommandReceived->nCommand == SCRIPT_STEP_CHANGE) {
					g_lastNetworkCommandTime = millis();
				}
				//test
				sprintf(sMsg, "Net Command Received: %d\n", pCommandReceived->nCommand);
				g_shiplog.LogEntry(sMsg, true);
				//end test
				pthread_mutex_lock(&g_remoteCommandsMutex);
				pRC->ExecuteCommand(pCommandReceived, false);
				if (pCommandReceived->nCommand == QUIT_PROGRAM) {
					g_bKeepGoing = false;//remote command to quit program was received
					g_bCancelFunctions = true;//cancel any navigation functions that might be in progress
				}
				RemoteCommand::DeleteCommand(pCommandReceived);
				pthread_mutex_unlock(&g_remoteCommandsMutex);
			}
			else {//some error occurred trying to get data, so look for new connection
				nErrorCount++;
				if (nErrorCount >= 60) {//~  1 minute of no data, so try for new connection
					close(new_socket);
					close(sockfd);
					bSocketsClosed = true;
					break;
				}
			}
		}
		if (!bSocketsClosed) {
			close(new_socket);
			close(sockfd);
		}
		delete pRC;
	}
	shutdown(sockfd, SHUT_WR);
	close(sockfd);
	g_shiplog.LogEntry((char*)"exiting server network thread.\n", true);
	g_bNetServerThreadRunning = false;
	return nullptr;
}

void *gpsCollectionFunction(void *pParam) {
	const unsigned int LOG_INTERVAL = 60000;//interval between gps updates to the log file in ms
	unsigned int uiLastLogTime = 0;
	g_bGPSThreadRunning = true;
	//test
	printf("started gps collection thread\n");
	//end test
	while (g_bKeepGoing) {
		if (g_navigator) {
			g_navigator->CollectGPSData((void *)&g_shiplog, &g_bKeepGoing);
			unsigned int uiCurrentTime = millis();
			if (uiLastLogTime==0||(uiCurrentTime - uiLastLogTime)>=LOG_INTERVAL) {
				uiLastLogTime = uiCurrentTime;
				char *log_status = g_navigator->GetStatusLogText();
				if (log_status) {
					g_shiplog.LogEntry(log_status,true);
					delete []log_status;
				}
			}
		}
		usleep(500000);//need to pause less than 1 second to ensure that GPS data is current
	}
	g_bGPSThreadRunning = false;
	//test
	printf("finished gps collection thread\n");
	//end test
	return nullptr;
}

/**
 * @brief Puts AMOS into a low power state by powering down the Pi board running this program. It does this by sending a message to the RFU220 radio module, which switches a relay controlling power to the Pi board.
 * 
 * @param nWaitTimeSec the length of time to power down the Pi board (in minutes).
 */
void EnterSleepMode(int nWaitTimeSec) {
	if (nWaitTimeSec<MIN_SLEEPTIME_SEC) {//power down AMOS if the wait time is MIN_SLEEPTIME_SEC seconds or longer
		return;
	}
	if (g_shipdiagnostics==nullptr) {
		printf("No ship diagnostics object, cannot enter low power mode.\n");
		return;
	}
	//make sure prefs.txt file is set for auto-start so that this program can wake itself back up properly
	char prefsFilename[PATH_MAX];
	char startupParams[1024];//startup parameters to include on the command line when starting this program
	int nNumStartupParams = g_commandLineParams.size();
	
	sprintf(prefsFilename,"%s/prefs.txt",g_rootFolder);
	filedata prefsFile(prefsFilename);
	prefsFile.writeData("[startup]","autostart",1);
	if (nNumStartupParams>0) {
		strcpy(startupParams,g_commandLineParams[0]);
		bool bContinueParam = false;//set to true if one of the parameters is the "continue" flag
		for (int i=1;i<nNumStartupParams;i++) {
			strcat(startupParams,(char *)" ");
			strcat(startupParams,g_commandLineParams[i]);
			if (strcmp(g_commandLineParams[i],(char*)"continue")==0) {
				bContinueParam = true;
			}
		}
		if (g_fileCommands!=nullptr&&!bContinueParam) {
			//need to add a "continue" parameter so that when the program restarts it will try to resume at the same place in the file commands where it left off
			strcat(startupParams,(char *)" continue");
		}
		prefsFile.writeData("[startup]","params",startupParams);	
	}
	prefsFile.closeDataFile();
	//exits this program and puts AMOS into a low-power state in which only the RFU220SU is powered up
	//test
	char sMsg[256];
	sprintf(sMsg,"About to enter sleep mode for %d minutes.\n",nWaitTimeSec/60);
	g_shiplog.LogEntry(sMsg,true);
	//end test
	if (!g_shipdiagnostics->EnterSleepMode(nWaitTimeSec)) {
		printf("Error trying to enter low power mode.\n");
		return;
	}
	//command has been sent to go to sleep, RFU220 should pause for 15 seconds, and then power down the Pi running this program. Calling function needs to call system("halt") before that
	g_bKeepGoing = false;
	g_bEnteringSleepMode = true;
}


void *sensorCollectionFunction(void *pParam) {
	const double BATT_VOLTAGE_CUTOFF = 6.0;//if the battery voltage falls below this level, then we can assume that the user must have switched off the +12V power supply
	const double FULL_POWER_VOLTAGE = 13.0;//if the battery voltage rises above this level, then we can assume that the battery has become fully charged and the boat is capable of full-speed operation
	const double LOW_POWER_VOLTAGE = 11.33;//if the battery voltage falls below this level, then there is not much charge left, need to conserve power so shutoff power to thrusters and hope for sunlight!
	const int MIN_LIDAR_DETECTIONS = 3;//number of consecutive LiDAR readings to require before taking some sort of evasive action (avoids false positives which tend to appear on bright days)
	unsigned int uiLastBattVoltageMeasurement = 0;//time in ms of last battery voltage measurement
	unsigned int uiLastLiDARMeasurement = 0;//time in ms of last LiDAR measurement
	unsigned int uiObjectCount = 0;//number of consecutive times that object has been detected using LiDAR
	g_bSensorThreadRunning = true;
	//test
	printf("started sensor collection thread\n");
	//end test
	while (g_bKeepGoing) {
		unsigned int uiCurrentTime = millis();
		//compass data
		if (g_navigator) {
			if (g_navigator->CollectCompassData((void *)&g_shiplog)) {
				double dBoatTemp = g_navigator->GetBoatTemp();
				if (g_sensorDataFile&&g_bLogBoatTemp) {
					g_sensorDataFile->SetData(BOAT_INTERIOR_TEMP_DATA, dBoatTemp);
				}
			}
			if (g_bLogGPSData&&g_navigator->HaveValidGPS()) {
				g_sensorDataFile->SetData(GPS_LATITUDE, g_navigator->GetLatitude());
				g_sensorDataFile->SetData(GPS_LONGITUDE, g_navigator->GetLongitude());
			}
		}
		//check leak sensors and respond accordingly if leak has occurred
		bool bLeak = g_leaksensor.CheckLeakSensors((void *)&g_shiplog);
		if (bLeak&& g_bKeepGoing) {
			if (!g_leaksensor.HasSentLeakNotification()) {
				g_leaksensor.SendLeakNotification((void *)&g_shiplog, &g_notifier);
			}
			if (!g_leaksensor.HasRespondedToLeak()) {
				int nLeakResponseType = g_leaksensor.GetLeakResponse();
				g_navigator->ExecuteLeakResponse(nLeakResponseType,(void *)&g_shiplog);//special emergency function for getting to shore as quickly as possible
				g_leaksensor.RespondedToLeak((void *)&g_shiplog, true);
			}
			else if (g_navigator->driftedFromEmergencyStop()) {//we have drifted a bit away from the previous emergency stopping point, need to try to get back to shore
				int nLeakResponseType = g_leaksensor.GetLeakResponse();
				g_navigator->ExecuteLeakResponse(nLeakResponseType,(void *)&g_shiplog);//special emergency function for getting to shore as quickly as possible
			}
		}
		//check to see if battery voltage measurement should be made
		if (((uiCurrentTime - uiLastBattVoltageMeasurement)>=60000)&& g_bKeepGoing) {//do battery voltage measurement every minute
			uiLastBattVoltageMeasurement = uiCurrentTime;
			double dBattVoltage = 0.0;
			if (g_atod->GetBatteryVoltage(dBattVoltage,g_batteryCharge)) {
				//check to see if battery voltage is too high (need to shut off solar input to charge controller)
				if (g_switchRelay->isOverVoltage(dBattVoltage)) {//voltage is too high
					//switch off solar input to charge controller
					g_shiplog.LogEntry((char *)"Turning off solar power.\n",true);
					g_switchRelay->TurnOnSolarPower(false);
				}
				else if (!g_switchRelay->isFullyCharged(dBattVoltage)) {//battery is not fully charged
					if (!g_switchRelay->isSwitchedOn()) {//solar input to controller was switched off previously, so switch it back on now
						g_shiplog.LogEntry((char *)"Turning on solar power.\n",true);
						g_switchRelay->TurnOnSolarPower(true);
					}
				}
				if (g_dBatteryVoltage>BATT_VOLTAGE_CUTOFF) {
					if (dBattVoltage<BATT_VOLTAGE_CUTOFF) {
						//user must have switched off battery voltage, stop program and set flag that +12 V power has been switched off
						//but first re-check voltage a to make sure that it wasn't just a momentary glitch
						if (g_atod->GetBatteryVoltage(dBattVoltage, g_batteryCharge)) {
							if (dBattVoltage<BATT_VOLTAGE_CUTOFF) {
								g_shiplog.LogEntry((char *)"User switched off power.\n",true);
								g_bKeepGoing=false;
								g_bCancelFunctions=true;
								g_bVoltageSwitchedOff=true;
							}
						}
					}
				}

				if (g_navigator&&!g_bVoltageSwitchedOff) {
					if (!g_batteryCharge->isChargeOK()) {//charge levels are getting pretty low, need to shut down thrusters, send alarm notification, go into low power mode, and hope for sunshine
						bool bFileCommandSleep = false;
						bool bTravelToSunnySpot = false;
						//check to see if the file commands object is already set to make AMOS go into sleep mode
						if (g_fileCommands!=nullptr) {
							if (g_fileCommands->m_bSleepTime) {
								bFileCommandSleep = true;
							}
							bTravelToSunnySpot = g_fileCommands->m_bTravelToSunnySpot;
							if (!g_fileCommands->m_bTraveledToSunnySpot&&!bTravelToSunnySpot) {
								char szMsg[256];
								sprintf(szMsg,"Battery is only %.2f V, set flag to go to sunny spot.\n",dBattVoltage);
								g_shiplog.LogEntry(szMsg,true);
								g_fileCommands->TravelToSunnySpot(dBattVoltage,g_batteryCharge);
								bTravelToSunnySpot = true;
							}
						}
						if (!bTravelToSunnySpot) {//not trying to travel to a sunny spot for recharging
							//turn off thrusters
							g_navigator->SetPowerMode(LOW_POWER_MODE,dBattVoltage,&g_shiplog);
						
							if (!bFileCommandSleep) {
								//send alarm notification that power levels are low
								char szAlarmMsg[256];
								char szSubject[256];
								sprintf(szAlarmMsg,"Low battery alarm! Voltage = %.2f V. Pos = %.6f, %.6f, Entering low power for one hour.\n",
									dBattVoltage,g_navigator->GetLatitude(),g_navigator->GetLongitude());
								strcpy(szSubject,(char *)"Low Battery Alarm!");
								g_notifier.IssueNotification(szAlarmMsg,szSubject,(void *)&g_shiplog);
								//enter low power mode for an hour
								EnterSleepMode(3600);
							}
						}
					}
					else if (dBattVoltage>=FULL_POWER_VOLTAGE) {//charge levels are pretty full, can enable full-speed operation if not already in that mode
						g_navigator->SetPowerMode(FULL_POWER_MODE,dBattVoltage,&g_shiplog);
					}
				}
				g_dBatteryVoltage = dBattVoltage;
			}
			else {
				//function call to get battery voltage failed for some reason
				g_shiplog.LogEntry((char *)"Error, failed to get battery voltage.\n",true);
				if (g_switchRelay->isSwitchedOn()) {
					//try switching solar power off
					g_switchRelay->TurnOnSolarPower(false);
					g_shiplog.LogEntry((char *)"Switching solar power off to see if that fixes voltage measurements.\n",true);
				}
				else {
					//try switching solar power on
					g_switchRelay->TurnOnSolarPower(true);
					g_shiplog.LogEntry((char *)"Switching solar power off to see if that fixes voltage measurements.\n",true);
				}
			}
		}
		//check to see if LiDAR measurement should be made
		if (g_bUseLiDAR&& g_bKeepGoing&&(uiCurrentTime - uiLastLiDARMeasurement)>=1000*g_dLiDAR_IntervalSec) {//do LiDAR measurement
			//double dDistMeters = m_miniLiDAR.getDistance()*.01; //--> uncomment for TF Mini LiDAR
			double dDistMeters = g_lidar->getDistance()*.01;
			//unsigned short signalStrength = m_miniLiDAR.getRecentSignalStrength(); //--> uncomment for TF Mini LiDAR
			//if (signalStrength>=MIN_SIGNAL_STRENGTH&&dDistMeters<13) { //--> uncomment for TF Mini LiDAR
			if (dDistMeters>0.05&&dDistMeters<13) {//sometimes get 0.01 false readings, so look for > 0.05
				//detected some sort of obstacle
				uiObjectCount++;
				if (uiObjectCount>=MIN_LIDAR_DETECTIONS) {
					char szTest[256];
					sprintf(szTest,"Object detected at %.2f m\n",dDistMeters);
					g_shiplog.LogEntry(szTest,true);
					if (g_bObjectPictures) {
						g_vision.CaptureFrameWithDistText(dDistMeters);//capture frame of video with an estimate of the distance to the object (in m) superimposed on the picture (use autocapture settings for filename)
					}
					if (g_bLiDARSafetyMode) {//make sure thrusters are turned off in order to minimize potential impact from collision
						if (g_thrusters!=nullptr&&!g_thrusters->isInSafetyMode()) {
							char obstacleMsg[128];
							sprintf(obstacleMsg,(char *)"Obstacle detected %.2f m away. Safety mode engaged, thrusters turned off.\n",dDistMeters);
							g_shiplog.LogEntry(obstacleMsg,true);
							g_thrusters->SetObstacleSafetyMode(true);
						}
					}
					else if (g_bLiDARAvoidanceMode) {
						pthread_mutex_lock(&g_remoteCommandsMutex);
						g_navigator->AddObstacleAtCurrentHeading((void *)&g_shiplog);//inform the navigation object that there is an obstacle at the current heading
						pthread_mutex_unlock(&g_remoteCommandsMutex);
					}
				}
			}
			else {//no obstacles, or at least nothing within 13 m
				uiObjectCount=0;
				if (g_bLiDARSafetyMode) {//make sure thrusters are turned off in order to minimize potential impact from collision
					if (g_thrusters&&g_thrusters->isInSafetyMode()) {		
						g_shiplog.LogEntry((char *)"Obstacle no longer detected. Safety mode disengaged.\n",true);
						g_thrusters->SetObstacleSafetyMode(false);
					}
				}
			}
			uiLastLiDARMeasurement = uiCurrentTime;
		}
		//check to see if it's time to log sensor data to file
		if (g_sensorDataFile!=nullptr&&g_nLoggingIntervalSec>0&& g_bKeepGoing) {
			time_t rawtime;
			struct tm * timeinfo;
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			if (g_sensorDataFile->isTimeToLogSensorData(timeinfo)) {
				pthread_mutex_lock(&g_remoteCommandsMutex);
				if (g_nSensorStabilizeTimeSec > 0&&g_bKeepGoing) {
					g_thrusters->Stop();
					usleep(g_nSensorStabilizeTimeSec*1000000);//wait for sensors to stabilize in water
				}
				if (g_bKeepGoing)
				{
					g_sensorDataFile->CollectData();
					g_sensorDataFile->SaveData(timeinfo, g_nLoggingIntervalSec);
				}
				pthread_mutex_unlock(&g_remoteCommandsMutex);
			}
		}
	}
	g_bSensorThreadRunning = false;
	return nullptr;
}

void *fileCommandsFunction(void *pParam) {
	g_bFileCommandsThreadRunning = true;
	g_bExitFileCommandsThread = false;
	while (g_bKeepGoing&&!g_bExitFileCommandsThread) {
		if (g_fileCommands) {
			if (g_lastNetworkCommandTime>0) {//don't execute file commands within 10 seconds of the last network command time
				while ((millis() - g_lastNetworkCommandTime)<10000&&g_bKeepGoing&&!g_bExitFileCommandsThread) {
					usleep(1000000);
				}
			}
			if (!g_bKeepGoing || g_bExitFileCommandsThread) {
				g_bFileCommandsThreadRunning = false;
				return nullptr;
			}
			int nCommandAction = g_fileCommands->DoNextCommand(&g_remoteCommandsMutex,&g_lastNetworkCommandTime,(void *)&g_shiplog);
			if (nCommandAction==FC_WAIT) {
				int nWaitTimeSec = g_fileCommands->m_nWaitTimeSeconds;
				if (nWaitTimeSec>=MIN_SLEEPTIME_SEC) {
					//increment index of file command and save to prefs.txt
					g_fileCommands->IncrementAndSaveCommandIndex();
					EnterSleepMode(nWaitTimeSec);
					g_bFileCommandsThreadRunning = false;
					return nullptr;
				}
				else if (nWaitTimeSec>0) {
					//just delay for the required number of seconds and then keep going in this thread
					delay(nWaitTimeSec*1000);
				}
			}
			else if (g_fileCommands->m_bSleepTime) {
				//time to put AMOS to sleep
				//increment index of file command and save to prefs.txt
				g_fileCommands->IncrementAndSaveCommandIndex();
				EnterSleepMode(g_fileCommands->GetSleepTimeHrs()*3600);
				g_bFileCommandsThreadRunning = false;
				return nullptr;
			}
		}
	}
	g_bFileCommandsThreadRunning = false;
	return nullptr;
}

bool isSomethingToLog() {//return true if boat is logging at least some data
	if (g_bLogWaterTemp||g_bLogBoatTemp||g_bLogGPSData||g_PHSensor!=nullptr||g_turbiditySensor!=nullptr||g_do2Sensor!=nullptr||g_tempConductivitySensor!=nullptr) {
		return true;
	}
	return false;
}


//program used to accept remote control commands for AMOS

void ExecuteCommand(REMOTE_COMMAND *pCommand) {//executes the remote command nCommand
	char szCommand[256];
	if (pCommand!=nullptr) {
		char *szCommandDescription = RemoteCommand::GetCommandDescription(pCommand);
		if (szCommandDescription!=nullptr) {
			sprintf(szCommand,"Command: %s",szCommandDescription);
			g_shiplog.LogEntry(szCommand,true);
			delete []szCommandDescription;
		}
		else {
			//do nothing
		}
	}
}

void Cleanup() {//remove any un-executed commands from the queue, do general cleanup of memory, and cancel any running threads
	//pause for a bit to allow network client thread to exit cleanly, so that socket connection (if any) gets properly closed
	//test
	printf("Entering cleanup.\n");
	//end test
	unsigned int uiTimeoutTime = millis() + 30000;//allow up to 30 seconds for threads to exit cleanly
	while (millis()<uiTimeoutTime&&(g_bNetServerThreadRunning||g_bSerportThreadRunning||g_bGPSThreadRunning||g_bSensorThreadRunning||g_bFileCommandsThreadRunning)) {
		usleep(100000);//sleep for 100 ms
	}
	if (millis()>uiTimeoutTime) {
		printf("timeout occurred\n");
		if (g_bNetServerThreadRunning) {
			printf("Net server thread did not exit cleanly.\n");
		}
		if (g_bSerportThreadRunning) {
			printf("Serial port thread did not exit cleanly.\n");
		}
		if (g_bGPSThreadRunning) {
			printf("GPS thread did not exit cleanly.\n");
		}
		if (g_bSensorThreadRunning&&!g_bEnteringSleepMode) {
			printf("Sensor thread did not exit cleanly.\n");
		}
	}
	if (g_sensorDataFile) {
		delete g_sensorDataFile;
		g_sensorDataFile=nullptr;
	}
	if (g_networkThreadId) {
		pthread_cancel(g_networkThreadId);
		g_networkThreadId=0;
	}
	if (g_serportThreadId) {
		pthread_cancel(g_serportThreadId);
		g_serportThreadId=0;
	}
	if (g_gpsThreadId) {
		pthread_cancel(g_gpsThreadId);
		g_gpsThreadId=0;
	}
	if (g_sensorThreadId&&!g_bEnteringSleepMode) {
		pthread_cancel(g_sensorThreadId);
		g_sensorThreadId=0;
	}
	if (g_threadFileCommandsId) {
		pthread_cancel(g_threadFileCommandsId);
		g_threadFileCommandsId=0;
	}
	if (g_fileCommands) {
		delete g_fileCommands;
		g_fileCommands=nullptr;
	}
	if (g_helixDepth) {
		delete g_helixDepth;
		g_helixDepth = nullptr;
	}
	if (g_navigator) {
		delete g_navigator;
		g_navigator=nullptr;
	}
	if (g_atod) {
		delete g_atod;
		g_atod = nullptr;
	}
	if (g_lidar) {
		delete g_lidar;
		g_lidar = nullptr;
	}
	if (g_batteryCharge) {
		delete g_batteryCharge;
		g_batteryCharge = nullptr;
	}
	if (g_thrusters) {
		delete g_thrusters;
		g_thrusters=nullptr;
	}
	if (g_switchRelay) {
		delete g_switchRelay;
		g_switchRelay = nullptr;
	}
	if (g_shipdiagnostics&&!g_bEnteringSleepMode) {//need to preserve g_shipdiagnostics if we are entering sleep mode
		delete g_shipdiagnostics;
		g_shipdiagnostics = nullptr;
	}
	if (g_szRTKPort != nullptr) {
		delete[] g_szRTKPort;
		g_szRTKPort = nullptr;
	}
}

void StartNetworkServerThread(char *remoteIPAddr) {//start thread to create server that allows incoming network connections to control the boat and get data
	strcpy(g_remote_ipaddr,remoteIPAddr);
	int nError = pthread_create(&g_networkThreadId, NULL, &serverNetFunction, (void *)g_remote_ipaddr);
	if (nError != 0) {
		printf("Can't create thread: %s", strerror(nError));
	}
}

void StartSerialClientThread(char *serPort) {//start thread to read in commands over serial port connection
	if (g_serPort!=nullptr) {
		delete []g_serPort;
		g_serPort=nullptr;
	}
	g_serPort = new char[strlen(serPort)+1];
	strcpy(g_serPort,serPort);
	int nError = pthread_create(&g_serportThreadId, NULL, &serportFunction, (void *)g_serPort);
	if (nError!=0) {
		printf("Can't create thread: %s", strerror(nError));
	}
}

void StartGPSThread() {//start thread for receiving GPS data
	char sMsg[256];
	int nError = pthread_create(&g_gpsThreadId, NULL, &gpsCollectionFunction, NULL);
	if (nError!=0) {
		sprintf(sMsg,"Can't create thread: %s",strerror(nError));
		g_shiplog.LogEntry(sMsg,true);
	}
}

void StartDataCollectionThread() {//start thread for receiving data from sensors (compass, leak sensors, etc.)
	char sMsg[256];
	int nError = pthread_create(&g_sensorThreadId, NULL, &sensorCollectionFunction, NULL);
	if (nError!=0) {
		sprintf(sMsg,"Can't create thread: %s",strerror(nError));
		g_shiplog.LogEntry(sMsg,true);
	}
}

void StartFileCommandsThread() {//start thread for executing commands from a text file
	char sMsg[256];
	int nError = pthread_create(&g_threadFileCommandsId, NULL, &fileCommandsFunction, NULL);
	if (nError!=0) {
		sprintf(sMsg,"Can't create thread: %s",strerror(nError));
		g_shiplog.LogEntry(sMsg,true);
	}
}

bool GetDataLoggingPreferences() {//get data logging preferences from prefs.txt file
	char sMsg[256];
	char sPrefsFilename[PATH_MAX];
	sprintf(sPrefsFilename,"%s/prefs.txt", g_rootFolder);
	if (filedata::getFileLength(sPrefsFilename) <= 0) {
		//no data in the prefs.txt file. This can happen sometimes if AMOS's power switch is abruptly switched off by the user without stopping this program first. If the program happens to be writing prefs.txt at the time that power is killed it will typically leave it as an empty file.
		strcpy(sMsg, "prefs.txt file is empty or does not exist (program not shutdown cleanly last time?). Looking for backup prefs file...");
		g_shiplog.LogEntry(sMsg, true);
		if (Util::BackupExists(sPrefsFilename)) {
			Util::RestoreFromBackup(sPrefsFilename);
			strcpy(sMsg, "prefs.txt file restored from backup.");
			g_shiplog.LogEntry(sMsg, true);
		}
		else {
			//backup does not exist, need to create a valid prefs.txt file manually
			strcpy(sMsg, "Error, backup file for prefs.txt does not exist, please contact In Nature Robotics Ltd. for assistance");
			g_shiplog.LogEntry(sMsg, true);
			return false;
		}
	}
	else {
		//prefs.txt file exists and is non-zero, so update the current backup file to match it.
		Util::CreateBackup(sPrefsFilename);
	}
	filedata prefsFile(sPrefsFilename);
	//compass setting
	double dDeclination = prefsFile.getDouble((char*)"[compass]", "declination");
	g_navigator->SetDeclination(dDeclination);

	g_bLogWaterTemp = (bool)prefsFile.getInteger((char *)"[sensors]",(char *)"watertemp");
	g_bLogBoatTemp = (bool)prefsFile.getInteger((char *)"[sensors]",(char *)"boattemp");
	g_bLogGPSData = (bool)prefsFile.getInteger((char *)"[sensors]",(char *)"gps");
	bool bGetPHData = (bool)prefsFile.getInteger((char *)"[sensors]",(char *)"ph_probe");
	if (g_PHSensor!=nullptr) {//delete pH sensor object if it already exists
		delete g_PHSensor;
		g_PHSensor = nullptr;
	}
	if (bGetPHData) {
		PH_CALIBRATION phcal;
		phcal.dLowCalVoltage = prefsFile.getDouble((char *)"[sensors]",(char *)"ph_lowvolt");
		phcal.dLowCalPH = prefsFile.getDouble((char *)"[sensors]",(char *)"ph_lowph");
		phcal.dMidCalVoltage = prefsFile.getDouble((char *)"[sensors]",(char *)"ph_midvolt");
		phcal.dMidCalPH = prefsFile.getDouble((char *)"[sensors]",(char *)"ph_midph");
		int nPHChannel = prefsFile.getInteger((char *)"[sensors]",(char *)"ph_channel");//A to D channel for pH measurements
		g_PHSensor = new PHSensor(nPHChannel, g_atod, &phcal);
	}
	if (g_turbiditySensor != nullptr) {
		delete g_turbiditySensor;
		g_turbiditySensor = nullptr;
	}
	if (g_do2Sensor != nullptr) {
		delete g_do2Sensor;
		g_do2Sensor = nullptr;
	}
	if (g_tempConductivitySensor != nullptr) {
		delete g_tempConductivitySensor;
		g_tempConductivitySensor = nullptr;
	}
	if (g_tempsensor != nullptr) {
		delete g_tempsensor;
		g_tempsensor = nullptr;
	}
	bool bGetDO2Data = (bool)prefsFile.getInteger((char*)"[sensors]", (char*)"do2_sensor");
	if (bGetDO2Data) {
		g_do2Sensor = new AtlasDO2Sensor(&g_i2cMutex);
	}
	bool bGetTempConductivityData = (bool)prefsFile.getInteger((char*)"[sensors]", (char*)"temp_conductivity_sensor");
	if (bGetTempConductivityData) {
		char* szTempConductivitySerport = NULL;
		prefsFile.getString("[sensors]", "temp_conductivity_port", &szTempConductivitySerport);
		if (szTempConductivitySerport != nullptr) {
			g_tempConductivitySensor = new AMLTempConductivity(szTempConductivitySerport);
			delete[]szTempConductivitySerport;
			if (g_tempsensor != nullptr) {
				delete g_tempsensor;
				g_tempsensor = nullptr;
			}
			g_tempsensor = new TempSensor(g_tempConductivitySensor);
			g_bLogWaterTemp = true;
		}
	}
	else if (g_bLogWaterTemp) {
		g_tempsensor = new TempSensor();
	}
	bool bGetTurbidity = (bool)prefsFile.getInteger((char *)"[sensors]",(char *)"turbidity_sensor");
	if (bGetTurbidity) {
		int nTurbidityChannel = prefsFile.getInteger((char *)"[sensors]",(char *)"turbidity_channel");//A to D channel for turbidity measurements
		g_turbiditySensor = new TurbiditySensor(nTurbidityChannel, g_atod);
	}
	//camera preferences
	bool bUseCamera = (bool)prefsFile.getInteger((char *)"[camera]",(char *)"use_camera");//whether or not to use camera to continuously snap pictures
	int nIntervalSeconds = prefsFile.getInteger((char *)"[camera]",(char *)"interval_sec");//the number of seconds to wait between pictures 
	char *storage_folder = NULL;
	prefsFile.getString((char *)"[camera]",(char *)"storage_folder",&storage_folder);
	char *filename_prefix = NULL;
	prefsFile.getString((char *)"[camera]",(char *)"filename_prefix",&filename_prefix);
	g_vision.SetLoggingConfig(bUseCamera, nIntervalSeconds, storage_folder, filename_prefix);//sets the configuration for continuously logging pictures from the camera
	if (storage_folder) {
		delete []storage_folder;
		storage_folder = NULL;
	}
	if (filename_prefix) {
		delete []filename_prefix;
		filename_prefix = NULL;
	}

	//LiDAR preferences
	g_bUseLiDAR = (bool)prefsFile.getInteger((char *)"[lidar]",(char *)"use_lidar");//whether or not to use LiDAR measurments to detect objects
	g_dLiDAR_IntervalSec = prefsFile.getDouble((char *)"[lidar]",(char *)"interval_sec");//interval between LiDAR measurements in seconds (if really fast LiDAR measurements are required, it should probably be sampled in its own thread, this program is currently setup to just sample the LiDAR in the sensor data collection thread)
	g_bObjectPictures = (bool)prefsFile.getInteger((char *)"[lidar]",(char *)"object_pictures");//whether or not to take a picture whenever an obstacle is encountered.	
	g_bLiDARSafetyMode = (bool)prefsFile.getInteger((char *)"[lidar]",(char *)"safety_mode");//whether or not to turn off thrusters whenever an obstacle is encountered
	g_bLiDARAvoidanceMode = (bool)prefsFile.getInteger((char *)"[lidar]",(char *)"avoidance_mode");//whether or not to alter heading in order to try to avoid obstacles
	if (g_bUseLiDAR) {
		g_lidar = new LIDARLite(&g_i2cMutex);
		g_lidar->TurnOn(true);//turn on LiDAR
	}
	else {
		if (g_lidar != nullptr)
		{
			g_lidar->TurnOn(false);//turn off LiDAR to save power
			delete g_lidar;
		}
		g_lidar = nullptr;
	}

	//sensor preferences
	char *loggingInterval = NULL;
	g_nLoggingIntervalSec=0;
	prefsFile.getString((char *)"[sensors]",(char *)"data_interval",&loggingInterval);
	if (loggingInterval!=NULL) {
		int nHrs=0, nMin=0, nSec=0;
		sscanf(loggingInterval,"%d:%d:%d",&nHrs, &nMin, &nSec);
		g_nLoggingIntervalSec = nHrs*3600 + nMin*60 + nSec;
		delete[] loggingInterval;
	}
	g_nSensorStabilizeTimeSec = prefsFile.getInteger("[sensors]", "stabilization_time_sec");
	
	if (isSomethingToLog()) {//g_bLogWaterTemp||g_bLogBoatTemp||g_bLogGPSData||g_PHSensor!=nullptr||g_turbiditySensor!=nullptr) {
		void *sensorObjects[MAX_SENSORS];
		int sensorTypes[MAX_SENSORS];
		for (int i=0;i<MAX_SENSORS;i++) {
			sensorObjects[i]=nullptr;
			sensorTypes[i]=0;
		}
		int nNumSensors=0;
		if (g_bLogGPSData) {
			sensorTypes[nNumSensors] = GPS_LATITUDE;
			nNumSensors++;
			sensorTypes[nNumSensors] = GPS_LONGITUDE;
			nNumSensors++;
		}
		if (g_bLogWaterTemp) {
			sensorTypes[nNumSensors] = WATER_TEMP_DATA;
			sensorObjects[nNumSensors] = (void *)g_tempsensor;
			nNumSensors++;
		}
		if (g_bLogBoatTemp) {
			sensorTypes[nNumSensors] = BOAT_INTERIOR_TEMP_DATA;
			sensorObjects[nNumSensors] = (void *)g_navigator;
			nNumSensors++;
		}
		if (g_PHSensor) {
			sensorTypes[nNumSensors] = PH_DATA;
			sensorObjects[nNumSensors] = (void *)g_PHSensor;
			nNumSensors++;
		}
		if (g_turbiditySensor) {
			sensorTypes[nNumSensors] = WATER_TURBIDITY;
			sensorObjects[nNumSensors] = (void *)g_turbiditySensor;
			nNumSensors++;
		}
		if (g_do2Sensor) {
			sensorTypes[nNumSensors] = DO2_DATA;
			sensorObjects[nNumSensors] = (void*)g_do2Sensor;
			nNumSensors++;
		}
		if (g_tempConductivitySensor) {
			sensorTypes[nNumSensors] = CONDUCTIVITY_DATA;
			sensorObjects[nNumSensors] = (void*)g_tempConductivitySensor;
			nNumSensors++;
		}
		//leak sensor
		sensorTypes[nNumSensors] = LEAK_DATA;
		sensorObjects[nNumSensors] = (void *)&g_leaksensor;
		nNumSensors++;
		//diagnostics sensor
		sensorTypes[nNumSensors] = DIAGNOSTICS_DATA;
		sensorObjects[nNumSensors] = (void *)g_shipdiagnostics;
		nNumSensors++;
		if (g_sensorDataFile != NULL) {
			delete g_sensorDataFile;
			g_sensorDataFile = NULL;
		}
		char* szSensorDataFilename = NULL;
		prefsFile.getString("[sensors]", "sensor_filename", &szSensorDataFilename);
		if (szSensorDataFilename != NULL) {
			g_sensorDataFile = new SensorDataFile(sensorTypes, nNumSensors, sensorObjects, szSensorDataFilename, g_nLoggingIntervalSec);
			delete[] szSensorDataFilename;
		}
		else {
			g_sensorDataFile = new SensorDataFile(sensorTypes, nNumSensors, sensorObjects, (char*)"sensordata.txt", g_nLoggingIntervalSec);
		}
		//depth readings
		bool bDepthReadings = (bool)prefsFile.getInteger((char*)"[sensors]", (char*)"depth");
		if (bDepthReadings) {
			char* szDepthSerport = NULL;
			prefsFile.getString("[sensors]", "depth_port", &szDepthSerport);
			if (szDepthSerport) {
				char* szDepthFilename = NULL;
				prefsFile.getString("[sensors]", "depth_filename", &szDepthFilename);
				if (szDepthFilename) {
					if (g_helixDepth != NULL) {
						delete g_helixDepth;
						g_helixDepth = NULL;
					}
					g_helixDepth = new HelixDepth(g_navigator, szDepthSerport, szDepthFilename);
					delete[]szDepthFilename;
				}
				delete[]szDepthSerport;
			}
		}
		//GPS RTK correction data
		if (g_szRTKPort != nullptr) {
			delete[]g_szRTKPort;
			g_szRTKPort = nullptr;
		}
		prefsFile.getString("[gps]", "rtk_port", &g_szRTKPort);
	}
	return true;
}

/**
 * @brief check to see if the user has typed the text "quit" to exit the program
 * 
 * @param nKeyboardChar the character code for the key most recently pressed by the user
 * @return true if the last 4 characters typed by the user are "quit".
 * @return false otherwise.
 */
bool CheckUserQuit(int nKeyboardChar) {
	//save key typed by user
	const char *check_word = "quit";//text we are looking for to return true
	g_keyboardBuf[g_keyTypedIndex] = nKeyboardChar;
	g_keyTypedIndex++;
	//check to see if last 4 characters typed are equal to "quit"
	for (int i=0;i<4;i++) {
		int j = g_keyTypedIndex-4+i;
		if (j<0) j+=4;
		if (g_keyboardBuf[j]!=check_word[i]) {
			return false;
		}	
	}
	return true;//user typed check_word
}

void baz() {
 int *foo = (int*)-1; // make a bad pointer
  printf("%d\n", *foo);       // causes segfault
}

#ifndef  IMUTEST_BUILD

int main(int argc, const char * argv[]) {
	getcwd(g_rootFolder,PATH_MAX);
	g_shipdiagnostics = nullptr;
	g_sensorDataFile = nullptr;
	g_tempsensor = nullptr;
	g_PHSensor = nullptr;
	g_turbiditySensor = nullptr;
	g_do2Sensor = nullptr;
	g_tempConductivitySensor = nullptr;
	g_helixDepth = nullptr;
	g_serPort = nullptr;
	g_szRTKPort = nullptr;
	g_switchRelay = nullptr;
	g_bEnteringSleepMode = false;
	g_bLogWaterTemp = false;
	g_bLogBoatTemp = false;
	g_bNetServerThreadRunning=false;
	g_bSerportThreadRunning=false;
	g_bGPSThreadRunning = false;
	g_bSensorThreadRunning = false;
	g_bExitFileCommandsThread = false;
	g_bFileCommandsThreadRunning = false;
	g_nLoggingIntervalSec = 0;
	g_lastNetworkCommandTime=0;
	g_bVoltageSwitchedOff=false;
	g_dBatteryVoltage=0.0;
	g_bUseLiDAR = false;
	g_dLiDAR_IntervalSec = 0.0;
	g_bObjectPictures = false;
	g_bLiDARSafetyMode = false;
	g_bLiDARAvoidanceMode = false;
	memset(g_keyboardBuf,0,4);
	g_keyTypedIndex = 0;
	g_nSensorStabilizeTimeSec = 0;

	g_navigator = new Navigation(-18,&g_i2cMutex);//set to - 18 degrees magnetic declination for New Brunswick, Canada

	struct ifaddrs * ifAddrStruct = NULL;
	struct ifaddrs * ifa = NULL;
	void * tmpAddrPtr = NULL;
	g_bKeepGoing = true;
	g_bCancelFunctions=false;
	g_networkThreadId = 0;
	g_serportThreadId = 0;
	g_fileCommands=nullptr;
	
	pthread_mutexattr_init(&g_remoteCommandsMutexAttr);
	pthread_mutexattr_settype(&g_remoteCommandsMutexAttr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&g_remoteCommandsMutex, &g_remoteCommandsMutexAttr);
	g_i2cMutex = PTHREAD_MUTEX_INITIALIZER;
	Thruster::OneTimeSetup();//do I/O setup for thrusters (once per power-cycle), this function calls the WiringPi setup function

	
	pinMode(ACTIVITY_PIN,OUTPUT);//setup activity pin as output
	g_switchRelay = new SwitchRelay((char *)"prefs.txt");
	if (g_shipdiagnostics!=nullptr) {
		g_shipdiagnostics->ActivityPulse();//send activity pulse out on activity pin to indicate that program is running
	}
	
	//g_thrusters = new Thruster(true,true,false);//Thruster constructor for left & right propellers in water
	g_thrusters = new Thruster(false,false,true);//Thruster constructor for air propeller & rudder
	g_thrusters->Initialize();//more initialization for thrusters (makes sure they begin "stopped")

	
	char *i2c_filename = (char*)"/dev/i2c-1";
	g_atod = new AToD(i2c_filename,ATOD_ADDRESS,&g_shiplog,&g_i2cMutex);
	g_lidar = nullptr;
	
	g_batteryCharge = new BatteryCharge((char *)"prefs.txt",&g_shiplog);
	g_shipdiagnostics = new DiagnosticsSensor(g_atod);

	bool bContainsHelpFlag = Util::ContainsHelpFlag(argc, argv);
	if (argc<2||bContainsHelpFlag) {
		ShowUsage();
		Cleanup();
		//printf("Usage: RemoteControlTest server_ip_address |  [optional command file]\n");
		return 1;
	}

	//check to see if battery is sufficiently charged to start program, if not, then go back to sleep
	double dVoltage = 0.0;
	bool bLowStartupVoltage = false;
	if (g_atod->GetBatteryVoltage(dVoltage,g_batteryCharge)) {
		if (dVoltage>5) {//make sure voltage is non-zero
			if (!g_batteryCharge->hasEnoughChargeToStart(dVoltage)) {
				char sMsg[256];
				sprintf(sMsg,"Startup voltage is: %.3f V, should be > %.2f V.\n",dVoltage,g_batteryCharge->GetMinStartupVoltage());
				g_shiplog.LogEntry(sMsg,true);
				if (argc>=3) {//file commands were specified, so make sure AMOS is set to go back into sleep mode and try again later
					g_batteryCharge->SetInsufficientStartupCharge();
				}
				bLowStartupVoltage = true;
			}
		}
	}

	//get data logging preferences from prefs.txt file
	if (!GetDataLoggingPreferences()) {
		//error trying to access prefs.txt file
		Cleanup();
		return 3;
	}		

	//test
	printf("Got data logging preferences.\n");
	//end test
	bool bContinuedFromPrevious = false;
	if (argc>=3) {
		char *commandFilename = (char *)argv[2];
		g_fileCommands = new FileCommands(g_rootFolder,commandFilename,g_navigator,g_thrusters,&g_bCancelFunctions);
		if (!g_fileCommands->m_bFileOK) {
			printf("Error trying to open or parse file: %s\n",commandFilename);
			Cleanup();
			return 2;
		}
		if (!FileCommands::HasGPSWaypoints(g_fileCommands)) {//don't worry about low startup voltage if boat is being manually navigated
			bLowStartupVoltage = false;
			g_batteryCharge->IgnoreInsufficentStartupCharge();
		}
		if (bLowStartupVoltage) {
			g_fileCommands->TravelToSunnySpot(dVoltage,g_batteryCharge);	
		}
		if (argc>=4) {
			if (strcmp(argv[3],(char *)"continue")==0) {
				g_fileCommands->ContinueFromPrevious();//continue at the last known stage of the file command (i.e. from previous program instance, useful for example in picking up where you left off if the program crashes for some mysterious reason)
				bContinuedFromPrevious = true;
			}
		} 
		//test
		g_fileCommands->PrintOutCommandList();
		g_navigator->PrintOutPlannedPts();
		//end test
	}
	//store command line parameters
	for (int i=1;i<argc;i++) {
		g_commandLineParams.push_back((char *)argv[i]);
	}

	
	if (Util::isSerPort(argv[1])) {
		char *serPort = (char *)argv[1];
		StartSerialClientThread(serPort);//start thread to read in commands over serial port connection
	}
	else {
		char *remoteIPAddr = (char *)argv[1];
		StartNetworkServerThread(remoteIPAddr);//start thread to create server that receives commands and sends back data
	}
	
	printf("Type \"quit\" to exit...\n");
	StartGPSThread();//start thread for receiving GPS data
	//execute this loop until we get some accurate GPS data before doing anything else
	bool bGotAccurateGPSData = false;
	if (!FileCommands::HasGPSWaypoints(g_fileCommands)) {
		bGotAccurateGPSData = true;//no need to wait for gps data if file commands do not contain any GPS waypoints
	}
	const int REQUIRED_NUM_SATELLITES = 4;//required number of GPS satellites for accurate position data
	while (g_bKeepGoing&&!bGotAccurateGPSData) {
		int nKeyboardChar = Util::getch_noblock();
		bool bUserQuit = false;
		if (nKeyboardChar>0) {
			bUserQuit = CheckUserQuit(nKeyboardChar);	
		}
		if (bUserQuit) {
			g_shiplog.LogEntry((char *)"User typed \"quit\" to exit program.\n",false);
			g_bKeepGoing = false;
			g_bCancelFunctions = true;
		}
		if (g_shipdiagnostics!=nullptr) {
			g_shipdiagnostics->ActivityPulse();//send pulse out activity pin to indicate that the program is still running
		}
		double dLatitude = g_navigator->GetLatitude();
		double dLongitude = g_navigator->GetLongitude();
		if (dLatitude!=0.0||dLongitude!=0.0) {
			char sMsg[256];
			int nNumSatellitesVisible = g_navigator->GetNumVisibleSatellites();//the number of currently visible GPS satellites
			int nNumSatellitesUsed = g_navigator->GetNumSatellitesUsed();//the number of GPS satellites that are used for the position calculation
			nPosSamples++;
			sprintf(sMsg,"pos = %.6f, %.6f, satellites visible = %d, satellites used = %d.\n",dLatitude,dLongitude,nNumSatellitesVisible,nNumSatellitesUsed);
			g_shiplog.LogEntry(sMsg,true);
			if (nNumSatellitesUsed>=REQUIRED_NUM_SATELLITES) {
				//have enough satellites
				bGotAccurateGPSData = true;
			}
		}
		usleep(1000000);//pause for 1 second
	}
	StartDataCollectionThread();//start thread for receiving sensor data
	if (g_fileCommands!=nullptr) {
		if (bContinuedFromPrevious) {
			//get closest GPS waypoint
			g_fileCommands->SetStepToClosestGPSWaypoint();//necessary to do this in case AMOS drifted a long distance while sleeping
		}
		StartFileCommandsThread();//start thread for executing commands from a text file
	}

	//loop receiving commands until ESC key is pressed
	while (g_bKeepGoing) {
		//if (_kbhit()) {
		bool bUserQuit = false;
		int nKeyboardChar = Util::getch_noblock();
		if (nKeyboardChar>0) {
			bUserQuit = CheckUserQuit(nKeyboardChar);	
		}
		if (bUserQuit) {
			g_shiplog.LogEntry((char *)"User typed \"quit\" to exit program.\n",false);
			g_bKeepGoing = false;
			g_bCancelFunctions = true;
		}
		if (g_shipdiagnostics!=nullptr) {
			g_shipdiagnostics->ActivityPulse();//send pulse out activity pin to indicate that the program is still running
		}
	}
	if (g_shipdiagnostics!=nullptr) {
		g_shipdiagnostics->ActivityBurst();//send short burst of pulses out activity pin to indicate that program was closed normally
	}
	if (ifAddrStruct != NULL) freeifaddrs(ifAddrStruct);
	Cleanup();
	if (g_bVoltageSwitchedOff) {
		//restart Pi to ensure that things can start afresh
		system("reboot");
	}
	else if (g_bEnteringSleepMode) {
		//halt Pi to make sure that it can be safely powered off by the RFU220SU
		system("halt");	
	}
	return 0;
}

#endif // ! IMUTEST_BUILD