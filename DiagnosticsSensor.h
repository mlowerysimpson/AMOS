//DiagnosticsSensor.h class interface for ship monitoring, power control, etc.

#pragma once
#include "Sensor.h"
#include "HumiditySensor.h"

#define DEFAULT_DIAG_PORT "/dev/serial0" //the default serial port interface to use for getting diagnostics data and control functions
#define MIN_SLEEPTIME_SEC 120 //minimum time that AMOS can be powered down by a "sleep" command
#define ACTIVITY_PIN 7 //pin used for outputting activity pulses to indicate that something is happening

/**
 * @brief diagnostics interface for ship monitoring, power control, etc.
 * 
 */
class DiagnosticsSensor : public Sensor {
public:
    DiagnosticsSensor(AToD *pAToD);//constructor
    ~DiagnosticsSensor();//destructor
    void ActivityPulse();//send activity pulse out on activity pin to indicate that program is running
    void ActivityBurst();//send short burst of pulses out activity pin to indicate that program was closed normally
    bool IsSolarCharging();//return true if there is sufficient sunlight (as measured by the differential voltage at the + and - solar input terminals of the charge controller) to charge the battery
    bool GetCurrentDraw(float &fCurrent);//return an estimate of the amount of current being consumed from the +12 V power supply
    bool GetWirelessRXPower(float &fRXPower);//return the received power level of the serial radio in dBm
    bool GetBatteryVoltage(float &fBatteryVoltage);//do an A to D measurement of the battery voltage and return the result
    bool GetHumidityAndTemp(float &fHumidity, float &fTemp, int nLocation);//get relative humidity and temperature from DHT22 sensor module in either the CPU box or the battery box
    void SetSerportHandle(int fd);//specify a serial port file descriptor to use for communications with the diagnostics hardware
    bool EnterSleepMode(int nSleepTimeSec);//send command to RFU220SU to power down this computer (i.e. the Raspberry Pi module of AMOS)
    static char * GetSensorFileHeader();//return the text header that should be used when saving diagnostics data to a sensor data file, calling function is responsible for deleting returned text

private:
    //data
    bool m_bSentActivityBurst;//true if an activity burst has been sent out the ACTIVITY_PIN, indicating that the program is shutting down
    unsigned int m_uiLastActivityPulseTime;//time in ms of the last activity pulse sent out
    int m_nActivityState;//set to 0 or 1 depending on the desired state (low or high) of the activity output pin (ACTIVITY_PIN)
    HumiditySensor m_humiditySensor;//used for getting relative humidity and temperature data from a DHT22 sensor located inside the main Pi compartment of AMOS.
    int m_fd;//file descriptor to use for communications with the diagnostics hardware
    bool m_bOpenedPort;//true if this object was used to open the serial port (i.e. it is then responsible for closing it when it is destroyed)

    //functions
    void SendPowerDownSequence(int nSleepTimeMinutes);//send command to RFU220 to power down the Pi board for nSleepTimeMinutes minutes
    int OpenDefaultPort();//tries to open the default serial port, and if successful returns a file descriptor to that serial port
    float ConvertCountsToAmps(int nNumCounts);//converts an integer number of counts from an A to D to a value in counts (based on size / length of shunt resistor used)
};