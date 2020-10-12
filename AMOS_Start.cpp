//AMOS_Start.cpp --> program used to start software for AMOS. Used to automatically restart software if a crash occurs

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <string.h>
#include <iostream>
#include <unistd.h>
#include "../RemoteControlTest/Util.h"
#include "../RemoteControlTest/ShipLog.h"
#include "../RemoteControlTest/filedata.h"
#include "../RemoteControlTest/DiagnosticsSensor.h"


/**
 * @brief saves a short text message to the log file: exitLog.txt
 * 
 * @param szMsg 
 */
void saveExitMsg(char *szMsg) {
  ShipLog logFile((char *)"exitLog.txt");
	logFile.LogEntry(szMsg,true);
}

int main (int argc, const char * argv[])
{
  //start RemoteControlTest software"Executing command DIR...\n");
  const char *start_program = "./RemoteControlTest";
  //check to see if prefs.txt is missing or of zero length and needs to be restored from backup
  if (filedata::getFileLength("prefs.txt") <= 0) {
      //restored from backup
      if (Util::BackupExists("prefs.txt")) {
          Util::RestoreFromBackup("prefs.txt");
          saveExitMsg("prefs.txt file restored from backup.");
      }
  }
  filedata prefsFile("prefs.txt");//preferences configuration file
  if (argc<=1) {//check to see if autostart flag is present in preferences file
    int nAutoStart = prefsFile.getInteger("[startup]","autostart");
    if (nAutoStart<=0) {//do not autostart, so just exit this program
      saveExitMsg((char *)"Autostart not enabled, exiting AMOS_Start.\n");
      return 0;
    }
  }
  wiringPiSetupGpio();//need to call this before doing anything with GPIO
  pinMode(ACTIVITY_PIN,INPUT);//set activity pin for input
  pullUpDnControl(ACTIVITY_PIN,PUD_DOWN);//set internal pull-up/down resistor on Pi to be weak pull-down (resistor has value of ~ 50 kohm)
  delay(20);//pause for 20 ms
  if (digitalRead(ACTIVITY_PIN)>0) {
    //RFU220SU must be pulling the activity pin high, indicating that AMOS's software should not be automatically started
    saveExitMsg((char *)"Activity pin is high, exiting auto-start.\n");
    return 0;//just exit this program, i.e. don't start anything
  } 
 
  char exitMsg[128];
  //int nTest = system(start_program);
  //printf("nTest = %d\n",nTest);
  int nCommandLength = strlen(start_program)+2;
  char *commandStr = NULL;
  
  if (argc<=1) {//program was started without command line arguments, likely after a reboot
    //need to grab parameters from prefs.txt file
    char *szParams = NULL;
    prefsFile.getString("[startup]","params",&szParams);
    if (!szParams) {
      saveExitMsg((char *)"No parameters included in prefs.txt file.\n");
      return 0;//no parameters included in prefs.txt file
    }
    nCommandLength+=(strlen(szParams)+3);
    commandStr = new char[nCommandLength];
    strcpy(commandStr, start_program);
    strcat(commandStr, (char *)" ");//add space character
    strcat(commandStr, szParams);//add program aruments
    delete []szParams;
  }
  else {
    for (int i=1;i<argc;i++) {
	    nCommandLength+=(strlen(argv[i])+2);
    }
    nCommandLength++;
    commandStr = new char[nCommandLength];
    strcpy(commandStr, start_program);
    for (int i=1;i<argc;i++) {
	    strcat(commandStr,(char *)" ");//add space character
	    strcat(commandStr, argv[i]);//add program argument
    }
  }
  bool bValidExitCode = false;
  while (!bValidExitCode) {//keep launching commandStr until a valid exit code is returned (i.e. a crash does not occur)
	  int nExitCode = system(commandStr)/256;//system function left-shifts actual exit code by 8-bits for some reason
	  sprintf(exitMsg,"exit code = %d\n",nExitCode);
	  ShipLog logFile((char *)"exitLog.txt");
	  saveExitMsg(exitMsg);
	  //check for a valid exit code
	  if (nExitCode==0||nExitCode==1||nExitCode==2) {
		  bValidExitCode=true;
		  break;
	  }
	  sleep(10);//sleep for 10 seconds between launches
  }
  delete []commandStr;
  return 0;
}