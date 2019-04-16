//gps_odometer.cpp  -- program for parsing gps data from ship logs to determine total distance that boat has travelled
//program takes as input a folder name and parses all of the "shiplog_*.txt" files in that folder to determine the total distance that the boat has travelled.
//The total distance is tracked in a file called total_distance.bin that resides in the folder where this program is located and run from.
//The total_distance.bin file also keeps track of which files have been parsed already, so that if the same file is parsed again, it won't count the same travel twice.
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <io.h>
#include <vector>
#include <windows.h>

using namespace std;
vector <time_t> g_shipLogTimes;



void ShowUsage() {//show format for using this program
	printf("gps_odometer <folder where ship's logs are stored>\);
	printf("or\n");
	printf("gps_odometer <-h | -help>\n");
}

FILE * GetTotalDistanceFile(char *rootFolder) {
	char szFilename[_MAX_PATH];
	sprintf(szFilename,"%s\\total_distance.bin",rootFolder);
	FILE *totalDistFile = fopen(szFilename,"r+");
	return totalDistFile;
}

bool FindTotalDistTraveled(FILE *totalDistFile, char *szShipLogFolder) {//find the total distance traveled 
	//based on the contents of totalDistFile (if it exists) and any shiplog_*.txt files in the szShipLogFolder
	double dTotalDistance=0.0;
	if (totalDistFile) {
		if (fread(&dTotalDistance,1,sizeof(double),totalDistFile)!=sizeof(double)) {
			printf("Error reading total distance from total_distance.bin file.\n");
			return false;
		}
		//read in times of files already parsed and included in the total_distance.bin file
		time_t shipLogTime;
		while (fread(&shipLogTime, 1, sizeof(time_t),totalDistFile)==sizeof(time_t)) {
			g_shipLogTimes.push_back(shipLogTime);
		}
	}
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir(szShipLogFolder)) != NULL) {
		/* look for shiplog_*.txt files within directory */
		while ((ent = readdir (dir)) != NULL) {
			int nYr=0,nMonth=0,nDay=0,nHr=0,nMin=0,nSec=0;
			if (sscanf(ent->d_name,"shiplog_%d_%d_%d_%d_%d_%d.txt",&nYr,&nMonth,&nDay,
				&nHr,&nMin,&nSec)==6) {
				struct tm fileTime;
				fileTime.tm_year = nYr - 1900;
				fileTime.tm_mon = nMonth - 1;
				fileTime.tm_mday = nDay;
				fileTime.tm_hour = nHr;
				fileTime.tm_min = nMin;
				fileTime.tm_sec = nSec;

				time_t tt = mktime(&fileTime);
				if (!timeExists(//left off here...
				g_shipLogTimes.push_back(tt);
				double dDistTraveled = GetDistTraveled(ent->d_name);
			}
		}
		closedir (dir);
	}
}

}

int main(int argc, const char * argv[]) {
    char currentFolder[_MAX_PATH];
	::GetCurrentDirectory(_MAX_PATH,currentFolder);
	int (argc!=2) {
		ShowUsage();
		return -1;
	}
	else if (isHelpText(argv[1])) {
		ShowUsage();
		return -2;
	}
	else {
		FILE *totalDistFile = GetTotalDistanceFile(currentFolder);
		char *shipLogFolder = argv[1];
		if (!FindTotalDistTraveled(totalDistFile, shipLogFolder)) {
			if (totalDistFile) {
				fclose(totalDistFile);
			}
			return -3;
		}
		if (totalDistFile) {
			fclose(totalDistFile);
		}
	}
    return 0;
}