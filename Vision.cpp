//Vision.cpp - implementation file for Vision class
#include "Vision.h"
#include "CommandList.h"
#include "FileCommands.h"
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>


void *PictureLoggingThread(void *pParam) {//function continuously logging pictures to files
	Vision *pVis = (Vision *)pParam;
	pVis->m_bPictureThreadRunning = true;
	pVis->m_bEndPictureLoggingThread = false;
	while (!pVis->m_bEndPictureLoggingThread) {
		pVis->AutoCapturePicture();
		pVis->WaitForNextAutoCapture();
	}
	pVis->m_bPictureThreadRunning = false;
	return nullptr;
}

Vision::Vision() {//constructor
	m_bCameraOpened=false;
	m_cap.set(CV_CAP_PROP_BUFFERSIZE, 10); // internal buffer will now store only 10 frames (may help to reduce lag from storage of video frames in buffer)
	m_cap.open(0);//use first camera on Pi for video capture
	if (m_cap.isOpened() == false)  
	{
		cout << "Cannot open the video camera" << endl;
	}
	else {
		m_bCameraOpened=true;
		double dWidth = m_cap.get(CAP_PROP_FRAME_WIDTH); //get the width of frames of video
		double dHeight = m_cap.get(CAP_PROP_FRAME_HEIGHT); //get the height of frames of video
		cout << "Resolution of video: " << dWidth << " x " << dHeight << endl;
	}
	m_bLogPictures=false;
	m_nPictureIntervalSeconds=0;
	m_szStorageFolder=NULL;
	m_szFilenamePrefix=NULL;
	m_bPictureThreadRunning=false;//true when the thread for logging pictures is active
	m_bEndPictureLoggingThread=false;//true when it is time to end the picture logging thread
	m_cameraThreadId = 0;
	m_uiLastAutoPictureTime = 0;
	m_uiPictureFileNumber = 0;
}

Vision::~Vision() {//destructor
	EndPictureLoggingThread();
	if (m_szStorageFolder) {
		delete []m_szStorageFolder;
		m_szStorageFolder = NULL;
	}
	if (m_szFilenamePrefix) {
		delete []m_szFilenamePrefix;
		m_szFilenamePrefix=NULL;
	}
	if (m_cap.isOpened()) {
		m_cap.release();
	}
}

//CaptureFrame: capture video frame using Canny edge detection or fast feature detection, and store the result to file
//nThresholdInfo = the 2nd most significant byte of nThresholdInfo is 0 for Canny Edge detection or 1 for fast feature detection
		//least significant byte is the low threshold for Canny Edge detection or the feature threshold for fast feature detection
		//2nd least significant byte is the high threshold for Canny Edge detection or zero for fast feature detection
//szCapFilename = the name of the file where the image will be saved, can be ".jpg", ".png", etc.
//szOverlayText = text to overlay on the image, set to NULL if no text should be overlayed
bool Vision::CaptureFrame(int nThresholdInfo, char *szCapFilename, char *overlayText/*=NULL*/) {
	//check to make sure that camera was opened
	if (!m_bCameraOpened) return false;
	bool bCannyEdgeDetection = true;
	if ((nThresholdInfo&0x00ff0000)>0) {
		bCannyEdgeDetection = false;
	}
	//Create Mat image using camera capture
	Mat img1;//original image from video camera
	Mat img1_out;//output image with keypoints shown
	Mat img_gray1;//grayscale image 
	
	vector<KeyPoint> keypoints;

	bool bSuccess = m_cap.read(img1); //read a new frame from video camera 
	if (bSuccess == false) {
		cout << "Video camera is disconnected" << endl;
		return false;
	}

	if (bCannyEdgeDetection) {//using Canny edge detection to process image
		int nLowCannyThreshold = (nThresholdInfo&0x000000ff);
		int nHighCannyThreshold = (nThresholdInfo&0x0000ff00)>>8;
		Canny(img1,img1_out,nLowCannyThreshold,nHighCannyThreshold);
	}
	else {//using fast feature detection to process image
		int nFeatureThreshold = (nThresholdInfo&0x000000ff);
		if (nFeatureThreshold==0) {
			img1_out = img1;
		}
		else {//find features in image
			//first convert image to grayscale
			cvtColor(img1,img_gray1,COLOR_RGB2GRAY);
			cv::FAST(img_gray1,keypoints,nFeatureThreshold);//use fast feature detection algorithm
			cv::drawKeypoints(img1,keypoints,img1_out,Scalar(0,0,255));//draws red keypoints
		}
	}
	if (overlayText) {
		cv::putText(img1_out, 
           overlayText,
            cv::Point(300,300), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,255,255), // BGR Color
            1); // Line Thickness (Optional)
            //cv::CV_AA); // Anti-alias (Optional)
	}
	string sOutputFilename = string(szCapFilename);
	return imwrite(sOutputFilename,img1_out);
}

/**
 * @brief send saved image file over network connection or wireless serial port connection.
 * 
 * @param nHandle the network socket handle (if bUseSerial is false) or the serial port file descriptor (if bUseSerial is true).
 * @param bUseSerial set to true if a wireless serial port link is being used, otherwise set to false if a network connection is being used.
 * @param szCapFilename the filename of the captured image.
 * @param pDiagSensor void pointer to a DiagnosticsSensor object that is used for signaling that AMOS is still active. This is necessary to ensure that AMOS does not get put to sleep when sending out large amounts of data over a slow connection.
 * @return true if the image data could be successfully sent.
 * @return false if there was a problem sending the image data.
 */
bool Vision::SendCapturedImage(int nHandle, bool bUseSerial, char *szCapFilename, void *pDiagSensor) {
	FILE *capFile = fopen(szCapFilename,"r");
	if (!capFile) return false;
	//find filesize
	fseek(capFile, 0L, SEEK_END);
	size_t fileSize = ftell(capFile);
	//go back to beginning of file
	rewind(capFile);
	unsigned char *szBytesToSend = new unsigned char[4+fileSize];//bytes to send over network connection
	if (fread(szBytesToSend,1,fileSize,capFile)!=fileSize) {
		delete []szBytesToSend;
		return false;
	}
	int nFileSize = (int)fileSize;
	//create and send BOAT_DATA structure for describing the size of the file
	BOAT_DATA *pBoatData = BoatCommand::CreateBoatData(VIDEO_DATA_PACKET);
	memcpy(pBoatData->dataBytes,&nFileSize,sizeof(int));
	pBoatData->checkSum = BoatCommand::CalculateChecksum(pBoatData);//calculate simple 8-bit checksum for BOAT_DATA structure
	if (!BoatCommand::SendBoatData(nHandle, bUseSerial, pBoatData, pDiagSensor)) {
		return false;
	}
	//also send out file bytes
	if (bUseSerial) {//using serial port link
		if (!BoatCommand::SendLargeSerialData(nHandle, szBytesToSend, nFileSize, pDiagSensor)) {//send large amount of data out serial port, need to get confirmation after sending each chunk
			return false;
		}
	}
	else {//using network link
		if (send(nHandle, szBytesToSend, nFileSize, 0)<0) {
			//error sending data
			return false;
		}
	}
	delete pBoatData;
	delete []szBytesToSend;
	fclose(capFile);
	return true;
}


//SetLoggingConfig: sets the configuration for continuously logging pictures from the camera
//bLogPictures = flag is true if the camera should be continously taking pictures and saving them to  folder, otherwise false
//nIntervalSec = the interval in seconds between pictures, is only applicable if bLogPictures is true
//storage_folder = the folder where to store the logged pictures, is only applicable if bLogPictures is true
//filename_prefix = the filename prefix to use for picture files, each filename will use this prefix, followed by a sequential number (ex: 00001, 00002, 00003, etc.)
void Vision::SetLoggingConfig(bool bLogPictures, int nIntervalSec, char *storage_folder, char *filename_prefix) {
	m_bLogPictures = bLogPictures;
	m_nPictureIntervalSeconds = nIntervalSec;
	//test
	printf("Log pictures = %d, interval = %d\n",(int)m_bLogPictures,nIntervalSec);
	//end test
	if (m_szStorageFolder) {
		delete []m_szStorageFolder;
		m_szStorageFolder = NULL;
	}
	if (m_szFilenamePrefix) {
		delete []m_szFilenamePrefix;
		m_szFilenamePrefix=NULL;
	}
	if (storage_folder) {
		//test
		printf("storage_folder = %s\n",storage_folder);
		//end test
		m_szStorageFolder = new char[strlen(storage_folder)+1];
		strcpy(m_szStorageFolder, storage_folder);
	}
	if (filename_prefix) {
		//test
		printf("filename_prefix = %s\n",filename_prefix);
		//end test
		m_szFilenamePrefix = new char[strlen(filename_prefix)+1];
		strcpy(m_szFilenamePrefix, filename_prefix);
	}
	if (m_bLogPictures) {
		//test
		printf("starting picture logging thread\n");
		//end test
		StartPictureLoggingThread();
	}
	else {
		//test
		printf("ending picture logging thread\n");
		//end test
		EndPictureLoggingThread();
	}
}

void Vision::StartPictureLoggingThread() {//start thread for logging pictures
	int nError = pthread_create(&m_cameraThreadId, NULL, &PictureLoggingThread, (void *)this);
	if (nError != 0) {
		printf("Can't create picture logging thread: %s", strerror(nError));
	}
}

void Vision::EndPictureLoggingThread() {//end thread for logging pictures
	m_bEndPictureLoggingThread = true;
	unsigned int uiTimeoutTime = millis() + 5000;//timeout after 5 seconds
	while (m_bPictureThreadRunning&&millis()<uiTimeoutTime) {
		usleep(100000);//sleep for 100 ms
	}
	if (m_cameraThreadId) {
		pthread_cancel(m_cameraThreadId);
		m_cameraThreadId=0;
	}
}



void Vision::AutoCapturePicture() {//capture a picture and log it to file
	char *szNextAutoFilename = GetNextAutoFilename();
	if (szNextAutoFilename) {
		int nFeatureDetect = 1;//set to use FAST feature detection, but set the threshold to 0, effectively disabling feature detection and just taking a normal picture
		int nThresholdInfo = (nFeatureDetect<<16);
		CaptureFrame(nThresholdInfo, szNextAutoFilename);
		m_uiLastAutoPictureTime = millis();
		delete []szNextAutoFilename;
	}
}

void Vision::WaitForNextAutoCapture() {//waits the required number of seconds before the next picture capture
	if (m_nPictureIntervalSeconds>0) {
		unsigned int uiStartTime = millis();//initial time in ms
		unsigned int uiMilliSecondsToWait = m_nPictureIntervalSeconds * 1000;
		unsigned int uiTimeoutTime = uiStartTime + uiMilliSecondsToWait;
		while (millis()<uiTimeoutTime) {
			usleep(250000);//sleep for 250 ms
		}
	}
}

char * Vision::GetNextAutoFilename() {//gets next available filename for automatically saving image data (calling function is responsible for deleting the returned character pointer)
	const unsigned int MAX_FILE_NUMBER = 999999;
	char *szRetval = new char[strlen(m_szFilenamePrefix)+strlen(m_szStorageFolder)+16];
	do {
		m_uiPictureFileNumber++;
		//check to see if folder name already has '/' character at the end of it 
		if (m_szStorageFolder[strlen(m_szStorageFolder)-1]=='/') {
			sprintf(szRetval,"%s%s%05u.jpg",m_szStorageFolder,m_szFilenamePrefix,m_uiPictureFileNumber);
		}
		else {//insert '/' character between folder and filename
			sprintf(szRetval,"%s/%s%05u.jpg",m_szStorageFolder,m_szFilenamePrefix,m_uiPictureFileNumber);
		}
		if (FileCommands::doesFileExist(szRetval)) {//file exists, increase # and try again
			continue;
		}
		else return szRetval;
	} while (m_uiPictureFileNumber<MAX_FILE_NUMBER);
	return szRetval;
}

bool Vision::CaptureFrameWithDistText(double dDist) {//capture frame of video with an estimate of the distance to the object (in m) superimposed on the picture (use autocapture settings for filename)
	char *szNextAutoFilename = GetNextAutoFilename();
	if (szNextAutoFilename) {
		int nFeatureDetect = 1;//set to use FAST feature detection, but set the threshold to 0, effectively disabling feature detection and just taking a normal picture
		int nThresholdInfo = (nFeatureDetect<<16);
		char szDistText[256];
		sprintf(szDistText,"Distance = %.2f m",dDist);
		CaptureFrame(nThresholdInfo, szNextAutoFilename, szDistText);
		m_uiLastAutoPictureTime = millis();
		delete []szNextAutoFilename;
	}
}