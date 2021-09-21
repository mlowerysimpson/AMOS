//Vision.h - class for handling vision and image-related functions
#pragma once
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <string> 
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <wiringPi.h>

using namespace std;
using namespace cv;

class Vision {
public:
	Vision();//constructor
	~Vision();//destructor

	bool CaptureFrameWithDistText(double dDist);//capture frame of video with an estimate of the distance to the object (in m) superimposed on the picture (use autocapture settings for filename)
	bool CaptureFrame(int nThresholdInfo, char *szCapFilename, char *szOverlayText=NULL);//capture video frame using Canny edge detection or fast feature detection, or just regular picture (with low-quality option) and store the result to file
	bool SendCapturedImage(int nHandle, bool bUseSerial, char *szCapFilename, void *pDiagSensor);//send saved image file over network connection or wireless serial port connection
	void SetLoggingConfig(bool bLogPictures, int nIntervalSec, char *storage_folder, char *filename_prefix);//sets the configuration for continuously logging pictures from the camera
	void AutoCapturePicture();//capture a picture and log it to file
	void AutoCaptureVideo();//auto-record a video to file
	void WaitForNextAutoCapture();//waits the required number of seconds before the next picture capture
	void SetUpsideDown(bool bUpsideDown);//function used to handle the case where the camera is positioned upside-down (e.g. underneath AMOS)
	void CloseCamera();//close video camera so that it can be accessed by other software
	void OpenCamera();//open video camera port for use by this software
	void SetVideoFilenamePrefix(char *videoFilenamePrefix);//sets the filename prefix to use for recording videos
	void StartVideoRecording(float fVideoDurationSec);//start recording a video

	//data
	bool m_bPictureThreadRunning;//true when the thread for logging pictures is active
	bool m_bEndPictureLoggingThread;//true when it is time to end the picture logging thread

private:
	//data
	float m_fVideoDurationSec;//duration of next video in seconds
	bool m_bVideoThreadRunning;//true if the video thread is running
	pthread_t m_videoThreadId;//thread id for recording videos
	char* m_szVideoFilenamePrefix;//the filename prefix to use for recording video, each filename will use this prefix, followed by a sequential number (ex: 00001, 00002, 00003, etc.)
	bool m_bUpsideDown;//true if the camera is mounted upside down
	unsigned int m_uiPictureFileNumber;//the number to append to the automatically saved picture filename (use 5 (or more) digits #####)
	unsigned int m_uiVideoFileNumber;//the number to append to the automatically saved video filename (use 5 (or more) digits #####)
	unsigned int m_uiLastAutoPictureTime;//the time of the last automatically saved picture in milliesconds
	pthread_t m_cameraThreadId;//thread id for thread that continously takes pictures with the camera
	cv::VideoCapture m_cap;//video capture object
	bool m_bCameraOpened;//flag is true if we were able to successfully open  the camera
	bool m_bLogPictures;//flag is true if pictures should be continously taken and saved from the camera
	int m_nPictureIntervalSeconds;//the number of seconds to wait between pictures, use 0 for no wait.
	char *m_szStorageFolder;//the folder where pictures will be stored
	char *m_szFilenamePrefix;//the filename prefix to use for picture files, each filename will use this prefix, followed by a sequential number (ex: 00001, 00002, 00003, etc.)

	//functions
	char * GetNextAutoFilename();//gets next available filename for automatically saving image data (calling function is responsible for deleting the returned character pointer)
	void StartPictureLoggingThread();//start thread for logging pictures
	void EndPictureLoggingThread();//end thread for logging pictures
};