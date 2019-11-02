#pragma once
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <vector>


using namespace std;
using namespace cv;

#define DEFAULT_LOW_THRESHOLD 3.0 //low threshold value for determining whether or not a particular block of pixels contains an object
#define DEFAULT_HIGH_THRESHOLD 4.0 //high threshold value for determining whether or not a particular block of pixels contains an object
#define BLOCKSIZE 20//width and height of square region to consider when looking for objects

struct OPENCSV_HSV {//style of HSV data used by openCSV
	unsigned char h;//value from 0 to 179 --> hue
	unsigned char s;//value from 0 to 255 --> saturation
	unsigned char v;//value from 0 to 255 --> value (brightness)
};

struct IMAGE_STATS {//image statistics
	double avg_brightness;//average brightness (0 to 255)
	double sd_brightness;//standard deviation of brightness
	double avg_red;//average red component (0 to 255)
	double avg_green;//average green component (0 to 255)
	double avg_blue;//average blue component (0 to 255)
	double avg_hue;//average hue component (0 to 255)
	double avg_sat;//average saturation component (0 to 255)
	double avg_value;//average value component (0 to 255)
	double sd_red;//standard deviation of red component
	double sd_green;//standard deviation of green comonent
	double sd_blue;//standard deviation of blue component
	double sd_hue;//standard deviation of hue component
	double sd_sat;//standard deviation of saturation component
	double sd_value;//standard deviation of value component
};

struct OBJECT_BLOCK {
	int nX;//horizontal x-axis coordinate of the top-left corner of an object block
	int nY;//vertical y-axis coordinate of the top-left corner of an object block
	int nWidth;//the width of the object block
	int nHeight;//the height of the object block
};

class ImageUtil //utility class for AMOS image post-processing
{

public:
	ImageUtil(void);
	~ImageUtil(void);

	//static void CalculateHorizon(int &nLeftHorizonPixelY, int &nRightHorizonPixelY, CImage *pImage);//calculate pixel coordinates for the horizon / coast line
	static unsigned int * GetImageStats(Mat img, int nLeftCoastY, int nRightCoastY, IMAGE_STATS *pStats);//get a number of statistics for the water part of the image, i.e. below the coast and not including the hull of the boat
	static bool GetHSVAt(int nX, int nY, Mat img, OPENCSV_HSV *pHSV);//get the OpenCSV style HSV coordinates at a point in an image
	static bool ConvertToHSV(unsigned char rVal, unsigned char gVal, unsigned char bVal, OPENCSV_HSV *pHSV);//get the OpenCSV style HSV coordinates at a point, given the r,g,b color of that point
	static vector <OBJECT_BLOCK *> GetObjectBlocks(unsigned int *pixelData,int nImageWidth,int nImageHeight,int nLeftHorizonPixelY,int nRightHorizonPixelY,
		double dLowObjectThreshold, double dHighObjectThreshold, IMAGE_STATS *waterStats);//determines objects (if any) that are floating around in the water in the boat's field of view, and below the horizon line in the water
	static unsigned int GetBlockColor(unsigned int *pixelData, int nX, int nY, int nImageWidth);//gets the average color of a BLOCKSIZE x BLOCKSIZE grouping of pixels whose upper-left corner starts at nX, nY
	static unsigned char GetRValue(unsigned int c);//gets the red component of an rgb color
	static unsigned char GetGValue(unsigned int c);//gets the green component of an rgb color
	static unsigned char GetBValue(unsigned int c);//gets the blue component of an rgb color

private:
	static unsigned int GetBGRPixel(Mat img,int nX,int nY);//gets BGR color of pixel at nX, nY in img
	static int GetVertStartingPt(int nX);//use piecewise linear function to determine vertical starting point that avoids the yellow hull of the boat
	static void SelectBestCoast(int nImageWidth,int *coastYCoords,int &nLeftCoastY,int &nRightCoastY);//find the coastline that matches up best with the coastYCoords found above
	static void GetAvgAndSD(vector <double>vals, double &avg_value, double &sd_value);//compute the average and standard deviation of a list of numbers
	static bool isObjectBlock(unsigned int blockColor, IMAGE_STATS *waterStats, double dObjectThreshold, bool bLowThresholdMode);//returns true if a block's color qualifies it as an object block
	static double CalculateHSVScore(OPENCSV_HSV *hsvColor, IMAGE_STATS *waterStats);//return a score based on how different a particular color is from the average water color
	static bool isObjectBlock(int nX, int nY, vector <OBJECT_BLOCK *>objects);//return true if one of the objects starts at nX, nY
	static bool isNearObjectBlock(int nX, int nY, vector <OBJECT_BLOCK *>objects);//return true if the block at nX, nY is close to one of the blocks listed in objects, either to left, right, above, below, or diagonally
};
