#pragma once

#include <vector>

/*struct GPS_POINT {
	double dLatitude;//latitude of location in degrees
	double dLongitude;//longitude of location in degrees
	CTime *readingTime;//time of the measurement (applicable for path points only)
};*/

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

using namespace std;


class MapUtil
{
public:
	MapUtil(void);
	~MapUtil(void);

static hsv   rgb2hsv(rgb in);//convert rgb values to hsv
static rgb   hsv2rgb(hsv in);//convert hsv values to rgb
static bool isRed(unsigned int color);//function returns true if color is mostly red
static void GetMinMaxPixelPositions(int nNumPts, int *xPos, int *yPos, int &nMinPixelX, int &nMaxPixelX, int &nMinPixelY, int &nMaxPixelY);//gets min and max pixel coordinates of a grouping of pixel pairs
/*
static void EstimatePixelCoords(GPS_POINT *readingLocation, GPS_POINT *waypointCenter, double dLatitudeRange, double dLongitudeRange, 
				int nXPixelRange, int nYPixelRange, int nPixelCenterX, int nPixelCenterY, int nImageWidth, int nImageHeight, int &nPixelX, int &nPixelY);//function returns approximate pixel locations on a map that correspond to the GPS coordinates of a point
static void cleanup(GPS_POINT *gpsPoint);//deletes memory allocated for gpsPoint
static double GetMinLatitude(vector <GPS_POINT> pts);//finds the minimum latitude in a grouping of GPS points
static double GetMaxLatitude(vector <GPS_POINT> pts);//finds the maximum latitude in a grouping of GPS points
static double GetMinLongitude(vector <GPS_POINT> pts);//finds the minimum longitude in a grouping of GPS points
static double GetMaxLongitude(vector <GPS_POINT> pts);//finds the maximum longitude in a grouping of GPS points
*/	

};
