#include "MapUtil.h"
#include "ImageUtil.h"
#include <math.h>

#ifdef WIN32
    #ifndef NAN
        static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
        #define NAN (*(const float *) __nan)
    #endif
#endif

MapUtil::MapUtil(void)
{
}

MapUtil::~MapUtil(void)
{
}

hsv MapUtil::rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

rgb MapUtil::hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

bool MapUtil::isRed(unsigned int color) {//function returns true if color is mostly red (actually accepts anything from about pink to orange)
	//convert rgb value to HSV, and then look at the hue to see how red it is
	//need to convert rgb byte values to floating point numbers ranging from 0 to 1
	rgb c1;
	c1.r = ImageUtil::GetRValue(color) / 255.0;
	c1.g = ImageUtil::GetGValue(color) / 255.0;
	c1.b = ImageUtil::GetBValue(color) / 255.0;
	hsv c2 = rgb2hsv(c1);
	if (c2.s<.5||c2.v<.5) {//too little saturation or brightness
		return false;
	}
	if (c2.h>=330&&c2.h<=360) {
		return true;
	}
	else if (c2.h>=0&&c2.h<=30) {
		return true;
	}
	return false;
}

//GetMinMaxPixelPositions: gets min and max pixel coordinates of a grouping of pixel pairs
//nNumPts = the size of the xPos and yPos arrays, i.e. the number of pair of pixel coordinates to check for min and max values
//xPos = the x-axis coordinates of the pixel points
//yPos = the y-axis coordinates of the pixel points
//nMinPixelX = the returned minimum value from the xPos array
//nMaxPixelX = the returned maximum value from the xPos array
//nMinPixelY = the returned minimum value from the yPos array
//nMaxPixelY = the returned maximum value from the yPos array
void MapUtil::GetMinMaxPixelPositions(int nNumPts, int *xPos, int *yPos, int &nMinPixelX, int &nMaxPixelX, int &nMinPixelY, int &nMaxPixelY) {
	if (nNumPts<=0) return;
	int nMinX = xPos[0];
	int nMaxX = xPos[0];
	int nMinY = yPos[0];
	int nMaxY = yPos[0];
	for (int i=1;i<nNumPts;i++) {
		if (xPos[i]<nMinX) {
			nMinX = xPos[i];
		}
		else if (xPos[i]>nMaxX) {
			nMaxX = xPos[i];
		}
		if (yPos[i]<nMinY) {
			nMinY = yPos[i];
		}
		else if (yPos[i]>nMaxY) {
			nMaxY = yPos[i];
		}
	}
	nMinPixelX = nMinX;
	nMaxPixelX = nMaxX;
	nMinPixelY = nMinY;
	nMaxPixelY = nMaxY;
}

/*
//EstimatePixelCoords: function returns approximate pixel locations on a map that correspond to the GPS coordinates of a point
//readingLocation: the GPS coordinates of the point that we are interested in, i.e. where we want to get pixel coordinates
//waypointCorner: the GPS coordinates of the center of the available waypoint data
//dLatitudeRange: the range in latitude (in degrees) for an area (typically a grouping of known points) on the map
//dLongitudeRange: the range in longitude (in degrees) for an area (typically a grouping of known points) on the map
//nXPixelRange: the width in pixels for the same area whose dimensions in degrees are dLongitudeRange x dLatitudeRange
//nYPixelRange: the height in pixels for the same area whose dimensions in degrees are dLongitudeRange x dLatitudeRange
//nPixelCenterX: the x-axis pixel coordinate of the center of the waypoint markers
//nPixelCenterY: the y-axis pixel coordinate of the center of the waypoint markers
//nImageWidth: the width of the image in pixels
//nImageHeight: the height of the image in pixels
//nPixelX: the estimated x-axis pixel coordinate of readingLocation
//nPixelY: the estimated y-axis pixel coordinate of readingLocation
void MapUtil::EstimatePixelCoords(GPS_POINT *readingLocation, GPS_POINT *waypointCenter, double dLatitudeRange, double dLongitudeRange, 
								  int nXPixelRange, int nYPixelRange, int nPixelCenterX, int nPixelCenterY, int nImageWidth, int nImageHeight, int &nPixelX, int &nPixelY) {
	if (!readingLocation||!waypointCenter) return;
	nPixelX = (int)((readingLocation->dLongitude - waypointCenter->dLongitude)*nXPixelRange / dLongitudeRange) + nPixelCenterX;
	nPixelY = nPixelCenterY - (int)((readingLocation->dLatitude - waypointCenter->dLatitude)*nYPixelRange / dLatitudeRange);
	if (nPixelX<0) nPixelX=0;
	else if (nPixelX>nImageWidth) nPixelX=nImageWidth;
	if (nPixelY<0) nPixelY=0;
	else if (nPixelY>nImageHeight) nPixelY=nImageHeight;
}

void MapUtil::cleanup(GPS_POINT *gpsPoint) {//deletes memory allocated for gpsPoint
	if (gpsPoint) {
		if (gpsPoint->readingTime) {
			delete gpsPoint->readingTime;
			gpsPoint->readingTime=NULL;
		}
		delete gpsPoint;
	}
}

double MapUtil::GetMinLatitude(vector <GPS_POINT> pts) {//finds the minimum latitude in a grouping of GPS points
	int nNumPts = pts.size();
	if (nNumPts<=0) return 0.0;
	double dMinLat = pts[0].dLatitude;
	for (int i=1;i<nNumPts;i++) {
		if (pts[i].dLatitude<dMinLat) {
			dMinLat = pts[i].dLatitude;
		}
	}
	return dMinLat;
}

double MapUtil::GetMaxLatitude(vector <GPS_POINT> pts) {//finds the maximum latitude in a grouping of GPS points
	int nNumPts = pts.size();
	if (nNumPts<=0) return 0.0;
	double dMaxLat = pts[0].dLatitude;
	for (int i=1;i<nNumPts;i++) {
		if (pts[i].dLatitude>dMaxLat) {
			dMaxLat = pts[i].dLatitude;
		}
	}
	return dMaxLat;
}

double MapUtil::GetMinLongitude(vector <GPS_POINT> pts) {//finds the minimum longitude in a grouping of GPS pointsd
	int nNumPts = pts.size();
	if (nNumPts<=0) return 0.0;
	double dMinLong = pts[0].dLongitude;
	for (int i=1;i<nNumPts;i++) {
		if (pts[i].dLongitude<dMinLong) {
			dMinLong = pts[i].dLongitude;
		}
	}
	return dMinLong;
}

double MapUtil::GetMaxLongitude(vector <GPS_POINT> pts) {//finds the maximum longitude in a grouping of GPS points
	int nNumPts = pts.size();
	if (nNumPts<=0) return 0.0;
	double dMaxLong = pts[0].dLongitude;
	for (int i=1;i<nNumPts;i++) {
		if (pts[i].dLongitude>dMaxLong) {
			dMaxLong = pts[i].dLongitude;
		}
	}
	return dMaxLong;
}*/