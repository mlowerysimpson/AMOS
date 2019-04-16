#include "ImageUtil.h"
#include "MapUtil.h"
#include <wiringPi.h>
#include <math.h>

ImageUtil::ImageUtil(void)
{
}

ImageUtil::~ImageUtil(void)
{
}

//CalculateHorizon: calculate pixel coordinates for the horizon / coast line
//nLeftHorizonPixelY: the returned Y-axis coordinate of the horizon at the left edge of the screen
//nRightHorizonPixelY: the returned Y-axis coordinate of the horizon at the right edge of the screen
//pImage: the image for which the horizon / coast line is being determined
/*void ImageUtil::CalculateHorizon(int &nLeftHorizonPixelY, int &nRightHorizonPixelY, CImage *pImage) {//
	const int MIN_PIXELS = 20;//minimum # of pixels to check before looking for coastline or horizon
	const double MIN_HUE_TRANSITION = 30;//look for this much of a transition in hue to indicate a transition from water to coastline (or sky)
	const unsigned char MIN_SATURATION = 13;//only look at pixels with at least this much saturation when trying to figure out where pixels transition from water to coastline
	const unsigned char MIN_VALUE = 64;//only look at pixels with at least this much value (brightness) when trying to figure out where pixels transition from water to coastline
	if (!pImage) return;

	int nImageWidth = pImage->GetWidth();
	int nImageHeight = pImage->GetHeight();
	//start from bottom of image (left and right edges) and go up in vertical lines, look at HSV values, require at least 5% S (13 counts), 25% V (64 counts), check
	//for abrupt transition in H (>60 deg (>30 counts) from avg of preceding points) for coastline detection.
	//transition from water to sky may be more subtle, may need to look for transition in V instead (sky should appear brighter than water)

	int nX = 0;//x-coordinate of pixel
	int *coastYCoords = new int[nImageWidth];
	memset(coastYCoords,0,nImageWidth*sizeof(int));
	for (int j=0;j<nImageWidth;j++) {
		nX=j;
		int nVertStartingPoint = GetVertStartingPt(nX);//use piecewise linear function to determine vertical starting point that avoids the yellow hull of the boat
		double dAvgH=0.0;//average of preceding values of hue (in counts from 0 to 180)
		double dSumH=0.0;//sum of all hue values
		int nPixelCount=0;//the number of pixels that meet the qualification criteria for saturation (MIN_SATURATION) and brightness (MIN_VALUE)
		for (int i=0;i<(nVertStartingPoint-2);i++) {
			int nY = nVertStartingPoint-i;//y-coordinate of pixel
			//get h,s,v values at pixel nX, nY
			OPENCSV_HSV hsv;
			GetHSVAt(nX,nY,pImage,&hsv);
			unsigned char h = hsv.h;
			unsigned char s = hsv.s;
			unsigned char v = hsv.v;
			if (s>=MIN_SATURATION&&v>=MIN_VALUE) {
				nPixelCount++;
				dSumH+=(double)(h);
				if (nPixelCount>=MIN_PIXELS) {
					dAvgH = dSumH / (nPixelCount);
					if (fabs(h-dAvgH)>=MIN_HUE_TRANSITION) {//have found possible tranisition from water to coastline (or possibly sky)
						//check next 2 pixels to see if they also exceed MIN_HUE_TRANSITION
						OPENCSV_HSV hsv2;
						GetHSVAt(nX,nY-1,pImage,&hsv2);
						unsigned char h2 = hsv2.h;
						unsigned char s2 = hsv2.s;
						unsigned char v2 = hsv2.v;
						if (s2>=MIN_SATURATION&&v2>=MIN_VALUE) {
							if (fabs(h2-dAvgH)>=MIN_HUE_TRANSITION) {
								OPENCSV_HSV hsv3;
								GetHSVAt(nX,nY-2,pImage,&hsv3);
								unsigned char h3 = hsv3.h;
								unsigned char s3 = hsv3.s;
								unsigned char v3 = hsv3.v;
								if (s3>=MIN_SATURATION&&v3>=MIN_VALUE) {
									if (fabs(h3-dAvgH)>=MIN_HUE_TRANSITION) {
										//look back to see if we might have missed the transition due to lack of saturation or brightness
										int k=nY+1;
										while (k<nImageHeight) {
											OPENCSV_HSV hsv_back;
											GetHSVAt(nX,nY,pImage,&hsv_back);
											unsigned char h_back = hsv_back.h;
											unsigned char s_back = hsv_back.s;
											unsigned char v_back = hsv_back.v;
											if (fabs(h_back-dAvgH)<MIN_HUE_TRANSITION) {
												break;
											}
											else if (s_back>=MIN_SATURATION&&v_back>=MIN_VALUE) {
												break;
											}
											nY=k;
											k++;
										}
										coastYCoords[j] = nY;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	int nLeftCoastY=0, nRightCoastY=0;
	SelectBestCoast(nImageWidth,coastYCoords,nLeftCoastY,nRightCoastY);//find the coastline that matches up best with the coastYCoords found above
	nLeftHorizonPixelY = nLeftCoastY;
	nRightHorizonPixelY = nRightCoastY;
	delete []coastYCoords;
}*/

int ImageUtil::GetVertStartingPt(int nX) {//use piecewise linear function to determine vertical starting point that avoids the yellow hull of the boat
	const int MIDPOINT = 300;//the horizontal pixel coordinate where the hull of the boat protrudes the most
	const int VAL_AT_MIDPOINT = 410;//vertical pixel coordinate of tip of hull
	const int MAX_X = 639;
	int nVertStartingPt=479;
	if (nX<=MIDPOINT) {
		double dFactor = ((double)(nX))/MIDPOINT;
		int nRetval = dFactor*VAL_AT_MIDPOINT + (1-dFactor)*nVertStartingPt;
		return nRetval;
	}
	else {
		int nVertEndingPt = 479;
		double dFactor = ((double)(nX-MIDPOINT)) / (MAX_X-MIDPOINT);
		int nRetval = dFactor*nVertEndingPt + (1-dFactor)*VAL_AT_MIDPOINT;
		return nRetval;
	}
	return nVertStartingPt;//should not get here
}

bool ImageUtil::GetHSVAt(int nX, int nY, Mat img, OPENCSV_HSV *pHSV) {//get the OpenCSV style HSV coordinates at a point in an image
	int nImageWidth = img.cols;
	int nImageHeight = img.rows;
	if (nX<0||nX>=nImageWidth) {
		return false;
	}
	if (nY<0||nY>=nImageHeight) {
		return false;
	}
	//get r,g,b values at pixel nX, nY
	unsigned int pixelColor = GetBGRPixel(img,nX,nY);
	unsigned char rVal = GetRValue(pixelColor);
	unsigned char gVal = GetGValue(pixelColor);
	unsigned char bVal = GetBValue(pixelColor);
	rgb c1;
	c1.r = rVal/255.0;
	c1.g = gVal/255.0;
	c1.b = bVal/255.0;

	hsv c2 = MapUtil::rgb2hsv(c1);
	//printf("h=%.1f, s=%.0f, v=%.0f\n",c2.h,c2.s*255,c2.v*255);
	unsigned char h = (unsigned char)(c2.h/2);
	unsigned char s = (unsigned char)(c2.s*255);
	unsigned char v = (unsigned char)(c2.v*255);
	pHSV->h = h;
	pHSV->s = s;
	pHSV->v = v;
	return true;
}

void ImageUtil::SelectBestCoast(int nImageWidth, int *coastYCoords, int &nLeftCoast, int &nRightCoast) {//find the coastline that matches up best with the coastYCoords found above
	unsigned int uiStartTime = millis();
	//try to narrow down the min and max over which to search to make this algorithm faster
	const int MIN_LEFT_Y = 207;//minimum pixel value for left side corresponding to edge of solar panel
	const int MIN_RIGHT_Y = 180;//minimum pixel value for right side corresponding to edge of solar panel
	int nMinRightY = 479;
	int nMaxLeftY = 0;
	int nMaxRightY = 0;
	for (int i=0;i<nImageWidth/2;i++) {
		if (coastYCoords[i]>nMaxLeftY) {
			nMaxLeftY=coastYCoords[i];
		}
	}
	for (int i=nImageWidth/2;i<nImageWidth;i++) {
		if (coastYCoords[i]>nMaxRightY) {
			nMaxRightY=coastYCoords[i];
		}
	}
	
	int nBestLeftSideY=0;//left side Y-coordinate of best coastline
	int nBestRightSideY=0;//right side Y-coordinate of best coastline
	int nBestScore=0;//count of the most number of pixel matches for a trial coastline
	for (int i=MIN_LEFT_Y;i<=nMaxLeftY;i++) {
		for (int j=MIN_LEFT_Y;j<=nMaxRightY;j++) {
			int nNumMatches=0;
			for (int k=0;k<nImageWidth;k++) {
				double dFactor = ((double)k)/(nImageWidth-1);
				int nY = (1.0-dFactor)*i + dFactor*j;
				if (nY==coastYCoords[k]) {
					nNumMatches++;
				}
			}
			if (nNumMatches>nBestScore) {
				nBestScore=nNumMatches;
				nBestLeftSideY=i;
				nBestRightSideY=j;
			}
		}
	}
	nLeftCoast=nBestLeftSideY;
	nRightCoast=nBestRightSideY;
	unsigned int uiEndTime = millis();
	printf("time to find horizon = %.3f sec\n",((double)(uiEndTime - uiStartTime))/1000.0);
}

//GetImageStats: get a number of statistics for the water part of the image, i.e. below the coast and not including the hull of the boat
//pImage = the image for which we are collecting statistics
//nLeftCoastY = the Y-axis pixel location of the coastline on the left edge of the screen
//nRightCoastY = the Y-axis pixel location of the coastline on the right edge of the screen
//pStats = the returned statistics for the water part of the image
//returns an array of unsigned int values that correspond to the colors of each pixel in the image
unsigned int * ImageUtil::GetImageStats(Mat img, int nLeftCoastY, int nRightCoastY, IMAGE_STATS *pStats) {
	int nImageWidth = img.cols;
	int nImageHeight = img.rows;

	vector <double>brightness;
	vector <double>red;
	vector <double>green;
	vector <double>blue;
	vector <double>hue;
	vector <double>sat;
	vector <double>value;

	for (int i=0;i<nImageWidth;i++) {
		int nVertStartingPoint = GetVertStartingPt(i);//use piecewise linear function to determine vertical starting point that avoids the yellow hull of the boat
		double dFactor = ((double)i)/(nImageWidth-1);
		double dHorizonPos = (1-dFactor)*nLeftCoastY + dFactor*nRightCoastY;
		int nHorizonPt = (int)(dHorizonPos+3);//+3 is so we make sure to avoid using any pixels on the horizon / coast
		for (int j=nVertStartingPoint;j>=nHorizonPt;j--) {
			Vec3b pixel(img.at<Vec3b>(j,i));//read pixel
			double b = (double)pixel.val[0];
			double g = (double)pixel.val[1];
			double r = (double)pixel.val[2];
			double bright = sqrt(r*r+g*g+b*b);	
			brightness.push_back(bright);
			red.push_back(r);
			green.push_back(g);
			blue.push_back(b);

			OPENCSV_HSV hsv;
			GetHSVAt(i,j,img,&hsv);
			hue.push_back(hsv.h);
			sat.push_back(hsv.s);
			value.push_back(hsv.v);
		}
	}
	GetAvgAndSD(brightness,pStats->avg_brightness,pStats->sd_brightness);
	GetAvgAndSD(red,pStats->avg_red,pStats->sd_red);
	GetAvgAndSD(green,pStats->avg_green,pStats->sd_green);
	GetAvgAndSD(blue,pStats->avg_blue,pStats->sd_blue);
	GetAvgAndSD(hue,pStats->avg_hue,pStats->sd_hue);
	GetAvgAndSD(sat,pStats->avg_sat,pStats->sd_sat);
	GetAvgAndSD(value,pStats->avg_value,pStats->sd_value);	

	//now get color info for every pixel in the image
	unsigned int *pColorArray = new unsigned int[nImageWidth*nImageHeight];
	for (int i=0;i<nImageWidth;i++) {
		for (int j=0;j<nImageHeight;j++) {
			pColorArray[j*nImageWidth+i] = GetBGRPixel(img,i,j);
		}
	}
	return pColorArray;
}

void ImageUtil::GetAvgAndSD(vector <double>vals, double &avg_value, double &sd_value) {//compute the average and standard deviation of a list of numbers
	double sum_values=0.0;
	int nNumValues = vals.size();
	for (int i=0;i<nNumValues;i++) {
		sum_values+=vals[i];
	}
	avg_value = sum_values / nNumValues;
	double sum_variances=0.0;
	for (int i=0;i<nNumValues;i++) {
		sum_variances+=pow((vals[i] - avg_value),2);
	}
	sd_value = sqrt(sum_variances/nNumValues);
}

bool ImageUtil::ConvertToHSV(unsigned char rVal, unsigned char gVal, unsigned char bVal, OPENCSV_HSV *pHSV) {//get the OpenCSV style HSV coordinates at a point, given the r,g,b color of that point
	rgb c1;
	c1.r = rVal/255.0;
	c1.g = gVal/255.0;
	c1.b = bVal/255.0;

	hsv c2 = MapUtil::rgb2hsv(c1);
	//printf("h=%.1f, s=%.0f, v=%.0f\n",c2.h,c2.s*255,c2.v*255);
	unsigned char h = (unsigned char)(c2.h/2);
	unsigned char s = (unsigned char)(c2.s*255);
	unsigned char v = (unsigned char)(c2.v*255);
	pHSV->h = h;
	pHSV->s = s;
	pHSV->v = v;
	return true;
}

//GetObjectBlocks: determines objects (if any) that are floating around in the water in the boat's field of view, and below the horizon line in the water
//pixelData: an array of COLOLORREF values that define the pixels of the image
//nImageWidth: the width of the image in pixels
//nImageHeight: the height of the image in pixels
//nLeftHorizonPixelY: the vertical coordinate of the horizon line on the left side of the image
//nRightHorizonPixelY: the vertical coordinate of the horizon line on the right side of the image
//dLowObjectThreshold: the lower threshold for determining whether or not a block connected to an object block, might also be part of the same object
//dHighObjectThreshold: the minimum threshold value required for defining the start of an object floating in the water below the horizon line
//waterStats: the average and standard deviation statistics for the water in the image
vector <OBJECT_BLOCK *> ImageUtil::GetObjectBlocks(unsigned int *pixelData,int nImageWidth,int nImageHeight,int nLeftHorizonPixelY,int nRightHorizonPixelY,
														  double dLowObjectThreshold, double dHighObjectThreshold, IMAGE_STATS *waterStats) {
	const int EXTRA_PIXELS = 0;//number of extra pixels to consider when determining the vertical number of blocks at a particular location in the image (can help with finding objects that are close to the horizon)
	int IGNORE_DIST = 3*BLOCKSIZE;//ignore this many pixels within the hull of the boat
	vector <OBJECT_BLOCK *>objects;

	for (int i=0;i<(nImageWidth-BLOCKSIZE);i+=BLOCKSIZE) {	
		int nStartY = GetVertStartingPt(i)-IGNORE_DIST;//subtract IGNORE_DIST to avoid counting parts of the hull
		double dFactor = ((double)(nImageWidth-1-i))/(nImageWidth-1);
		int nEndY = dFactor*nLeftHorizonPixelY + (1.0-dFactor)*nRightHorizonPixelY;
		
		//nEndY corresponds to a boundary that is divisible by BLOCKSIZE
		//start on block boundary
		nStartY = ((nStartY+BLOCKSIZE/2)/BLOCKSIZE)*BLOCKSIZE;
		int nHeight = nStartY - nEndY;
		if (nHeight<0) continue;
		nHeight = ((nHeight+EXTRA_PIXELS)/BLOCKSIZE)*BLOCKSIZE;//make sure height being checked is a multiple of BLOCKSIZE
		nEndY = nStartY - nHeight;
		for (int j=nStartY;j>=nEndY;j-=BLOCKSIZE) {
			unsigned int avgColor = GetBlockColor(pixelData, i, j, nImageWidth);
		
			if (isObjectBlock(avgColor,waterStats,dHighObjectThreshold,false)) {
				OBJECT_BLOCK *pBlock = new OBJECT_BLOCK;
				pBlock->nX = i;
				pBlock->nY = j;
				pBlock->nWidth = BLOCKSIZE;
				pBlock->nHeight = BLOCKSIZE;
				objects.push_back(pBlock);
			}
		}
	}
	if (objects.size()<=0) return objects;
	//now find other object blocks whose score might not be quite as high, but is still somewhat high and are connected to
	//already existing blocks, either to left, right, above, or diagonally.
	int nNumOtherBlocksAdded=0;
	
	do {
		nNumOtherBlocksAdded=0;
		for (int i=0;i<(nImageWidth-BLOCKSIZE);i+=BLOCKSIZE) {	
			int nStartY = GetVertStartingPt(i)-IGNORE_DIST;//subtract IGNORE_DIST to avoid counting parts of the hull
			double dFactor = ((double)(nImageWidth-1-i))/(nImageWidth-1);
			int nEndY = dFactor*nLeftHorizonPixelY + (1.0-dFactor)*nRightHorizonPixelY;
			//nEndY corresponds to a boundary that is divisible by BLOCKSIZE
			//start on block boundary
			nStartY = ((nStartY+BLOCKSIZE/2)/BLOCKSIZE)*BLOCKSIZE;
			int nHeight = nStartY - nEndY;
			if (nHeight<0) continue;
			nHeight = ((nHeight+EXTRA_PIXELS)/BLOCKSIZE)*BLOCKSIZE;//make sure height being checked is a multiple of BLOCKSIZE
			nEndY = nStartY - nHeight;
			for (int j=nStartY;j>=nEndY;j-=BLOCKSIZE) {
				if (!isObjectBlock(i,j,objects)) {//not one of the previously found objects, but maybe close to one and still with a high-ish score
					unsigned int avgColor = GetBlockColor(pixelData, i, j, nImageWidth);
					if (isObjectBlock(avgColor,waterStats,dLowObjectThreshold,true)) {//qualifies for the low threshold
						if (isNearObjectBlock(i,j,objects)) {//and is also near an existing object, so consider it to be an object too
							OBJECT_BLOCK *pBlock = new OBJECT_BLOCK;
							pBlock->nX = i;
							pBlock->nY = j;
							pBlock->nWidth = BLOCKSIZE;
							pBlock->nHeight = BLOCKSIZE;
							objects.push_back(pBlock);
							nNumOtherBlocksAdded++;
						}
					}
				}
			}
		}
	} while (nNumOtherBlocksAdded>0);
	return objects;
}

unsigned int ImageUtil::GetBlockColor(unsigned int *pixelData, int nX, int nY, int nImageWidth) {//gets the average color of a BLOCKSIZE x BLOCKSIZE grouping of pixels whose upper-left corner starts at nX, nY
	double dRedSum=0.0;
	double dGreenSum=0.0;
	double dBlueSum=0.0;
	if (!pixelData) return 0;
	for (int i=nX;i<(nX+BLOCKSIZE);i++) {
		for (int j=nY;j<(nY+BLOCKSIZE);j++) {
			unsigned int pixelColor=pixelData[i+j*nImageWidth];
			dRedSum+=GetRValue(pixelColor);
			dGreenSum+=GetGValue(pixelColor);
			dBlueSum+=GetBValue(pixelColor);
		}
	}
	unsigned char avgRed = (unsigned char)(dRedSum/(BLOCKSIZE*BLOCKSIZE));
	unsigned char avgGreen = (unsigned char)(dGreenSum/(BLOCKSIZE*BLOCKSIZE));
	unsigned char avgBlue = (unsigned char)(dBlueSum/(BLOCKSIZE*BLOCKSIZE));

	unsigned int avgColor = avgRed + (avgGreen<<8) + (avgBlue<<16);
	return avgColor;
}

bool ImageUtil::isObjectBlock(unsigned int blockColor, IMAGE_STATS *waterStats, double dObjectThreshold, bool bLowThresholdMode) {//returns true if a block's color qualifies it as an object block
	const int MIN_WHITE_VALUE = 180;//minimum value for saturation to be considered an object when hue and saturation are both zero
	const int MIN_SAT_VALUE = 11;//threshold for low-saturation 
	const int BLUE_FILTER_VAL = 5;//filter out any block color that is mostly blue
	const int BLACK_FILTER_VAL = 85;//filter out block colors that are too dark
	unsigned char rVal = GetRValue(blockColor);
	unsigned char gVal = GetGValue(blockColor);
	unsigned char bVal = GetBValue(blockColor);

	int nDifBG = bVal - gVal;
	int nDifBR = bVal - rVal;
	double dBrightnessFactor = 255.0 * 3 / (rVal+gVal+bVal);//factor is larger the darker the color is... used in conjunction with "blue" filter below
	double dDifBG = (bVal - gVal)*dBrightnessFactor;
	double dDifBR = (bVal - rVal)*dBrightnessFactor;
	if (dDifBG>=BLUE_FILTER_VAL&&dDifBR>=BLUE_FILTER_VAL) {
		return false;
	}
	//convert blockColor to HSV
	OPENCSV_HSV hsv;
	ConvertToHSV(rVal, gVal, bVal, &hsv);//get the OpenCSV style HSV coordinates at a point, given the r,g,b color of that point
	//filter out blocks that have less saturation than MIN_SAT_VALUE and less brightness than MIN_WHITE_VALUE, as these are most likely caused by water droplets on the camera window
	if (!bLowThresholdMode&&hsv.s<=MIN_SAT_VALUE&&hsv.v<MIN_WHITE_VALUE) {
		return false;
	}
	if (hsv.v<BLACK_FILTER_VAL) {
		return false;//too dark
	}
	double dScore = CalculateHSVScore(&hsv,waterStats);
	bool bRetval = (dScore>=dObjectThreshold);
	return bRetval;
}

double ImageUtil::CalculateHSVScore(OPENCSV_HSV *hsvColor, IMAGE_STATS *waterStats) {//return a score based on how different a particular color is from the average water color
	if (!hsvColor||!waterStats) {
		return 0.0;
	}
	double sh = fabs((hsvColor->h - waterStats->avg_hue) / waterStats->sd_hue);
	double ss = fabs((hsvColor->s - waterStats->avg_sat) / waterStats->sd_sat);
	//scale sh downward if saturation is lower than average
	double dSatFactor = max(waterStats->avg_sat,(double)30);
	if (hsvColor->s<dSatFactor) {
		sh*=((double)hsvColor->s)/dSatFactor;
	}
	double sv = fabs((hsvColor->v - waterStats->avg_value) / waterStats->sd_value);
	double dRetval = sh + ss + sv;
	return dRetval;
}

bool ImageUtil::isObjectBlock(int nX, int nY, vector <OBJECT_BLOCK *>objects) {//return true if one of the objects starts at nX, nY
	int nNumObjects = objects.size();
	for (int i=0;i<nNumObjects;i++) {
		if (objects[i]->nX==nX&&objects[i]->nY==nY) {
			return true;
		}
	}
	return false;
}

bool ImageUtil::isNearObjectBlock(int nX, int nY, vector <OBJECT_BLOCK *>objects) {//return true if the block at nX, nY is close to one of the blocks listed in objects, either to left, right, above, below, or diagonally
	int nNumObjects = objects.size();
	for (int i=0;i<nNumObjects;i++) {
		if ((nX-BLOCKSIZE)==objects[i]->nX) {//check to see if object is to left of nX, nY
			if (nY>=(objects[i]->nY-BLOCKSIZE)&&nY<=objects[i]->nY+BLOCKSIZE) {
				return true;
			}
		}
		else if ((nX+BLOCKSIZE)==objects[i]->nX) {//check to see if object is to right of nX, nY
			if (nY>=(objects[i]->nY-BLOCKSIZE)&&nY<=objects[i]->nY+BLOCKSIZE) {
				return true;
			}
		}
		else if (nX==objects[i]->nX) {//check to see if object is directly above or below nX, nY
			if (nY==(objects[i]->nY-BLOCKSIZE)||nY==(objects[i]->nY+BLOCKSIZE)) {
				return true;
			}
		}
	}
	return false;
}

unsigned char ImageUtil::GetRValue(unsigned int c) {//gets the red component of an rgb color
	unsigned char retVal = (unsigned char)(c&0x000000ff);
	return retVal;
}

unsigned char ImageUtil::GetGValue(unsigned int c) {//gets the green component of an rgb color
	unsigned char retVal = (unsigned char)((c&0x0000ff00)>>8);
	return retVal;
}

unsigned char ImageUtil::GetBValue(unsigned int c) {//gets the blue component of an rgb color
	unsigned char retVal = (unsigned char)((c&0x00ff0000)>>16);
	return retVal;
}

unsigned int ImageUtil::GetBGRPixel(Mat img,int nX,int nY) {//gets BGR color of pixel at nX, nY in img
	Vec3b pixel(img.at<Vec3b>(nY,nX));//read pixel
	unsigned char b = pixel.val[0];
	unsigned char g = pixel.val[1];
	unsigned char r = pixel.val[2];
	unsigned long retVal = r + (g<<8) + (b<<16);
	return retVal;
}