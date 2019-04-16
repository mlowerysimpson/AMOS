#include "FeatureGroups.h"

FeatureGroups::FeatureGroups(vector <cv::Point2f> allPoints, int nImageWidth, int nImageHeight, float fMaxSeparation) {
	m_nImageHeight = nImageHeight;
	m_nImageWidth = nImageWidth;
	m_fMaxSeparation = fMaxSeparation;
	try {
		MakeGroups(allPoints);
	}
	catch (std::exception& e) {
		std::cerr << "Exception catched : " << e.what() << std::endl;
	}
}

FeatureGroups::~FeatureGroups() {//destructor
	int nNumGroups = GetCount();
	for (int i=0;i<nNumGroups;i++) {
		FEATURE_GROUP *pGroup = GetFeatureGroup(i);
		if (pGroup) {
			delete pGroup;
			pGroup = nullptr;
		}
	}
}

int FeatureGroups::GetCount() {//return the number of feature groups that were found in the image
	return m_features.size();
}

FEATURE_GROUP * FeatureGroups::GetFeatureGroup(int nFeatureGroupIndex) {//return a given feature group
	//nFeatureGroupIndex = the index of the group to get
	//returns an elliptical grouping of features of type FEATURE_GROUP
	int nNumGroups = GetCount();
	if (nFeatureGroupIndex<0||nFeatureGroupIndex>=nNumGroups) {
		return nullptr;
	}
	return m_features[nFeatureGroupIndex];
}

void FeatureGroups::MakeGroups(vector <cv::Point2f>allPoints) {//go through all of the key points and make groups from connected features
	int nNumPts = allPoints.size();
	for (int i=0;i<nNumPts;i++) {
		//FEATURE_GROUP *pNearbyGroup = isNearExistingGroup(allPoints[i]);//get group (if any) that is nearby or enclosing pt
		//if (pNearbyGroup) {
		//	AddPtToGroup(pNearbyGroup, allPoints[i]);
		//}
		FEATURE_GROUP *pConnectedToGroup = isConnectedToExistingGroup(allPoints[i]);//get group (if any) that is connected to pt
		if (pConnectedToGroup) {
			AddPtToGroup(pConnectedToGroup, allPoints[i]);
		}	
		else {//begin a new group
			BeginNewGroup(allPoints[i]);
		}	
	}
	//CombineOverlappedGroups();
	DiscardTinyGroups();//get rid of the smallest groups of features
}
	
FEATURE_GROUP * FeatureGroups::isNearExistingGroup(cv::Point2f pt) { //return existing FEATURE_GROUP if pt happens to be nearby or within it
	//pt = point to check for closeness to existing groups of points
	//returns FEATURE_GROUP that is close to or enclosing pt, or nullptr if pt is not near any group
	int nNumGroups = GetCount();
	for (int i=0;i<nNumGroups;i++) {
		if (pt.x>=(m_features[i]->fCenterX-m_features[i]->fWidth/2)) {
			if (pt.x<=(m_features[i]->fCenterX+m_features[i]->fWidth/2)) {
				if (pt.y>=(m_features[i]->fCenterY-m_features[i]->fHeight/2)) {
					if (pt.y<=(m_features[i]->fCenterY+m_features[i]->fHeight/2)) {
						//point is close or already within group i
						return m_features[i];
					}
				}
			}
		}
	}
	return nullptr;
}

void FeatureGroups::BeginNewGroup(cv::Point2f pt) {//create a new group of points, starting at pt	
	//pt = point around which to create the new group of points
	FEATURE_GROUP *pNewGroup = new FEATURE_GROUP();
	pNewGroup->fCenterX = pt.x;
	pNewGroup->fCenterY = pt.y;
	pNewGroup->fHeight = 2*m_fMaxSeparation+1;
	pNewGroup->fWidth = 2*m_fMaxSeparation+1;
	pNewGroup->fDist=0.0;
	pNewGroup->pts.push_back(pt);
	m_features.push_back(pNewGroup);
}
	
	
void FeatureGroups::AddPtToGroup(FEATURE_GROUP *pGroup, cv::Point2f pt) {//add pt to a group of points and update the centroid and possibly the height and width	
	//pGroup = the feature group to which the point pt will be added
	//pt = the new point that is being added to the feature group pGroup
	//first get old limits of group
	float fMaxY = pGroup->fCenterY + pGroup->fHeight/2;
	float fMinY = pGroup->fCenterY - pGroup->fHeight/2;
	float fMaxX = pGroup->fCenterX + pGroup->fWidth/2;
	float fMinX = pGroup->fCenterX - pGroup->fWidth/2;
	pGroup->pts.push_back(pt);
	//check to see if limits of group should be updated
	float fTestMinX = pt.x - m_fMaxSeparation;
	float fTestMaxX = pt.x + m_fMaxSeparation;
	float fTestMinY = pt.y - m_fMaxSeparation;
	float fTestMaxY = pt.y + m_fMaxSeparation;
	if (fTestMinX<fMinX) {
		fMinX = fTestMinX;
	}
	if (fTestMaxX>fMaxX) {
		fMaxX = fTestMaxX;
	}
	if (fTestMinY<fMinY) {
		fMinY = fTestMinY;
	}
	if (fTestMaxY>fMaxY) {
		fMaxY = fTestMaxY;
	}
	pGroup->fCenterX = (fMinX+fMaxX)/2;
	pGroup->fCenterY = (fMinY+fMaxY)/2;
	pGroup->fHeight = fMaxY - fMinY;
	pGroup->fWidth = fMaxX - fMinX;	
}
	
	
void FeatureGroups::DrawFeatureGroup(InputOutputArray img, int nFeatureGroupIndex, bool bDrawDistText) {//draw an ellipse that surrounds the feature group	
	//img = the image upon which to draw the found feature groups (feature groups drawn as ellipses)
	//bDrawDistText = flag which is true if text should be overlaid on the image indicating the distance (in m) to each stereo matched feature
	FEATURE_GROUP *pGroup = GetFeatureGroup(nFeatureGroupIndex);
	if (!pGroup) return;
	Scalar objectColor = Scalar(255,0,0);//default to blue
	Scalar textColor = Scalar(0,255,255);//text color set to yellow
	if (pGroup->fDist!=0.0) {//was able to find a stereo correspondence for this object (i.e. it was detected by both the left and right cameras, so use a different color to indicate that)
		objectColor = Scalar(0,255,0);//green
		if (bDrawDistText) {//draw text indicating distance (in m) to stereo matched feature
			char distText[32];
			Point textOrigin(pGroup->fCenterX - pGroup->fWidth, pGroup->fCenterY);
			sprintf(distText,"%.2f",pGroup->fDist);
			putText(img, distText, textOrigin, FONT_HERSHEY_SIMPLEX, .5, textColor);
		}
	}
	Size2f featureSize(pGroup->fWidth, pGroup->fHeight);
	RotatedRect box(Point2f(pGroup->fCenterX, pGroup->fCenterY),featureSize,0);
	ellipse(img, box, objectColor, 1, LINE_8 );
}
	
void FeatureGroups::DrawAllFeatureGroups(InputOutputArray img, bool bDisplayDistText) {//draw ellipses around each feature group, with optional distance text to stereo matched features
	//img = the image upon which to draw the found feature groups as ellipses
	//bDisplayDistText = true if you want to see descriptive text indicating the distance (in m) to each stereo matched feature group
	int nNumGroups = GetCount();
	for (int i=0;i<nNumGroups;i++) {
		DrawFeatureGroup(img,i,bDisplayDistText);
	}
}
	
void FeatureGroups::CombineOverlappedGroups() {//combine groups that overlap into a single larger group	
	int nNumGroups = GetCount();
	if (nNumGroups<=1) return;
	vector<FEATURE_GROUP *>::iterator it = m_features.begin();
	vector<FEATURE_GROUP *>::iterator it_next;
	for (int i=0;i<nNumGroups;i++) {
		FEATURE_GROUP *pGrp1 = GetFeatureGroup(i);
		it_next = std::next(it);
		for (int j=i+1;j<nNumGroups;j++) {
			FEATURE_GROUP *pGrp2 = GetFeatureGroup(j);
			if (GroupsOverlap(pGrp1,pGrp2)) {
				float fMinX = min(pGrp1->fCenterX - pGrp1->fWidth/2, pGrp2->fCenterX - pGrp2->fWidth/2);
				float fMaxX = max(pGrp1->fCenterX + pGrp1->fWidth/2, pGrp2->fCenterX + pGrp2->fWidth/2);
				float fMinY = min(pGrp1->fCenterY - pGrp1->fHeight/2, pGrp2->fCenterY - pGrp2->fHeight/2);
				float fMaxY = max(pGrp1->fCenterY + pGrp1->fHeight/2, pGrp2->fCenterY + pGrp2->fHeight/2);
				float fNewCenterX = (fMinX + fMaxX)/2;
				float fNewCenterY = (fMinY + fMaxY)/2;
				pGrp1->fCenterX = fNewCenterX;
				pGrp1->fCenterY = fNewCenterY;
				pGrp1->fWidth = fMaxX - fMinX;
				pGrp1->fHeight = fMaxY - fMinY;
				//copy points from group 2 over to group 1
				int nNumPts = pGrp2->pts.size();
				for (int k=0;k<nNumPts;k++) {
					pGrp1->pts.push_back(pGrp2->pts[k]);
				}
				//delete group 2
				delete m_features[j];
				m_features[j]=nullptr;
				m_features.erase(it_next);
				CombineOverlappedGroups();//recursively call this function with reduced number of groups
				return;
			}
			it_next++;
		}
		it++;
	}
}
	
	
bool FeatureGroups::GroupsOverlap(FEATURE_GROUP *pGrp1, FEATURE_GROUP *pGrp2) {//return true if pGrp1 and pGrp2 overlap	
	//just do a simple rectangular test for overlapping; this is not really correct for elliptical shapes, but is easy and close enough
	float fMinX1 = pGrp1->fCenterX - pGrp1->fWidth/2;
	float fMaxX1 = pGrp1->fCenterX + pGrp1->fWidth/2;
	float fMinY1 = pGrp1->fCenterY - pGrp1->fHeight/2;
	float fMaxY1 = pGrp1->fCenterY + pGrp1->fHeight/2;
	float fMinX2 = pGrp2->fCenterX - pGrp2->fWidth/2;
	float fMaxX2 = pGrp2->fCenterX + pGrp2->fWidth/2;
	float fMinY2 = pGrp2->fCenterY - pGrp2->fHeight/2;
	float fMaxY2 = pGrp2->fCenterY + pGrp2->fHeight/2;
	//test for fMinX1, fMinY1 in 2nd rect
	if (fMinX1>=fMinX2&&fMinX1<=fMaxX2) {
		if (fMinY1>=fMinY2&&fMinY1<=fMaxY2) {
			return true;
		}
		//test for fMinX1, fMaxY1 in 2nd rect
		if (fMaxY1>=fMinY2&&fMaxY1<=fMaxY2) {
			return true;
		}
	}
	//test for fMaxX1, fMinY1 in 2nd rect
	if (fMaxX1>=fMinX2&&fMaxX1<=fMaxX2) {
		if (fMinY1>=fMinY2&&fMinY1<=fMaxY2) {
			return true;
		}
		//test for fMaxX1, fMaxY1 in 2nd rect
		if (fMaxY1>=fMinY2&&fMaxY1<=fMaxY2) {
			return true;
		}
	}
	
	
	//test for fMinX2, fMinY2 in 1st rect
	if (fMinX2>=fMinX1&&fMinX2<=fMaxX1) {
		if (fMinY2>=fMinY1&&fMinY2<=fMaxY1) {
			return true;
		}
		//test for fMinX2, fMaxY2 in 1st rect
		if (fMaxY2>=fMinY1&&fMaxY2<=fMaxY1) {
			return true;
		}
	}
	//test for fMaxX2, fMinY2 in 1st rect
	if (fMaxX2>=fMinX1&&fMaxX2<=fMaxX1) {
		if (fMinY2>=fMinY1&&fMinY2<=fMaxY1) {
			return true;
		}
		//test for fMaxX2, fMaxY2 in 1st rect
		if (fMaxY2>=fMinY1&&fMaxY2<=fMaxY1) {
			return true;
		}
	}
	return false;
}
	
void FeatureGroups::DiscardTinyGroups() {//get rid of the smallest groups of features	
	const int SMALLSIZE_LIMIT = 1;//get rid of any groups that have this many features or less
	vector<FEATURE_GROUP *>::iterator it = m_features.end();
	it--;//start by pointing to last element
	int nNumGroups = GetCount();
	for (int i=nNumGroups-1;i>=0;i--) {
		FEATURE_GROUP *pGroup = GetFeatureGroup(i);
		int nNumPts = pGroup->pts.size();
		if (nNumPts<=SMALLSIZE_LIMIT) {
			delete pGroup;
			m_features.erase(it);
		}
		it--;
	}
}

FEATURE_GROUP *FeatureGroups::isConnectedToExistingGroup(cv::Point2f pt) {//return existing FEATURE_GROUP if pt happens to be connected to it
	//pt = point to check for connectedness to existing groups of points
	//returns first FEATURE_GROUP that is connected to pt, or nullptr if pt is not connected to any group
	int nNumGroups = GetCount();
	for (int i=0;i<nNumGroups;i++) {
		int nNumGroupPts = m_features[i]->pts.size();
		for (int j=0;j<nNumGroupPts;j++) {
			float fMinX = m_features[i]->pts[j].x - m_fMaxSeparation;
			float fMaxX = m_features[i]->pts[j].x + m_fMaxSeparation;
			float fMinY = m_features[i]->pts[j].y - m_fMaxSeparation;
			float fMaxY = m_features[i]->pts[j].y + m_fMaxSeparation;
			if (pt.x>=fMinX&&pt.x<=fMaxX) {
				if (pt.y>=fMinY&&pt.y<=fMaxY) {//point is close enough to group i to be considered connected to it
					return m_features[i];
				}
			}
		}
	}
	return nullptr;
}
	
void FeatureGroups::StereoMatchFeatures(FeatureGroups *leftCamGroups, FeatureGroups *rightCamGroups, int nImageWidth, int nImageHeight, float fCameraSeparation) {//look for common features from both cameras and try to figure out distance to each of those common features
	//leftCamGroups = FeatureGroups object from the left camera
	//rightCamGroups = FeatuerGroups object from the right camera
	//nImageWidth = the width of the left and right camera images in pixels
	//nImageHeight = the height of the left and right camera images in pixels
	//fCameraSeparation = lateral separation between cameras in m
	const float HORIZ_TOLERANCE = 30;//objects must be horizontally within this # of pixels to be considered the same object
	const float VERT_TOLERANCE = 30;//objects must be vertically within this # of pixels to be considered the same object
	const float SIZE_TOLERANCE = .5;//relative size limit to use when comparing objects
		
	int nNumLeftGroups = leftCamGroups->GetCount();
	for (int i=0;i<nNumLeftGroups;i++) {
		FEATURE_GROUP *pLeftGroup = leftCamGroups->GetFeatureGroup(i);
		/*for (int j=0;j<nNumRightGroups;j++) {
			FEATURE_GROUP *pRightGroup = rightCamGroups->GetFeatureGroup(j);
			if (featuresMatch(pLeftGroup,pRightGroup,HORIZ_TOLERANCE,VERT_TOLERANCE,SIZE_TOLERANCE, nImageWidth, nImageHeight)) {
				UpdateMatchingStereoData(pLeftGroup,pRightGroup,nImageWidth,nImageHeight,fCameraSeparation);
				break;
			}	
		}*/
		FEATURE_GROUP *pBestRightMatch = FindBestMatch(pLeftGroup, rightCamGroups, HORIZ_TOLERANCE, VERT_TOLERANCE, SIZE_TOLERANCE, nImageWidth, nImageHeight);
		if (pBestRightMatch!=nullptr) {
			UpdateMatchingStereoData(pLeftGroup, pBestRightMatch, nImageWidth, nImageHeight, fCameraSeparation);
		}
	}
}
	
bool FeatureGroups::featuresMatch(FEATURE_GROUP *pLeftGroup, FEATURE_GROUP *pRightGroup, float horiz_tolerance, float vert_tolerance, float size_tolerance, 
	int nImageWidth, int nImageHeight) {//do tests to determine whether or not two feature groups (from left and right cameras) correspond to the same physical feature in the real world
	if (pLeftGroup->fCenterX < pRightGroup->fCenterX) {//left camera group appears to left of right camera group, which should not be possible if pLeftGroup and pRightGroup correspond to the same real world object
		return false;
	}
	//check vertical tolerance
	if (fabsf(pLeftGroup->fCenterY - pRightGroup->fCenterY)>vert_tolerance) {
		return false;
	}
	//check horizontal tolerance
	if ((pLeftGroup->fCenterX - pRightGroup->fCenterX)>horiz_tolerance) {
		return false;
	}
	//check size tolerance
	float fLeftSize = pLeftGroup->fWidth * pLeftGroup->fHeight;
	float fRightSize = pRightGroup->fWidth * pRightGroup->fHeight;
	float fSizeTest = min(fLeftSize,fRightSize) / max(fLeftSize,fRightSize);
	if (fSizeTest<size_tolerance) {
		return false;
	}
	return true;
}
	
void FeatureGroups::UpdateMatchingStereoData(FEATURE_GROUP *pLeftMatch,FEATURE_GROUP *pRightMatch, int nImageWidth, int nImageHeight, float fCameraSeparation) {//calculate and store depth data for matching feature groups
	//pLeftMatch = feature object from left camera view that is the same physical object as pRightMatch from the right camera view
	//pRightMatch = feature object from right camera view that is the same physical object as pLeftMatch from the left camera view
	//nImageWidth = the width of the images taken by the left and right camears in pixels
	//nImageHeight = the height of the images taken by the left and right cameras in pixels
	//fCameraSeparation = lateral separation of cameras in m
	//use equation 	D = X1 / tan(ang1), where:
	//D = normal distance from plane of cameras to image plane where the feature object of interest (FOI) lies 
	//X1 = lateral distance (in m) of FOI from left camera
	//ang1 = lateral angular position of FOI in left camera image (assumes 10 pixels per degree for PiCam v2)
	//also have this equation for X1:
	//X1 = d / tan(ang2) / (1 / tan(ang1) - 1 / tan(ang2))
	//d = lateral separation between cameras (in m)
	//ang2 = lateral angular position of FOI in right camera image (also assumes 10 pixels per degree for same PiCam v2)
	float d = .079;//lateral separation between cameras (in m), i.e. how far the "eyes" are apart
	const float RADIANS_PER_PIXEL = 0.0017;//approximate empirical determination of radians per pixel of viewing angle for Raspberry Pi camera V2. (corresponds to ~ 10 pixels per degree)
	const float LARGE_DISTANCE = 1000.0;//use this distance to avoid divide by zero infinite errors
	
	float origin_x = nImageWidth/2;//x-axis coordinate in pixels of the center of the image
	float origin_y = nImageHeight/2;//y-axis coordinate in pixes of the center of the image
	float ang1 = (pLeftMatch->fCenterX - origin_x) * RADIANS_PER_PIXEL;//lateral viewing angle of object as seen by left camera
	float ang2 = (pRightMatch->fCenterX - origin_x) * RADIANS_PER_PIXEL;//lateral viewing angle of object as seen by right camera
	float D = LARGE_DISTANCE;
	if (ang2!=0&&ang1!=0) {
		if (ang2!=ang1) {
			float X1 = d / tanf(ang2) / (1 / tanf(ang1) - 1 / tanf(ang2));//lateral distance (in m) of object from left camera
			D = X1 / tanf(ang1);
		}
	}
	else if (ang1!=0) {//ang2 is zero
		D = d / tan(ang1);
	}
	else if (ang2!=0) {//ang1 is zero
		D = d / tan(ang2);
	}
	//D is just the normal distance from the camera plane to the image plane, need to figure out the actual distance, from the robot to the image feature
	//get vertical angles to 
	float vert_ang1 = (pLeftMatch->fCenterY - origin_y)*RADIANS_PER_PIXEL;//vertical viewing angle of object as seen by left camera
	float vert_ang2 = (pRightMatch->fCenterY - origin_y)*RADIANS_PER_PIXEL;//vertical viewing angle of object as seen by right camera
	float X1 = tanf(ang1)*D;//lateral distance from left camera to object
	float X2 = tanf(ang2)*D;//lateral distance from right camera to object
	float Y1 = tanf(vert_ang1)*D;//vertical distance from left camera to object
	float Y2 = tanf(vert_ang2)*D;//vertical distance from right camera to object
	float fDist1 = sqrt(D*D + X1*X1 + Y1*Y1);//total distance from left camera to object
	float fDist2 = sqrt(D*D + X2*X2 + Y2*Y2);//total distance from right camera to object
	float fAvgDist = (fDist1+fDist2)/2;//average distance from cameras (i.e. robot) to object
	pLeftMatch->fDist = fAvgDist;
	pRightMatch->fDist = fAvgDist;
}
	
	
FEATURE_GROUP * FeatureGroups::FindBestMatch(FEATURE_GROUP *pGroup, FeatureGroups *pGroupCollection, float horiz_tolerance, float vert_tolerance, float size_tolerance, int nImageWidth, int nImageHeight) {//find the best match for pGroup out of all the features in pGroupCollection	
	//pGroup = the feature group for which we are trying to find a match
	//pGroupCollection = a FeatureGroups object containing potential feature groups that could match pGroup, this function returns the best match, if possible or nullptr if no match could be found
	int nNumToCheck = pGroupCollection->GetCount();
	if (nNumToCheck<=0) return nullptr;
	FEATURE_GROUP *pFirst = pGroupCollection->GetFeatureGroup(0);
	float fSize1 = pGroup->fWidth * pGroup->fHeight;
	float fDifX = fabsf(pGroup->fCenterX - pFirst->fCenterX);
	float fDifY = fabsf(pGroup->fCenterY - pFirst->fCenterY);
	float fSep = sqrt(fDifX*fDifX + fDifY*fDifY);
	float fMinSep = fSep;//set minimum separation to be the separation between pGroup and the first group in pGroupCollection
	FEATURE_GROUP *pBestMatch = nullptr;
	if (fDifX<horiz_tolerance&&fDifY<vert_tolerance) {//close enough to be a potential match
		//check size to make sure that it is similar 
		float fSize2 = pFirst->fWidth * pFirst->fHeight;
		float fSizeTest = min(fSize1,fSize2) / max(fSize1,fSize2);
		if (fSizeTest>=size_tolerance) {
			pBestMatch = pFirst;//set pBestMatch to the first feature group
		}
	}
	//go through remaining feature groups to see if a better match can be found
	for (int i=1;i<nNumToCheck;i++) {
		FEATURE_GROUP *pNext = pGroupCollection->GetFeatureGroup(i);
		fDifX = fabsf(pGroup->fCenterX - pNext->fCenterX);
		fDifY = fabsf(pGroup->fCenterY - pNext->fCenterY);
		fSep = sqrt(fDifX*fDifX + fDifY*fDifY);
		if (fSep<fMinSep) {
			if (fDifX<horiz_tolerance&&fDifY<vert_tolerance) {//close enough to be a potential match
				//check size to make sure that it is similar
				float fSize2 = pNext->fWidth * pNext->fHeight;
				float fSizeTest = min(fSize1, fSize2) / max(fSize1, fSize2);
				if (fSizeTest>=size_tolerance) {//size is similar
					pBestMatch = pNext;
					fMinSep = fSep;
				}
			}
		}
	}
	return pBestMatch;
}
	
	
	
	
	
	
		

