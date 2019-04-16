#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.hpp>

using namespace std;
using namespace cv;

struct FEATURE_GROUP {//assume the shape of an ellipse for each feature
	float fCenterX;//x-axis centroid of feature (in pixels)
	float fCenterY;//y-axis centroid of feature (in pixels)
	float fHeight;//the height of the feature ellipse (in pixels)
	float fWidth;//the width of the feature ellipse (in pixels)
	float fDist;//distance from object to plane of cameras (in m) (will be zero if a stereo feature match could not be found)
	vector <cv::Point2f> pts;
};

class FeatureGroups {
public:
	FeatureGroups(vector <cv::Point2f> allPoints, int nImageWidth, int nImageHeight, float fMaxSeparation);
	~FeatureGroups();
	
	int GetCount();//return the number of feature groups that were found in the image
	FEATURE_GROUP * GetFeatureGroup(int nFeatureGroupIndex);//return a given feature group
	void DrawFeatureGroup(InputOutputArray img, int nFeatureGroupIndex, bool bDrawDistText);//draw a blue ellipse that surrounds the feature group
	void DrawAllFeatureGroups(InputOutputArray img, bool bDisplayDistText);//draw blue ellipses around each feature group
	static void StereoMatchFeatures(FeatureGroups *leftCamGroups, FeatureGroups *rightCamGroups, int nImageWidth, int nImageHeight, float fCameraSeparation);//look for common features from both cameras and try to figure out distance to each of those common features
	static FEATURE_GROUP * FindBestMatch(FEATURE_GROUP *pGroup, FeatureGroups *pGroupCollection, float horiz_tolerance, float vert_tolerance, float size_tolerance, int nImageWidth, int nImageHeight);//find the best match for pGroup out of all the features in pGroupCollection
		 
private:
	//data	
	vector <FEATURE_GROUP *> m_features;//all of the ellipse features that were found in the image
	float m_fMaxSeparation;//maximum allowed separation in pixels allowed between feature key points
	vector <cv::Point2f> m_allPts;//all of the feature points in the image that need to be formed into groups
	int m_nImageWidth;//width of the image
	int m_nImageHeight;//height of the image
	
	//functions
	bool GroupsOverlap(FEATURE_GROUP *pGrp1, FEATURE_GROUP *pGrp2);//return true if pGrp1 and pGrp2 overlap
	void MakeGroups(vector <cv::Point2f> allPoints);//go through all of the key points and make groups from connected features
	FEATURE_GROUP * isNearExistingGroup(cv::Point2f pt); //return existing FEATURE_GROUP if pt happens to be nearby or within it
	FEATURE_GROUP *isConnectedToExistingGroup(cv::Point2f pt);//return existing FEATURE_GROUP if pt happens to be connected to it
	void BeginNewGroup(cv::Point2f pt);//create a new group of points, starting at pt
	void AddPtToGroup(FEATURE_GROUP *pGroup, cv::Point2f pt);//add pt to a group of points and update the centroid and possibly the height and width
	void CombineOverlappedGroups();//combine groups that overlap into a single larger group
	void DiscardTinyGroups();//get rid of the smallest groups of features
	static bool featuresMatch(FEATURE_GROUP *pLeftGroup, FEATURE_GROUP *pRightGroup, float horiz_tolerance, float vert_tolerance, float size_tolerance, int nImageWidth, int nImageHeight);//do tests to determine whether or not two feature groups (from left and right cameras) correspond to the same physical feature in the real world
	static void UpdateMatchingStereoData(FEATURE_GROUP *pLeftMatch,FEATURE_GROUP *pRightMatch, int nImageWidth, int nImageHeight, float fCameraSeparation);//calculate and store depth data for matching feature groups
};
