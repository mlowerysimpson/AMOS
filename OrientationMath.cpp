// OrientationMath.cpp: implementation of the OrientationMath class.
//
//////////////////////////////////////////////////////////////////////
#include "OrientationMath.h"
#include "3DMATH.H"
#include <math.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

OrientationMath::OrientationMath()
{

}

OrientationMath::~OrientationMath()
{

}

void OrientationMath::MatMult(double mat1[3][3], double mat2[3][3], double result[3][3]) {
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			result[i][j] = mat1[i][0]*mat2[0][j] + mat1[i][1]*mat2[1][j] + mat1[i][2]*mat2[2][j];
		}
	}
}

void OrientationMath::MatMult(float mat1[3][3], float mat2[3][3], float result[3][3]) {
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			result[i][j] = mat1[i][0]*mat2[0][j] + mat1[i][1]*mat2[1][j] + mat1[i][2]*mat2[2][j];
		}
	}
}

void OrientationMath::MatrixInverse(double newmat[3][3], double origmat[3][3]) {
	//finds inverse of a 3X3 orientation matrix
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			newmat[i][j]=origmat[j][i];
		}
	}
}

void OrientationMath::MatVectMult(double mat[][3], double vect[], double result[]) {
	//multiplies a matrix by a vector to get a vector result
	for (int i=0;i<3;i++) {
		result[i] = mat[i][0]*vect[0] + mat[i][1]*vect[1] + mat[i][2]*vect[2];
	}
}

void OrientationMath::MatVectMult(float mat[][3], float vect[], float result[]) {
	//multiplies a matrix by a vector to get a vector result
	for (int i=0;i<3;i++) {
		result[i] = mat[i][0]*vect[0] + mat[i][1]*vect[1] + mat[i][2]*vect[2];
	}
}

void OrientationMath::rotline(double* fpL1, double* fpLA, double fAlpha)
{
	// The center of rotation is (0,0,0) 
	// and all lines must start at (0,0,0).
	// alpha is the rotation angle in radians
	// fAlpha = alpha*pi/180;
	double cosa, sina, vera;
	double x, y, z;
	double rot[3][3];
	double oldxyz[2][3];
	double newxyz[2][3];
	double length1;
	double norm1;
	cosa = cos(fAlpha);
	sina = sin(fAlpha);
	vera = 1.0f - cosa;
	
	//The line is rotated about a rotation axis [x y z]=fpLA
	x = fpLA[0];
	y = fpLA[1];
	z = fpLA[2];
	rot[0][0] = cosa + x*x*vera;
	rot[1][0] = x*y*vera - z*sina;
	rot[2][0] = x*z*vera + y*sina;
	rot[0][1] = x*y*vera + z*sina;
	rot[1][1] = cosa + y*y*vera;
	rot[2][1] = y*z*vera - x*sina;
	rot[0][2] = x*z*vera - y*sina;
	rot[1][2] = y*z*vera + x*sina;
	rot[2][2] = cosa + z*z*vera;
	oldxyz[0][0] = fpL1[0];
	oldxyz[0][1] = fpL1[1];
	oldxyz[0][2] = fpL1[2];
	
	// Rotate fpL1 and fpL2 about fpLA by -fAlpha.
	// Matrix multiplication the tedious way
	newxyz[0][0] = oldxyz[0][0]*rot[0][0] + oldxyz[0][1]*rot[1][0] + oldxyz[0][2]*rot[2][0];
	newxyz[0][1] = oldxyz[0][0]*rot[0][1] + oldxyz[0][1]*rot[1][1] + oldxyz[0][2]*rot[2][1];
	newxyz[0][2] = oldxyz[0][0]*rot[0][2] + oldxyz[0][1]*rot[1][2] + oldxyz[0][2]*rot[2][2];
	
	// now we ensure that the new vectors have the same length as the originals
	// lengths 1 & 2 and norms 1 & 2 are my additions - L. Malloch
	length1 = sqrt(fpL1[0]*fpL1[0] + fpL1[1]*fpL1[1] + fpL1[2]*fpL1[2]);
	norm1 = sqrt(newxyz[0][0]*newxyz[0][0]+newxyz[0][1]*newxyz[0][1]+newxyz[0][2]*newxyz[0][2]);
	fpL1[0] = newxyz[0][0] * length1/norm1;
	fpL1[1] = newxyz[0][1] * length1/norm1;
	fpL1[2] = newxyz[0][2] * length1/norm1;
}

void OrientationMath::cross(double A[3], double B[3], double C[3]) {
	// A cross B = C
	C[0] = A[1]*B[2]-A[2]*B[1];
	C[1] = A[2]*B[0]-A[0]*B[2];
	C[2] = A[0]*B[1]-A[1]*B[0];
}


void OrientationMath::CopyMatrix(double source_mat[3][3], double dest_mat[3][3]) {
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			dest_mat[i][j]=source_mat[i][j];
		}
	}
}

void OrientationMath::NormVector(double *vec) {
	double dMag = sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
	vec[0]/=dMag;
	vec[1]/=dMag;
	vec[2]/=dMag;
}

void OrientationMath::NormVector(float *vec) {
	float fMag = sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
	vec[0]/=fMag;
	vec[1]/=fMag;
	vec[2]/=fMag;
}

void OrientationMath::GetPosFromOrientation(double pos[], double u[], double dLength) {
	//Function takes an initial base position in Cartesian space in the pos[] array, and calculates
	//what the end point position would be if it started at pos and was translated along the vector u 
	//by a length dLength.
	pos[0]+=u[0]*dLength;
	pos[1]+=u[1]*dLength;
	pos[2]+=u[2]*dLength;
}

void OrientationMath::GetPosFromOrientation(double pos[], double yaw, double pitch, double dLength) {
	//Function takes an initial base position in Cartesian space in the pos[] array, and calculates
	//what the end point position would be if it started at pos and was translated along the vector u 
	//by a length dLength.
	//get u vector from yaw and pitch orientation angles
	const double dDegtoRad = 0.017453292519943296;
	double u[3];
	double dYaw = yaw*dDegtoRad;
	double dPitch = pitch*dDegtoRad;
	u[0]=cos(dYaw)*cos(dPitch);
	u[1]=sin(dPitch);
	u[2]=sin(dYaw)*cos(dPitch);
	GetPosFromOrientation(pos,u,dLength);
}



void OrientationMath::unb2rpy(double *u, double *n, double *b, double *roll_angle
					   , double *pitch_angle, double *yaw_angle)
{
	//This function converts U,N, and B vectors into roll pitch 
	//and yaw euler angles.

	//The first two rotations (yaw and pitch) take U from [1 0 0] 
	//to its final orientation. The third rotation affects B and N only.
	//The yaw and pitch rotations are simply the azimuth and elevation
	//angles of U in spherical coordinates. For confirmation, try
	//quatdemo.m in Matlab, and observe the transformation matrices
	//in the reference, particularly the cos(yaw)*cos(pitch) term in A.
	//We have a 'y-up' coordinate system, so the base plane for the
	//spherical coordinates is yz, not xy. Some if statements are required
	//to create yaw and roll that vary from -180 to + 180, and to
	//account for signs in various quadrants.

	const double pole_alert = 0.000001; //used to detect potential pole conditions, i.e. 
										//if pitch is +90deg or -90deg. In these cases, just assume
										//that sum of roll and yaw is zero, since Euler angle math
										//breaks down - get 0 / 0 conditions
	double Ux = u[0];
	double Uy = u[1];
	double Uz = u[2];
	double Nx = n[0];
	double Ny = n[1];
	double Nz = n[2];
	double Bx = b[0];
	double By = b[1];
	double Bz = b[2];

	double yaw=0;
	double pitch=0;
	double roll=0;

	//pitch
	if (Uy>1) Uy = 1;
	else if (Uy<-1) Uy = -1;
	pitch = asin(Uy);
	//yaw
	double dTest = Ux / cos(pitch);
	if (dTest>1) dTest=1;
	else if (dTest<-1) dTest = -1;
	yaw = acos(dTest);
	//check sign
	dTest = sin(yaw)*cos(pitch)*Uz;
	if (dTest<0) yaw=-yaw; //check to see if sign of yaw needs to be reversed
	//roll
	dTest = Ny/cos(pitch);
	if (dTest>1) dTest=1;
	else if (dTest<-1) dTest=-1;
	roll=acos(dTest);

	//check sign (and check for pole condition)
	dTest=cos(pitch)*sin(roll)*By;
	double dTest2=cos(pitch);
	if (fabs(dTest2)<pole_alert) {
		//approximate total yaw/roll
		double dTotal_yawroll;
		if (Uy<0.0) {//near -90 deg pitch
			if (Nx>0.0) dTotal_yawroll=atan(Nz/Nx);
			else if (Nz<0) dTotal_yawroll=-3.14159/2-atan(Nx/Nz);
			else /*Nz>=0*/ dTotal_yawroll=3.14159/2-atan(Nx/Nz);
			roll=-dTotal_yawroll+yaw;
		}
		else {//near +90 deg pitch
			if (Nx<0.0) dTotal_yawroll=atan(Nz/Nx);
			else if (Nz<0) dTotal_yawroll=3.14159/2-atan(Nx/Nz);
			else /*Nz>=0*/ dTotal_yawroll=-3.14159/2-atan(Nx/Nz);
			roll=dTotal_yawroll - yaw;
		}
	}
	else if (dTest<0) roll=-roll;

	*roll_angle=roll;
	*pitch_angle=pitch;
	*yaw_angle=yaw;
}

void OrientationMath::CalculateNormalVector(double p1[3], double p2[3], double p3[3], double vec[3]) {
	//function calculates a unit normal vector given 3 points that form a plane
	//equation of plane is Ax + By + Cz + D = 0
	//first point on plane is at the index knuckle
	double x1 = p1[0];
	double y1 = p1[1];
	double z1 = p1[2];
	//2nd point on plane is at the wrist joint
	double x2 = p2[0];
	double y2 = p2[1];
	double z2 = p2[2];
	//3rd point on plane is at the pinky knuckle
	double x3 = p3[0];
	double y3 = p3[1];
	double z3 = p3[2];
		
	//now find A, B, C, D
	double A = y1*(z2-z3)+y2*(z3-z1)+y3*(z1-z2);
	double B = z1*(x2-x3)+z2*(x3-x1)+z3*(x1-x2);
	double C = x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2);
	double D = -(x1*(y2*z3-y3*z2)+x2*(y3*z1-y1*z3)+x3*(y1*z2-y2*z1));
	
	//A,B,C is the normal vector to the plane 
	vec[0]=A;
	vec[1]=B;
	vec[2]=C;
	OrientationMath::NormVector(vec);//normalize 
}

void OrientationMath::MatEquals(double sourcemat[][3], double destmat[][3]) {
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			destmat[i][j]=sourcemat[i][j];
		}
	}
}

void OrientationMath::TransformAngles(double input_angles[3], double trans_angles[3])
{
	//function transforms input_angles by the Euler trans_angles
	//computes new global roll, pitch, and yaw based on 
	//convert input angles to radians
	const double dDegtoRad = 0.017453292519943296;
	double roll=input_angles[0]*dDegtoRad;
	double pitch=input_angles[1]*dDegtoRad;
	double yaw=input_angles[2]*dDegtoRad;
	//compute u,n,b for input_angles
	double u[3],n[3],b[3];
	double cosyaw=cos(yaw);
	double sinyaw=sin(yaw);
	double cospitch=cos(pitch);
	double sinpitch=sin(pitch);
	double cosroll=cos(roll);
	double sinroll=sin(roll);
	u[0] = cosyaw*cospitch;
	u[1] = sinpitch;
	u[2] = sinyaw*cospitch;
	n[0] = -cosyaw*sinpitch*cosroll + sinyaw*sinroll;
	n[1] = cospitch*cosroll;
	n[2] = -sinyaw*sinpitch*cosroll - cosyaw*sinroll;
	b[0] = -cosyaw*sinpitch*sinroll - sinyaw*cosroll;
	b[1] = cospitch*sinroll;
	b[2] = -sinyaw*sinpitch*sinroll + cosyaw*cosroll;
	//compute angles transformed by base orientation angles
	double uu[3], nn[3], bb[3];
	cosroll=cos(trans_angles[0]*dDegtoRad);
	sinroll=sin(trans_angles[0]*dDegtoRad);
	cospitch=cos(trans_angles[1]*dDegtoRad);
	sinpitch=sin(trans_angles[1]*dDegtoRad);
	cosyaw=cos(trans_angles[2]*dDegtoRad);
	sinyaw=sin(trans_angles[2]*dDegtoRad);
	
	uu[0] = cosyaw*cospitch*u[0] + (sinyaw*sinroll-cosyaw*sinpitch*cosroll)*u[1]
		- (cosyaw*sinpitch*sinroll + sinyaw*cosroll)*u[2];
	uu[1] = sinpitch*u[0] + cospitch*cosroll*u[1] + cospitch*sinroll*u[2];
	uu[2] = sinyaw*cospitch*u[0] - (sinyaw*sinpitch*cosroll + cosyaw*sinroll)*u[1]
		+ (cosyaw*cosroll - sinyaw*sinpitch*sinroll)*u[2];
	nn[0] = cosyaw*cospitch*n[0] + (sinyaw*sinroll-cosyaw*sinpitch*cosroll)*n[1]
		- (cosyaw*sinpitch*sinroll + sinyaw*cosroll)*n[2];
	nn[1] = sinpitch*n[0] + cospitch*cosroll*n[1] + cospitch*sinroll*n[2];
	nn[2] = sinyaw*cospitch*n[0] - (sinyaw*sinpitch*cosroll + cosyaw*sinroll)*n[1]
		+ (cosyaw*cosroll - sinyaw*sinpitch*sinroll)*n[2];
	bb[0] = cosyaw*cospitch*b[0] + (sinyaw*sinroll-cosyaw*sinpitch*cosroll)*b[1]
		- (cosyaw*sinpitch*sinroll + sinyaw*cosroll)*b[2];
	bb[1] = sinpitch*b[0] + cospitch*cosroll*b[1] + cospitch*sinroll*b[2];
	bb[2] = sinyaw*cospitch*b[0] - (sinyaw*sinpitch*cosroll + cosyaw*sinroll)*b[1]
		+ (cosyaw*cosroll - sinyaw*sinpitch*sinroll)*b[2];

	//now compute transformed roll, pitch, yaw
	OrientationMath::unb2rpy(uu,nn,bb,&roll,&pitch,&yaw);
	input_angles[0]=roll/dDegtoRad;
	input_angles[1]=pitch/dDegtoRad;
	input_angles[2]=yaw/dDegtoRad;
}


void OrientationMath::TransformPoint(double point[3], double basepoint[3], double trans_angles[3]) {
	//check to see if tail tape, cube (3dm-g), or step tracking is being used
	const double dDegtoRad = 0.017453292519943296;
	double cos_roll = cos(trans_angles[0]*dDegtoRad);
	double sin_roll = sin(trans_angles[0]*dDegtoRad);
	double cos_pitch = cos(trans_angles[1]*dDegtoRad);
	double sin_pitch = sin(trans_angles[1]*dDegtoRad);
	double cos_yaw = cos(trans_angles[2]*dDegtoRad);
	double sin_yaw = sin(trans_angles[2]*dDegtoRad);

	double rotmatrix[3][3]; //first index is row, second is column
	rotmatrix[0][0] = cos_yaw*cos_pitch;
	rotmatrix[0][1] = (-cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll);
	rotmatrix[0][2] = (-cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll);
	rotmatrix[1][0] = sin_pitch;
	rotmatrix[1][1] = cos_pitch*cos_roll;
	rotmatrix[1][2] = cos_pitch*sin_roll;
	rotmatrix[2][0] = sin_yaw*cos_pitch;
	rotmatrix[2][1] = -sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll;
	rotmatrix[2][2] = -sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll;

	//perform rotation about basepoint
	point[0]-=basepoint[0];
	point[1]-=basepoint[1];
	point[2]-=basepoint[2];
	double temp[3];
	temp[0] = rotmatrix[0][0]*point[0] + rotmatrix[0][1]*point[1] + rotmatrix[0][2]*point[2];
	temp[1] = rotmatrix[1][0]*point[0] + rotmatrix[1][1]*point[1] + rotmatrix[1][2]*point[2];
	temp[2] = rotmatrix[2][0]*point[0] + rotmatrix[2][1]*point[1] + rotmatrix[2][2]*point[2];
	point[0]= temp[0] + basepoint[0];
	point[1]= temp[1] + basepoint[1];
	point[2]= temp[2] + basepoint[2];
}

void OrientationMath::TransformVector(double vec[3], double trans_angles[3]) {
	const double dDegtoRad = 0.017453292519943296;
	double cos_roll = cos(trans_angles[0]*dDegtoRad);
	double sin_roll = sin(trans_angles[0]*dDegtoRad);
	double cos_pitch = cos(trans_angles[1]*dDegtoRad);
	double sin_pitch = sin(trans_angles[1]*dDegtoRad);
	double cos_yaw = cos(trans_angles[2]*dDegtoRad);
	double sin_yaw = sin(trans_angles[2]*dDegtoRad);
		
	double rotmatrix[3][3]; //first index is row, second is column
	rotmatrix[0][0] = cos_yaw*cos_pitch;
	rotmatrix[0][1] = (-cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll);
	rotmatrix[0][2] = (-cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll);
	rotmatrix[1][0] = sin_pitch;
	rotmatrix[1][1] = cos_pitch*cos_roll;
	rotmatrix[1][2] = cos_pitch*sin_roll;
	rotmatrix[2][0] = sin_yaw*cos_pitch;
	rotmatrix[2][1] = -sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll;
	rotmatrix[2][2] = -sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll;

	//perform rotation
	double temp[3];
	temp[0] = rotmatrix[0][0]*vec[0] + rotmatrix[0][1]*vec[1] + rotmatrix[0][2]*vec[2];
	temp[1] = rotmatrix[1][0]*vec[0] + rotmatrix[1][1]*vec[1] + rotmatrix[1][2]*vec[2];
	temp[2] = rotmatrix[2][0]*vec[0] + rotmatrix[2][1]*vec[1] + rotmatrix[2][2]*vec[2];

	//save back in vec
	vec[0]=temp[0];
	vec[1]=temp[1];
	vec[2]=temp[2];
}

void OrientationMath::GetYawPitch(double vec[], double &yaw, double &pitch) {
	//function computes the yaw and pitch Euler angles for the vector vec
	OrientationMath::NormVector(vec);
	double Ux = vec[0]; 
	double Uy = vec[1]; 
	double Uz = vec[2];
	//pitch
	if (Uy>1) Uy = 1;
	else if (Uy<-1) Uy = -1;
	pitch = asin(Uy);
	//yaw
	double dTest = Ux / cos(pitch);
	if (dTest>1) dTest=1;
	else if (dTest<-1) dTest = -1;
	yaw = acos(dTest);
	//check sign
	dTest = sin(yaw)*cos(pitch)*Uz;
	if (dTest<0) yaw=-yaw; //check to see if sign of yaw needs to be reversed
}

void EulerToMatrix(float mat[3][3], float roll, float pitch, float yaw) {
	//takes roll, pitch, and yaw angles in degrees, and converts them to a 3X3 orientation matrix
	const float fDegtoRad = 0.017453293;
	double cosroll = cos(roll*fDegtoRad);
	double sinroll = sin(roll*fDegtoRad);
	double cospitch = cos(pitch*fDegtoRad);
	double sinpitch = sin(pitch*fDegtoRad);
	double cosyaw = cos(yaw*fDegtoRad);
	double sinyaw = sin(yaw*fDegtoRad);
	mat[0][0]=cosyaw*cospitch;
	mat[0][1]=-cosyaw*sinpitch*cosroll+sinyaw*sinroll;
	mat[0][2]=-cosyaw*sinpitch*sinroll-sinyaw*cosroll;
	mat[1][0]=sinpitch;
	mat[1][1]=cospitch*cosroll;
	mat[1][2]=cospitch*sinroll;
	mat[2][0]=sinyaw*cospitch;
	mat[2][1]=-sinyaw*sinpitch*cosroll-cosyaw*sinroll;
	mat[2][2]=-sinyaw*sinpitch*sinroll+cosyaw*cosroll;
}

void OrientationMath::EulerToMatrix(double mat[3][3], double roll, double pitch, double yaw) {
	//takes roll, pitch, and yaw angles in degrees, and converts them to a 3X3 orientation matrix
	const double dDegtoRad = 0.017453292519943296;
	double cosroll = cos(roll*dDegtoRad);
	double sinroll = sin(roll*dDegtoRad);
	double cospitch = cos(pitch*dDegtoRad);
	double sinpitch = sin(pitch*dDegtoRad);
	double cosyaw = cos(yaw*dDegtoRad);
	double sinyaw = sin(yaw*dDegtoRad);
	mat[0][0]=cosyaw*cospitch;
	mat[0][1]=-cosyaw*sinpitch*cosroll+sinyaw*sinroll;
	mat[0][2]=-cosyaw*sinpitch*sinroll-sinyaw*cosroll;
	mat[1][0]=sinpitch;
	mat[1][1]=cospitch*cosroll;
	mat[1][2]=cospitch*sinroll;
	mat[2][0]=sinyaw*cospitch;
	mat[2][1]=-sinyaw*sinpitch*cosroll-cosyaw*sinroll;
	mat[2][2]=-sinyaw*sinpitch*sinroll+cosyaw*cosroll;
}

void OrientationMath::EulerToMatrix(float mat[3][3], float roll, float pitch, float yaw) {
	//takes roll, pitch, and yaw angles in degrees, and converts them to a 3X3 orientation matrix
	const float fDegtoRad = 0.01745329;
	double cosroll = cos(roll*fDegtoRad);
	double sinroll = sin(roll*fDegtoRad);
	double cospitch = cos(pitch*fDegtoRad);
	double sinpitch = sin(pitch*fDegtoRad);
	double cosyaw = cos(yaw*fDegtoRad);
	double sinyaw = sin(yaw*fDegtoRad);
	mat[0][0]=cosyaw*cospitch;
	mat[0][1]=-cosyaw*sinpitch*cosroll+sinyaw*sinroll;
	mat[0][2]=-cosyaw*sinpitch*sinroll-sinyaw*cosroll;
	mat[1][0]=sinpitch;
	mat[1][1]=cospitch*cosroll;
	mat[1][2]=cospitch*sinroll;
	mat[2][0]=sinyaw*cospitch;
	mat[2][1]=-sinyaw*sinpitch*cosroll-cosyaw*sinroll;
	mat[2][2]=-sinyaw*sinpitch*sinroll+cosyaw*cosroll;
}

void OrientationMath::EulerToMatrix(double mat[3][3], double pitch, double yaw) {
	const double dDegtoRad = 0.017453292519943296;
	double cospitch = cos(pitch*dDegtoRad);
	double sinpitch = sin(pitch*dDegtoRad);
	double cosyaw = cos(yaw*dDegtoRad);
	double sinyaw = sin(yaw*dDegtoRad);
	mat[0][0]=cosyaw*cospitch;
	mat[0][1]=-cosyaw*sinpitch;
	mat[0][2]=-sinyaw;
	mat[1][0]=sinpitch;
	mat[1][1]=cospitch;
	mat[1][2]=0.0;
	mat[2][0]=sinyaw*cospitch;
	mat[2][1]=-sinyaw*sinpitch;
	mat[2][2]=cosyaw;
}

void OrientationMath::MatrixToEuler(double &roll, double &pitch, double &yaw, double mat[][3]) {
	//takes 3X3 orientation matrix and converts to roll, pitch, yaw orientation angles in degrees
	const double dRadtoDeg = 57.295779513082;
	double u[3], n[3], b[3];
	for (int i=0;i<3;i++) {
		u[i]=mat[i][0];
		n[i]=mat[i][1];
		b[i]=mat[i][2];
	}
	unb2rpy(u,n,b,&roll,&pitch,&yaw);
	roll*=dRadtoDeg;
	pitch*=dRadtoDeg;
	yaw*=dRadtoDeg;
}

void OrientationMath::GetLocalMatrix(double local_mat[][3], double parent_mat[][3], double child_mat[][3]) {
	//function finds the local 3x3 matrix given 2 global 3X3 matrices (parent and child)
	//get inverse of parent_mat
	double inv_parent[3][3];
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			inv_parent[i][j]=parent_mat[j][i];
		}
	}
	MatMult(inv_parent,child_mat,local_mat);
}

void OrientationMath::u2py(double u[], double &roll, double &pitch, double &yaw) {
	//calculates pitch and yaw (in that order, in radians) from u-vector
	//allows pitch to range from -180 deg to +180 deg and limits yaw to be from -90 to +90
	const double dRadtoDeg = 57.295779513082;
	NormVector(u);
	//get rid of rounding errors
	if (u[2]>1.0) u[2]=1.0;
	else if (u[2]<-1.0) u[2]=-1.0;
	yaw = asin(u[2]);
	double dTest = u[0]/cos(yaw);
	if (dTest>1.0) dTest=1.0;
	else if (dTest<-1.0) dTest=-1.0;
	pitch=acos(dTest);
	//check sign of pitch
	dTest=u[1]*sin(pitch)*cos(yaw);
	if (dTest<0) pitch=-pitch;

	//build orientation vectors
	double n[3], b[3];
	n[0]=-sin(pitch); b[0]=-cos(pitch)*sin(yaw);
	n[1]=cos(pitch); b[1]=-sin(pitch)*sin(yaw);
	n[2]=0.0; b[2]=cos(yaw);
	unb2rpy(u,n,b,&roll,&pitch,&yaw);

	roll*=dRadtoDeg;
	pitch*=dRadtoDeg;
	yaw*=dRadtoDeg;
}

double OrientationMath::distance(double pt1[], double pt2[]) {
	//computes distance between 2 points
	return sqrt(pow(pt1[0]-pt2[0],2)+pow(pt1[1]-pt2[1],2)+pow(pt1[2]-pt2[2],2));
}

float OrientationMath::distance(float pt1[], float pt2[]) {
	//computes distance between 2 points
	return sqrt(pow(pt1[0]-pt2[0],2)+pow(pt1[1]-pt2[1],2)+pow(pt1[2]-pt2[2],2));
}

void OrientationMath::RotAverage(double rotangles1[], double rotangles2[], double weight) {
	//averages the 2 sets of Euler angles rotangles1 and rotangles2
	//weight is a number between 0 and 1.0 that refers to how much importance to give to rotangles1
	//function uses Quaternion SLERP averaging to achieve the result
	//result is returned in rotangles1
	const double dRadtoDeg = 57.295779513082;
	quaternion2 quat1(rotangles1[0],rotangles1[1],rotangles1[2]);
	quaternion2 quat2(rotangles2[0],rotangles2[1],rotangles2[2]);
	quat1.slerp(quat2,weight);
	tmatrix rotmat = quat1.getRotMatrix(); //get rotation matrix
	rotmat.getRPY(rotangles1[0],rotangles1[1],rotangles1[2]);//get Euler angles from matrix
	//convert from radians to degrees
	rotangles1[0]*=dRadtoDeg;
	rotangles1[1]*=dRadtoDeg;
	rotangles1[2]*=dRadtoDeg;
}

void OrientationMath::SetUnity(double mat[][3]) {
	//sets input matrix to the unity matrix
	mat[0][0]=1.0; mat[0][1]=0.0; mat[0][2]=0.0;
	mat[1][0]=0.0; mat[1][1]=1.0; mat[1][2]=0.0;
	mat[2][0]=0.0; mat[2][1]=0.0; mat[2][2]=1.0;
}

void OrientationMath::MatrixToUNB(double u[], double n[], double b[], double mat[][3]) {
	u[0]=mat[0][0]; n[0]=mat[0][1]; b[0]=mat[0][2];
	u[1]=mat[1][0]; n[1]=mat[1][1]; b[1]=mat[1][2];
	u[2]=mat[2][0]; n[2]=mat[2][1]; b[2]=mat[2][2];
}

void OrientationMath::TransformUNBwithMat(double u[], double n[], double b[], double mat[][3]) {
	double unb_mat[3][3]; //orientation matrix formed from using u,n,b vectors
	double trans_mat[3][3]; //transformed matrix resulting from multiplying unb_mat by mat
	unb_mat[0][0]=u[0]; unb_mat[0][1]=n[0]; unb_mat[0][2]=b[0];
	unb_mat[1][0]=u[1]; unb_mat[1][1]=n[1]; unb_mat[1][2]=b[1];
	unb_mat[2][0]=u[2]; unb_mat[2][1]=n[2]; unb_mat[2][2]=b[2];
	MatMult(unb_mat,mat,trans_mat);
	//re-assign new values to u,n,b
	u[0]=trans_mat[0][0]; n[0]=trans_mat[0][1]; b[0]=trans_mat[0][2];
	u[1]=trans_mat[1][0]; n[1]=trans_mat[1][1]; b[1]=trans_mat[1][2];
	u[2]=trans_mat[2][0]; n[2]=trans_mat[2][1]; b[2]=trans_mat[2][2];
}


void OrientationMath::RotatePoint(double point[], double basepoint[], double mat[][3]) {
	double newpoint[3];
	//subtract basepoint
	point[0]-=basepoint[0];
	point[1]-=basepoint[1];
	point[2]-=basepoint[2];
	MatVectMult(mat,point,newpoint);
	//add basepoint back on
	point[0]=newpoint[0]+basepoint[0];
	point[1]=newpoint[1]+basepoint[1];
	point[2]=newpoint[2]+basepoint[2];
}

void OrientationMath::MidPoint(double midpoint[], double pt1[], double pt2[]) {
	//gets the midpoint of pt1 and pt2
	midpoint[0] = (pt1[0]+pt2[0])/2;
	midpoint[1] = (pt1[1]+pt2[1])/2;
	midpoint[2] = (pt1[2]+pt2[2])/2;
}

void OrientationMath::unb2Matrix(double mat[][3], double u[], double n[], double b[]) {
	mat[0][0]=u[0]; mat[0][1]=n[0]; mat[0][2]=b[0];
	mat[1][0]=u[1]; mat[1][1]=n[1]; mat[1][2]=b[1];
	mat[2][0]=u[2]; mat[2][1]=n[2]; mat[2][2]=b[2];
}

void OrientationMath::EulerToUNB(double dRoll, double dPitch, double dYaw, double u[], 
								 double n[], double b[]) {
	double mat[3][3];
	OrientationMath::EulerToMatrix(mat,dRoll,dPitch,dYaw);
	u[0]=mat[0][0]; n[0]=mat[0][1]; b[0]=mat[0][2];
	u[1]=mat[1][0]; n[1]=mat[1][1]; b[1]=mat[1][2];
	u[2]=mat[2][0]; n[2]=mat[2][1]; b[2]=mat[2][2];
}

void OrientationMath::RotateUNB(double u[], double n[], double b[], double rotmat[][3]) {
	//rotates u,n,b vectors by rotmat
	double mat[3][3];//fill mat with u,n,b vectors
	mat[0][0]=u[0]; mat[0][1]=n[0]; mat[0][2]=b[0];
	mat[1][0]=u[1]; mat[1][1]=n[1]; mat[1][2]=b[1];
	mat[2][0]=u[2]; mat[2][1]=n[2]; mat[2][2]=b[2];
	double result[3][3];
	OrientationMath::MatMult(mat,rotmat,result);
	u[0]=result[0][0]; n[0]=result[0][1]; b[0]=result[0][2];
	u[1]=result[1][0]; n[1]=result[1][1]; b[1]=result[1][2];
	u[2]=result[2][0]; n[2]=result[2][1]; b[2]=result[2][2];
}

void OrientationMath::Align(double &roll, double &pitch, double &yaw, double vec[]) {
	//align orientation specified by roll, pitch, and yaw with vec vector. Tries to 
	//preserve original roll angle as much as possible
	double current_mat[3][3];
	OrientationMath::NormVector(vec);//make sure vec is normalized
	OrientationMath::EulerToMatrix(current_mat,roll,pitch,yaw);
	double current_b[3];//current b vector
	current_b[0] = current_mat[0][2];
	current_b[1] = current_mat[1][2];
	current_b[2] = current_mat[2][2];
	double new_n[3];//new n vector (formed by taking cross-product of new_b and vec
	OrientationMath::cross(current_b,vec,new_n);
	OrientationMath::NormVector(new_n);//normalize
	//take another cross-product to get new b vector
	double new_b[3];
	OrientationMath::cross(vec,new_n,new_b);
	OrientationMath::NormVector(new_b);//normalize
	double new_mat[3][3];//store results in new matrix
	new_mat[0][0] = vec[0];
	new_mat[1][0] = vec[1];
	new_mat[2][0] = vec[2];
	new_mat[0][1] = new_n[0];
	new_mat[1][1] = new_n[1];
	new_mat[2][1] = new_n[2];
	new_mat[0][2] = new_b[0];
	new_mat[1][2] = new_b[1];
	new_mat[2][2] = new_b[2];
	OrientationMath::MatrixToEuler(roll,pitch,yaw,new_mat);
}

void OrientationMath::RZRXRYToMatrix(double rz, double rx, double ry, double mat[][3]) {
	const double dDegToRad = 0.0174532925199432958;
	const double dRadToDeg = 57.295779513082320877;
	double crx = cos(rx*dDegToRad);
	double srx = sin(rx*dDegToRad);
	double cry = cos(ry*dDegToRad);
	double sry = sin(ry*dDegToRad);
	double crz = cos(rz*dDegToRad);
	double srz = sin(rz*dDegToRad);
	mat[0][0]=crz*cry - srz*srx*sry;
	mat[0][1]=-srz*crx;
	mat[0][2]=crz*sry + srz*srx*cry;
	mat[1][0]=srz*cry+crz*srx*sry;
	mat[1][1]=crz*crx;
	mat[1][2]=srz*sry-crz*srx*cry;
	mat[2][0]=-crx*sry;
	mat[2][1]=srx;
	mat[2][2]=crx*cry;
}

void OrientationMath::GLMatTo3x3Matrix(double *glMat, double mat[][3]) {
	mat[0][0]=glMat[0]; mat[0][1]=glMat[4]; mat[0][2]=glMat[8];
	mat[1][0]=glMat[1]; mat[1][1]=glMat[5]; mat[1][2]=glMat[9];
	mat[2][0]=glMat[2]; mat[2][1]=glMat[6]; mat[2][2]=glMat[10];
}