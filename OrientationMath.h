// OrientationMath.h: interface for the OrientationMath class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ORIENTATIONMATH_H__94026277_82D1_4656_A065_3FB5C469BAD0__INCLUDED_)
#define AFX_ORIENTATIONMATH_H__94026277_82D1_4656_A065_3FB5C469BAD0__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class OrientationMath  
{
public:
	static void GLMatTo3x3Matrix(double *glMat, double mat[][3]);
	static void RZRXRYToMatrix(double rz, double rx, double ry, double mat[][3]);
	static void Align(double &roll, double &pitch, double &yaw, double vec[]);
	static void RotateUNB(double u[], double n[], double b[], double rotmat[3][3]);
	static void EulerToUNB(double dRoll, double dPitch, double dYaw, double u[], double n[], double b[]);
	static void MidPoint(double midpoint[], double pt1[], double pt2[]);
	static void RotatePoint(double point[], double basepoint[], double mat[][3]);
	static void MatrixToUNB(double u[3], double n[3], double b[3], double mat[3][3]);
	static void SetUnity(double mat[][3]);
	static double distance(double pt1[3], double pt2[3]);
	static float distance(float pt1[3], float pt2[3]);
	static void u2py(double u[3], double &roll, double &pitch, double &yaw);
	static void GetLocalMatrix(double local_mat[3][3], double parent_mat[3][3], double child_mat[3][3]);
	static void GetYawPitch(double vec[3], double &yaw, double &pitch);
	static void MatEquals(double sourcemat[3][3], double destmat[3][3]);
	static void CalculateNormalVector(double p1[3], double p2[3], double p3[3], double vec[3]);
	static void unb2rpy(double *u, double *n, double *b, double *roll_angle, double *pitch_angle, double *yaw_angle);
	static void unb2Matrix(double mat[][3], double u[], double n[], double b[]);
	static void GetPosFromOrientation(double pos[], double yaw, double pitch, double dLength);
	static void GetPosFromOrientation(double pos[], double u[], double dLength);
	static void NormVector(double *vec);
	static void NormVector(float *vec);
	static void CopyMatrix(double source_mat[3][3], double dest_mat[3][3]);
	static void cross(double A[3], double B[3], double C[3]);
	static void rotline(double* fpL1, double* fpLA, double fAlpha);
	static void MatMult(double mat1[3][3], double mat2[3][3], double result[3][3]);
	static void MatMult(float mat1[3][3], float mat2[3][3], float result[3][3]);
	static void MatVectMult(double mat[][3], double vect[], double result[]);
	static void MatVectMult(float mat[][3], float vect[], float result[]);
	static void TransformAngles(double input_angles[3], double trans_angles[3]);
	static void TransformPoint(double point[3], double basepoint[3], double trans_angles[3]);
	static void TransformVector(double vec[3], double trans_angles[3]);
	static void TransformUNBwithMat(double u[], double n[], double b[], double mat[][3]);
	static void EulerToMatrix(double mat[3][3], double roll, double pitch, double yaw);
	static void EulerToMatrix(float mat[3][3], float roll, float pitch, float yaw);
	static void EulerToMatrix(double mat[3][3], double pitch, double yaw);
	static void MatrixToEuler(double &roll, double &pitch, double &yaw, double mat[][3]);
	static void MatrixInverse(double newmat[3][3], double origmat[3][3]);//finds inverse of 3X3 orientation matrix
	static void RotAverage(double rotangles1[3], double rotangles2[3], double weight);
	OrientationMath();
	virtual ~OrientationMath();

};

#endif // !defined(AFX_ORIENTATIONMATH_H__94026277_82D1_4656_A065_3FB5C469BAD0__INCLUDED_)
