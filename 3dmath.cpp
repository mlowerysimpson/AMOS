// this file uses JavaDoc style comment blocks for automatic extraction of source code documentation.

/*!\file
 @short implementation for 3dmath classes
 @version 0.1
 \date 15/06/99
 @author Alessandro Falappa
*/

#include "3DMATH.H"
#include "OrientationMath.h"

//-----------------------------------------------------------------------------
// static members

int vector3d::counter=0;// global counter initialization
int tmatrix::counter=0;// global counter initialization
int quaternion2::counter=0;// global counter initialization

//-----------------------------------------------------------------------------
// useful constants (definition)

//!the vector representing the origin
const vector3d ORIGIN(0,0,0);
//!the unit vector associated to the x axys
const vector3d X_AXIS(1,0,0);
//!the unit vector associated to the y axys
const vector3d Y_AXIS(0,1,0);
//!the unit vector associated to the z axys
const vector3d Z_AXIS(0,0,1);
//!the greek pi constant
const double G_PI=3.14159265359;
//! greek pi / 2
const double G_HALF_PI= 1.570796326795;
//!2 * greek pi
const double G_DOUBLE_PI= 6.28318530718;

//-----------------------------------------------------------------------------
// vector friends and members

vector3d operator+(const vector3d& v1,const vector3d& v2)
{
	return vector3d(v1.vec[0]+v2.vec[0], v1.vec[1]+v2.vec[1], v1.vec[2]+v2.vec[2]);
}

vector3d operator-(const vector3d& v1,const vector3d& v2)
{
	return vector3d(v1.vec[0]-v2.vec[0], v1.vec[1]-v2.vec[1], v1.vec[2]-v2.vec[2]);
}

vector3d operator-(const vector3d& v1)
{
	return vector3d(-v1.vec[0],-v1.vec[1],-v1.vec[2]);
}

vector3d operator^(const vector3d& v1,const vector3d& v2)
{
	return vector3d( v1.vec[1]*v2.vec[2]-v1.vec[2]*v2.vec[1],
				  -v1.vec[0]*v2.vec[2]+v1.vec[2]*v2.vec[0],
				   v1.vec[0]*v2.vec[1]-v1.vec[1]*v2.vec[0]);
}

double operator*(const vector3d& v1,const vector3d& v2)
{
	return v1.vec[0]*v2.vec[0] + v1.vec[1]*v2.vec[1] + v1.vec[2]*v2.vec[2];
}

/*!
This function together with operator*(real,vector) implements the commutative product of a scalar by a vector
*/
vector3d operator*(const vector3d& v,const double& fact)
{
	return vector3d(fact*v.vec[0],fact*v.vec[1],fact*v.vec[2]);
}

/*!
This function together with operator*(vector,real) implements the commutative product of a scalar by a vector
*/
vector3d operator*(const double& fact,const vector3d& v)
{
	return vector3d(fact*v.vec[0],fact*v.vec[1],fact*v.vec[2]);
}

vector3d operator/(const vector3d& v,const double& fact)
{
	assert(fabs(fact)>=epsilon2 );
	return vector3d(v.vec[0]/fact,v.vec[1]/fact,v.vec[2]/fact);
}
	vector3d Bisect(vector3d v0,vector3d v1);//!< returns the unit vector which halves the arc between v0 and v1

vector3d Bisect(vector3d v0,vector3d v1)
{
	//add the vectors
	vector3d v=v0+v1;
	// normalize the sum vector or fill it with a standar vector
	double norm=v.length2();
	if(norm<1e-5) v=Z_AXIS;
	else v/=sqrt(norm);
	return v;
}

#ifdef __AFXWIN_H__ // see if we are using MFC
#ifdef _DEBUG
CDumpContext& operator<<(CDumpContext& cd,const vector3d& vect)
{
	cd<<"[ "<<vect.vec[0]<<" ; "<<vect.vec[1]<<" ; "<<vect.vec[2]<<" ]";
	return cd;
}
#endif
#endif

/*!
This function modifies the vector upon which has been called.
\return the length of the vector before normalization, this is useful to check if the normalization operation has been numerically precise.
*/
double vector3d::normalize()
{
	double len=length();
	assert(fabs(len)>=epsilon2);
	(*this)/=len;
	return len;
}

/*!
This function doesn't modifies the vector upon which has been called, it returns a new vector instead.
\return the normalized copy of the vector
*/
vector3d vector3d::normalized() const
{
	double len=length();
	assert(fabs(len)>=epsilon2);
	return vector3d(this->vec[0]/len,this->vec[1]/len,this->vec[2]/len);
}

void vector3d::epsilonCorrect(const vector3d& v)
{
	if(simpleabs(vec[0])<epsilon2 && simpleabs(vec[1])<epsilon2 && simpleabs(vec[2])<epsilon2)	*this=v;
}

double vector3d::dot(vector3d &other)
{
	// DotProduct = (x1*x2 + y1*y2 + z1*z2)
	return (this->x() * other.x() + this->y() * other.y() + this->z() * other.z());
}

vector3d vector3d::cross(vector3d &other)
{
	// CrossProduct=(v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0])
	double x, y, z;
	x = this->y()*other.z()-this->z()*other.y();
	y = this->z()*other.x()-this->x()*other.z();
	z = this->x()*other.y()-this->y()*other.x();
	return vector3d(x,y,z);
}

/*
operator vector3d::operator char*()
{
}
*/

//-----------------------------------------------------------------------------
// quaternion friends and members

quaternion2 operator+(const quaternion2& q1,const quaternion2& q2)
{
	return quaternion2(q1.s+q2.s, q1.v+q2.v);
}

quaternion2 operator-(const quaternion2& q1,const quaternion2& q2)
{
	return quaternion2(q1.s-q2.s, q1.v-q2.v);
}

quaternion2 operator-(const quaternion2& q1)
{
	return quaternion2( -(q1.s), -(q1.v) );
}

quaternion2 operator*(const quaternion2& q1,const quaternion2& q2)
{
	double ts=q1.s*q2.s-q1.v*q2.v;
	vector3d tv=q1.s*q2.v+q2.s*q1.v+(q1.v^q2.v);
	return quaternion2(ts,tv);
}

/*!
This function together with operator*(real,quaternion2) implements the commutative product of a scalar by a quaternion2
*/
quaternion2 operator*(const quaternion2& q,const double& fact)
{
	return quaternion2(fact*q.s,fact*q.v);
}

/*!
This function together with operator*(quaternion2,real) implements the commutative product of a scalar by a quaternion2
*/
quaternion2 operator*(const double& fact,const quaternion2& q)
{
	return quaternion2(fact*q.s,fact*q.v);
}

quaternion2 operator/(const quaternion2& q,const double& fact)
{
	return fact*q.inversed();
}

quaternion2 operator/(const quaternion2& q1,const quaternion2& q2)
{
	return q1*q2.inversed();
}

#ifdef __AFXWIN_H__ // see if we are using MFC
#ifdef _DEBUG
CDumpContext& operator<<(CDumpContext& cd,const quaternion2& q)
{
	cd<<"< "<<q.s<<" , "<<q.v<<" >";
	return cd;
}
#endif
#endif

quaternion2& quaternion2::operator*=(const quaternion2& other)
{
	double temp=s;
	s=s*other.s-v*other.v;
	v=temp*other.v+other.s*v+(v^other.v);
	return *this;
}

quaternion2& quaternion2::operator/=(const quaternion2& other)
{
	quaternion2 temp=other.inversed();
	double ts=s;
	s=ts*temp.s-v*temp.v;
	v=ts*temp.v+temp.s*v+(v^temp.v);
	return *this;
}
/*!
This function check if two quaternion2s are equal.
*/
int operator==(const quaternion2& q1,const quaternion2& q2)
{
	if(q1.s==q2.s && q1.v==q2.v) return 1;
	else return 0;
}

/*!
This function check if two quaternion2s are not equal.
*/
int operator!=(const quaternion2& q1,const quaternion2& q2)
{
	if(q1.s==q2.s && q1.v==q2.v) return 0;
	else return 1;
}

/*!
\b NOTE: the norm is comparable to the squared length of a vector not to the length
*/
double quaternion2::norm() const
{
	return (s*s+v.length2());
}

/*!
This function modifies the quaternion2 upon which has been called.
\return the length of the quaternion2 before normalization, this is useful to check if the normalization operation has been numerically precise.
*/
double quaternion2::normalize()
{
	double len=length();
	assert(fabs(len)>=epsilon2);
	s/=len;
	v/=len;
	return len;
}

/*!
This function doesn't modifies the quaternion2 upon which has been called, it returns a new quaternion2 instead.
\return the normalized copy of the quaternion2
*/
quaternion2 quaternion2::normalized() const
{
	double len=length();
	assert(fabs(len)>=epsilon2);
	return quaternion2(s/len,v/len);
}

/*!
This function modifies the quaternion2 upon which has been called.
\return the norm of the quaternion2 before the internal normalization, this is useful to check if the normalization operation has been numerically precise.
*/
double quaternion2::inverse()
{
	double n=norm();
	assert(fabs(n)>=epsilon2);
	s/=n;
	v/=-n;
	return n;
}

/*!
This function doesn't modifies the quaternion2 upon which has been called, it returns a new quaternion2 instead.
*/
quaternion2 quaternion2::inversed() const
{
	double n=norm();
	assert(fabs(n)>=epsilon2);
	return quaternion2( s/n, v/(-n) );
}
/*!
Generates a rotation matrix even from non unit quaternion2s (only for unit
quaternoin the result is the same as unitquaternion2::getRotMatrix).
The generated rotation matrix is OpenGL compatible, it is intended to be
post-multiplied to the current trasformation matrix to achieve the rotation.
\return the rotation matrix
*/
tmatrix quaternion2::getRotMatrix()
{
	tmatrix result;
	double n=norm();
	double s=n>0?2.0/n:0.0;
	double xs=x()*s;	double ys=y()*s;	double zs=z()*s;
	double wx=w()*xs;	double wy=w()*ys;	double wz=w()*zs;
	double xx=x()*xs;	double xy=x()*ys;	double xz=x()*zs;	
	double yy=y()*ys;	double yz=y()*zs;	double zz=z()*zs;	
	result(0,0) = 1.0 - (yy + zz);
	result(1,0) = xy - wz;
	result(2,0) = xz + wy;
	result(3,0) = 0.0;

	result(0,1) = xy + wz;
	result(1,1) = 1.0 - (xx+ zz);
	result(2,1) = yz - wx;
	result(3,1) = 0.0;

	result(0,2) = xz - wy;
	result(1,2) = yz + wx;
	result(2,2) = 1.0 - (xx + yy);
	result(3,2) = 0.0;

	result(0,3) = 0.0;
	result(1,3) = 0.0;
	result(2,3) = 0.0;
	result(3,3) = 1.0;
	return result;
}

void tmatrix::inverse()
{
	tmatrix *inverse;
	double sign;
	double determinate;
	int i, j, k, l, m, a[3], b[3];

	inverse = new tmatrix(*this);

	determinate = inverse->Determinate();
	sign = 1.0;
	for (i=0;i<4;i++)
	{
		sign *= -1.0;
		for (j=0;j<4;j++)
		{
			sign *= -1.0;
			l = 0;
			m = 0;
			for (k=0;k<4;k++)
			{
				if (k!=i)
				{
					a[l]=k;
					l++;
				}
				if (k!=j)
				{
					b[m]=k;
					m++;
				}
			}
			mat[j][i] = (double)(sign * inverse->covalue(a[0], a[1], a[2], b[0], b[1], b[2]) / determinate);
		}
	}

	delete inverse;
}

double tmatrix::covalue(int a1, int a2, int a3, int b1, int b2, int b3)
{
	double cv;

	cv = mat[a1][b1]*mat[a2][b2]*mat[a3][b3];
	cv += mat[a2][b1]*mat[a3][b2]*mat[a1][b3];
	cv += mat[a3][b1]*mat[a1][b2]*mat[a2][b3];
	cv -= mat[a1][b3]*mat[a2][b2]*mat[a3][b1];
	cv -= mat[a2][b3]*mat[a3][b2]*mat[a1][b1];
	cv -= mat[a3][b3]*mat[a1][b2]*mat[a2][b1];

	return cv;
}

double tmatrix::Determinate()
{
	int i, j, k;
	double sign;
	double result;

	result = 0.0;
	sign = 1.0;
	
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			if (i != j)
			{
				sign *= -1.0;
				
				for (k = 0; k < 4; k++)
				{
					if (k != i && k != j)
					{
						sign *= -1;
						result += sign*(mat[i][0] * mat[j][1] * mat[k][2] * mat[6-i-j-k][3]);
					}
				}
			}
		}
	}

	return result;
}

//-----------------------------------------------------------------------------
// unitquaternion


/*!
The generated rotation matrix is OpenGL compatible, it is intended to be
post-multiplied to the current trasformation matrix to achieve the rotation.
\return the rotation matrix
*/
tmatrix unitquaternion::getRotMatrix()
{
	tmatrix result;
	register double t1,t2,t3;
/*	the code below performs the following calculations but has been
	reorganized to exploit three temporaries variables efficiently
	
	result(0,0) = 1.0 - 2.0*(y()*y() + z()*z());
	result(1,0) = 2.0*(x()*y() - z()*s);
	result(2,0) = 2.0*(z()*x() + y()*s);
	result(3,0) = 0.0;

	result(0,1) = 2.0*(x()*y() + z()*s);
	result(1,1) = 1.0 - 2.0*(z()*z()+ x()*x());
	result(2,1) = 2.0*(y()*z() - x()*s);
	result(3,1) = 0.0;

	result(0,2) = 2.0*(z()*x() - y()*s);
	result(1,2) = 2.0*(y()*z() + x()*s);
	result(2,2) = 1.0 - 2.0*(y()*y() + x()*x());
	result(3,2) = 0.0;

	result(0,3) = 0.0;
	result(1,3) = 0.0;
	result(2,3) = 0.0;
	result(3,3) = 1.0;
*/
	t1=2.0*x()*x();
	t2=2.0*y()*y();
	t3=2.0*z()*z();
	result(0,0) = 1.0 - t2 - t3;
	result(1,1) = 1.0 - t3 - t1;
	result(2,2) = 1.0 - t2 - t1;

	t1=2.0*x()*y();
	t2=2.0*z()*s;
	result(1,0) = t1 - t2;
	result(0,1) = t1 + t2;

	t1=2.0*z()*x();
	t2=2.0*y()*s;
	result(2,0) = t1 + t2;
	result(0,2) = t1 - t2;

	t1=2.0*y()*z();
	t2=2.0*x()*s;
	result(2,1) = t1 - t2;
	result(1,2) = t1 + t2;

	result(3,0) = 	result(3,1) = 	result(3,2) = 0.0;
	result(0,3) = 	result(1,3) = 	result(2,3) = 0.0;
	result(3,3) = 1.0;
	return result;
}


/*!
the rotation represented by the unitquaternion would transform the first vector
into the second. The vectors are of unit length (so they are placed on a unit sphere)
\param vfrom the first vector
\param vto the second vector
*/
void unitquaternion::getVectorsOnSphere(vector3d& vfrom,vector3d& vto)
{
	unitquaternion tmp=(*this)*(*this);
	double s=sqrt(tmp.x()*tmp.x()+tmp.y()*tmp.y());
	if(s<=epsilon2) vfrom=Y_AXIS;
	else vfrom=vector3d(-tmp.y()/s,tmp.x()/s,0.0);
	vto.x()=tmp.w()*vfrom.x()-tmp.z()*vfrom.y();
	vto.y()=tmp.w()*vfrom.y()+tmp.z()*vfrom.x();
	vto.z()=tmp.x()*vfrom.y()-tmp.y()*vfrom.x();
	if(w()<0.0) vfrom=-vfrom;
}

unitquaternion& unitquaternion::operator*=(const unitquaternion& other)
{
	double temp=s;
	s=s*other.s-v*other.v;
	v=temp*other.v+other.s*v+(v^other.v);
	return *this;
}

/*!
This function has been defined to trap the use of an operation which is not allowed
*/
unitquaternion operator+(const unitquaternion& q1,const unitquaternion& q2)
{
	// THIS OPERATION IS NOT ALLOWED CAUSE DOESN'T MANTAIN THE UNIT LENGTH
	assert(false);
	return q1+q2;// this return will never be executed
}

/*!
This function has been defined to trap the use of an operation which is not allowed
*/
unitquaternion operator-(const unitquaternion& q1,const unitquaternion& q2)
{
	// THIS OPERATION IS NOT ALLOWED CAUSE DOESN'T MANTAIN THE UNIT LENGTH
	assert(false);
	return q1-q2;// this return will never be executed
}


/*!
This function has been defined to trap the use of an operation which is not allowed
*/
unitquaternion operator*(const unitquaternion& q,const double& s)
{
	// THIS OPERATION IS NOT ALLOWED CAUSE DOESN'T MANTAIN THE UNIT LENGTH
	assert(false);
	return q*s;// this return will never be executed
}


/*!
This function has been defined to trap the use of an operation which is not allowed
*/
unitquaternion operator*(const double& s,const unitquaternion& q)
{
	// THIS OPERATION IS NOT ALLOWED CAUSE DOESN'T MANTAIN THE UNIT LENGTH
	assert(false);
	return q*s;// this return will never be executed
}


/*!
This function has been defined to trap the use of an operation which is not allowed
*/
unitquaternion operator/(const unitquaternion& q,const double& s)
{
	// THIS OPERATION IS NOT ALLOWED CAUSE DOESN'T MANTAIN THE UNIT LENGTH
	assert(false);
	return q;// this return will never be executed
}


//-----------------------------------------------------------------------------
// tmatrix friends and members

#ifdef _AFXDLL // see if we are using MFC
#ifdef _DEBUG
CDumpContext& operator<<(CDumpContext& cd,const tmatrix& m)
{
	cd<<"[ "<<m.mat[0][0]<<" "<<m.mat[1][0]<<" "<<m.mat[2][0]<<" "<<m.mat[3][0]<<" ]\n";
	cd<<"[ "<<m.mat[0][1]<<" "<<m.mat[1][1]<<" "<<m.mat[2][1]<<" "<<m.mat[3][1]<<" ]\n";
	cd<<"[ "<<m.mat[0][2]<<" "<<m.mat[1][2]<<" "<<m.mat[2][2]<<" "<<m.mat[3][2]<<" ]\n";
	cd<<"[ "<<m.mat[0][3]<<" "<<m.mat[1][3]<<" "<<m.mat[2][3]<<" "<<m.mat[3][3]<<" ]\n";
	return cd;
}
#endif
#endif

void tmatrix::loadIdentity()
{
	// set to zero all the elements except the diagonal
	for (int i=0;i<4;i++)
		for(int j=i+1;j<4;j++)
			mat[i][j]=mat[j][i]=0.0;
	// set to 1 the diagonal
	mat[0][0]=mat[1][1]=mat[2][2]=mat[3][3]=1.0;
}

tmatrix operator+(const tmatrix& t1,const tmatrix& t2)
{
	tmatrix temp;
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)temp.mat;
	double* rp1=(double*)t1.mat;
	double* rp2=(double*)t2.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=rp1[pos]+rp2[pos];
	return temp;
}

tmatrix operator-(const tmatrix& t1,const tmatrix& t2)
{
	tmatrix temp;
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)temp.mat;
	double* rp1=(double*)t1.mat;
	double* rp2=(double*)t2.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=rp1[pos]-rp2[pos];
	return temp;
}

tmatrix operator*(const tmatrix& t1,const tmatrix& t2)
{
	tmatrix temp;
	for(int c=0;c<4;c++)
		for(int r=0;r<4;r++)
		{
			temp.mat[r][c]=t1.mat[0][c]*t2.mat[r][0];
			for(int p=1;p<4;p++) temp.mat[r][c]+=t1.mat[p][c]*t2.mat[r][p];
		};
	return temp;
}

tmatrix operator*(const tmatrix& tmat,const double& fact)
{
	tmatrix temp;
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)temp.mat;
	double* rp1=(double*)tmat.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=rp1[pos]*fact;
	return temp;
}

tmatrix operator*(const double& fact,const tmatrix& tmat)
{
	tmatrix temp;
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)temp.mat;
	double* rp1=(double*)tmat.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=rp1[pos]*fact;
	return temp;
}

tmatrix operator/(const tmatrix& tmat,const double& fact)
{
	assert(fact>=epsilon2);
	tmatrix temp;
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)temp.mat;
	double* rp1=(double*)tmat.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=rp1[pos]/fact;
	return temp;
}

/*!
\param bInit if set to true initializes the matrix to the identity matrix
*/
tmatrix::tmatrix()
{
	counter++;
}

tmatrix::tmatrix(const double v[16],ordermode ord)
{
	if (ord==ROW)
		for (int r=0;r<4;r++)
			for (int c=0;c<4;c++)
				mat[r][c]=v[r+4*c];
// which is faster?
/*	else
		for (int r=0;r<4;r++)
			for (int c=0;c<4;c++)
				mat[r][c]=v[c+4*r];
*/
	else
	{
		double* rp=(double*)mat;
		for (int pos=0;pos<16;pos++)
			rp[pos]=v[pos];
		
	}
	counter++;
}

tmatrix::tmatrix(const double vector[16])
{
	for (int column = 0;column < 4;column++)
	{
		for (int row = 0;row < 4;row++)
		{
			mat[column][row] = vector[row + 4*column];
		}
	}
	counter++;
}

tmatrix::tmatrix(const double& val)
{
	// "quick & dirty" approach accessing the internal matrix as a vector
	double* rp=(double*)mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=val;
	counter++;
}

tmatrix& tmatrix::operator=(const tmatrix& other)
{
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)mat;
	double* rp2=(double*)other.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=rp2[pos];
	return *this;
}

tmatrix& tmatrix::operator+=(const tmatrix& other)
{
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)mat;
	double* rp2=(double*)other.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]+=rp2[pos];
	return *this;
}

tmatrix& tmatrix::operator-=(const tmatrix& other)
{
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)mat;
	double* rp2=(double*)other.mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]-=rp2[pos];
	return *this;
}

tmatrix& tmatrix::operator*=(const tmatrix& other)
{
	tmatrix temp=*this;
	for(int c=0;c<4;c++)
		for(int r=0;r<4;r++)
		{
			this->mat[r][c]=temp.mat[0][c]*other.mat[r][0];
			for(int p=1;p<4;p++) this->mat[r][c]+=temp.mat[p][c]*other.mat[r][p];
		};
	return *this;
}

double* tmatrix::times(const double vector[4])
{
	double *result = new double[4];
	result[0]=0;
	result[1]=0;
	result[2]=0;
	result[3]=0;
	for (int column=0;column<4;column++)
	{
		for (int row=0;row<4;row++)
		{
			result[column] += (this->mat[row][column] * vector[row]);
		}
	}

	return result;
}

tmatrix& tmatrix::operator*=(const double& fact)
{
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]*=fact;
	return *this;
}

tmatrix& tmatrix::operator/=(const double& fact)
{
	assert(fact>=epsilon2);
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]/=fact;
	return *this;
}

tmatrix& tmatrix::operator-()
{
	// "quick & dirty" approach accessing the internal matrices as vectors
	double* rp=(double*)mat;
	for (int pos=0;pos<16;pos++)
		rp[pos]=-rp[pos];
	return *this;
}

#define RADIANS 57.295827 

void tmatrix::setAngles(float roll, float pitch, float yaw)
{
	float A,B,C,D,E,F,AD,BD;
	A       = cos(roll);
    B       = sin(roll);
    C       = cos(yaw);
    D       = sin(yaw);
    E       = cos(pitch);
    F       = sin(pitch);

    AD      =   A * D;
    BD      =   B * D;

    mat[0][0]  =   C * E;
    mat[0][1]  =  -C * F;
    mat[0][2]  =  -D;
    mat[1][0]  = -BD * E + A * F;
    mat[1][1]  =  BD * F + A * E;
    mat[1][2]  =  -B * C;
    mat[2][0]  =  AD * E + B * F;
    mat[2][1]  = -AD * F + B * E;
    mat[2][2] =   A * C;

    mat[0][3]  =  mat[1][3] = mat[2][3] = mat[3][0] = mat[3][1] = mat[3][2] = 0;
    mat[3][3] =  1;
// code from http://www.flipcode.com/documents/matrfaq.html#Q37
}



//-----------------------------------------------------------------------------
// global functions



void tmatrix::getRPY(double &roll, double &pitch, double &yaw)
{
	//assumes that yaw is performed first, followed by pitch, and then roll.
	//pitch is CW pos, while yaw and roll are CW positive
	//yaw is about y-axis, pitch is about z-axis, roll is about x-axis.
	
	double dMat[3][3];
	dMat[0][0]=mat[0][0]; dMat[0][1]=mat[0][1]; dMat[0][2]=mat[0][2];
	dMat[1][0]=mat[1][0]; dMat[1][1]=mat[1][1]; dMat[1][2]=mat[1][2];
	dMat[2][0]=mat[2][0]; dMat[2][1]=mat[2][1]; dMat[2][2]=mat[2][2];
	
	OrientationMath::MatrixToEuler(roll,pitch,yaw,dMat);
}



/**
 * @brief gets the roll, pitch, and yaw in degrees, as used for AMOS
 * 
 * @param roll the returned roll angle of the orientation in degrees.
 * @param pitch the returned pitch angle of the orientation in degrees
 * @param yaw the returned yaw angle of the orientation in degrees
 */
void tmatrix::getAMOSRPY(double &roll, double &pitch, double &yaw) {
	double dMat[3][3];
	dMat[0][0]=mat[0][0]; dMat[0][1]=mat[0][1]; dMat[0][2]=mat[0][2];
	dMat[1][0]=mat[1][0]; dMat[1][1]=mat[1][1]; dMat[1][2]=mat[1][2];
	dMat[2][0]=mat[2][0]; dMat[2][1]=mat[2][1]; dMat[2][2]=mat[2][2];
	OrientationMath::MatrixToEuler(roll,pitch,yaw,dMat);
	pitch=-pitch;
	//check limits of yaw (should be 0 to 360) and roll (should be -180 to 180)
	if (yaw<0) {
		yaw+=360;
	}
	else if (yaw>360) {
		yaw-=360;
	}
	if (roll<-180) {
		roll+=360;
	}
	else if (roll>180) {
		roll-=360;
	}
}

double  * tmatrix::getMat()
{
	return (double*)mat;
}

/**
 * @brief performs spherical linear interpolation between this quaternion and another quaternion
 * 
 * @param other the other quaternion to spherically linearly interpolate with this quaternion
 * @param t the scaling factor to use, should be between 0 and 1. As t-->0 mostly this quaternion gets used in the interpolated result. As t-->1 most of the other quaternion gets used in the interpolated result.
 */
void quaternion2::slerp(const quaternion2 &other, double t) {	
	//returns interpolated result in this quaternion
	quaternion2 qcopy(other);
	double dDotproduct = other.s*s + other.v.x()*v.x() + other.v.y()*v.y() + other.v.z()*v.z();
	if (dDotproduct<0) {
		qcopy*=-1.0;
		dDotproduct*=-1.0;
	}
	if (dDotproduct>0.99999) return; //practically the same
	double dAngle = acos(dDotproduct);
	double dFactor1 = sin((1-t)*dAngle)/sin(dAngle);
	double dFactor2 = sin(t*dAngle)/sin(dAngle);
	double rFactor1 = (double)dFactor1;
	double rFactor2 = (double)dFactor2;
	qcopy*=rFactor2;
	*this*=rFactor1;
	*this+=qcopy;
}

void tmatrix::getGLMat(double glMat[][4]) {
	for (int i=0;i<4;i++) {
		for (int j=0;j<4;j++) {
			glMat[i][j]=mat[j][i];
		}
	}
}

void tmatrix::get3x3Mat(double m[][3]) {
	m[0][0]=mat[0][0]; m[0][1]=mat[0][1]; m[0][2]=mat[0][2];
	m[1][0]=mat[1][0]; m[1][1]=mat[1][1]; m[1][2]=mat[1][2];
	m[2][0]=mat[2][0]; m[2][1]=mat[2][1]; m[2][2]=mat[2][2];
}
