#include "HpMotionMath.h"

//HuboVPBody.cpp
Vec3 vectorToVec3(const Eigen::Vector3d &v)
{
	return Vec3(v.data());
}
Eigen::Vector3d Vec3Tovector(const Vec3 &v)
{
	return Eigen::Vector3d(v[0], v[1], v[2]);
}
Eigen::Matrix3d vectorToSkewMat(const Eigen::Vector3d &v)
{
	Eigen::Matrix3d m;
	m.setZero();
	m(1, 0) = v(3);
	m(2, 0) = -v(2);
	m(2, 1) = v(1);
	m(0, 1) = -v(3);
	m(0, 2) = v(2);
	m(1, 2) = -v(1);

	return m;
}

//HuboMotionData.cpp
Eigen::Quaterniond quaterMinus(Eigen::Quaterniond &q2, Eigen::Quaterniond &q1)
{
	Eigen::Quaterniond q;
	q.w() = q2.w() - q1.w();
	q.x() = q2.x() - q1.x();
	q.y() = q2.y() - q1.y();
	q.z() = q2.z() - q1.z();

	return q;
}

Eigen::Quaterniond quaterRealMult(Eigen::Quaterniond &q, double a)
{
	Eigen::Quaterniond newq;
	newq.w() = q.w() *a;
	newq.x() = q.x() *a;
	newq.y() = q.y() *a;
	newq.z() = q.z() *a;
	return newq;
}

//HuboVpController.cpp
void matrixStackUptoDown(Eigen::MatrixXd &a, Eigen::MatrixXd &b)
{
	// a b | [a]
	//       [b]
	assert(a.cols() == b.cols());
	a.conservativeResize(a.rows()+b.rows(), a.cols());
	a.block(a.rows(), 0, b.rows(), b.cols()) = b;
}

//IKSolver.cpp
Eigen::Vector3d ln(Eigen::Quaterniond &q)
{
	double sc = q.vec().norm();
    double theta = atan2(sc, q.w());
	if(sc > DBL_EPSILON)
        sc = theta / sc;
    else  sc = 1.0 ;
	return sc*q.vec();
}

Eigen::Vector3d diffQuat(Eigen::Quaterniond &q1, Eigen::Quaterniond &q2)
{
	Eigen::Quaterniond q = q2.inverse() * q1;
	return ln(q);
}
