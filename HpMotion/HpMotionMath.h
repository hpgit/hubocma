//#define _HPMOTIONMATH_H_
#ifndef _HPMOTIONMATH_H_
#define _HPMOTIONMATH_H_

#include <Eigen/Dense>
#include <VP/vphysics.h>


//HuboVPBody.cpp
Vec3 vectorToVec3(const Eigen::Vector3d &v);
Eigen::Vector3d Vec3Tovector(const Vec3 &v);
Eigen::Matrix3d vectorToSkewMat(const Eigen::Vector3d &v);

//HuboMotionData.cpp
Eigen::Quaterniond quaterMinus(Eigen::Quaterniond &q2, Eigen::Quaterniond &q1);
Eigen::Quaterniond quaterRealMult(Eigen::Quaterniond &q, double a);

//HuboVpController.cpp
void matrixStackUptoDown(Eigen::MatrixXd &a, Eigen::MatrixXd &b);

//IKSolver.cpp
Eigen::Vector3d ln(Eigen::Quaterniond &q);
Eigen::Vector3d diffQuat(Eigen::Quaterniond &q1, Eigen::Quaterniond &q2);



#endif
