#ifndef _MOTION_H_
#define _MOTION_H_

#pragma once

#include <Eigen/Dense>
//#define M_PI 3.141592653589793238462643383279
//#include "MATHCLASS\mathclass.h"

using std::string;
using std::sin;
using std::cos;
using std::asin;
using std::acos;
using std::atan2;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::aligned_allocator;


class Motion{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Motion(): translation(Vector3d(0,0,0)), rotation(Quaterniond(1,0,0,0)){};

	Motion(Vector3d &_v, Quaterniond &_q): translation(_v), rotation(_q){};

public:
	Vector3d translation;
	Quaterniond rotation;
	
public:
	void setRotation(Quaterniond &_rotation);
	void setTranslation(Vector3d &_translation);
	Quaterniond getRotation();
	Vector3d getTranslation();
	//double getAngle();//deprecated
	void reset();
};

#endif
