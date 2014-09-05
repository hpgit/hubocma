#ifndef _UNIFORMBSPLINE_H_
#define _UNIFORMBSPLINE_H_

#include "Eigen/Dense"
#include <map>

class UniformBspline
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//std::map<int, Eigen::VectorXd, aligned_allocator<Eigen::VectorXd>> ctrlPoint;
	std::map<int, Eigen::VectorXd> ctrlPoint;

	void setControlPoint(int idx, Eigen::VectorXd &controlPoint)
	{
		ctrlPoint[idx] = controlPoint;
	}
	
	Eigen::VectorXd getValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;
		if (idx == ctrlPoint.size()-3)
		{
			idx--;
			t = 1;
		}
		double invt = 1-t;
		double sqt = t*t;
		double sqinvt = invt*invt;

		return (
			sqinvt*invt*ctrlPoint[(idx-1)]
			+ (3*(sqinvt*t + invt)+1)*ctrlPoint[(idx)]
			+ (3*(sqt*invt + t)+1)*ctrlPoint[(idx+1)]
			+ sqt*t*ctrlPoint[(idx+2)]
			)/6.0;
				
	}

	Eigen::VectorXd getDifferentialValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;
		if (idx == ctrlPoint.size()-3)
		{
			idx--;
			t = 1;
		}
		double invt = 1 - t;
		double sqt = t*t;
		double sqinvt = invt*invt;

		return (
			-sqinvt * ctrlPoint[(idx-1)]
			+ (3 * sqt - 4 * t) * ctrlPoint[(idx)]
			+ (-3 * sqt + 2 * t + 1) * ctrlPoint[(idx+1)]
			+ sqt * ctrlPoint[(idx+2)]
			) / 2.0;
	}

	Eigen::VectorXd getSecondDerivationValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;
		if (idx == ctrlPoint.size()-3)
		{
			idx--;
			t = 1;
		}

		return (
			(1-t) * ctrlPoint[(idx-1)]
			+ (3 * t - 2 ) * ctrlPoint[(idx)]
			+ (-3 * t + 1 ) * ctrlPoint[(idx+1)]
			+ t * ctrlPoint[(idx+2)]
			) ;
	}

	void clear()
	{
		ctrlPoint.clear();
	}
};

class UniformQuaternionBspline
{
private:
	Quaterniond affineQuater(double a, double b, double c, double d, int idx)
	{

		return ctrlPoint[idx - 1].slerp(b / (a + b), ctrlPoint[idx])
			.slerp(
			(a + b) / (a+b+c+d),
			ctrlPoint[idx + 1].slerp(d / (c + d), ctrlPoint[idx+2])
			);
	}
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//std::map<int, Eigen::Quaterniond, aligned_allocator<Eigen::Quaterniond>> ctrlPoint;
	std::map<int, Eigen::Quaterniond> ctrlPoint;

	void setControlPoint(int idx, Eigen::Quaterniond &controlPoint)
	{
		ctrlPoint[idx] = controlPoint;
	}
	
	Eigen::Quaterniond getValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;
		double invt = 1-t;
		double sqt = t*t;
		double sqinvt = invt*invt;

		return affineQuater(
			sqinvt*invt/6.0,
			(3*(sqinvt*t + invt)+1)/6.0,
			(3*(sqt*invt + t)+1)/6.0,
			sqt*t/6.0,
			idx
			);
				
	}

	Eigen::Quaterniond getDifferentialValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;
		double invt = 1 - t;
		double sqt = t*t;
		double sqinvt = invt*invt;

		return affineQuater(
			-sqinvt/2.0,
			 (3 * sqt - 4 * t) /2.0,
			 (-3 * sqt + 2 * t + 1) /2.0,
			 sqt /2.0,
			 idx
			) ;
	}

	Eigen::Quaterniond getSecondDerivationValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;

		return affineQuater(
			(1-t), 
			(3 * t - 2 ) ,
			(-3 * t + 2 ),
			t ,
			idx
			) ;
	}

	void clear()
	{
		ctrlPoint.clear();
	}
};

class BezierSpline
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::map<int, Eigen::VectorXd> ctrlPoint;

	void setControlPoint(int idx, Eigen::VectorXd &controlPoint)
	{
		ctrlPoint[idx] = controlPoint;
	}
	
	Eigen::VectorXd getValue(double tt)
	{
		int idx = 3*((int)tt);
		double t = tt - (int)tt;
		if (ctrlPoint.size()-1 == idx)
		{
			idx-=3;
			t = 1;
		}
		double invt = 1-t;
		double sqt = t*t;
		double sqinvt = invt*invt;

		return (
			sqinvt*invt*ctrlPoint[(idx)]
			+ (3*sqinvt*t)*ctrlPoint[(idx+1)]
			+ (3*sqt*invt)*ctrlPoint[(idx+2)]
			+ sqt*t*ctrlPoint[(idx+3)]
			);
				
	}
	/*
	Eigen::VectorXd getDifferentialValue(double tt)
	{
		int idx = 3*((int)tt);
		double t = tt - (int)tt;
		if (ctrlPoint.size()-1 == idx)
		{
			idx-=3;
			t = 1;
		}
		double invt = 1 - t;
		double sqt = t*t;
		double sqinvt = invt*invt;

		return (
			-sqinvt * ctrlPoint[(idx-1)]
			+ (3 * sqt - 4 * t) * ctrlPoint[(idx)]
			+ (-3 * sqt + 2 * t + 1) * ctrlPoint[(idx+1)]
			+ sqt * ctrlPoint[(idx+2)]
			) / 2.0;
	}

	Eigen::VectorXd getSecondDerivationValue(double tt)
	{
		int idx = (int)tt;
		double t = tt - (int)tt;
		if (idx == ctrlPoint.size()-3)
		{
			idx--;
			t = 1;
		}

		return (
			(1-t) * ctrlPoint[(idx-1)]
			+ (3 * t - 2 ) * ctrlPoint[(idx)]
			+ (-3 * t + 1 ) * ctrlPoint[(idx+1)]
			+ t * ctrlPoint[(idx+2)]
			) ;
	}
	*/

	void clear()
	{
		ctrlPoint.clear();
	}
};

#endif