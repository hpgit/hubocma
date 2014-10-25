#ifndef _IKSOLVER_H_
#define _IKSOLVER_H_

#include "MotionData.h"

#ifndef GOLDENRATIO
#define GOLDENRATIO 1.618
#endif

#define LIMITCOEFF 1
#define COMTHRESH 0.01
#define CONSTRAINTTHRESH 0.01
#define CONSTRAINTANGULARTHRESH 0.02

class IKSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		IKSolver(MotionData *_obj)
		: maxIter(200), weightPos(1), weightAng(10),
		  stepSize(0.01), ikEps(0.001), obj(_obj)

	{}
	

	int maxIter;
	double ikEps;
	double weightPos;
	double weightAng;
	double stepSize;

	MotionData *obj;
	std::vector<Joint*> Parts;
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd fullJacobian;
	Vector3d goal;
	Quaterniond goalQuat;
	string goalname;

	std::vector<Joint*> dontMoveJointCorrespodingChildBody;

	std::vector<Joint*>constraints;
	std::vector<Vector3d, aligned_allocator<Vector3d> > constraintPosition;
	std::vector<Quaterniond, aligned_allocator<Quaterniond> > constraintOrientation;

	void makeParts();
	void run_r(string PartName, Vector3d &goalPosition, Quaterniond &goalOrientation);
	void run_gradient_descent(string PartName, Vector3d &goalPosition, Quaterniond &goalOrientation);

	void initpose();

	void solve(Eigen::VectorXd &dp, int nStep);
	int solve_r(Eigen::VectorXd &dp, int nStep);

	double computeJacobian();
	void computeFullJacobian(); // Joint movement to Body(child of Joint) movement mapping
	void computeConstraintFullJacobian(Joint* joint, Eigen::MatrixXd &constraintJacobian);
	void setConstraintJoint(Joint *joint);
	void releaseConstraintJoint(Joint *joint);
	double computepenalty();

	double computePenalty(Eigen::VectorXd &p);
	double computePenaltyGradient(Eigen::VectorXd &p, Eigen::VectorXd &dp);

};

#endif
