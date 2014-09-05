#ifndef _IKSOLVER_H_
#define _IKSOLVER_H_

#include "MotionData.h"
#define IKMOTIONSIZE 200

#ifndef GOLDENRATIO
#define GOLDENRATIO 1.618
#endif

#ifndef MAXITER
#define MAXITER 200
#endif

#define LIMITCOEFF 1
#define IKEPSILON 0.02
#define COMTHRESH 0.01
#define CONSTRAINTTHRESH 0.01
#define CONSTRAINTANGULARTHRESH 0.02

class IKSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IKSolver(MotionData *_obj) : obj(_obj) {}
	

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
	void run_r(string PartName, Vector3d &GoalPosition, Quaterniond &goalOrientation);
	void run_gradient_descent(string PartName, Vector3d &GoalPosition);

	static Vector3d diffQuat(Quaterniond &q1, Quaterniond &q2);
	static Vector3d ln(Quaterniond &q);

	void initpose();

	void solve(Eigen::VectorXd &dp, int nStep);
	int solve_r(Eigen::VectorXd &dp, int nStep);

	void computeJacobian();
	void computeFullJacobian(); // Joint movement to Body(child of Joint) movement mapping
	void computeConstraintFullJacobian(Joint* joint, Eigen::MatrixXd &constraintJacobian);
	void setConstraintJoint(Joint *joint);
	void releaseConstraintJoint(Joint *joint);
	double computepenalty();
	void updateGoldenSection(int dim, double x[], double x2[], double x4[], double d[], double &a1, double &a2, double &a3);
	double findMinAlongGradient(int dim, double x0[], double d[], double xmin[], double xmax[]);
};

#endif
