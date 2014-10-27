#ifndef _HUBOVPCONTROLLER_H_
#define _HUBOVPCONTROLLER_H_

#include "HuboVPBody.h"
#include <fstream>

class HuboVpController 
{
private:
	HuboMotionData *huboMotion;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	HuboVpController() :
		huboMotion(NULL),huboVpBody(NULL), world(NULL),
		ground(NULL), groundGeom(NULL), timestep(0.001),
		ks(2000), kd(89.44),
		//grfKs(75000), grfDs(100)
		grfKs(7500), grfDs(100)
	{
		init();
	}
	~HuboVpController();
	HuboVPBody *huboVpBody;
	vpWorld *world;
	vpBody *ground;
	vpGeom *groundGeom;

	std::ofstream fout;

	//Simulator parameters
	double timestep;

	//PD contol parameters
	double ks;
	double kd;

	//Ground parameters
	double grfKs;
	double grfDs;

	//Center of Pressure
	Vector3d cp;
	Vector3d cpBeforeOneStep;

	//TODO:
	//deperecated
	//com tracking
	Vector3d comTrackPoint;

	std::vector<Joint*>constraints;
	std::vector<Vector3d, aligned_allocator<Vector3d> > constraintPosition;
	std::vector<Quaterniond, aligned_allocator<Quaterniond> > constraintOrientation;

	bool leftHandHoldCheck;
	bool leftFootHoldCheck;
	bool rightHandHoldCheck;
	bool rightFootHoldCheck;

	void importHuboSkeleton(char* filename);
	void init();
	void initController();

	void setTimeStep(double _timestep);
	double getTimeStep();

	void stepAheadWithPenaltyForces();

	void applyPdControlTorque(Eigen::VectorXd &desireAngles, Eigen::VectorXd &desireAngularVelocities);
	void applyPdControlTorque(
			Eigen::VectorXd &ngles,
			Eigen::VectorXd &desireAngularVelocities,
			Eigen::VectorXd &

			);

	void getDesiredDofAngAccelForRoot(
			Eigen::Quaterniond &orienRefer,
			Eigen::Quaterniond &orienVp,
			Eigen::Vector3d &angleRateRefer,
			Eigen::Vector3d &angleRateVp,
			Eigen::Vector3d &angleAccelRefer,
			Eigen::Vector3d &acc
			);

	void getDesiredDofAccelForRoot(
			Eigen::Vector3d &posRefer,
			Eigen::Vector3d &posVp,
			Eigen::Vector3d &posRateRefer,
			Eigen::Vector3d &posRateVp,
			Eigen::Vector3d &posAccelRefer,
			Eigen::Vector3d &acc
			);

	void getDesiredDofAccel(
			Eigen::VectorXd &angleRefer,
			Eigen::VectorXd &angleVp,
			Eigen::VectorXd &angleRateRefer,
			Eigen::VectorXd &angleRateVp,
			Eigen::VectorXd &angleAccelRefer,
			Eigen::VectorXd &acc
			);
	static void motionPdTrackingThread(HuboVpController *cont, HuboMotionData *referMotion);

	//void balancing(HuboMotionData *refer, double time);
	void balancing(
			HuboMotionData *refer,
			double time,
			double kl, double kh,
			double weightTrack, double weightTrackAnkle, double weightTrackUpper
			);

	void comTrackingWithoutPhysics(double comx, double comy, double comz);
	void comSolve(
			Eigen::VectorXd &dAngles,
			int nStep,
			std::vector<Joint*> &constraints,
			std::vector<Vector3d, aligned_allocator<Vector3d> > &constraintPosition,
			std::vector<Quaterniond, aligned_allocator<Quaterniond> > &constraintOrientation
			);
	int computePenaltyWithCom(
			std::vector<Joint*> &constraints,
			std::vector<Vector3d, aligned_allocator<Vector3d> > &constraintPosition,
			std::vector<Quaterniond, aligned_allocator<Quaterniond> > &constraintOrientation
			);
	void getConstraintStates();

	static unsigned int sendupdatedstate_test(void* aParam);
	static unsigned int sendupdatedstate(void* aParam);

	//TODO:
	//timestep, Ks, Kd, elasticity,
};
#endif
