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
		ground(NULL), groundGeom(NULL)
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
	int manualContactForces;

	//PD contol parameters
	double ks;
	double kd;

	//PD control parameters for torque
	Eigen::VectorXd ksTorqueVector;
	Eigen::VectorXd kdTorqueVector;

	//Ground parameters
	double grfKs;
	double grfDs;
	double mu;

	//Center of Pressure
	Eigen::Vector3d cp;
	Eigen::Vector3d cpBeforeOneStep;

	//TODO:
	//deperecated
	//com tracking
	Eigen::Vector3d comTrackPoint;

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
	void getConstants(char* filename);

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
	void getDesiredDofTorque(
			Eigen::VectorXd &angleRefer,
			Eigen::VectorXd &angleVp,
			Eigen::VectorXd &angleRateRefer,
			Eigen::VectorXd &angleRateVp,
			Eigen::VectorXd &angleAccelRefer,
			Eigen::VectorXd &torque
			);

	static void motionPdTrackingThread(HuboVpController *cont, HuboMotionData *referMotion);

	void balance(
			HuboMotionData *refer,
			double time,
			double kl, double kh,
			double weightTrack, double weightTrackUpper
			);
	void balanceQP(
			HuboMotionData *refer,
			double time,
			double kl, double kh,
			double weightTrack, double weightTrackUpper
			);
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

};
#endif
