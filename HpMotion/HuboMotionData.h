#ifndef _HUBOMOTIONDATA_H_
#define _HUBOMOTIONDATA_H_

#include "MotionData.h"

class HuboMotionData : public MotionData {
	
public:
	HuboMotionData()
		: startPeriodFrame(0), contactPeriod(0)
	{
		if (naturalToBsplineFourPointIsReady == 0)
		{
			Eigen::MatrixXd m(6, 6);
			m << 1, -2, 1, 0, 0, 0,
				1, 4, 1, 0, 0, 0,
				0, 1, 4, 1, 0, 0,
				0, 0, 1, 4, 1, 0,
				0, 0, 0, 1, 4, 1,
				0, 0, 0, 1, -2, 1;
			m /= 6;
			naturalToBsplineFourPoint = m.inverse();
			naturalToBsplineFourPointIsReady = 1;
		}
	}
	string filename;

	static Eigen::MatrixXd naturalToBsplineFourPoint;
	static int naturalToBsplineFourPointIsReady;

	std::vector<Joint*> activeJoints;
	std::vector<int> rFootContact;
	std::vector<int> lFootContact;

	int startPeriodFrame;
	int contactPeriod;

	int import(const char* _filename, int firstFrame, int lastFrame = 0);
	int importContactAnnotation(const char* _filename, int firstFrame, int lastFrame = 0);
	int importContactPeriodAnnotation(const char* _filename, int firstFrame, int lastFrame = 0);
	int importSkeleton(const char* _filename);
	void printAllMotion(char* _filename);
	
	void save(const char* filename, int firstFrame, int lastFrame = 0);

	void setOBJon();
	void unsetOBJon();

	void makeAllJointAngleSpline();

	double getPeriodPhaseInTime(double time);


	Vector3d getHuboComGlobalPositionInTime(double t);
	Vector3d getHuboComGlobalPosition(int frame);
	Vector3d getHuboComGlobalPosition();

	Vector3d getHipJointGlobalPositionInTime(double t);
	Quaterniond getHipJointGlobalOrientationInTime(double t);
	Vector3d getHipJointVelInHuboMotionInTime(double t);
	Vector3d getHipJointAngVelInHuboMotionInTime(double t);
	Vector3d getHipJointAccelInHuboMotionInTime(double t);
	Vector3d getHipJointAngAccelInHuboMotionInTime(double t);
	
	void getAllHipStateInTime(double t,
		Eigen::Vector3d &Pos,
		Eigen::Quaterniond &Ori,
		Eigen::Vector3d &Vel,
		Eigen::Vector3d &AngVel,
		Eigen::Vector3d &Accel,
		Eigen::Vector3d &AngAccel);

	void getAllAngleInHuboMotion(int frame, Eigen::VectorXd &angles);
	void getAllAngleInHuboMotion(int firstFrame, double t, Eigen::VectorXd &angles);
	void getAllAngleRateInHuboMotion(int frame, Eigen::VectorXd &angles);
	void getAllAngleRateInHuboMotion(int firstFrame, double t, Eigen::VectorXd &angles);
	void getAllAngleAccelInHuboMotion(int frame, Eigen::VectorXd &angles);
	void getAllAngleAccelInHuboMotion(int firstFrame, double t, Eigen::VectorXd &angles);
	void getAllAngleInHuboMotionInTime(double t, Eigen::VectorXd &angles);
	void getAllAngleRateInHuboMotionInTime(double t, Eigen::VectorXd &angles);
	void getAllAngleAccelInHuboMotionInTime(double t, Eigen::VectorXd &angles);

	void getAllStateInTime(double t, Eigen::VectorXd &ang, Eigen::VectorXd &angVel, Eigen::VectorXd &angAcc);

	double getHuboMass();
	void getHuboAllMassMatrix(Eigen::MatrixXd &massMatrix, int applyOrientation);
	void makeGroundContact();

	friend void transformToHuboMotionData(MotionData *md, HuboMotionData *hmd);
	friend void transformToHuboMotionDataOnlyUpperBody(MotionData *md, HuboMotionData *hmd);
	friend void transformToHuboMotionDataIK(MotionData *md, HuboMotionData *hmd, std::map<string, string> endeffctormap);
	friend void transformToHuboMotionDataIKOnlyUpperBody(MotionData *md, HuboMotionData *hmd, std::map<string, string> endeffctormap);
};

#endif
