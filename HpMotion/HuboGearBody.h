#ifndef _HUBOGEARBODY_H_
#define _HUBOGEARBODY_H_

#include <gear/gear.h>
#include "HuboMotionData.h"
#include "IKSolver.h"

class HuboGearBody
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	HuboGearBody();
	~HuboGearBody();
	enum {RIGHT=0, LEFT};
	enum JOINTNUM{
		eWST = 0,
		eRSP, eRSR, eRSY, eREB, eRWY, eRWP,
		eNKY,
		eRHY, eRHR, eRHP, eRKN, eRAP, eRAR,
		eLHY, eLHR, eLHP, eLKN, eLAP, eLAR,
		eLSP, eLSR, eLSY, eLEB, eLWY, eLWP
		};
	enum BODYNUM{
		eHip = 0, eTorso, eHead,
		eRShoulderP, eRShoulderR, eRUpperArm, eRElbow, eRWrist, eRHand,
		eRPelvisY, eRPelvisR, eRUpperLeg, eRLowerLeg, eRAnkle, eRFoot,
		eLShoulderP, eLShoulderR, eLUpperArm, eLElbow, eLWrist, eLHand,
		eLPelvisY, eLPelvisR, eLUpperLeg, eLLowerLeg, eLAnkle, eLFoot
	};

	Vector3d com;
	double mass;
	double grfKs;
	double grfDs;
	double mu;

	
	GBody *Hip, *Torso, *Head;
	GBody *ShoulderP[2], *ShoulderR[2], *UpperArm[2], *Elbow[2], *Wrist[2], *Hand[2];
	GBody *PelvisY[2], *PelvisR[2], *UpperLeg[2], *LowerLeg[2], *Ankle[2], *Foot[2];
	GJointRevolute *WST, *NKY, *HeadX, HeadZ; // upper part
	GJointRevolute *RSP, *RSR, *RSY, *REB, *RWY, *RWP; // upper right
	GJointRevolute *RHY, *RHR, *RHP, *RKN, *RAP, *RAR; // lower right
	GJointRevolute *LSP, *LSR, *LSY, *LEB, *LWY, *LWP; // upper left
	GJointRevolute *LHY, *LHR, *LHP, *LKN, *LAP, *LAR; // lower left



	IKSolver *pIKSolver;
	HuboMotionData *pHuboMotion;
	std::vector<GBody*>bodies; // Right links first
	//std::vector<vpGeom*>bodyGeoms; 
	std::vector<GJointRevolute*>joints; // Right joints first
	std::map<Joint *, GJointRevolute *>vpHuboJointToVpmap;
	std::map<GJointRevolute *, Joint *>vptohuboJointmap;
	std::map<GBody *, Joint *>vpBodytohuboParentJointmap;
	std::map<GJointRevolute *, GBody *>vpJointtoBodymap;
	std::map<GBody *, GBody *>vpBodytoParentBody;
	std::map<GBody *, GJointRevolute *>vpBodytoJointmap;;
	//*/

	void initBody();
	void initJoint();
	void setInitBodyJoint(GBody *pBody, GJoint *pJoint, string bodyName, GBody *pParentBody, double elasticity, double damping);
	void initHybridDynamics(bool floatingBase);
	void solveHybridDynamics();
	void create(GSystem *pWorld, HuboMotionData *pHuboImporter);
	void stepAhead(GSystem *pWorld, GBody *pGround);

	void drawBodyBoundingBox(GBody *body);
	void drawAllBoundingBox();

	void ignoreVpHuboBodyCollision(GSystem *pWorld);
	void ignoreVpGroundBodyCollision(GSystem *pWorld, GBody *pGround);
	void getAllJointTorque(Eigen::VectorXd &torque);
	void applyAllJointValueVptoHubo();
	void applyAddAllBodyForce(Eigen::VectorXd &force);
	void applyAllJointTorque(Eigen::VectorXd &torque);
	void applyAllJointTorque(
		double WST, double NKY, 
		double RSP, double RSR, double RSY, double REB, double RWY, double RWP, 
		double RHY, double RHR, double RHP, double RKN, double RAP, double RAR,
		double LSP, double LSR, double LSY, double LEB, double LWY, double LWP,
		double LHY, double LHR, double LHP, double LKN, double LAP, double LAR 
		);
	void applyRootJointDofAccel(Vector3d &accHip, Vector3d &angAccHip);
	void applyAllJointDofAccel(Eigen::VectorXd &acc);

	void setInitialHuboHipFromMotion(HuboMotionData *refer);
	void setInitialHuboAngleFromMotion(HuboMotionData *refer);
	void setInitialHuboAngleRateFromMotion(HuboMotionData *refer);

	GJointRevolute *getParentJoint(GJointRevolute *joint);
	
	void getHuboLimit(int minOrMax, std::vector<double> &bound);//min :0, max : 1
	HuboMotionData *getHuboImporter();
	Vector3d getSupportRegionCenter();
	Vector3d getVpHipRotationalJointPosition();
	Vector3d getVpJointPosition(GJointRevolute *joint);
	Vector3d getVpJointAxis(GJointRevolute *joint);
	Vector3d getCOMposition();
	Vector3d getCOMvelocity();
	Vector3d getCOPposition(GSystem *pWorld, GBody *pGround);
	Vec3 getCOM();
	Vec3 getCOMLinvel();
	Vec3 getCOP(GSystem *pWorld, GBody *pGround);
	int getMainContactFoot(GSystem *pWorld, GBody *pGround, double &leftRate, double &rightRate);
	Vector3d getHipDirection();
	void getHuboHipState(Eigen::Vector3d &Pos, Eigen::Quaterniond &Ori, Eigen::Vector3d &Vel, Eigen::Vector3d &AngVel);
	Quaterniond getOrientation(GBody *pBody);

	void getBodyState(GBody *pBody, Eigen::Vector3d &pos, Eigen::Quaterniond &ori, Eigen::Vector3d &vel, Eigen::Vector3d &angVel);
	void getFootSole(int LEFTorRIGHT, Eigen::Vector3d &pos, Eigen::Quaterniond &ori);

	void getAllAngle(Eigen::VectorXd &angles); // radian
	void getAllAngularVelocity(Eigen::VectorXd &angVel);

	void getFootPoints(int LEFTorRIGHT, std::vector<Vector3d, aligned_allocator<Vector3d> > &points);
	Vector3d getFootPenaltyPosition(int LEFTorRIGHT, double &retmomentsum);

	void calcPenaltyForce(
		GSystem *pWorld, GBody* pGround,
		std::vector<GBody*> &checkBodies,
		std::vector<GBody*> &collideBodies,
		std::vector<Vec3> &positions,
		std::vector<Vec3> &positionLocals,
		std::vector<Vec3> &forces,
		double ks, double ds, double mu
		);
	bool _calcPenaltyForce(
		GSystem *pWorld,
		const GBody *pBody, const Vec3 &position, const Vec3 &velocity, 
		Vec3 &force, double ks, double ds, double mu
		); 

	void calcPenaltyForceBoxGround(
		GSystem *pWorld, GBody* pGround,
		std::vector<GBody*> &checkBodies,
		std::vector<GBody*> &collideBodies,
		std::vector<Vec3> &positions,
		std::vector<Vec3> &positionLocals,
		std::vector<Vec3> &forces,
		double ks, double ds, double mu
		);
	bool _calcPenaltyForceBoxGround(
		const Vec3 &boxSize, const SE3 &boxFrame, const GBody *pBody, 
		const Vec3 &position, const Vec3 &velocity, 
		Vec3 &force, double ks, double ds, double mu
		); 
	void applyPenaltyForce( 
		const std::vector<GSystem*> &collideBodies, 
		const std::vector<Vec3> &positionLocals, 
		const std::vector<Vec3> &forces
		);

	void getFootSupJacobian(Eigen::MatrixXd &fullJ, Eigen::MatrixXd &Jsup, int RIGHTorLEFT);
	void getDifferentialFootSupJacobian(Eigen::MatrixXd &fulldJ, Eigen::MatrixXd &dJsup, int RIGHTorLEFT);
	void getJacobian(Eigen::MatrixXd &J, int inlucdeRoot = 0);
	void getDifferentialJacobian(Eigen::MatrixXd &dJ, int inlucdeRoot = 0);
	void getLinkMatrix(Eigen::MatrixXd &M);
	void getDifferentialLinkMatrix(Eigen::MatrixXd &dM);

	void getSingleFootRootJacobian(Eigen::MatrixXd &J, int isLEFT);
	void getSingleFootRootLinkMatrix(Eigen::MatrixXd &M, int isLEFT);
	
	void getSingleFootRootToHipJacobian(Eigen::MatrixXd &J, int isLEFT);
	//*/
};

#endif // _HUBOGEARBODY_H_
