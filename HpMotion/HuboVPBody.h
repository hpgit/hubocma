#ifndef _HUBOVPBODY_H_
#define _HUBOVPBODY_H_

#include <VP/vphysics.h>
#include "HuboMotionData.h"
#include "IKSolver.h"

class HuboVPBody
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	HuboVPBody();
	~HuboVPBody();
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
	/*
	enum id{
	RHY=0, RHR, RHP, RKN, RAP, RAR, LHY, LHR, LHP, LKN, LAP, LAR,
	RSP, RSR, RSY, REB, RWY, RWP, LSP, LSR, LSY, LEB, LWY, LWP,
	NKY, HeadX, HeadZ, WST
	};
	enum {Hip=0, Torso, Head,
		RShoulderP, RShoulderR, RUpperArm, RElbow, RWrist, RHand,
		RPelvisY, RPelvisR, RUpperLeg, RLowerLeg, RAnkle, RFoot,
		LShoulderP, LShoulderR, LUpperArm, LElbow, LWrist, LHand,
		LPelvisY, LPelvisR, LUpperLeg, LLowerLeg, LAnkle, LFoot};
		*/

	vpBody *Hip, *Torso, *Head;
	vpBody *ShoulderP[2], *ShoulderR[2], *UpperArm[2], *Elbow[2], *Wrist[2], *Hand[2];
	vpBody *PelvisY[2], *PelvisR[2], *UpperLeg[2], *LowerLeg[2], *Ankle[2], *Foot[2];
	vpRJoint *WST, *NKY, *HeadX, HeadZ; // upper part
	vpRJoint *RSP, *RSR, *RSY, *REB, *RWY, *RWP; // upper right
	vpRJoint *RHY, *RHR, *RHP, *RKN, *RAP, *RAR; // lower right
	vpRJoint *LSP, *LSR, *LSY, *LEB, *LWY, *LWP; // upper left
	vpRJoint *LHY, *LHR, *LHP, *LKN, *LAP, *LAR; // lower left
	vpBJoint b;


	Eigen::Vector3d com;
	double mass;

	IKSolver *pIKSolver;
	HuboMotionData *pHuboMotion;
	std::vector<vpBody*>bodies; // Right links first
	std::vector<vpGeom*>bodyGeoms; 
	std::vector<vpRJoint*>joints; // Right joints first
	std::map<Joint *, vpRJoint *>vpHuboJointToVpmap;
	std::map<vpRJoint *, Joint *>vptohuboJointmap;
	std::map<vpBody *, Joint *>vpBodytohuboParentJointmap;
	std::map<vpJoint *, vpBody *>vpJointtoBodymap;
	std::map<vpBody *, vpBody *>vpBodytoParentBody;
	std::map<vpBody *, vpRJoint *>vpBodytoJointmap;;


	// contact info
	double grfKs;
	double grfDs;
	double mu;
	std::vector<vpBody*>collideCheckBodies;
	std::vector<vpBody*>collideBodies;
	std::vector<Vec3>positions;
	std::vector<Vec3>positionsLocal;
	std::vector<Vec3>forces;

	void initBody();
	void initJoint();
	void setInitBodyJoint(vpBody *pBody, vpRJoint *pJoint, string bodyName, vpBody *pParentBody, double elasticity, double damping);
	void initHybridDynamics(bool floatingBase);
	void solveHybridDynamics();
	void create(vpWorld *pWorld, HuboMotionData *pHuboImporter);
	void stepAhead(vpWorld *pWorld, vpBody *pGround);

	void drawBodyBoundingBox(vpBody *body);
	void drawAllBoundingBox();

	void ignoreVpHuboBodyCollision(vpWorld *pWorld);
	void ignoreVpGroundBodyCollision(vpWorld *pWorld, vpBody *pGround);
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

	vpRJoint *getParentJoint(vpRJoint *joint);
	
	void getHuboLimit(int minOrMax, std::vector<double> &bound);//min :0, max : 1
	HuboMotionData *getHuboImporter();
	Eigen::Vector3d getSupportRegionCenter();
	Eigen::Vector3d getVpHipRotationalJointPosition();
	Eigen::Vector3d getVpJointPosition(vpRJoint *joint);
	Eigen::Vector3d getVpJointAxis(vpRJoint *joint);
	Eigen::Vector3d getCOMposition();
	Eigen::Vector3d getCOMvelocity();
	Eigen::Vector3d getCOPposition(vpWorld *pWorld, vpBody *pGround);
	Vec3 getCOM();
	Vec3 getCOMLinvel();
	Vec3 getCOP(vpWorld *pWorld, vpBody *pGround);
	int getMainContactFoot(vpWorld *pWorld, vpBody *pGround, double &leftRate, double &rightRate);
	dse3 getBodyContactForcesToEachCom(vpBody *pBody);
	Eigen::Vector3d getHipDirection();
	void getHuboHipState(Eigen::Vector3d &Pos, Eigen::Quaterniond &Ori, Eigen::Vector3d &Vel, Eigen::Vector3d &AngVel);
	Eigen::Quaterniond getOrientation(vpBody *pBody);

	void getBodyState(vpBody *pBody, Eigen::Vector3d &pos, Eigen::Quaterniond &ori, Eigen::Vector3d &vel, Eigen::Vector3d &angVel);
	void getFootSole(int LEFTorRIGHT, Eigen::Vector3d &pos, Eigen::Quaterniond &ori);

	void getAllAngle(Eigen::VectorXd &angles); // radian
	void getAllAngularVelocity(Eigen::VectorXd &angVel);
	void getAllAngularAcc(Eigen::VectorXd &angacc);
	void getAllJointTorque(Eigen::VectorXd &torque);

	void setAllAngularAcc(Eigen::VectorXd &angacc);
	void setAllJointTorque(Eigen::VectorXd &torque);

	void getFootPoints(int LEFTorRIGHT, std::vector<Vector3d, aligned_allocator<Vector3d> > &points);
	Eigen::Vector3d getFootPenaltyPosition(int LEFTorRIGHT, double &retmomentsum);

	void calcPenaltyForce(
		vpWorld *pWorld, vpBody* pGround,
		std::vector<vpBody*> &checkBodies,
		double ks, double ds, double mu
		);
	bool _calcPenaltyForce(
		vpWorld *pWorld,
		const vpBody *pBody, const Vec3 &position, const Vec3 &velocity, 
		Vec3 &force, double ks, double ds, double mu
		); 

	void calcPenaltyForceBoxGround(
		vpWorld *pWorld, vpBody* pGround,
		std::vector<vpBody*> &checkBodies,
		double ks, double ds, double mu
		);
	bool _calcPenaltyForceBoxGround(
		const Vec3 &boxSize, const SE3 &boxFrame, const vpBody *pBody, 
		const Vec3 &position, const Vec3 &velocity, 
		Vec3 &force, double ks, double ds, double mu
		); 
	void applyPenaltyForce();

	void getBodyJacobian(vpBody *body, Eigen::MatrixXd &J);
	void getBodyDifferentialJacobian(vpBody *body, Eigen::MatrixXd &dJ);
	void getFootSupJacobian(Eigen::MatrixXd &fullJ, Eigen::MatrixXd &Jsup, int RIGHTorLEFT);
	void getDifferentialFootSupJacobian(Eigen::MatrixXd &fulldJ, Eigen::MatrixXd &dJsup, int RIGHTorLEFT);
	void getJacobian(Eigen::MatrixXd &J, int inlucdeRoot = 0);
	void getDifferentialJacobian(Eigen::MatrixXd &dJ, int inlucdeRoot = 0);
	void getJacobian2(Eigen::MatrixXd &J);
	void getDifferentialJacobian2(Eigen::MatrixXd &dJ);
	void getLinkMatrix(Eigen::MatrixXd &M);
	void getDifferentialLinkMatrix(Eigen::MatrixXd &dM);

	void getSingleFootRootJacobian(Eigen::MatrixXd &J, int isLEFT);
	void getSingleFootRootLinkMatrix(Eigen::MatrixXd &M, int isLEFT);
	
	void getSingleFootRootToHipJacobian(Eigen::MatrixXd &J, int isLEFT);

	void getEquationsOfMotion(vpWorld *world, Eigen::MatrixXd &M, Eigen::VectorXd &b);
};
#endif
