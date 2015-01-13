#include "HuboVpQpController.h"
#include "HpMotionMath.h"

void HuboVpQpController::solveQp(Eigen::VectorXd &ddq, Eigen::VectorXd &tau, Eigen::VectorXd &lambda)
{
	//TODO:
	//set inequality constraints

	// currently, only equality constraints are considered
	// so the QP is solved linear equations.

	// min w1*|| ddtheta_des - ddtheta ||^2 + w2*|| tau || ^2
	// w.r.t.	M*ddtheta + b = S*tau + J_c^T * f_c
	//			J_c * ddtheta = dJ_c * dtheta + acc_c_des
	// => min 0.5*x^T*W*x - | ddtheta_des^T*W1 0 |*x
	// w.r.t. Cx = d 

	// S =  | 0 0 |   => underactuated root
	//		| 0 I |

	// C =	| M   -S |
	//		| J_c 0  |

	// d =	| -b + J_c^T * f_c           |
	//		| -dJ_c * dtheta + acc_c_des |


	//solve Ax = b_A

	// A =  | W C^T |
	//		| C   0 |

	// b_A =| W1 * ddtheta_des |
	//		| d                |

	Eigen::MatrixXd Mass;
	Eigen::VectorXd b;

	huboVpBody->getEquationsOfMotion(world, Mass, b);

	Eigen::MatrixXd S(32, 32);
	S.setIdentity();
	S.block(0, 0, 6, 6).setZero();

	Eigen::MatrixXd J_cR, dJ_cR, J_cL, dJ_cL;
	Eigen::MatrixXd J_c, dJ_c;

	Eigen::VectorXd cForces;
	Eigen::VectorXd dtheta, acc_c_des;

	//TODO:
	// examine that acc_c_des to PD

	

	double footRate[2];
	int mainContactFoot = huboVpBody->getMainContactFoot(world, ground, footRate[0], footRate[1]);
	int numContactFoot = 0;
	if (mainContactFoot == HuboVPBody::LEFT || mainContactFoot == HuboVPBody::RIGHT)
	{
		vpBody *body = huboVpBody->Foot[mainContactFoot];
		huboVpBody->getBodyJacobian(body, J_c);
		huboVpBody->getBodyDifferentialJacobian(body, dJ_c);
		dse3 cforce_dse3 = huboVpBody->getBodyContactForcesToEachCom(body);
		cForces.resize(6);
		cForces(0) = cforce_dse3[3];
		cForces(1) = cforce_dse3[4];
		cForces(2) = cforce_dse3[5];
		cForces(3) = cforce_dse3[0];
		cForces(4) = cforce_dse3[1];
		cForces(5) = cforce_dse3[2];
		acc_c_des.resize(6);
		acc_c_des.setZero();

		numContactFoot = 1; 
	}
	else if (mainContactFoot == 2)
	{
		huboVpBody->getBodyJacobian(huboVpBody->Foot[HuboVPBody::RIGHT], J_c);
		huboVpBody->getBodyDifferentialJacobian(huboVpBody->Foot[HuboVPBody::RIGHT], dJ_c);
		huboVpBody->getBodyJacobian(huboVpBody->Foot[HuboVPBody::LEFT], J_cL);
		huboVpBody->getBodyDifferentialJacobian(huboVpBody->Foot[HuboVPBody::LEFT], dJ_cL);
		matrixStackUptoDown(J_c, J_cL);
		matrixStackUptoDown(dJ_c, dJ_cL);
		dse3 cforce_dse3_R = huboVpBody->getBodyContactForcesToEachCom(huboVpBody->Foot[HuboVPBody::RIGHT]);
		dse3 cforce_dse3_L = huboVpBody->getBodyContactForcesToEachCom(huboVpBody->Foot[HuboVPBody::LEFT]);
		cForces.resize(12);
		cForces(0) = cforce_dse3_R[3];
		cForces(1) = cforce_dse3_R[4];
		cForces(2) = cforce_dse3_R[5];
		cForces(3) = cforce_dse3_R[0];
		cForces(4) = cforce_dse3_R[1];
		cForces(5) = cforce_dse3_R[2];
		cForces(6) = cforce_dse3_L[3];
		cForces(7) = cforce_dse3_L[4];
		cForces(8) = cforce_dse3_L[5];
		cForces(9) = cforce_dse3_L[0];
		cForces(10) = cforce_dse3_L[1];
		cForces(11) = cforce_dse3_L[2];
		acc_c_des.resize(12);
		acc_c_des.setZero();
		numContactFoot = 2; 
	}

	if (numContactFoot != 0)
	{
		dtheta.resize(32);
		
		Eigen::Vector3d hippos, hipvel, hipangvel;
		Eigen::Quaterniond hipori;
		Eigen::VectorXd angVels;
		huboVpBody->getAllAngularVelocity(angVels);
		huboVpBody->getHuboHipState(hippos, hipori, hipvel, hipangvel);
		dtheta.head(3) = hipvel;
		dtheta.segment(3, 3) = hipangvel;
		dtheta.tail(26) = angVels;
	}

	
	Eigen::MatrixXd C(Mass.rows() + numContactFoot * 6, Mass.cols() + S.cols());
	C.setZero();
	C.topLeftCorner(Mass.rows(), Mass.cols()) = Mass;
	if(numContactFoot != 0)
		C.bottomLeftCorner(J_c.rows(), J_c.cols()) = J_c;
	C.topRightCorner(S.rows(), S.cols()) = -S;



	Eigen::VectorXd d(Mass.rows() + 6 * numContactFoot);
	d.head(Mass.rows()) = -b;
	if (numContactFoot != 0)
	{
		d.head(Mass.rows()) += J_c.transpose() *cForces;
		d.tail(J_c.rows()) = -J_c * dtheta + acc_c_des;
	}

	Eigen::MatrixXd A(W.rows()+C.rows(), W.cols()+C.rows());
	A.setZero();
	A.topLeftCorner(W.rows(), W.cols()) = W;
	A.topRightCorner(C.cols(), C.rows()) = C.transpose();
	A.bottomLeftCorner(C.rows(), C.cols()) = C;

	Eigen::VectorXd b_A(a.size()+d.size());
	b_A.head(a.size()) = a;
	b_A.tail(d.size()) = d;

	//Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(a);
	Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_A);

	ddq.resize(32);
	tau.resize(32);
	ddq = x.head(32);
	tau = x.segment(32, 32);
}

void HuboVpQpController::balanceQp(
			HuboMotionData *refer,
			double time,
			double kl, double kh,
			double weightTrack, double weightTrackUpper
			)
{
	double dl=2*std::sqrt(kl), dh=2*std::sqrt(kh);

	//[Macchietto 2009] for momentum term
	Eigen::MatrixXd M, dM, J, dJ;
	huboVpBody->getJacobian2(J);
	huboVpBody->getLinkMatrix(M);
	huboVpBody->getDifferentialJacobian2(dJ);
	huboVpBody->getDifferentialLinkMatrix(dM);

	Eigen::VectorXd angles, angVels, angAccels;
	Eigen::VectorXd dofVels;
	huboVpBody->getAllAngle(angles);
	huboVpBody->getAllAngularVelocity(angVels);
	dofVels.resize(32);

	Vec3 rootVel, rootAngVel;
	rootVel = huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0));
	rootAngVel = huboVpBody->Hip->GetAngVelocity();

	dofVels.segment(0, 3) = Vec3Tovector(rootVel);
	dofVels.segment(3, 3) = Vec3Tovector(rootAngVel);
	dofVels.tail(26) = angVels;

	Eigen::MatrixXd RS = M*J;
	Eigen::MatrixXd R = RS.block(0, 0, 3, 32);
	Eigen::MatrixXd S = RS.block(3, 0, 3, 32);

	Eigen::VectorXd rs = (dM*J+M*dJ)*dofVels;
	Eigen::Vector3d rbias = rs.head(3);
	Eigen::Vector3d sbias = rs.tail(3);

	Eigen::Vector3d LdotDes, HdotDes;
	
	//TODO :
	//using real VP value
	//Eigen::Vector3d supCenter = huboVpBody->getSupportRegionCenter();
	Eigen::Vector3d supCenter = refer->getFootCenterInTime(time);
	Eigen::Vector3d comPlane = huboVpBody->getCOMposition();
	comPlane.y() = 0;
	//std::cout << supCenter.transpose() <<std::endl;

	//Eigen::Vector3d comVel = (M*J*dofVels).head(3);

	//LdotDes
	LdotDes = huboVpBody->mass *(
		kl * (supCenter - comPlane)
		- dl * huboVpBody->getCOMvelocity()
		//- dl * comVel
			);
	LdotDes.y() = 0;

	//HdotDes
	//cp and cpBeforeOneStep is calculated in stepAheadWithPenaltyForces()
	//Eigen::Vector3d cp = huboVpBody->getCOPposition(world, ground);
	Eigen::Vector3d cpOld = this->cpBeforeOneStep;

	if (cp.y() < 0 || cpOld.y() < 0)
		HdotDes = Vector3d(0, 0, 0);
	else
	{
		Eigen::Vector3d pdes, dp, ddpdes;
		Eigen::Vector3d refSupCenter =
				refer->getFootCenterInTime(time);

		dp = cp - cpOld;

		ddpdes = kh*(refSupCenter - cp) - dh*dp;
		pdes = cp + dp*timestep + 0.5 * ddpdes*timestep*timestep;
		HdotDes = (pdes - comPlane).cross(LdotDes - huboVpBody->mass * Vector3d(0, -9.8, 0));
	}

	// tracking term
	Eigen::VectorXd desDofAccel;
	desDofAccel.resize(32);
	{
		Eigen::VectorXd angleRefer, angleVelRefer, angleAccelRefer;
		Eigen::VectorXd desAccel;

		Eigen::Vector3d hipPosRefer, hipVelRefer, hipAccelRefer, hipDesAccel;
		Eigen::Vector3d hipPos, hipVel;
		Eigen::Vector3d hipAngVelRefer, hipAngAccelRefer, hipDesAngAccel;
		Eigen::Vector3d hipAngVel;
		Eigen::Quaterniond hipOrienRefer;
		Eigen::Quaterniond hipOrien;
		 
		// get desired acceleration for instance time
		hipPosRefer = refer->getHipJointGlobalPositionInTime(time);
		hipOrienRefer = refer->getHipJointGlobalOrientationInTime(time);
		hipVelRefer = refer->getHipJointVelInHuboMotionInTime(time);
		hipAngVelRefer = refer->getHipJointAngVelInHuboMotionInTime(time);
		hipAccelRefer = refer->getHipJointAccelInHuboMotionInTime(time);
		hipAngAccelRefer = refer->getHipJointAngAccelInHuboMotionInTime(time);

		hipPos = Vec3Tovector(huboVpBody->Hip->GetFrame().GetPosition());
		hipVel = Vec3Tovector(huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0)));
		hipOrien = huboVpBody->getOrientation(huboVpBody->Hip);
		hipAngVel = Vec3Tovector(huboVpBody->Hip->GetAngVelocity());

		refer->getAllAngleInHuboMotionInTime(time, angleRefer);
		refer->getAllAngleRateInHuboMotionInTime(time, angleVelRefer);
		refer->getAllAngleAccelInHuboMotionInTime(time, angleAccelRefer);

		getDesiredDofAccelForRoot(hipPosRefer, hipPos, hipVelRefer, hipVel, hipAccelRefer, hipDesAccel);
		getDesiredDofAngAccelForRoot(hipOrienRefer, hipOrien, hipAngVelRefer, hipAngVel, hipAngAccelRefer, hipDesAngAccel);
		getDesiredDofAccel(angleRefer, angles, angleVelRefer, angVels, angleAccelRefer, desAccel);

		desDofAccel.head(3) = hipDesAccel;
		desDofAccel.segment(3,3) = hipDesAngAccel;
		desDofAccel.tail(26) = desAccel;
	}
}