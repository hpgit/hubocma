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
