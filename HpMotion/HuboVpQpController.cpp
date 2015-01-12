#include "HuboVpQpController.h"

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



	huboVpBody->getEquationsOfMotion(world, Mass, b);

	Eigen::MatrixXd S(32, 32);
	S.setIdentity();
	S.block(0, 0, 6, 6).setZero();

	Eigen::MatrixXd J_c, dJ_c;
	//TODO:
	//get contact Jacobian

	
	Eigen::MatrixXd C(Mass.rows() + J_c.rows(), Mass.cols() + S.cols());
	C.setZero();
	C.topLeftCorner(Mass.rows(), Mass.cols()) = Mass;
	C.bottomLeftCorner(J_c.rows(), J_c.cols()) = J_c;
	C.topRightCorner(S.rows(), S.cols()) = -S;

	Eigen::VectorXd forces;
	Eigen::VectorXd ddtheta_des, dtheta, acc_c_des;
	//TODO:
	//get contact forces, dtheta, acc_c_des

	Eigen::VectorXd d(J_c.rows() + J_c.cols());
	d.head(J_c.cols()) = -b + J_c.transpose() *forces;
	d.tail(J_c.rows()) = -J_c * dtheta + acc_c_des;

	Eigen::MatrixXd A(32+C.rows(), 32+C.rows());
	A.setZero();
	A.topLeftCorner(32, 32).setIdentity();
	//TODO:
	//set weight
	Eigen::MatrixXd W1, W2;

	A.topRightCorner(C.cols(), C.rows()) = C.transpose();
	A.bottomLeftCorner(C.rows(), C.cols()) = C;

	Eigen::VectorXd b_A(32+d.size());
	b_A.head(32) = W1 * ddtheta_des;
	b_A.tail(d.size()) = d;

	//Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(a);
	Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_A);

	ddq.resize(32);
	tau.resize(32);
	ddq = x.head(32);
	tau = x.segment(32, 32);

	


}