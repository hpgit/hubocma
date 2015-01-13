#ifndef _HUBOVPQPCONTROLLER_H_
#define _HUBOVPQPCONTROLLER_H_

#include "HuboVpController.h"
#include <fstream>

class HuboVpQpController : public HuboVpController
{
public:
	// QP Matrices
	// problem formulation : min 0.5*x^T *W*x + a^T x
	Eigen::MatrixXd W;
	Eigen::VectorXd a;

	//void stepAheadWithPenaltyForces();

	// problem formulation : min 0.5*x^T *W*x + a^T x
	// x^T = [ddq^T tau^T]^T
	void addQpObj(Eigen::MatrixXd &W, Eigen::VectorXd &a);
	void solveQp(Eigen::VectorXd &q, Eigen::VectorXd &tau, Eigen::VectorXd &lambda);

};

#endif // _HUBOVPQPCONTROLLER_H_
