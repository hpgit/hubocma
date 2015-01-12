#ifndef _HUBOVPQPCONTROLLER_H_
#define _HUBOVPQPCONTROLLER_H_

#include "HuboVpController.h"
#include <fstream>

class HuboVpQpController : public HuboVpController
{
public:
	// QP Matrices
	Eigen::MatrixXd Mass;
	Eigen::VectorXd b;

	void stepAheadWithPenaltyForces();

	void solveQp(Eigen::VectorXd &q, Eigen::VectorXd &tau, Eigen::VectorXd &lambda);

};

#endif // _HUBOVPQPCONTROLLER_H_
