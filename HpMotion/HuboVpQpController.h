#ifndef _HUBOVPQPCONTROLLER_H_
#define _HUBOVPQPCONTROLLER_H_

#include "HuboVpController.h"
#include <fstream>

class HuboVpQpController : public HuboVpController
{
public:
	// QP Matrices
	// problem formulation : min 0.5*x^T *W*x + a^T x
	int qpMatrixInit = 0;
	Eigen::MatrixXd W;
	Eigen::VectorXd a;

	//void stepAheadWithPenaltyForces();

	// problem formulation : min 0.5*x^T *W*x + a^T x
	// x^T = [ddq^T tau^T]^T
	//void addQpObj(double w, Eigen::VectorXd &a);
	void addQpObj(Eigen::MatrixXd &W, Eigen::VectorXd &a);
	void solveQp(Eigen::VectorXd &ddq, Eigen::VectorXd &tau, Eigen::VectorXd &lambda);

	void balanceQp(
			HuboMotionData *refer,
			double time,
			double kl, double kh,
			double trackWeight, double trackUpperWeight,
			double linWeight, double angWeight,
			double torqueWeight
			);
};

#endif // _HUBOVPQPCONTROLLER_H_
