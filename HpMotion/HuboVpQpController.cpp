#include "HuboVpQpController.h"

void HuboVpQpController::solveQp(Eigen::VectorXd &q, Eigen::VectorXd &tau, Eigen::VectorXd &lambda)
{
	huboVpBody->getEquationsOfMotion(world, Mass, b);


}