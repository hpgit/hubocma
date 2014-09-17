#include "IKSolver.h"
#include <iostream>
#include "HpMotionMath.h"


void IKSolver::initpose()
{
	obj->resetMotion();
}

void IKSolver::run_r(string PartName, Vector3d &goalPosition, Quaterniond &goalOrientation)
{
	const int frame = obj->getCurrentFrame();

	Eigen::VectorXd dp;
	Joint *EndEffector = obj->jointMap[PartName];
	double angle;
	double penalty;
	double _stepsize;
	int cnt = 0;
	
	
	Parts.clear();
	goalname = PartName;
	goal = goalPosition;
	goalQuat = goalOrientation;
	makeParts();

	penalty = computepenalty();

	for(int j=0; j<maxIter && penalty > ikEps; j++)
	//for(int j=0; j<1 && computepenalty() > IKEPSILON; j++)
	{
		solve(dp, 10);
		//std::cout << dp.transpose() << std::endl;

		///*
		_stepsize = stepSize * 1024;

		//1. copy frame to ikframe;
		obj->backUpMotionForIk(frame);

		do{
			obj->restoreMotionForIk(frame);
			if (cnt++ % 2 == 0)
				_stepsize /= 2.0;
			else
				_stepsize *= -1;

			//2. modify this frame
			for (int i = 0; i < Parts.size(); i++)
			{
				angle = Parts.at(i)->getAngle(frame) + dp(i)*_stepsize;

				/*
				if (angle < Parts.at(i)->constraintAngle[0])
					angle = Parts.at(i)->constraintAngle[0];
				else if (angle > Parts.at(i)->constraintAngle[1])
					angle = Parts.at(i)->constraintAngle[1];
				*/
				Quaterniond q =
					Quaterniond(Eigen::AngleAxisd(angle, Parts.at(i)->constraintAxis));

				Parts.at(i)->setRotation(frame, q);
			}
			if ( abs(stepSize / _stepsize) > 2048)
			{
				obj->restoreMotionForIk(frame);
				break;
			}
			
		} while (computepenalty() > penalty);
		//*/
		/*
		for (int i = 0; i < Parts.size(); i++)
		{
			angle = Parts.at(i)->getAngle(frame) + dp(i)*_stepsize;

			if (angle < Parts.at(i)->constraintAngle[0])
				angle = Parts.at(i)->constraintAngle[0];
			else if (angle > Parts.at(i)->constraintAngle[1])
				angle = Parts.at(i)->constraintAngle[1];
			Quaterniond q =
				Quaterniond(Eigen::AngleAxisd(angle, Parts.at(i)->constraintAxis));

			Parts.at(i)->setRotation(frame, q);
		}
		*/

		penalty = computepenalty();
	}
}

void IKSolver::solve(Eigen::VectorXd &dp, int nStep)
{
	const int frame = obj->getCurrentFrame();
	Eigen::VectorXd b;
	
	Vector3d btemp = ( goal - obj->jointMap[goalname]->getGlobalBoundingBoxPosition(frame) );
	Quaterniond orien = obj->jointMap[goalname]->getGlobalOrientation(frame);
	Vector3d bQuatTemp = diffQuat(goalQuat,orien);

	//if (btemp.squaredNorm() >= DBL_EPSILON)
	//	btemp.normalize();

	b.resize(6);
	
	b.segment(0,3) = btemp;
	b.segment(3,3) = bQuatTemp;
	//std::cout << "diffVec : " << btemp.transpose() << std::endl;
	//std::cout << "diffQuat : " << bQuatTemp.transpose() << std::endl;

	computeJacobian();
	dp = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	//dp = jacobian.inverse()*b;
}

double IKSolver::computeJacobian()
{
	const int frame = obj->getCurrentFrame();

	jacobian.resize(6, Parts.size());

	for(int i=0; i<Parts.size(); i++)
	{
		//Vector3d endEffV = rotateAxis.at(i) * ( GoalPosition - Parts.at(i)->getGlobalPosition(obj->frameTotal-1) );
		//std::cout << Parts.at(i)->name << " " << Parts.at(i)->getGlobalRotationAxis(frame).transpose() << std::endl;
		Vector3d endEffV = Parts.at(i)->getGlobalRotationAxis(frame).cross( 
			( obj->jointMap[goalname]->getGlobalBoundingBoxPosition(frame) - Parts.at(i)->getGlobalPosition(frame)) );
		Vector3d endEffW = Parts.at(i)->getGlobalRotationAxis(frame);

		jacobian.block(0, i, 3, 1) = endEffV;
		jacobian.block(3, i, 3, 1) = endEffW;
	}	
	//std::cout << "jacobian :" << std::endl;
	//std::cout << jacobian << std::endl;

	double dist_penalty = (goal - obj->jointMap[goalname]->getGlobalPosition(frame)).squaredNorm();
    Quaterniond curQuat = obj->jointMap[goalname]->getGlobalOrientation(frame);
    dist_penalty += ::diffQuat(goalQuat, curQuat).squaredNorm();

	return dist_penalty;
}

void IKSolver::computeFullJacobian()
{
	const int frame = obj->getCurrentFrame();

	const int bodiessize = obj->joints.size();
	const int jointssize = obj->joints.size()+5; 

	std::vector<Joint*> &bodies = obj->joints;
	
	fullJacobian.resize(6*bodiessize, jointssize);

	// jacobian initialization
	fullJacobian.setZero();

	// root translation

	for(int i=0; i<3*bodiessize; i++)
	{
		fullJacobian( i, i%3)= 1;
	}

	// root orientation
	{
		Vector3d x(1,0,0), y(0,1,0), z(0,0,1);
		Vector3d axisx(1,0,0), axisy(0,1,0), axisz(0,0,1);
		Vector3d r, valongx, valongy, valongz;

		for(int i=0; i < 3*bodiessize; i+=3)
		{
			r = bodies[i/3]->getGlobalComPosition(frame) - obj->jointMap["Hip"]->getGlobalPosition(frame);

			valongx = axisx.cross(r);
			valongy = axisy.cross(r);
			valongz = axisz.cross(r);

			fullJacobian.block(i, 3, 3, 1) = valongx;
			fullJacobian.block(i, 4, 3, 1) = valongy;
			fullJacobian.block(i, 5, 3, 1) = valongz;
		}

		{
			for(int i=3*bodiessize; i < 6*bodiessize; i+=3)
			{
				fullJacobian.block(i, 3, 3, 1) = Vector3d::UnitX();
				fullJacobian.block(i, 4, 3, 1) = Vector3d::UnitY();
				fullJacobian.block(i, 5, 3, 1) = Vector3d::UnitZ();
			}
		}
	}
	
	
	// joint part
	
	Vector3d rcom, rk;
	Vector3d w, v;
	
	
	for(int j=1; j<jointssize-5; j++)
	{
		// linear velocity part
		for(int i=0; i< 3*bodiessize; i+=3)
		{
			if( obj->joints[j]->isAncestor(bodies[i/3]) || obj->joints[j] == bodies[i/3] )
			{
				rcom = bodies[i/3]->getGlobalComPosition(frame);

				rk = obj->joints[j]->getGlobalPosition(frame);
				w = obj->joints[j]->getGlobalRotationAxis(frame).normalized();
				v = w.cross( (rcom-rk) );

				fullJacobian.block(i, j+5, 3, 1) = v;
			}
			else
			{
				fullJacobian.block(i, j+5, 3, 1) = Vector3d::Zero();
			}
		}
	}

	{
		// orientation part
		for(int j=1; j<jointssize-5; j++)
		{
			for(int i=3*bodiessize; i<6*bodiessize; i+=3)
			{
				
				if( obj->joints[j]->isAncestor(obj->joints[i/3]) || obj->joints[j] == obj->joints[i/3] )
				{
					w = obj->joints[j]->getGlobalRotationAxis(frame).normalized();

					fullJacobian.block(i, j+5, 3, 1) = w;
				}
				else
				{
					fullJacobian.block(i, j+5, 3, 1) = Vector3d::Zero();
				}
			}
		}
	}
}

void IKSolver::computeConstraintFullJacobian(Joint* joint, Eigen::MatrixXd &constraintJacobian)
{
	const int frame = obj->getCurrentFrame();


	const int jointssize = obj->joints.size()+5;

	constraintJacobian.resize(6, jointssize);

	//
	constraintJacobian.setZero();

	// root translation

	for(int i=0; i<3; i++)
	{
		constraintJacobian( i, i)= 1;
	}

	// root orientation
	{
		Vector3d x(1,0,0), y(0,1,0), z(0,0,1);
		Vector3d axisx, axisy, axisz;
		Vector3d r, valongx, valongy, valongz;

		axisx = x;
		axisy = y;
		axisz = z;

		
		r = joint->getGlobalComPosition(frame) - obj->jointMap["Hip"]->getGlobalPosition(frame);

		valongx = axisx .cross(r);
		valongy = axisy .cross(r);
		valongz = axisz .cross(r);

		constraintJacobian.block(0, 3, 3, 1) = valongx;
		constraintJacobian.block(0, 4, 3, 1) = valongy;
		constraintJacobian.block(0, 5, 3, 1) = valongz;
		//

		fullJacobian.block(3, 3, 3, 1) = Vector3d::UnitX();
		fullJacobian.block(3, 4, 3, 1) = Vector3d::UnitY();
		fullJacobian.block(3, 5, 3, 1) = Vector3d::UnitZ();
	}

	
	// joint part
	
	// rcom : link to CoM, rk : , w : normalized angular velocity of link
	Vector3d rcom, rk;
	Vector3d w, v;
	

	for(int j=1; j<jointssize-5; j++)
	{
		// linear velocity part
		if( obj->joints[j]->isAncestor(joint) || obj->joints[j] == joint )
		{
			rcom = joint->getGlobalComPosition(frame);

			rk = obj->joints[j]->getGlobalPosition(frame);
			w = obj->joints[j]->getGlobalRotationAxis(frame).normalized();
			v = w.cross( (rcom-rk) );

			constraintJacobian.block(0, j+5, 3, 1) = v;
		}
		else
		{
			constraintJacobian.block(0, j+5, 3, 1) = Vector3d::Zero();
		}
	}

	{
		//orientation part
		for(int j=1; j<jointssize-5; j++)
		{
			if( obj->joints[j]->isAncestor(joint) || obj->joints[j] == joint )
			{
				w = obj->joints[j]->getGlobalRotationAxis(frame).normalized();

				constraintJacobian.block(3, j+5, 3, 1) = w;
			}
			else
			{
				constraintJacobian.block(3, j+5, 3, 1) = Vector3d::Zero();
			}
		}
	}
}


void IKSolver::makeParts()
{
	Joint *tempPart = obj->jointMap[goalname];

	for( ; tempPart->name != obj->rootJoint->name ; tempPart = tempPart->parent)
	//for( ; tempPart != obj->rootJoint ; tempPart = tempPart->parent)
	{
		if(tempPart->constraintAxis.norm()>0.5)
			Parts.push_back(tempPart);
	}
}

void IKSolver::setConstraintJoint(Joint *joint)
{
	int frame = obj->getCurrentFrame();
	
	constraints.push_back(joint);
	constraintPosition.push_back(joint->getGlobalPosition(frame));
	constraintOrientation.push_back(joint->getGlobalOrientation(frame));
}

void IKSolver::releaseConstraintJoint(Joint *joint)
{
	std::vector<Vector3d, aligned_allocator<Vector3d> >::iterator posIt = constraintPosition.begin();
	std::vector<Quaterniond, aligned_allocator<Quaterniond> >::iterator oriIt =  constraintOrientation.begin();
	for(std::vector<Joint*>::iterator it = constraints.begin(); it != constraints.end(); it++)
	{
		if((*it) == joint)
		{
			constraints.erase(it);
			constraintPosition.erase(posIt);
			constraintOrientation.erase(oriIt);
			break;
		}
		posIt++;
		oriIt++;
	}
}

double IKSolver::computepenalty()
{
	const int frame = obj->getCurrentFrame();
	
	//distance penalty
	double dist_penalty = weightPos * (goal - obj->jointMap[goalname]->getGlobalPosition(frame)).squaredNorm();
    Quaterniond curQuat = obj->jointMap[goalname]->getGlobalOrientation(frame);
    dist_penalty += weightAng * diffQuat(goalQuat, curQuat).squaredNorm();

	/*
	//angle limit penalty
	double limit_penalty=0;
	double limit_center = 0;
	double limit_sub = 0;
	for(int i=0; i<Parts.size(); i++)
	{
		limit_sub = (Parts.at(i)->constraintAngle[1] - Parts.at(i)->constraintAngle[0]) /2;
		limit_center = ( Parts.at(i)->constraintAngle[0] + Parts.at(i)->constraintAngle[1] ) /2;
		limit_penalty += std::pow( (Parts.at(i)->getAngle() - limit_center)/limit_sub, 4);
	}
	limit_penalty /= LIMITCOEFF;
	*/


	//self collision penalty
	// TODO:

	//return dist_penalty + limit_penalty;
	return sqrt(dist_penalty);
}

double IKSolver::computePenalty(Eigen::VectorXd &p)
{
	const int frame = obj->getCurrentFrame();

	//1. copy frame to ikframe;
	obj->backUpMotionForIk(frame);

	//2. modify this frame
	for(int i=0; i < Parts.size(); i++)
	{
		double angle = Parts.at(i)->getAngle(frame) + p(i);
		if(angle < Parts.at(i)->constraintAngle[0])
			angle = Parts.at(i)->constraintAngle[0];
		else if(angle > Parts.at(i)->constraintAngle[1])
			angle = Parts.at(i)->constraintAngle[1];

		Quaterniond q = 
			Quaterniond(Eigen::AngleAxisd(angle, Parts.at(i)->constraintAxis));

		Parts.at(i)->setRotation(frame,	q);
	}	
		
	//distance penalty
	double dist_penalty = (goal - obj->jointMap[goalname]->getGlobalPosition(frame)).squaredNorm();
    Quaterniond curQuat = obj->jointMap[goalname]->getGlobalOrientation(frame);
    dist_penalty += ::diffQuat(goalQuat, curQuat).squaredNorm();

	//3. restore this frame from ikframe
	obj->restoreMotionForIk(frame);

	return sqrt(dist_penalty);
}

double IKSolver::computePenaltyGradient(Eigen::VectorXd &p, Eigen::VectorXd &dp)
{
	double dist;

	const int frame = obj->getCurrentFrame();
	Eigen::VectorXd b;
	
	//1. copy frame to ikframe;
	obj->backUpMotionForIk(frame);

	//2. modify this frame
	for(int i=0; i < Parts.size(); i++)
	{
		double angle = Parts.at(i)->getAngle(frame) + p(i);
		if(angle < Parts.at(i)->constraintAngle[0])
			angle = Parts.at(i)->constraintAngle[0];
		else if(angle > Parts.at(i)->constraintAngle[1])
			angle = Parts.at(i)->constraintAngle[1];

		Quaterniond q = 
			Quaterniond(Eigen::AngleAxisd(angle, Parts.at(i)->constraintAxis));

		Parts.at(i)->setRotation(frame,	q);
	}	

	Vector3d btemp = ( goal - obj->jointMap[goalname]->getGlobalBoundingBoxPosition(frame) );
	Quaterniond orien = obj->jointMap[goalname]->getGlobalOrientation(frame);
	Vector3d bQuatTemp = diffQuat(goalQuat,orien);

	b.resize(6);
	b.segment(0,3) = btemp;
	b.segment(3,3) = bQuatTemp;

	dist = computeJacobian();
	dp = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	//3. restore this frame from ikframe
	obj->restoreMotionForIk(frame);

	return sqrt(dist);
}

void IKSolver::run_gradient_descent(std::string _goalName, Eigen::Vector3d &goalpos, Eigen::Quaterniond &goalOri)
{
	Parts.clear();
	goalname = _goalName;
	goal = goalpos;
	goalQuat = goalOri;
	makeParts();

	int iter = 0;
	double f;
	Eigen::VectorXd p;
	p.resize(Parts.size());
	gradient_descent(p, Parts.size(), 0.1, iter, f, this);
	
	const int frame = obj->getCurrentFrame();
	double angle;
	for(int i=0; i < Parts.size(); i++)
	{
		angle = Parts.at(i)->getAngle(frame) + p(i);

		//if(angle < Parts.at(i)->constraintAngle[0])
		//	angle = Parts.at(i)->constraintAngle[0];
		//else if(angle > Parts.at(i)->constraintAngle[1])
		//	angle = Parts.at(i)->constraintAngle[1];
		Quaterniond q = 
			Quaterniond(Eigen::AngleAxisd(angle, Parts.at(i)->constraintAxis));

		Parts.at(i)->setRotation(frame,	q);
	}	
}
