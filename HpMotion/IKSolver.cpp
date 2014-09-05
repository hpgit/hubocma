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
	
	
	Parts.clear();
	goalname = PartName;
	goal = goalPosition;
	goalQuat = goalOrientation;
	makeParts();

	for(int j=0; j<IKMOTIONSIZE && computepenalty() > IKEPSILON; j++)
	{
		solve(dp, 10);
		for(int i=0; i < Parts.size(); i++)
		{
			angle = Parts.at(i)->getAngle(frame) + dp(i);

			if(angle < Parts.at(i)->constraintAngle[0])
				angle = Parts.at(i)->constraintAngle[0];
			else if(angle > Parts.at(i)->constraintAngle[1])
				angle = Parts.at(i)->constraintAngle[1];
			Quaterniond q = 
				Quaterniond(Eigen::AngleAxisd(angle, Parts.at(i)->constraintAxis));

			Parts.at(i)->setRotation(frame,	q);
		}	
	}
}

void IKSolver::solve(Eigen::VectorXd &dp, int nStep)
{
	const int frame = obj->getCurrentFrame();
	Eigen::MatrixXd Jt, Jp;
	Eigen::VectorXd bp, b;
	Vector3d btemp = ( goal - obj->jointMap[goalname]->getGlobalPosition(frame) ) / nStep;
	Quaterniond orien = obj->jointMap[goalname]->getGlobalOrientation(frame);
	Vector3d bQuatTemp = ::diffQuat(goalQuat,orien);
	btemp.normalize();

	b.resize(3);
	
	b.segment(0,3) = btemp;
	b.segment(3,3) = bQuatTemp;
	//b.head(3) = btemp;
	//b.tail(3) = bQuatTemp;

	computeJacobian();
	dp = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

void IKSolver::computeJacobian()
{
	const int frame = obj->getCurrentFrame();

	jacobian.resize(6, Parts.size());

	for(int i=0; i<Parts.size(); i++)
	{
		//Vector3d endEffV = rotateAxis.at(i) * ( GoalPosition - Parts.at(i)->getGlobalPosition(obj->frameTotal-1) );
		Vector3d endEffV = Parts.at(i)->getGlobalRotationAxis(frame).cross( 
			( obj->jointMap[goalname]->getGlobalPosition(frame) - Parts.at(i)->getGlobalPosition(frame)) );
		Vector3d endEffW = Parts.at(i)->constraintAxis;

		jacobian.block(0, i, 3, 1) = endEffV;
		jacobian.block(3, i, 3, 1) = endEffW;
		//jacobian(0, i)= Parts.at(i)->IKweight * endEffV.x();
		//jacobian(1, i)= Parts.at(i)->IKweight * endEffV.y();
		//jacobian(2, i)= Parts.at(i)->IKweight * endEffV.z();
		//jacobian(3, i)= Parts.at(i)->IKweight * endEffW.x();
		//jacobian(4, i)= Parts.at(i)->IKweight * endEffW.y();
		//jacobian(5, i)= Parts.at(i)->IKweight * endEffW.z();
	}	
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
			//fullJacobian(i  ,3)=valongx[0];
			//fullJacobian(i+1,3)=valongx[1];
			//fullJacobian(i+2,3)=valongx[2];
			//fullJacobian(i  ,4)=valongy[0];
			//fullJacobian(i+1,4)=valongy[1];
			//fullJacobian(i+2,4)=valongy[2];
			//fullJacobian(i  ,5)=valongz[0];
			//fullJacobian(i+1,5)=valongz[1];
			//fullJacobian(i+2,5)=valongz[2];
			
		}

		{
			for(int i=3*bodiessize; i < 6*bodiessize; i+=3)
			{
				fullJacobian.block(i, 3, 3, 1) = Vector3d::UnitX();
				fullJacobian.block(i, 4, 3, 1) = Vector3d::UnitY();
				fullJacobian.block(i, 5, 3, 1) = Vector3d::UnitZ();
				//fullJacobian(i  , 3)= 1;
				//fullJacobian(i+1, 3)= 0;
				//fullJacobian(i+2, 3)= 0;
				//fullJacobian(i  , 4)= 0;
				//fullJacobian(i+1, 4)= 1;
				//fullJacobian(i+2, 4)= 0;
				//fullJacobian(i  , 5)= 0;
				//fullJacobian(i+1, 5)= 0;
				//fullJacobian(i+2, 5)= 1;
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
				//fullJacobian(i  , j+5)= v[0];
				//fullJacobian(i+1, j+5)= v[1];
				//fullJacobian(i+2, j+5)= v[2];
			}
			else
			{
				fullJacobian.block(i, j+5, 3, 1) = Vector3d::Zero();
				//fullJacobian(i  , j+5)= 0;
				//fullJacobian(i+1, j+5)= 0;
				//fullJacobian(i+2, j+5)= 0;
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
					//fullJacobian(i  , j+5)= w[0];
					//fullJacobian(i+1, j+5)= w[1];
					//fullJacobian(i+2, j+5)= w[2];
				}
				else
				{
					fullJacobian.block(i, j+5, 3, 1) = Vector3d::Zero();
					//fullJacobian(i  , j+5)=0;
					//fullJacobian(i+1, j+5)=0;
					//fullJacobian(i+2, j+5)=0;
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
		//constraintJacobian(0,3)=valongx[0];
		//constraintJacobian(1,3)=valongx[1];
		//constraintJacobian(2,3)=valongx[2];
		//constraintJacobian(0,4)=valongy[0];
		//constraintJacobian(1,4)=valongy[1];
		//constraintJacobian(2,4)=valongy[2];
		//constraintJacobian(0,5)=valongz[0];
		//constraintJacobian(1,5)=valongz[1];
		//constraintJacobian(2,5)=valongz[2];
		//

		fullJacobian.block(3, 3, 3, 1) = Vector3d::UnitX();
		fullJacobian.block(3, 4, 3, 1) = Vector3d::UnitY();
		fullJacobian.block(3, 5, 3, 1) = Vector3d::UnitZ();
		//constraintJacobian(3,3)= 1;
		//constraintJacobian(4,3)= 0;
		//constraintJacobian(5,3)= 0;
		//constraintJacobian(3,4)= 0;
		//constraintJacobian(4,4)= 1;
		//constraintJacobian(5,4)= 0;
		//constraintJacobian(3,5)= 0;
		//constraintJacobian(4,5)= 0;
		//constraintJacobian(5,5)= 1;
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
			//constraintJacobian(0, j+5)= v[0];
			//constraintJacobian(1, j+5)= v[1];
			//constraintJacobian(2, j+5)= v[2];
		}
		else
		{
			constraintJacobian.block(0, j+5, 3, 1) = Vector3d::Zero();
			//constraintJacobian(0, j+5)= 0;
			//constraintJacobian(1, j+5)= 0;
			//constraintJacobian(2, j+5)= 0;
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
				//constraintJacobian(3,j+5)= w[0];
				//constraintJacobian(4,j+5)= w[1];
				//constraintJacobian(5,j+5)= w[2];
			}
			else
			{
				constraintJacobian.block(3, j+5, 3, 1) = Vector3d::Zero();
				//constraintJacobian(3,j+5)=0;
				//constraintJacobian(4,j+5)=0;
				//constraintJacobian(5,j+5)=0;
			}
		}
	}
}


void IKSolver::makeParts()
{
	Joint *tempPart = obj->jointMap[goalname];

	for( ; tempPart->parent->name != obj->rootJoint->name ; tempPart = tempPart->parent)
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

/*
void IKSolver::updateGoldenSection(int dim, double x[], double x2[], double x4[], double d[], double &a1, double &a2, double &a3)
{
	double a4 = ( a2 - a1 > a3 - a2) ? 
		a2 + (a1-a2)/(1+GOLDENRATIO) : a2 + (a3-a2)/(1+GOLDENRATIO);
	
	for(int i=0; i<dim; i++)
		x2[i] = x[i] + a2 * d[i];
	for(int i=0; i<dim; i++)
		x4[i] = x[i] + a4 * d[i];

	if( computepenalty(x4) > computepenalty(x2))
	{
		if(a2 < a4) a3 = a4;
		else a1 = a4;
	}
	else
	{
		if(a2 < a4)
		{
			a1 = a2;
			a2 = a4;
		}
		else
		{
			a3 = a2;
			a2 = a4;
		}
	}
	
}

double IKSolver::findMinAlongGradient(int dim, double x0[], double d[], double xmin[], double xmax[])
{
	double a = 0;
	double amaxForXmin = DBL_MAX;
	double amaxForXmax = -DBL_MAX;
	double amax = 0;
	
	if(dim <=0 )
		return 0;

	double *x2 = new double [dim];
	double *x4 = new double [dim];


	if(xmin != NULL)
	{
		double temp = 0;
		for(int i=0; i < dim; i++)
		{
			if( std::abs(d[i]) > 0 )
			{
				temp = (xmin[i]-x0[i]) / d[i];
				if(temp > 0 && temp < amaxForXmin)
					amaxForXmin = temp;
			}
		}
	}

	if(xmax != NULL)
	{
		double temp = 0;
		for(int i=0; i < dim; i++)
		{
			if( std::abs(d[i]) > 0 )
			{
				temp = (xmax[i]-x0[i]) / d[i];
				if(temp > 0 && temp < amaxForXmax)
					amaxForXmax = temp;
			}
		}
	}

	amax = amaxForXmin < amaxForXmax ? amaxForXmin:amaxForXmax;
	
	unsigned iter = 0;
	double a1 = 0, a2 = amax / (1+GOLDENRATIO), a3 = amax;		// for golden ratio
	
	for(iter = 0; iter < MAXITER ; iter++)
	{
		updateGoldenSection(dim, x0, x2, x4, d, a1, a2, a3);
		if( std::abs(a2-a1) < DBL_EPSILON) 
			break;
	}
	
	delete []x2;
	delete []x4;

	return a2;
}
*/

double IKSolver::computepenalty()
{
	Joint *EndEffector = obj->jointMap[goalname];
	
	//distance penalty
	double dist_penalty = (goal - EndEffector->getGlobalPosition(obj->frameTotal-1)).norm();

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
	return dist_penalty;
}

/*
void IKSolver::run_gradient_descent(string PartName, Vector3d &GoalPosition)
{
	Eigen::VectorXd dp;
	Joint *EndEffector = obj->jointMap[PartName];
	Motion IKmotion;
	double alpha = 0;
	double *angles = NULL, *dangles = NULL;
	double *anglemax = NULL, *anglemin = NULL;
	
	Parts.clear();
	goalname = PartName;
	goal = GoalPosition;
	makeParts();

	angles = new double [Parts.size()];
	dangles = new double [Parts.size()];
	anglemax = new double [Parts.size()];
	anglemin = new double [Parts.size()];
	
	for(unsigned i=0; i<Parts.size(); i++)
	{
		IKmotion.setTranslation(Parts.at(i)->IKmotion.at(Parts.at(i)->IKmotion.size()-1)->getTranslation());
		IKmotion.setRotation(Parts.at(i)->IKmotion.at(Parts.at(i)->IKmotion.size()-1)->getRotation());
		Parts.at(i)->IKmotion.push_back(new Motion(IKmotion.getTranslation(), IKmotion.getRotation()));
	}
	for(unsigned i=0; i<Parts.size(); i++)
		anglemin[i] = Parts.at(i)->constraintAngle[0];
	for(unsigned i=0; i<Parts.size(); i++)
		anglemax[i] = Parts.at(i)->constraintAngle[1];
	for(unsigned i=0; i<Parts.size(); i++)
		angles[i] = Parts.at(i)->IKmotion.at(Parts.at(i)->IKmotion.size()-1)->getAngle();

	for(unsigned j=0; j<IKMOTIONSIZE && computepenalty(angles) > IKEPSILON ; j++)
	{
		solve(dp, 10);
		dp(dangles);
		alpha = findMinAlongGradient(Parts.size(), angles, dangles, anglemin, anglemax);
		for(unsigned i=0; i<Parts.size(); i++)
			angles[i] += alpha * dangles[i];
		for(unsigned i=0; i<Parts.size(); i++)
		{
			if(angles[i] < Parts.at(i)->constraintAngle[0])
				angles[i] = Parts.at(i)->constraintAngle[0];
			else if(angles[i] > Parts.at(i)->constraintAngle[1])
				angles[i] = Parts.at(i)->constraintAngle[1];
		}
	}
	delete []angles;
	delete []dangles;

}
*/
