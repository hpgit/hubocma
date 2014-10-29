#include "HuboMotionData.h"
#include "HpMotionMath.h"
#include <fstream>
#include <iostream>

Eigen::MatrixXd HuboMotionData::naturalToBsplineFourPoint;
int HuboMotionData::naturalToBsplineFourPointIsReady = 0;

int HuboMotionData::import(const char* _filename, int firstFrame, int lastFrame)
{
	std::ifstream fin;
	fin.open(_filename);

	string str;
	string frameString; // "Frame"
	int totalFrame;
	int frame;
	double t[3];
	double xyz[3];
	double angle;

	fin >> str;
	fin >> totalFrame;

	if (lastFrame == 0 || lastFrame > totalFrame-1)
		lastFrame = totalFrame;
	else
		totalFrame = lastFrame+1;

	setMotionSize(totalFrame-firstFrame);
	
	setFrameRate(30.0);

	// "Frame"
	fin >> frameString;

	for (int i = 0; i < totalFrame; i++)
	{
		
		fin >> frame;
		if (i + 1 != frame) break;

		{
			// Hip 
			fin >> str;
			fin >> t[0] >> t[1] >> t[2];
			fin >> xyz[0] >> xyz[1] >> xyz[2];
			if (i >= firstFrame)
			{
				Vector3d temptrans(t);
				Quaterniond temprot(
					Eigen::AngleAxisd(xyz[2], Vector3d::UnitZ())
					*Eigen::AngleAxisd(xyz[1], Vector3d::UnitY())
					*Eigen::AngleAxisd(xyz[0], Vector3d::UnitX())
					);
				jointMap["Hip"]->setTranslation(i-firstFrame, temptrans);
				jointMap["Hip"]->setRotation(
					i-firstFrame,
					temprot
					);
			}
		}

		fin >> str; // second joint name

		while ( !fin.eof() && strncmp(frameString.data(), str.data(), 5)) // until finding "Frame"
		{
			fin >> angle;
			if (i >= firstFrame)
				jointMap[str]->setAngle(i - firstFrame, angle);
			fin >> str;
		}
	}
	fin.close();

	setCurrentFrame(0);
	//makeAllJointAngleSpline();
	return 0;
}

int HuboMotionData::importContactAnnotation(const char* _filename, int firstFrame, int lastFrame)
{
	std::ifstream fin;
	fin.open(_filename);

	char buf[256];
	int frame = 0;
	int rightFoot = 1;
	int leftFoot = 1;
	

	while (!fin.eof())
	{
		fin.getline(buf, 256);
		if (buf[0] == '#')
			continue;
		else
		{
			if (frame >= firstFrame && (lastFrame == 0 || frame <= lastFrame) )
			{
				while (rFootContact.size() <= frame-firstFrame)
				{
					rFootContact.push_back(rightFoot);
					lFootContact.push_back(leftFoot);
				}
			}

			sscanf(buf, "%d %d %d", &frame, &rightFoot, &leftFoot);

			if (lastFrame != 0 && frame > lastFrame)
				break;
		}
	}

	while (rFootContact.size() <= firstFrame - lastFrame)
	{
		rFootContact.push_back(rFootContact.back());
		lFootContact.push_back(lFootContact.back());
	}

	return 0;
}

int HuboMotionData::importContactPeriodAnnotation(const char* _filename, int firstFrame, int lastFrame)
{
	std::stringstream ss;
	std::ifstream fin;
	std::string str;
	fin.open(_filename);

	char buf[256];
	char temp[256];
	int frame = 0;
	int period = 0;
	

	while (!fin.eof())
	{
		fin.getline(buf, 256);
		if (buf[0] == '#')
			continue;
		else if (buf[0] == 'S')
		{
			ss.clear();
			ss<<buf;
			ss >> str >> frame;
			startPeriodFrame = frame - firstFrame;
		}
		else if (buf[0] == 'T')
		{
			ss.clear();
			ss<<buf;
			ss >> str >> period;
			contactPeriod = period;
		}
	}
	
	return 0;
}

int HuboMotionData::importSkeleton(const char* _filename)
{
	std::ifstream in;
	in.open(_filename);

	Joint *joint;
	string str;
	string rotateAxis;    
	double minAngle, maxAngle;

	while(!in.eof())
	{
		in>>str;
		in>>str;
		while(str != "}")
		{
			if(str == "name")
			{
				in>>str;
				joint = new Joint(str);
				jointMap[str] = joint;
				jointNames.push_back(str);
			}
			if(str == "id")
			{
				int id;
				in >> id;
				jointIdMap[joint->name] = id;
			}
			else if(str == "file")
			{
				char buf[60];
				in>>str;
				strcpy(buf, str.data());
				joint->importOBJ(buf);
				joint->hasObjects = 1;
			}
			else if( str == "translation")
			{
				double offset[3];
				in >> offset[0] >> offset[1] >> offset[2];
				Vector3d tempoffset(offset);
				joint->setOffset(tempoffset);
			}
			else if( str == "RotateAxis")
			{
				in>>rotateAxis;
				joint->setConstraintAxis(rotateAxis);
				activeJoints.push_back(joint);
			}
			else if(str == "Constraint")
			{
				in>>minAngle>>maxAngle;
				joint->setConstraintAngle(minAngle, maxAngle);
			}
			else if(str == "parent")
			{
				in>>str;
				jointMap[str]->setChild(joint);
			}
			else if(str == "com")
			{
				double temp[3];
				in >> temp[0] >> temp[1] >> temp[2];
				joint->childBodyCom = Vector3d(temp);
			}
			else if(str == "inertia")
			{
				double temp[9];
				for(int i=0; i<9 ; i++)
					in >> temp[i];
				joint->childBodyInertia <<
					temp[0], temp[1], temp[2], 
					temp[3], temp[4], temp[5], 
					temp[6], temp[7], temp[8];
			}
			else if(str == "mass")
			{
				double temp;
				in >> temp;
				joint->childBodyMass = temp;
			}
			in>>str;
		}
		if(joint != NULL)
		{
			joints.push_back(joint);
			joint = NULL;
		}
	}
	rootJoint = joints[0];

	in.close();

	return 0;
}

void HuboMotionData::setOBJon()
{
	for(int i = 0; i < joints.size(); i++)
		joints.at(i)->setOBJon(1);
}

void HuboMotionData::unsetOBJon()
{
	for(int i = 0; i < joints.size(); i++)
		joints.at(i)->setOBJon(0);
}
/*
void HuboMotionData::makeAllJointAngleSpline()
{
	for (int i = 0; i < activeJoints.size(); i++)
		activeJoints.at(i)->makeAngleSpline();
}
*/

double HuboMotionData::getPeriodPhaseInTime(double time)
{
	double tt = (time - startPeriodFrame*getFrameTime()) / (contactPeriod*getFrameTime());
	if (tt < 0)
		return -1;
	return tt - (int)tt;
}

Vector3d HuboMotionData::getHuboComGlobalPositionInTime(double t)
{
	double totalMass = 0;
	Vector3d sumOfMoment(0,0,0);
	int frame = (int)(t / getFrameTime());
	double tt = t / getFrameTime() - frame;

	for(int i = 0; i < joints.size(); i++)
	{
		totalMass += joints.at(i)->childBodyMass;
		sumOfMoment += joints.at(i)->childBodyMass * joints.at(i)->getGlobalBoundingBoxPosition(frame, tt);
	}

	return sumOfMoment / totalMass;
}

Vector3d HuboMotionData::getHuboComGlobalPosition(int frame)
{
	double totalMass = 0;
	Vector3d sumOfMoment(0,0,0);

	for(int i = 0; i < joints.size(); i++)
	{
		totalMass += joints.at(i)->childBodyMass;
		sumOfMoment += joints.at(i)->childBodyMass * joints.at(i)->getGlobalBoundingBoxPosition(frame);
	}

	return sumOfMoment / totalMass;
}

Vector3d HuboMotionData::getHuboComGlobalPosition()
{
	double totalMass = 0;
	Vector3d sumOfMoment(0,0,0);

	for(int i = 0; i < joints.size(); i++)
	{
		totalMass += joints.at(i)->childBodyMass;
		sumOfMoment += joints.at(i)->childBodyMass * joints.at(i)->getGlobalBoundingBoxPosition(frame);
	}

	return sumOfMoment / totalMass;
}

Vector3d HuboMotionData::getHipJointGlobalPositionInTime(double t)
{
	/*
	//TODO:fixit
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}
	UniformBspline sp;
	Vector3d hippos[4];
	Eigen::VectorXd spPoint[3];
	Eigen::VectorXd naturalPoint[3];
	Eigen::VectorXd temp;

	Vector3d hipposSpPoint[6];

	for (int i = 0; i < 6; i++)
	{
		naturalPoint[i].resize(6);
		naturalPoint[i].setZero();
	}
	
	hippos[0] = hip->getGlobalPosition(_frame-1 >=0 ? _frame - 1 : 0);
	hippos[1] = hip->getGlobalPosition(_frame);
	hippos[2] = hip->getGlobalPosition(_frame + 1 < frameTotal? _frame+1: frameTotal-1);
	hippos[3] = hip->getGlobalPosition(_frame + 2 < frameTotal? _frame+2: frameTotal-1);

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++)
			naturalPoint[j](i + 1) = hippos[i](j);

	for (int i = 0; i < 3; i++)
		spPoint[i] = naturalToBsplineFourPoint * naturalPoint[i];

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 3; j++)
			hipposSpPoint[i](j) = spPoint[j](i);

	sp.clear();
	for (int i = 0; i < 6; i++)
	{
		temp = hipposSpPoint[i];
		sp.setControlPoint(i - 1, temp);
	}
	return sp.getValue(time).head(3);
	*/
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame >= frameTotal-1)
		return hip->getGlobalBoundingBoxPosition(frameTotal-1);
	Vector3d q1 = hip->getGlobalBoundingBoxPosition(_frame);
	Vector3d q2 = hip->getGlobalBoundingBoxPosition(_frame+1);
	return q1*(1-time) + q2*time;
}

Quaterniond HuboMotionData::getHipJointGlobalOrientationInTime(double t)
{
	//TODO:fixit
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}
	Quaterniond q1 = hip->getGlobalOrientation(_frame);
	Quaterniond q2 = hip->getGlobalOrientation(_frame+1);
	return q1.slerp(time, q2);
}


Vector3d HuboMotionData::getHipJointVelInHuboMotionInTime(double t)
{
	/*
	//TODO:fixit
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}
	UniformBspline sp;
	Vector3d hippos[4];
	Eigen::VectorXd spPoint[3];
	Eigen::VectorXd naturalPoint[3];
	Eigen::VectorXd temp;

	Vector3d hipposSpPoint[6];

	for (int i = 0; i < 6; i++)
	{
		naturalPoint[i].resize(6);
		naturalPoint[i].setZero();
	}
	
	hippos[0] = hip->getGlobalPosition(_frame-1 >=0 ? _frame - 1 : 0);
	hippos[1] = hip->getGlobalPosition(_frame);
	hippos[2] = hip->getGlobalPosition(_frame + 1 < frameTotal? _frame+1: frameTotal-1);
	hippos[3] = hip->getGlobalPosition(_frame + 2 < frameTotal? _frame+2: frameTotal-1);

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++)
			naturalPoint[j](i + 1) = hippos[i](j);

	for (int i = 0; i < 3; i++)
		spPoint[i] = naturalToBsplineFourPoint * naturalPoint[i];

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 3; j++)
			hipposSpPoint[i](j) = spPoint[j](i);

	sp.clear();
	for (int i = 0; i < 6; i++)
	{
		temp = hipposSpPoint[i];
		sp.setControlPoint(i - 1, temp);
	}
	return sp.getDifferentialValue(time).head(3);
	*/
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}

	Vector3d q[2];

	double frameTime = getFrameTime();
	if (_frame + 1 < frameTotal)
	{
		q[0] = hip->getGlobalPosition(_frame);
		q[1] = hip->getGlobalPosition(_frame+1);
	}
	else
	{
		q[0] = hip->getGlobalPosition(_frame >= 0 ? _frame-1: 0);
		q[1] = hip->getGlobalPosition(_frame);
	}

	return (q[1]-q[0])*frameRate;

}
Vector3d HuboMotionData::getHipJointAngVelInHuboMotionInTime(double t)
{
	//TODO:fixit
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}

	Quaterniond q[3];

	double frameTime = getFrameTime();
	if (_frame + 1 < frameTotal)
	{
		q[0] = hip->getGlobalOrientation(_frame);
		q[1] = hip->getGlobalOrientation(_frame+1);
	}
	else
	{
		q[0] = hip->getGlobalOrientation(_frame >= 0 ? _frame-1: 0);
		q[1] = hip->getGlobalOrientation(_frame);
	}

	Quaterniond qinv = q[0].inverse();
	Quaterniond qqq = quaterMinus(q[1], q[0])*qinv;
	Quaterniond qq = quaterRealMult(qqq, frameRate);
	return 2*qq.vec();

}

Vector3d HuboMotionData::getHipJointAccelInHuboMotionInTime(double t)
{
	/*
	//TODO:fixit
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}
	UniformBspline sp;
	Vector3d hippos[4];
	Eigen::VectorXd spPoint[3];
	Eigen::VectorXd naturalPoint[3];
	Eigen::VectorXd temp;

	Vector3d hipposSpPoint[6];

	for (int i = 0; i < 6; i++)
	{
		naturalPoint[i].resize(6);
		naturalPoint[i].setZero();
	}
	
	hippos[0] = hip->getGlobalPosition(_frame-1 >=0 ? _frame - 1 : 0);
	hippos[1] = hip->getGlobalPosition(_frame);
	hippos[2] = hip->getGlobalPosition(_frame + 1 < frameTotal? _frame+1: frameTotal-1);
	hippos[3] = hip->getGlobalPosition(_frame + 2 < frameTotal? _frame+2: frameTotal-1);

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++)
			naturalPoint[j](i + 1) = hippos[i](j);

	for (int i = 0; i < 3; i++)
		spPoint[i] = naturalToBsplineFourPoint * naturalPoint[i];

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 3; j++)
			hipposSpPoint[i](j) = spPoint[j](i);

	sp.clear();
	for (int i = 0; i < 6; i++)
	{
		temp = hipposSpPoint[i];
		sp.setControlPoint(i - 1, temp);
	}
	return sp.getSecondDerivationValue(time).head(3);
	*/
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	Vector3d v[3];
	if (_frame >= frameTotal-1)
	{
		v[0] = hip->getGlobalPosition(_frame-2);
		v[1] = hip->getGlobalPosition(_frame-1);
		v[2] = hip->getGlobalPosition(_frame);
	}
	else if (_frame >= frameTotal-2)
	{
		v[0] = hip->getGlobalPosition(_frame-1);
		v[1] = hip->getGlobalPosition(_frame);
		v[2] = hip->getGlobalPosition(_frame+1);
	}
	else
	{
		v[0] = hip->getGlobalPosition(_frame);
		v[1] = hip->getGlobalPosition(_frame+1);
		v[2] = hip->getGlobalPosition(_frame+2);
	}

	return (v[2]-2*v[1]+v[0])*frameRate*frameRate;


}

Vector3d HuboMotionData::getHipJointAngAccelInHuboMotionInTime(double t)
{
	//TODO:fixit
	Joint* hip = jointMap["Hip"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;
	if (_frame == frameTotal)
	{
		_frame--;
		time = 1.0;
	}

	Quaterniond q[3];

	double frameTime = getFrameTime();

	q[0] = hip->getGlobalOrientation(_frame-1 >= 0 ? _frame-1: 0);
	q[1] = hip->getGlobalOrientation(_frame);
	q[2] = hip->getGlobalOrientation(_frame+1 <frameTotal ? _frame+1:frameTotal-1);

	Quaterniond qinv = q[1].inverse();

	Quaterniond q2minusq1 = quaterMinus(q[2], q[1]);
	Quaterniond q1minusq0 = quaterMinus(q[1], q[0]);
	Quaterniond q2minusq1minusq0 = quaterMinus(q2minusq1, q1minusq0);	

	Quaterniond dq = quaterRealMult(q2minusq1, frameRate);
	Quaterniond ddq = quaterRealMult(q2minusq1minusq0, frameRate*frameRate );

	Quaterniond ddqqinv = ddq*qinv;
	Quaterniond ddq2 = quaterRealMult(ddqqinv, 2);

	Quaterniond dqqinvsq = dq*qinv*dq*qinv;
	Quaterniond dqdq = quaterRealMult(dqqinvsq, 2);

	return quaterMinus(ddq2, dqdq).vec();
}

void HuboMotionData::getAllHipStateInTime(
	double time,
	Eigen::Vector3d &Pos,
	Eigen::Quaterniond &Ori,
	Eigen::Vector3d &Vel,
	Eigen::Vector3d &AngVel,
	Eigen::Vector3d &Accel,
	Eigen::Vector3d &AngAccel)
{
	Pos = getHipJointGlobalPositionInTime(time);
	Ori = getHipJointGlobalOrientationInTime(time);
	Vel = getHipJointVelInHuboMotionInTime(time);
	AngVel = getHipJointAngVelInHuboMotionInTime(time);
	Accel = getHipJointAccelInHuboMotionInTime(time);
	AngAccel = getHipJointAngAccelInHuboMotionInTime(time);

}

Eigen::Vector3d HuboMotionData::getFootCenter()
{
	Joint* footr = jointMap["RAR"];
	Joint* footl = jointMap["LAR"];

	Eigen::Vector3d qr, ql;
	Eigen::Vector3d q(0,0,0);

	qr = footr->getGlobalBoundingBoxPosition(this->frame);
	ql = footl->getGlobalBoundingBoxPosition(this->frame);

	if(qr.y() <0.1 && ql.y() < 0.1)
	{
		q = (qr+ql)/2;
	}
	else if(qr.y() < 0.1)
	{
		q = qr;
	}
	else if(ql.y() < 0.1)
	{
		q = ql;
	}
	else
	{
		q = (qr+ql)/2;
	}

	q.y() = 0.;
	return q;
}


Eigen::Vector3d HuboMotionData::getFootCenterInTime(double t)
{
	Joint* footr = jointMap["RAR"];
	Joint* footl = jointMap["LAR"];
	double tt = t * frameRate;
	int _frame = (int)tt;
	double time = tt - _frame;

	Eigen::Vector3d qr, ql;
	Eigen::Vector3d q(0,0,0);

	if (_frame >= frameTotal-1)
	{
		qr = footr->getGlobalBoundingBoxPosition(frameTotal-1);
		ql = footl->getGlobalBoundingBoxPosition(frameTotal-1);
	}
	else
	{
		Vector3d qr1 = footr->getGlobalBoundingBoxPosition(_frame);
		Vector3d qr2 = footl->getGlobalBoundingBoxPosition(_frame+1);
		Vector3d ql1 = footr->getGlobalBoundingBoxPosition(_frame);
		Vector3d ql2 = footl->getGlobalBoundingBoxPosition(_frame+1);
		qr = qr1*(1-time) + qr2*time;
		ql = ql1*(1-time) + ql2*time;
	}

	if(qr.y() <0.1 && ql.y() < 0.1)
	{
		q = (qr+ql)/2;
	}
	else if(qr.y() < 0.1)
	{
		q = qr;
	}
	else if(ql.y() < 0.1)
	{
		q = ql;
	}
	else
	{
		q = (qr+ql)/2;
	}

	q.y() = 0.;
	return q;
}

void HuboMotionData::getAllAngleInHuboMotion(int frame, Eigen::VectorXd &angles)
{
	angles.resize(26);
	
	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngle(frame);
}

void HuboMotionData::getAllAngleInHuboMotion(int firstFrame, double t, Eigen::VectorXd &angles)
{
	angles.resize(26);

	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngle(firstFrame, t);
}

void HuboMotionData::getAllAngleRateInHuboMotion(int frame, Eigen::VectorXd &angleRates)
{
	angleRates.resize(26);

	for (int i = 0; i < activeJoints.size(); i++)
		angleRates(i) = activeJoints.at(i)->getAngleRateInTime(frame);

	//Eigen::VectorXd firstAngles, lastAngles;
	//if (frame >= getMotionSize() - 1)
	//	frame = getMotionSize() - 2;
	//getAllAngleInHuboMotion(frame, firstAngles);
	//getAllAngleInHuboMotion(frame, lastAngles);
	//angleRates = (lastAngles - firstAngles)/getFrameTime();
}

void HuboMotionData::getAllAngleRateInHuboMotion(int firstFrame, double t, Eigen::VectorXd &angleRates)
{
	angleRates.resize(26);

	for (int i = 0; i < activeJoints.size(); i++)
		angleRates(i) = activeJoints.at(i)->getAngleRateInTime(firstFrame+t);

	//angleRates.resize(26);
	//Eigen::VectorXd firstAngles, lastAngles;
	//if (firstFrame >= getMotionSize() - 1)
	//	firstFrame = getMotionSize() - 2;
	//getAllAngleInHuboMotion(firstFrame, firstAngles);
	//getAllAngleInHuboMotion(firstFrame, lastAngles);
	//angleRates = (lastAngles - firstAngles)/getFrameTime();
}

void HuboMotionData::getAllAngleAccelInHuboMotion(int frame, Eigen::VectorXd &angles)
{
	angles.resize(26);
	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngleSecondDerivateInTime(frame);

}

void HuboMotionData::getAllAngleAccelInHuboMotion(int firstFrame, double t, Eigen::VectorXd &angles)
{

	angles.resize(26);
	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngleSecondDerivateInTime(firstFrame+t);
}

void HuboMotionData::getAllAngleInHuboMotionInTime(double t, Eigen::VectorXd &angles)
{
	angles.resize(26);
	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngleInTime(t / getFrameTime());
}

void HuboMotionData::getAllAngleRateInHuboMotionInTime(double t, Eigen::VectorXd &angles)
{
	angles.resize(26);
	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngleRateInTime(t / getFrameTime()) * frameRate;
}

void HuboMotionData::getAllAngleAccelInHuboMotionInTime(double t, Eigen::VectorXd &angles)
{
	angles.resize(26);
	for (int i = 0; i < activeJoints.size(); i++)
		angles(i) = activeJoints.at(i)->getAngleSecondDerivateInTime(t / getFrameTime()) *frameRate * frameRate;
}

void HuboMotionData::getAllStateInTime(double time, Eigen::VectorXd &ang, Eigen::VectorXd &angVel, Eigen::VectorXd &angAcc)
{
	getAllAngleInHuboMotionInTime(time, ang);
	getAllAngleRateInHuboMotionInTime(time, angVel);
	getAllAngleAccelInHuboMotionInTime(time, angAcc);
}

double HuboMotionData::getHuboMass()
{
	double totalMass = 0;
	for(int i = 0; i < joints.size(); i++)
		totalMass += joints.at(i)->childBodyMass;

	return totalMass;
}

void HuboMotionData::getHuboAllMassMatrix(Eigen::MatrixXd &massMatrix, int applyOrientation)
{
	const int bodiessize = joints.size();

	if(applyOrientation)massMatrix.resize(6, 6*bodiessize); 
	else massMatrix.resize(3, 3*bodiessize);

	for(int i=0; i<massMatrix.rows(); i++)
		for(int j=0; j<massMatrix.cols(); j++)
			massMatrix(i,j) = 0.0;


	double m;

	for(int i=0; i<bodiessize; i++)
	{
		m = joints[i]->childBodyMass;
		for(int j=0; j<3; j++)
		{
			massMatrix(j,3*i+j) = m;
		}
	}

	if(applyOrientation)
	{
		Vector3d com = getHuboComGlobalPosition();

	}
}

void HuboMotionData::makeGroundContact()
{	
	double height = min(jointMap["LAR"]->getGlobalComPosition(frame).y(), jointMap["RAR"]->getGlobalComPosition(frame).y());

	Vector3d v = jointMap["Hip"]->getTranslation(frame) + Vector3d(0,-(height-0.035),0);

	jointMap["Hip"]->setTranslation(frame, v);
}

void HuboMotionData::printAllMotion(char* filename)
{
	std::ofstream out(filename);
	
	out << frameTotal << std::endl;

	for(int i=0; i<frameTotal; i++)
		out << jointMap[string("LEB")]->getAngle(i) << " "
			<< jointMap[string("LSP")]->getAngle(i) << " "
			<< jointMap[string("LSR")]->getAngle(i) << " "
			<< jointMap[string("LSY")]->getAngle(i) << " "
			<< jointMap[string("REB")]->getAngle(i) << " "
			<< jointMap[string("RSP")]->getAngle(i) << " "
			<< jointMap[string("RSR")]->getAngle(i) << " "
			<< jointMap[string("RSY")]->getAngle(i) << " "
			<< std::endl;

	for(int i=frameTotal-1; i>=0; i--)
		out << jointMap[string("LEB")]->getAngle(i) << " "
			<< jointMap[string("LSP")]->getAngle(i) << " "
			<< jointMap[string("LSR")]->getAngle(i) << " "
			<< jointMap[string("LSY")]->getAngle(i) << " "
			<< jointMap[string("REB")]->getAngle(i) << " "
			<< jointMap[string("RSP")]->getAngle(i) << " "
			<< jointMap[string("RSR")]->getAngle(i) << " "
			<< jointMap[string("RSY")]->getAngle(i) << " "
			<< std::endl;

	out.close();
}

void HuboMotionData::save(const char *_filename, int firstFrame, int lastFrame)
{
	if (lastFrame == 0)
		lastFrame = frameTotal - 1;
	if (firstFrame > lastFrame || firstFrame <0 || lastFrame > frameTotal - 1)
	{
		printf("save error: invalid frame setting");
		return;
	}

	const int numFrame = lastFrame - firstFrame + 1;
	std::ofstream fout;
	fout.open(_filename);

	// total frame
	fout << "TotalFrames " << numFrame << std::endl;

	for (int i = firstFrame; i <= lastFrame; i++)
	{
		fout << "Frame " << (i - firstFrame + 1) << std::endl;

		//hip translation and orientation
		Eigen::Quaterniond q = rootJoint->motions.at(i)->getRotation();
		Eigen::Vector3d v = qToEulerZYX(q);
		fout << "Hip "
			<< rootJoint->motions.at(i)->getTranslation().transpose()
			<< " "
			<< v.reverse().transpose()
			<< std::endl;

		for (int j = 0; j < activeJoints.size(); j++)
		{
			fout << activeJoints.at(j)->name << " "
				<< activeJoints.at(j)->getAngle(i)
				<< std::endl;
		}
	}
	std::cout << "motionSize:" << rootJoint->motions.size() <<std::endl;

	fout.close();
}
