#include "MotionData.h"
#include <queue>
#include <iostream>
#include <fstream>
#include "HpMotionMath.h"

MotionData::~MotionData()
{
	for(int i=0; i<joints.size(); i++)
		delete joints.at(i);
}

void MotionData::init()
{
	frame = 0;
	setMotionSize(0);
}

void MotionData::draw(int _frame)
{
	assert(_frame < frameTotal);
	rootJoint->drawHierarchical(_frame);
}

void MotionData::draw()
{
	rootJoint->drawHierarchical(getCurrentFrame());
}

void MotionData::drawBox(int _frame)
{
	assert(_frame < frameTotal);
	rootJoint->drawBoxHierarchical(_frame);
}

void MotionData::drawBox()
{
	rootJoint->drawBoxHierarchical(getCurrentFrame());
}

void MotionData::setFrameRate(double _frameRate)
{
	assert(_frameRate > 0);
	frameRate = _frameRate;
}

double MotionData::getFrameRate()
{
	return frameRate;
}

void MotionData::setFrameTime(double _frameTime)
{
	assert(_frameTime > 0);
	frameRate = 1.0 / _frameTime;
}

double MotionData::getFrameTime()
{
	return 1.0 / frameRate;
}

int MotionData::getMotionSize()
{
	return frameTotal;
}
void MotionData::setMotionSize(int num)
{
	assert(num >= 0);
	if (num == 0)
		num = 1;
	int i, k;

	//for (std::vector<Joint*>::iterator it = joints.begin(); it != joints.end(); it++)
	for(i=0; i<joints.size(); i++)
	{
		if((joints.at(i))->motions.size() != 0)
		{
			for(std::vector<Motion*>::iterator it=joints.at(i)->motions.begin(); it!=joints.at(i)->motions.end(); it++)
			{
					delete (*it);
			}
		}
	}

	for (std::vector<Joint*>::iterator it = joints.begin(); it != joints.end(); it++)
	{
		(*it)->motions.clear();
		(*it)->angles.clear();
	}

	for (std::vector<Joint*>::iterator it = joints.begin(); it != joints.end(); it++)
	{
		for (int j = 0; j < num; j++)
		{
			(*it)->angles.push_back(0);
			(*it)->motions.push_back(new Motion());
		}
	}

	

	frameTotal = num;
	if( frame >= frameTotal )
		setCurrentFrame(frameTotal-1);
}

void MotionData::addMotionSize(int num)
{
	Motion m;
	
	for(std::vector<Joint*>::iterator it=joints.begin(); it!=joints.end(); it++)
	{
		if(frameTotal > 0)
		{
			Quaterniond q =(*it)->motions.back()->getRotation(); 
			m.setRotation(q);
			Vector3d v= (*it)->motions.back()->getTranslation();
			m.setTranslation(v);
		}
		for(int j=0; j < num ; j++)
		{
			Quaterniond q = m.getRotation();
			Vector3d v = m.getTranslation();
			(*it)->motions.push_back(new Motion(v, q));
		}
	}

	frameTotal += num;

}

void MotionData::resetMotion(int _frame)
{
	assert(_frame < frameTotal);
	for(std::vector<Joint*>::iterator it=joints.begin(); it!=joints.end(); it++)
		(*it)->motions.at(_frame)->reset();
}

void MotionData::resetMotion()
{
	for(std::vector<Joint*>::iterator it=joints.begin(); it!=joints.end(); it++)
		(*it)->motions.at(frame)->reset();
}

void MotionData::setCurrentFrame(int _frame)
{
	assert(_frame < frameTotal);
	frame = _frame;
	//Joint::setCurrentFrameNumber(_frame);
}

int MotionData::getCurrentFrame()
{
	return frame;
}

int MotionData::canGoOneFrame()
{
	if (frame >= frameTotal - 1)
		return 0;
	return 1;
}

void MotionData::backUpMotionForIk(int _frame)
{
	for (int i = 0; i < joints.size(); i++)
		joints.at(i)->backUpMotionForIk = *(joints.at(i)->motions.at(_frame));
}

void MotionData::restoreMotionForIk(int _frame)
{
	for (int i = 0; i < joints.size(); i++)
		*(joints.at(i)->motions.at(_frame)) = joints.at(i)->backUpMotionForIk;
}

void MotionData::backUpMotion(int _frame)
{
	for (int i = 0; i < joints.size(); i++)
		joints.at(i)->backUpMotion = *(joints.at(i)->motions.at(_frame));
}

void MotionData::restoreMotion(int _frame)
{
	for (int i = 0; i < joints.size(); i++)
		*(joints.at(i)->motions.at(_frame)) = joints.at(i)->backUpMotion;
}

void MotionData::copyOneMotion(MotionData *src, int srcFrame, int dstFrame)
{
	for (int i = 0; i < joints.size(); i++)
		*(joints.at(i)->motions.at(dstFrame)) = *(src->joints.at(i)->motions.at(srcFrame));
}

void MotionData::copyAllMotion(MotionData *src)
{
	const int motionSize = src->getMotionSize();
	setMotionSize(motionSize);
	for (int j = 0; j < motionSize; j++)
		for (int i = 0; i < joints.size(); i++)
			*(joints.at(i)->motions.at(j)) = *(src->joints.at(i)->motions.at(j));

}

void MotionData::propagateDiffer(int beginFrame, int endFrame)
{
	Motion currM;
	Motion m;

	Eigen::Vector3d currDiffv;
	Eigen::Quaterniond currDiffq;

	Eigen::Vector3d v;
	Eigen::Quaterniond q;
	Eigen::Quaterniond qtarget;
	double ratio;
	for (int j = 0; j < joints.size(); j++)
	{
		currM = *(joints.at(j)->motions.at(frame));
		//currDiffv = currM.getTranslation() - joints.at(j)->backUpMotion.getTranslation();
		Eigen::Quaterniond q2 = currM.getRotation();
		Eigen::Quaterniond q1 = joints.at(j)->backUpMotion.getRotation();
		currDiffq = q1.inverse() * q2;

		for (int i = beginFrame + 1; i < frame; i++)
		{
			ratio = ((double)(i - beginFrame)) / (frame - beginFrame);
			m = *(joints.at(j)->motions.at(i));
			//v = m.getTranslation() + ratio * currDiffv;
			qtarget = m.getRotation() * currDiffq;
			q = m.getRotation().slerp(ratio, qtarget);
			
			//joints.at(j)->motions.at(i)->setTranslation(v);
			joints.at(j)->motions.at(i)->setRotation(q);
		}

		for (int i = frame + 1; i < endFrame; i++)
		{
			ratio = ((double)(endFrame - i)) / (endFrame - frame);
			m = *(joints.at(j)->motions.at(i));
			v = m.getTranslation() + ratio * currDiffv;
			qtarget = m.getRotation() * currDiffq;
			q = m.getRotation().slerp(ratio, qtarget);
			
			//joints.at(j)->motions.at(i)->setTranslation(v);
			joints.at(j)->motions.at(i)->setRotation(q);
		}
	}
}