#include "MotionData.h"
#include <queue>
#include <iostream>
#include <fstream>

MotionData::~MotionData()
{
	for(int i=0; i<joints.size(); i++)
		delete joints.at(i);
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
