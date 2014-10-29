#ifndef _MOTIONDATA_H_
#define _MOTIONDATA_H_

#pragma once

#include "Joint.h"
#include <map>


class MotionData
{
public:
	MotionData() 
		:frame(0), frameRate(100), frameTotal(1), rootJoint(NULL)
	{}
	~MotionData();

protected:
	int frame; 
public:
    double frameRate;
    int frameTotal;

	
	//Joint
	Joint* rootJoint;
	std::vector<string>jointNames;
	std::vector<Joint*>joints;
	std::map<string, int> jointIdMap;
	std::map<string, Joint*> jointMap;

	
	//methods
	void init();
	int importSkeleton(string filename);
	int import(string filename, int firstFrame, int lastFrame);
	void draw(int _frame);
	void draw();
	void drawBox(int _frame);
	void drawBox();
	void setFrameRate(double _frameRate);
	double getFrameRate();
	void setFrameTime(double _frameTime);
	double getFrameTime();
	int getMotionSize();
	void setMotionSize(int num);
	void addMotionSize(int num);
	void printAllMotion(string filename);
	void resetMotion(int _frame);
	void resetMotion();
	void setCurrentFrame(int _frame);
	int getCurrentFrame();	
	int canGoOneFrame();

	double timeToTimeBetweenFrame(double time);
	int timeToFrame(double time);

	void backUpMotion(int _frame);
	void restoreMotion(int _frame);
	void backUpMotionForIk(int _frame);
	void restoreMotionForIk(int _frame);

	void cutMotion(int firstFrame, int lastFrame = 0 );
	void copyOneMotion(MotionData *src, int srcFrame, int dstFrame);
	void copyAllMotion(MotionData *src);
	//TODO:
	void copyMotions(MotionData *src, int srcBeginFrame, int srcEndFrame, MotionData *dst, int dstBeginFrame);

	// This function always runs at currentFrame.
	// Before using this function, 
	// use backUpMotion(frame) in right position.
	// DON'T OVERWRITE backUpMotion UNEXPECTEDLY!!!
	void propagateDiffer(int beginFrame, int endFrame);
};

#endif
