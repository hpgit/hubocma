#ifndef _BVHMOTIONDATA_H_
#define _BVHMOTIONDATA_H_

#include "MotionData.h"

class BVHMotionData : public MotionData {
	
public:
    BVHMotionData(){};
	string filename;

	int import(char*  _filename);
	int importSkeleton(char* _filename);
};

#endif
