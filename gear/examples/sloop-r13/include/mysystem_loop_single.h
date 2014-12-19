#ifndef _MY_SYSTEM_SINGLE_LOOP
#define _MY_SYSTEM_SINGLE_LOOP

#include "mysystem.h"
#include "gear.h"

//===================================================================
//			MySystemSingleLoop
//===================================================================
class MySystemSingleLoop : public MySystem
{
public:
    std::vector<RigidBody*> plinks;
	std::vector<RevoluteJoint*> pjoints;
	GJointFree root_joint_free;

	// temporal root joints for approximating ground contact
	GJointRevolute root_joint_rotate_x;

	// seesaw
	RigidBody seesaw_bar, seesaw_base;
	RevoluteJoint rjoint_seesaw;
	GJointFixed fxjoint_seesaw;

public:
	MySystemSingleLoop() {}
	~MySystemSingleLoop() { 
		for (size_t i=0; i<plinks.size(); i++) { delete plinks[i]; }
		for (size_t i=0; i<pjoints.size(); i++) { delete pjoints[i]; } 
	}

public:
	bool build(const char *dirpath = NULL);
	bool set_properties();

	void approx_contact_model(bool b_);

	bool render(bool b_update_kine_);
	double getRadiusOfMinimalEnclosingSphere();
	Vec3 getCenterOfMinimalEnclosingSphere();

	// test function for developer
	bool test_func();
};

#endif

