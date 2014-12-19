#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include "mysystem_loop_single.h"

#include <vector>
#include <algorithm>
#include "liegroup_rmatrix3_ext.h"
#include "cdfunc.h"

using namespace std;

// skeleton
static int _nL = 12;			// number of links
static double _L_link = 0.2;	// length of link
static double _W_link = 0.2;	// width of link
static double _H_link = 0.04;	// height of link
static double _mass_link = 0.1;	// mass of link

// location of ground plane
static double _yloc = 0.0;
static double _zloc = 0.0;

// for handling change of base link
static vector<bool> b_reversed_joints;

bool MySystemSingleLoop::build(const char *dirpath)
{
	char cstr[500];

	system.setName("Single-loop");
	skeleton.setName("Single-loop skeleton");

	string dir(dirpath); dir += string("/");

	plinks.resize(_nL);
	pjoints.resize(_nL);
	for (int i=0; i<_nL; i++) {
		plinks[i] = new RigidBody;
		pjoints[i] = new RevoluteJoint;
	}

	// ----------------------------------------------------------------------------------------------------
	//                Skeleton Definition
	// ----------------------------------------------------------------------------------------------------

	// ground
	if ( !ground.loadSurfaceMesh((dir+string("plane.msh")).c_str(), 20*Vec3(5, 5, 0), SE3(Vec3(0, _yloc, _zloc)) ) ) return false;

	// links
	for (int i=0; i<_nL; i++) 
	{
		sprintf(cstr, "link %d", i); 
		plinks[i]->setName(string(cstr));										// name
		plinks[i]->setBoxObject(_W_link, 0.9*_L_link, _H_link, _mass_link, SE3(), true, 1, 8, 1);	// mass
		plinks[i]->setMarkerPosition(Vec3(0,0,0));								// marker
		plinks[i]->setReferencePlane(Vec3(0,0,0), Vec3(1,0,0));					// reference plane
		plinks[i]->enableCollision(true);										// enabling contact
	}

	// root joint
	root_joint_free.connectBodies(&ground, plinks[0]);
	root_joint_free.setPosition(Vec3(0,0,0), Vec3(0,0,-_H_link/2.));
	root_joint_free.setName("root joint free");

	// joints
	for (int i=0; i<_nL-1; i++) {
		sprintf(cstr, "joint %d", i); 
		pjoints[i]->setName(string(cstr));
		pjoints[i]->connectBodies(plinks[i], plinks[i+1]);
		pjoints[i]->setPosition(Vec3(0, _L_link/2., 0), Vec3(0, -_L_link/2., 0));
		pjoints[i]->setAxis(1,0,0);
		pjoints[i]->set_qLL(0./180.*3.14159);
		pjoints[i]->set_qUL(90./180.*3.14159);
	}

	sprintf(cstr,"joint %d", _nL-1); 
	pjoints[_nL-1]->setName(string(cstr));
	pjoints[_nL-1]->connectBodies(plinks[_nL-1], plinks[0]);
	pjoints[_nL-1]->setPosition(Vec3(0, _L_link/2., 0), Vec3(0, -_L_link/2., 0));
	pjoints[_nL-1]->setAxis(1,0,0);
	pjoints[_nL-1]->set_qLL(0./180.*3.14159);
	pjoints[_nL-1]->set_qUL(90./180.*3.14159);

	// skeleton
	skeleton.buildSystem(&ground);

	// build system
	system.buildSystem(&ground);

	// ----------------------------------------------------------------------------------------------------
	//                initial pose
	// ----------------------------------------------------------------------------------------------------

	// for circular shape
	double q0 = 2.*3.14159/double(_nL);
	for (size_t i=0; i<pjoints.size(); i++) {
		pjoints[i]->coordinate.q = q0;
	}

	root_joint_free.set_q(RMatrix("0 0 0 0 -1 1"));

	update_kinematics();

	// ----------------------------------------------------------------------------------------------------
	//                Objects
	// ----------------------------------------------------------------------------------------------------

	double length_seesaw = 5.0;
	double height_seesaw = 0.5;
	Vec3 initpos_heavybox(0, 0.5, height_seesaw+0.7), initpos_smallbox(0, 1.5, height_seesaw+0.5);

	seesaw_base.setName("seesaw_base");
	seesaw_bar.setName("seesaw_bar");

	fxjoint_seesaw.connectBodies(&ground, &seesaw_base);
	rjoint_seesaw.connectBodies(&seesaw_base, &seesaw_bar);

	fxjoint_seesaw.setPosition(Vec3(0,0,0), Vec3(0,0,0));
	rjoint_seesaw.setPosition(Vec3(0,0,height_seesaw), Vec3(0,0,0));
	rjoint_seesaw.setAxis(1,0,0);

	if ( !seesaw_base.addSurfaceMesh((dir+string("seesaw_base.tri")).c_str(), Vec3(1,1,1), SE3(), false, 0) ) return false;
	if ( !seesaw_base.addSurfaceMesh((dir+string("cylinder_rjoint_sloop.tri")).c_str(), Vec3(1,3,3), SE3(Vec3(0.5, 0, height_seesaw)), false, 0) ) return false;
	if ( !seesaw_base.addSurfaceMesh((dir+string("cylinder_rjoint_sloop.tri")).c_str(), Vec3(1,3,3), SE3(Vec3(-0.5, 0, height_seesaw)), false, 0) ) return false;
	seesaw_bar.setBoxObject(1.0, length_seesaw, 0.15, 10.0);

	seesaw_bar.enableCollision(true);
	seesaw_bar.setBoundingBoxAutomatically();

	vector<GBody*> childin;
	childin.push_back(&seesaw_base);
	if ( !system_objects.buildSystemWith(&ground, childin) ) return false;
	system_objects.updateKinematics();

	// ----------------------------------------------------------------------------------------------------
	//                Set pointers defined in MySystem
	// ----------------------------------------------------------------------------------------------------

	list<GCoordinate*>::iterator iter_pcoord;

	pLinks.resize(_nL);
	for (int i=0; i<_nL; i++) {
		pLinks[i] = plinks[i];
	}

	pActuatingJoints.clear();
	for (int i=0; i<_nL; i++) {
		pActuatingJoints.push_back(pjoints[i]);
	}

	pRootJoints.clear();
	pRootJoints.push_back(&root_joint_free);

	pCoordinatesActuator.clear();
	for (size_t i=0; i<pActuatingJoints.size(); i++) {
		for (iter_pcoord = pActuatingJoints[i]->pCoordinates.begin(); iter_pcoord != pActuatingJoints[i]->pCoordinates.end(); iter_pcoord++) {
			pCoordinatesActuator.push_back(*iter_pcoord);
		}
	}

	pCoordinatesRootJoint = vector<GCoordinate*>(root_joint_free.pCoordinates.begin(), root_joint_free.pCoordinates.end());
	pCoordinatesRootJointTranslational = vector<GCoordinate*>(root_joint_free.translational_joint.pCoordinates.begin(), root_joint_free.translational_joint.pCoordinates.end());
	pCoordinatesRootJointRotational = vector<GCoordinate*>(root_joint_free.spherical_joint.pCoordinates.begin(), root_joint_free.spherical_joint.pCoordinates.end());

	// joint loops
	list<GConstraintJointLoop>::iterator iter_jointloop;
	pJointLoops.clear();
	for (iter_jointloop = skeleton.closedJointLoopConstraints.begin(); iter_jointloop != skeleton.closedJointLoopConstraints.end(); iter_jointloop++) {
		pJointLoops.push_back(&(*iter_jointloop));
	}

	// objects
	pObjects.clear();
	pObjects.push_back(&seesaw_base);
	pObjects.push_back(&seesaw_bar);

	// ----------------------------------------------------------------------------------------------------
	//                Naming elements
	// ----------------------------------------------------------------------------------------------------

	for (size_t i=0; i<pLinks.size(); i++) { pLinks[i]->setID(i); }
	for (size_t i=0; i<pActuatingJoints.size(); i++) { pActuatingJoints[i]->setID(i); }

	return true;
}

bool MySystemSingleLoop::set_properties()
{
	list<GCoordinate *>::iterator iter_pcoord;

	// ----------------------------------------------------------------------------------------------------
	//                System Properties
	// ----------------------------------------------------------------------------------------------------

	// gravity
	system.setGravity(Vec3(0, 0, -9.81));

	// integrator and step size
	integrator = EXPLICIT_EULER;
	step_size = 0.001;

	// for the external dampers acting on the mass particles in the elastic bodies
	C_particle_damper = 0;

	// spring-damper on the actuating joints
	q_actuator_ref.assign(pCoordinatesActuator.size(), 0.0);
	K_q_actuator.assign(pCoordinatesActuator.size(), 0.0);
	C_dq_actuator.assign(pCoordinatesActuator.size(), 0.0);

	// collision
	b_check_collision = true;

	K_rr = 3000;
	C_rr = 30;

	// friction
	Mu_s_rr = 0.7;
	Mu_d_rr = 0.7;

	// type of driving actuator
	set_actuator_prescribed(true);

	m_constr_follower_elem_type = ELM_COM;

	// ----------------------------------------------------------------------------------------------------
	//                Rendering
	// ----------------------------------------------------------------------------------------------------

	rndr_scale_size = getRadiusOfMinimalEnclosingSphere();
	rndr_scale_force = 0.3;
	b_rndr_show_constraints_for_user_guided_simul = true;
	b_rndr_show_contact_force = false;
	b_rndr_show_sys_momentum = false;
	b_rndr_show_sys_com = true;
	b_rndr_show_position_of_temporal_root_joint = true;

	// ground
	ground.surface.b_render_mesh = false;
	//ground.surface.setDrawType(GLSUB_WIRE);
	//ground.surface.setColor(Vec3(0, 0, 0));
	//ground.surface.setAlphaBlending(0.5);

	// links
	for (size_t i=0; i<plinks.size(); i++) {
		plinks[i]->surface.setColor((1./256.)*Vec3(140,23,23));
		plinks[i]->surface.setAlphaBlending(1.0);
		plinks[i]->surface.setDrawType(GLSUB_SOLID);		// 0 = solid, 1 = wired
		plinks[i]->SetRadiusMarker(0.02*rndr_scale_size);
	}

	// joints
	for (size_t i=0; i<pjoints.size(); i++) {
		pjoints[i]->setRadius(_H_link/2.);
		pjoints[i]->setHeight(_W_link/2.);
		//pjoints[i]->setColorMapMode(true, GLSUB_COLOR_MAP_AUTUMN_INVERSE, 2.0);
		pjoints[i]->setColor((1./256.)*Vec3(124,252,0));
		//pjoints[i]->setAlphaBlending(1.0);
		pjoints[i]->setDrawType(GLSUB_SOLID);	// 0 = solid, 1 = wired
	}

	// seesaw
	for (size_t i=0; i<seesaw_base.surfaces_for_rendering_only.size(); i++) {
		seesaw_base.surfaces_for_rendering_only[i]->setDrawType(GLSUB_SOLID);
	}
	seesaw_base.surfaces_for_rendering_only[0]->setColor(Vec3(0.2,0.2,0.5));
	seesaw_base.surfaces_for_rendering_only[1]->setColor((1./256.)*Vec3(135,31,120));
	seesaw_base.surfaces_for_rendering_only[2]->setColor((1./256.)*Vec3(135,31,120));
	//seesaw_base.surfaces_for_rendering_only[1]->setColor(Vec3(0,0.5,0));
	//seesaw_base.surfaces_for_rendering_only[2]->setColor(Vec3(0,0.5,0));
	//seesaw_bar.surface.setColor((1./256.)*Vec3(255,193,37));
	seesaw_bar.surface.setColor((1./256.)*Vec3(255,185,15));
	seesaw_bar.surface.setDrawType(GLSUB_SOLID);
	rjoint_seesaw.setColor(Vec3(0,0.5,0));
	rjoint_seesaw.setDrawType(GLSUB_SOLID);

	//base_joint_rc.setColor(Vec3(0,0,0));
	//base_joint_rc.setDrawType(GLSUB_WIRE);

	return true;
}

bool MySystemSingleLoop::render(bool b_update_kine_)
{
	if ( !b_rndr_show_system ) return true;

	// ground
	if ( b_rndr_show_ground ) {
		double color0[4] = {0.0, 0.0, 0.0, 1.0};
		double color1[4] = {1.0, 1.0, 1.0, 0.6};
		double color2[4] = {0.5, 0.5, 0.5, 0.6};
		glPushMatrix();
		glTranslated(0, _yloc, _zloc);
		glsubPlane(20, 20, 10, 10, NULL, color1, color2, GLSUB_SOLID);
		glPopMatrix();	
	}

	return MySystem::render(b_update_kine_);
}

double MySystemSingleLoop::getRadiusOfMinimalEnclosingSphere()
{
	return 0.4;
}

Vec3 MySystemSingleLoop::getCenterOfMinimalEnclosingSphere()
{
	return Vec3(0, 0.4, 0.05);
}

void MySystemSingleLoop::approx_contact_model(bool b_)
{
	if ( b_ ) {

		vector<bool> b_link_contact(pLinks.size());
		vector<int> num_link_contact_points(pLinks.size());
		for (size_t i=0; i<pLinks.size(); i++) { 
			b_link_contact[i] = false; 
			num_link_contact_points[i] = 0;
		}

		// investigate if links contact with the ground
		for (size_t i=0; i<pLinks.size(); i++) {
			for (int j=0; j<pLinks[i]->surface.getNumNode(); j++) {
				if ( pLinks[i]->surface.bContact[j] ) {
					b_link_contact[i] = true;
					num_link_contact_points[i]++;
				}
			}
		}

		// if there is no link which contacts with the ground, return
		int num_link_contact = 0;
		for (size_t i=0; i<pLinks.size(); i++) {
			if ( num_link_contact_points[i] > 0 ) num_link_contact++;
		}
		if ( num_link_contact == 0 ) return;

		// find the base link
		int idx_base = -1;
		int num_max_link_cp = 0;
		Vec3 pos_com_sys = skeleton.getPositionCOMGlobal();

		// select the link with the most contact points
		for (int i=0; i<pLinks.size(); i++) {
			if ( num_link_contact_points[i] > num_max_link_cp ) {
				num_max_link_cp = num_link_contact_points[i];
				idx_base = i;
			}
		}

		//// select the nearest link in contact from c.o.m.
		//double dist = 100;
		//for (size_t i=0; i<pLinks.size(); i++) {
		//	if ( num_link_contact_points[i] > 0 ) {
		//		double dist_i = Norm(pLinks[i]->getPositionCOMGlobal() - pos_com_sys);
		//		if ( dist_i < dist ) {
		//			num_max_link_cp = num_link_contact_points[i];
		//			dist = dist_i;
		//			idx_base = i;
		//		}
		//	}
		//}

		if ( idx_base < 0 ) return;
		if ( num_max_link_cp == 0 ) return;

		// base_link = the selected link
		RigidBody &base_link = *(pLinks[idx_base]);

		// get contact information on the contact points of the selected link
		vector<Vec3> pos_cg, pos_cl;	// position of the contact nodes in {global} and a local frame, {base_link}
		for (int i=0; i<base_link.surface.getNumNode(); i++) {
			if ( base_link.surface.bContact[i] ) {
				pos_cg.push_back(base_link.surface.xg[i]);
				pos_cl.push_back(base_link.surface.r0[i]);
			}
		}

		// remove existing root joint
		for (size_t i=0; i<pRootJoints.size(); i++) { pRootJoints[i]->disconnectBodies(); }
		pRootJoints.clear();
		pCoordinatesRootJoint.clear();

		// new root joint
		root_joint_rotate_x.coordinate.init();
		root_joint_rotate_x.connectBodies(&ground, &base_link);
		if ( Norm(Log(base_link.getOrientationGlobal())) < 0.01 ) {
			root_joint_rotate_x.setPosition(base_link.getMarkerPositionGlobal(), base_link.getMarkerPosition());
		} else {
			root_joint_rotate_x.setPosition(0.5*(pos_cg[0]+pos_cg[1]), 0.5*(pos_cl[0]+pos_cl[1]));
		}
		root_joint_rotate_x.setOrientation(base_link.getOrientationGlobal(), SO3());
		root_joint_rotate_x.setAxis(1,0,0);
		pRootJoints.push_back(&root_joint_rotate_x);
		pCoordinatesRootJoint.push_back(&root_joint_rotate_x.coordinate);

		pos_temporal_root_joint = pRootJoints[0]->T_left.GetPosition();

		vector<GBody*> childin; childin.push_back(&base_link);
		skeleton.buildSystemWith(&ground, childin);
		system.buildSystemWith(&ground, childin);

		// joint loops
		list<GConstraintJointLoop>::iterator iter_jointloop;
		pJointLoops.clear();
		for (iter_jointloop = skeleton.closedJointLoopConstraints.begin(); iter_jointloop != skeleton.closedJointLoopConstraints.end(); iter_jointloop++) {
			pJointLoops.push_back(&(*iter_jointloop));
		}

	} else {

		// remove the temporary root joint
		for (size_t i=0; i<pRootJoints.size(); i++) { pRootJoints[i]->disconnectBodies(); }
		pRootJoints.clear();
		pCoordinatesRootJoint.clear();

		// root joint
		root_joint_free.connectBodies(&ground, plinks[0]);
		root_joint_free.setPosition(Vec3(0,0,0), Vec3(0,0,-_H_link/2.));
		pRootJoints.push_back(&root_joint_free);
		pCoordinatesRootJoint = vector<GCoordinate*>(root_joint_free.pCoordinates.begin(), root_joint_free.pCoordinates.end());

		vector<GBody*> childin; childin.push_back(plinks[0]);
		skeleton.buildSystemWith(&ground, childin); // includes the branch starting from plinks[0] only
		system.buildSystemWith(&ground, childin);

		// joint loops
		list<GConstraintJointLoop>::iterator iter_jointloop;
		pJointLoops.clear();
		for (iter_jointloop = skeleton.closedJointLoopConstraints.begin(); iter_jointloop != skeleton.closedJointLoopConstraints.end(); iter_jointloop++) {
			pJointLoops.push_back(&(*iter_jointloop));
		}
	}
}

bool MySystemSingleLoop::test_func()
{
	cout << "number of bodies = " << skeleton.pBodies.size() << endl;
	cout << "number of joints = " << skeleton.pJoints.size() << endl;
	cout << endl;
	return true;
}
