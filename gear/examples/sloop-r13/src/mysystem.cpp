#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <list>
#include <vector>
#include <algorithm>
#include "triad.h"
#include "mysystem.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "liegroup_rmatrix3_ext.h"
#include "glsub.h"
#include "gear.h"
#include "cdfunc.h"
#include "rigid_body.h"
#include "myQuadProg.h"
//#include "myimsl_matrix.h"

using namespace std;

MySystem::MySystem()
{
	b_created = false;

	integrator = EXPLICIT_EULER;
	step_size = 0.001;

	C_particle_damper = 30;

	b_check_collision = true;
	b_check_collision_dr = true;
	b_check_collision_rr = true;

	K_dr = 300;
	C_dr = 0;
	d0_nc_dr = 0;

	K_dd = 30;
	C_dd = 0;
	d0_nc_dd = 0;

	K_rr = 3000;
	C_rr = 300;
	d0_nc_rr = 0.001;

	Mu_s_dr = 0.7;
	Mu_d_dr = 0.7;

	Mu_s_dd = 0.0;
	Mu_d_dd = 0.0;

	Mu_s_rr = 3.0;
	Mu_d_rr = 3.0;

	b_rndr_naming = false;
	b_rndr_show_system = true;
	b_rndr_show_skeleton = true;
	b_rndr_show_body_surface = true;
	b_rndr_show_body_surface_extended = true;
	b_rndr_show_joint = true;
	b_rndr_show_body_frame = false;
	b_rndr_show_body_com = false;
	b_rndr_show_body_com_vel = false;
	b_rndr_show_body_com_acc = false;
	b_rndr_show_ground = true;
	b_rndr_show_contact_force = false;
	b_rndr_show_dbody_ebody = false;
	b_rndr_show_dbody_surf = true;
	b_rndr_show_dbody_surf_fake = false;
	b_rndr_show_bounding_box = false;
	b_rndr_show_sys_com = false;
	b_rndr_show_sys_momentum = false;
	b_rndr_show_constraints_for_user_guided_simul = true;
	b_rndr_hide_constraints_for_user_guided_simul_when_replay_running = true;
	b_rndr_show_position_of_temporal_root_joint = false;
	b_rndr_hide_desired_acceleration_for_user_guided_simul = true;
	b_rndr_show_vnode_body_connection = false;
	rndr_scale_force = 1.0;
	rndr_scale_vel = 1.0;
	rndr_scale_acc = 1.0;
	rndr_scale_size = 1.0;

	rndr_color_com = rndr_color_com_normal = Vec3(1,0,1);
	rndr_line_width_com = rndr_line_width_com_normal = 1;

	m_simul_num_skip_frames_for_updating_command_input = 0;
}

MySystem::~MySystem()
{
}

bool MySystem::create(const char * dirpath)
{
	if ( b_created ) return true;

	if ( !build(dirpath) ) return false;
	if ( !set_variables_automatically() ) return false;
	if ( !set_properties() ) return false;
	if ( !set_data_to_be_saved() ) return false;

	b_created = true;

	return true;
}

bool MySystem::set_variables_automatically()
{
	init_variables_for_user_guided_simulation();

	list<GCoordinate*>::iterator iter_pcoord;
	list<GJoint*>::iterator iter_pjoint;

	contactPositions.resize(pLinks.size());
	for (size_t i=0; i<contactPositions.size(); i++) {
		contactPositions[i].clear();
	}
	
	pNonActuatingJoints.clear();
	for (iter_pjoint = skeleton.pJoints.begin(); iter_pjoint != skeleton.pJoints.end(); iter_pjoint++) {
		if ( find(pActuatingJoints.begin(), pActuatingJoints.end(), *iter_pjoint) == pActuatingJoints.end() ) {
			pNonActuatingJoints.push_back(*iter_pjoint);
		}
	}

	pCoordinatesNonActuator.clear();
	for (size_t i=0; i<pNonActuatingJoints.size(); i++) {
		for (iter_pcoord = pNonActuatingJoints[i]->pCoordinates.begin(); iter_pcoord != pNonActuatingJoints[i]->pCoordinates.end(); iter_pcoord++) {
			pCoordinatesNonActuator.push_back(*iter_pcoord);
		}
	}

	pBodiesDistalOfpCoordinatesActuator.resize(pCoordinatesActuator.size());
	idxLocalOfpCoordinatesActuator.resize(pCoordinatesActuator.size());

	for (size_t i=0; i<pCoordinatesActuator.size(); i++) {
		pBodiesDistalOfpCoordinatesActuator[i] = NULL;
		idxLocalOfpCoordinatesActuator[i] = -1;

		// find a joint holding pCoordinatesActuator[i]
		GJoint *pjoint = NULL;
		for (iter_pjoint = skeleton.pJoints.begin(); iter_pjoint != skeleton.pJoints.end(); iter_pjoint++) {
			if ( find((*iter_pjoint)->pCoordinates.begin(), (*iter_pjoint)->pCoordinates.end(), pCoordinatesActuator[i]) != (*iter_pjoint)->pCoordinates.end() ) {
				pjoint = *iter_pjoint;
				break;
			}
		}
		if ( pjoint == NULL ) return false;

		// find local index of pCoordinatesActuator[i] in pjoint->pCoordinates
		int idx = 0;
		for (iter_pcoord = pjoint->pCoordinates.begin(); iter_pcoord != pjoint->pCoordinates.end(); iter_pcoord++) {
			if ( *iter_pcoord == pCoordinatesActuator[i] ) {
				idxLocalOfpCoordinatesActuator[i] = idx;
			}
			idx++;
		}

		// find a body whose base joint is pjoint
		if ( pjoint->isCut() ) {
			pBodiesDistalOfpCoordinatesActuator[i] = NULL;
		} else {
			pBodiesDistalOfpCoordinatesActuator[i] = (GBody*)(pjoint->pRightBody);
		}

	}

	m_constr_b_primary_joint_angle_limit = false;

	int na = pCoordinatesActuator.size();

	m_simul_Kp_mouse_drag = 2000;	m_simul_Kv_mouse_drag = 100;
	m_simul_Kp_link_pos = 2000;		m_simul_Kv_link_pos = 100;
	m_simul_Kp_link_ori = 2000;		m_simul_Kv_link_ori = 100;
	m_simul_Kp_joint = 100;			m_simul_Kv_joint = 10;
	m_simul_Kp_com = Vec3(2000.);	m_simul_Kv_com = Vec3(100.);
	m_simul_Kp_momentum = dse3(Vec3(100.),Vec3(100.));
	m_simul_alpha_acc = 0.1;//0.1;//0.003;//0.1;
	m_simul_alpha_trq = 1;//0.003;//0.1;
	m_simul_alpha_vel = 0.003;
	m_b_simul_limit_joint_angle = true;
	m_b_simul_limit_joint_torque = false; //true;
	m_b_simul_uneven_joint_weighting = false;
	m_b_simul_automatic_weighting = false;
	for (int i=0; i<na; i++) {
		m_constr_joint_b_selected[i] = 1;
		m_constr_joint_b_primary[i] = 0;
		m_constr_joint_type[i] = JOINT_CONSTR_TYPE_USER_Q;
		m_constr_joint_user_given_q[i] = pCoordinatesActuator[i]->q;
	}

	m_simul_weight_qa.resize(na);
	for (int i=0; i<na; i++) {
		m_simul_weight_qa[i] = 1.0;
	}

	return true;
}

bool MySystem::set_data_to_be_saved()
{
	list<GCoordinate*>::iterator iter_pcoord;
	list<GJoint*>::iterator iter_pjoint;
	vector<double *> ptr_data_double;
	vector<bool *> ptr_data_bool;
	vector<int *> ptr_data_int;

	// system coordinates
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		ptr_data_double.push_back(&((*iter_pcoord)->q));
		ptr_data_double.push_back(&((*iter_pcoord)->dq));
		ptr_data_double.push_back(&((*iter_pcoord)->ddq));
		ptr_data_double.push_back(&((*iter_pcoord)->tau));
	}
	for (iter_pcoord = system_objects.pCoordinates.begin(); iter_pcoord != system_objects.pCoordinates.end(); iter_pcoord++) {
		ptr_data_double.push_back(&((*iter_pcoord)->q));
		ptr_data_double.push_back(&((*iter_pcoord)->dq));
		ptr_data_double.push_back(&((*iter_pcoord)->ddq));
		ptr_data_double.push_back(&((*iter_pcoord)->tau));
	}

	// coordinate chart for spherical joints
	for (iter_pjoint = system.pJoints.begin(); iter_pjoint != system.pJoints.end(); iter_pjoint++) {
		if ( (*iter_pjoint)->jointType == GJoint::GJOINT_SPHERICAL ) {
			ptr_data_int.push_back(&(((GJointSpherical*)(*iter_pjoint))->coord_chart));
		}
		if ( (*iter_pjoint)->jointType == GJoint::GJOINT_FREE_ST ) {
			ptr_data_int.push_back(&(((GJointFree*)(*iter_pjoint))->spherical_joint.coord_chart));
		}
		if ( (*iter_pjoint)->jointType == GJoint::GJOINT_FREE_TS ) {
			if ( ((GJointFreeTS*)(*iter_pjoint))->pJoint1->jointType == GJoint::GJOINT_SPHERICAL ) {
				ptr_data_int.push_back(&(((GJointSpherical*)(((GJointFreeTS*)(*iter_pjoint))->pJoint1))->coord_chart));
			}
			if ( ((GJointFreeTS*)(*iter_pjoint))->pJoint2->jointType == GJoint::GJOINT_SPHERICAL ) {
				ptr_data_int.push_back(&(((GJointSpherical*)(((GJointFreeTS*)(*iter_pjoint))->pJoint2))->coord_chart));
			}
		}
	}
	for (iter_pjoint = system_objects.pJoints.begin(); iter_pjoint != system_objects.pJoints.end(); iter_pjoint++) {
		if ( (*iter_pjoint)->jointType == GJoint::GJOINT_SPHERICAL ) {
			ptr_data_int.push_back(&(((GJointSpherical*)(*iter_pjoint))->coord_chart));
		}
		if ( (*iter_pjoint)->jointType == GJoint::GJOINT_FREE_ST ) {
			ptr_data_int.push_back(&(((GJointFree*)(*iter_pjoint))->spherical_joint.coord_chart));
		}
		if ( (*iter_pjoint)->jointType == GJoint::GJOINT_FREE_TS ) {
			if ( ((GJointFreeTS*)(*iter_pjoint))->pJoint1->jointType == GJoint::GJOINT_SPHERICAL ) {
				ptr_data_int.push_back(&(((GJointSpherical*)(((GJointFreeTS*)(*iter_pjoint))->pJoint1))->coord_chart));
			}
			if ( ((GJointFreeTS*)(*iter_pjoint))->pJoint2->jointType == GJoint::GJOINT_SPHERICAL ) {
				ptr_data_int.push_back(&(((GJointSpherical*)(((GJointFreeTS*)(*iter_pjoint))->pJoint2))->coord_chart));
			}
		}
	}

	// contact forces on the rigid body surface
	for (size_t i=0; i<pLinks.size(); i++) {
		for (int j=0; j<pLinks[i]->surface.getNumNode(); j++) {
			ptr_data_int.push_back(&(pLinks[i]->surface.bContact[j]));
			ptr_data_double.push_back(&(pLinks[i]->surface.fg[j][0]));
			ptr_data_double.push_back(&(pLinks[i]->surface.fg[j][1]));
			ptr_data_double.push_back(&(pLinks[i]->surface.fg[j][2]));
		}
	}
	for (size_t i=0; i<pObjects.size(); i++) {
		for (int j=0; j<pObjects[i]->surface.getNumNode(); j++) {
			ptr_data_int.push_back(&(pObjects[i]->surface.bContact[j]));
			ptr_data_double.push_back(&(pObjects[i]->surface.fg[j][0]));
			ptr_data_double.push_back(&(pObjects[i]->surface.fg[j][1]));
			ptr_data_double.push_back(&(pObjects[i]->surface.fg[j][2]));
		}
	}

	// interactive user controls
	ptr_data_int.push_back(&m_constr_follower_elem_type);
	ptr_data_int.push_back(&m_constr_follower_idx_pbody);
	ptr_data_bool.push_back(&(m_constr_follower_b_primary));
	ptr_data_double.push_back(&(m_constr_follower_pos_target[0]));
	ptr_data_double.push_back(&(m_constr_follower_pos_target[1]));
	ptr_data_double.push_back(&(m_constr_follower_pos_target[2]));
	ptr_data_double.push_back(&(m_constr_follower_acc_target[0]));
	ptr_data_double.push_back(&(m_constr_follower_acc_target[1]));
	ptr_data_double.push_back(&(m_constr_follower_acc_target[2]));

	for (size_t i=0; i<pLinks.size(); i++) {
		ptr_data_int.push_back(&(m_constr_link_b_pbodies_selected[i]));
		for (int j=0; j<6; j++) {
			ptr_data_bool.push_back(&(m_constr_link_b_coords_selected[i][j]));
			ptr_data_bool.push_back(&(m_constr_link_b_coords_primary[i][j]));
		}
		for (int j=0; j<16; j++) {
			ptr_data_double.push_back(&(m_constr_link_user_given_SE3_selected[i][j]));
		}
	}
	ptr_data_int.push_back(&m_constr_link_idx_cube_control);

	for (size_t i=0; i<pCoordinatesActuator.size(); i++) {
		ptr_data_int.push_back(&(m_constr_joint_b_selected[i]));
		ptr_data_int.push_back(&(m_constr_joint_b_primary[i]));
		ptr_data_double.push_back(&(m_constr_joint_user_given_q[i]));
		ptr_data_double.push_back(&(m_constr_joint_user_given_dq[i]));
		ptr_data_double.push_back(&(m_constr_joint_user_given_ddq[i]));
	}

	ptr_data_bool.push_back(&(m_constr_com_b_selected));
	ptr_data_bool.push_back(&(m_constr_com_b_coords_selected[0]));
	ptr_data_bool.push_back(&(m_constr_com_b_coords_selected[1]));
	ptr_data_bool.push_back(&(m_constr_com_b_coords_selected[2]));
	ptr_data_bool.push_back(&(m_constr_com_b_primary[0]));
	ptr_data_bool.push_back(&(m_constr_com_b_primary[1]));
	ptr_data_bool.push_back(&(m_constr_com_b_primary[2]));
	ptr_data_double.push_back(&(m_constr_com_pos_target[0]));
	ptr_data_double.push_back(&(m_constr_com_pos_target[1]));
	ptr_data_double.push_back(&(m_constr_com_pos_target[2]));

	ptr_data_int.push_back(&saved_simul_input_type);
	ptr_data_int.push_back(&saved_simul_user_guidance_type);

	ptr_data_double.push_back(&(m_desired_position[0]));
	ptr_data_double.push_back(&(m_desired_position[1]));
	ptr_data_double.push_back(&(m_desired_position[2]));

	// set data address
	sysdata.set_data_address_double(ptr_data_double);
	sysdata.set_data_address_bool(ptr_data_bool);
	sysdata.set_data_address_int(ptr_data_int);

	return true;
}

void MySystem::set_actuator_prescribed(bool b_)
{
	list<GCoordinate*>::iterator iter_pcoord;

	b_actuators_prescribed = b_;

	for (size_t i=0; i<pActuatingJoints.size(); i++) {
		pActuatingJoints[i]->setPrescribed(b_);
	}

	system._scanCoordinatesPrescribed();
	skeleton._scanCoordinatesPrescribed();
}

void MySystem::set_rootjoint_prescribed(bool b_) 
{
	for (size_t i=0; i<pRootJoints.size(); i++) {
		pRootJoints[i]->setPrescribed(b_);
		if ( pRootJoints[i]->isPrescribed() ) {
			pRootJoints[i]->set_tau(Zeros(pRootJoints[i]->getDOF(), 1).GetPtr());
		}
	}

	system._scanCoordinatesPrescribed();
	skeleton._scanCoordinatesPrescribed();
}

void MySystem::save_state(const char *file_name)
{
	// Binary file
	//-------------------------------------------------------
	// n							// n = system.getNumCoordinates()
	// q							// q = system.pCoordinates[]->q
	// dq							// dq = system.pCoordinates[]->dq
	// ddq							// ddq = system.pCoordinates[]->ddq
	// tau							// tau = system.pCoordinates[]->tau
	//-------------------------------------------------------

	ofstream fout(file_name, ios::binary);
	list<GCoordinate *>::iterator iter_pcoord;
	int n = system.getNumCoordinates();

	// n
	fout.write((char *)&n, sizeof(int));
	// q
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fout.write((char *)(&((*iter_pcoord)->q)), sizeof(double));
	}
	// dq
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fout.write((char *)(&((*iter_pcoord)->dq)), sizeof(double));
	}
	// ddq
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fout.write((char *)(&((*iter_pcoord)->ddq)), sizeof(double));
	}
	// tau
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fout.write((char *)(&((*iter_pcoord)->tau)), sizeof(double));
	}

	fout.close();
}

bool MySystem::load_state(const char *file_name)
{
	// Binary file
	//-------------------------------------------------------
	// n							// n = system.getNumCoordinates()
	// q							// q = system.pCoordinates[]->q
	// dq							// dq = system.pCoordinates[]->dq
	// ddq							// ddq = system.pCoordinates[]->ddq
	// tau							// tau = system.pCoordinates[]->tau
	//-------------------------------------------------------

	ifstream fin(file_name, ios::binary);
	list<GCoordinate *>::iterator iter_pcoord;
	int n;

	// n
	fin.read((char *)&n, sizeof(int));

	if ( n != int(system.getNumCoordinates()) ) { fin.close(); return false; }

	// q
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fin.read((char *)(&((*iter_pcoord)->q)), sizeof(double));
	}
	// dq
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fin.read((char *)(&((*iter_pcoord)->dq)), sizeof(double));
	}
	// ddq
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fin.read((char *)(&((*iter_pcoord)->ddq)), sizeof(double));
	}
	// tau
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		fin.read((char *)(&((*iter_pcoord)->tau)), sizeof(double));
	}

	fin.close();

	update_kinematics();

	return true;
}

void MySystem::save_reference_pose(const char *file_name)
{
	ofstream fout(file_name);

	fout << m_ref_qa_style_based_control.size() << endl << pCoordinatesActuator.size() << endl << endl;
	for (size_t i=0; i<m_ref_qa_style_based_control.size(); i++) {
		for (size_t j=0; j<pCoordinatesActuator.size(); j++) {
			fout << m_ref_qa_style_based_control[i][j] << "  ";
		}
		fout << endl;
	}

	fout.close();
}

bool MySystem::load_reference_pose(const char *file_name)
{
	// Text file
	//-------------------------------------------------------
	// m							// number of reference poses
	// na							// na = pCoordinatesActuator.size()
	// qa_1							
	// ...
	// qa_na
	//-------------------------------------------------------

	ifstream fin(file_name);
	int m, na;
	fin >> m >> na;

	if ( m <= 0 ) return true;
	if ( na != pCoordinatesActuator.size() ) return false;

	m_ref_qa_style_based_control.resize(m);
	RMatrix qa(na, 1);

	for (int i=0; i<m; i++) {
		for (int j=0; j<na; j++) {
			fin >> qa[j];
		}
		m_ref_qa_style_based_control[i] = qa;
	}

	fin.close();

	return true;
}

bool MySystem::load_low_dim_pose_space(const char *file_name, int m_)
{
	// Text file
	//-------------------------------------------------------
	// na							// na = pCoordinatesActuator.size()
	// qam							// pose origin, size = (1,na) or (na,1)
	// lambda_1, ..., lambda_na		// eigen values of covariance matrix, size = (1,na)
	// [Phi_1, ..., Phi_na]			// each column corresponds to a basis vector, size = (na,na)
	//-------------------------------------------------------

	ifstream fin(file_name);
	int na;

	fin >> na;

	if ( na != pCoordinatesActuator.size() ) return false;
	if ( m_ <= 0 || m_ > na ) return false;

	RMatrix qam(na, 1), lambda(na, 1), Phi(na, na);

	for (int i=0; i<na; i++) {
		fin >> qam[i];
	}

	for (int i=0; i<na; i++) {
		fin >> lambda[i];
	}

	for (int i=0; i<na; i++) {
		for (int j=0; j<na; j++) {
			fin >> Phi(i,j);
		}
	}

	m_pca_qam = qam;
	m_pca_lambda = lambda.Sub(0, m_-1, 0, 0);
	m_pca_Phi = Phi.Sub(0, na-1, 0, m_-1);

	m_pca_A = ~Nullspace(~Phi);
	m_pca_b = m_pca_A * m_pca_qam;

	fin.close();

	return true;
}

int MySystem::calc_num_basis_low_dim_pose_space(const char *file_name, double score_)
{
	ifstream fin(file_name);
	int na;

	fin >> na;

	if ( score_ <= 0 || score_ > 1.0 ) return false;
	if ( na != pCoordinatesActuator.size() ) return false;

	RMatrix qam(na, 1), lambda(na, 1), Phi(na, na);

	for (int i=0; i<na; i++) {
		fin >> qam[i];
	}

	for (int i=0; i<na; i++) {
		fin >> lambda[i];
	}

	fin.close();

	double sum_lambda = 0;
	for (int i=0; i<na; i++) {
		sum_lambda += lambda[i];
	}
	if ( sum_lambda < 1E-6 ) return false;

	int cnt = 0;
	double acc_score = 0;
	for (int i=0; i<na; i++) {
		acc_score += lambda[i]/sum_lambda;
		cnt++;
		if ( acc_score >= score_ ) break;
	}

	return cnt;
}

bool MySystem::adjust_pose_to_low_dim_space(double *pdist_)
{
	int na = pCoordinatesActuator.size();
	int m = m_pca_Phi.ColSize();	// dimension of the reduced pose space

	m_b_pca_ready = false;

	if ( m <= 0 ) return false;
	if ( m_pca_qam.RowSize() != na ) return false;
	if ( m_pca_Phi.RowSize() != na ) return false;
	if ( m_pca_lambda.RowSize() != m_pca_Phi.ColSize() ) return false;

	RMatrix qa(na,1), dqa(na,1), ddqa(na,1), qa_new, dqa_new, ddqa_new;
	for (int i=0; i<na; i++) {
		qa[i] = pCoordinatesActuator[i]->q;
		dqa[i] = pCoordinatesActuator[i]->dq;
		ddqa[i] = pCoordinatesActuator[i]->ddq;
	}
	
	RMatrix r, dr, ddr;
	solve_Ax_b_pInv(r, m_pca_Phi, qa - m_pca_qam);
	solve_Ax_b_pInv(dr, m_pca_Phi, dqa);
	solve_Ax_b_pInv(ddr, m_pca_Phi, ddqa);

	qa_new = m_pca_Phi * r + m_pca_qam;
	dqa_new = m_pca_Phi * dr;
	ddqa_new = m_pca_Phi * ddr;

	if ( pdist_ != NULL ) {
		*pdist_ = FNorm(qa_new - qa);
	}

	for (int i=0; i<na; i++) {
		pCoordinatesActuator[i]->q = qa_new[i];
		pCoordinatesActuator[i]->dq = dqa_new[i];
		pCoordinatesActuator[i]->ddq = ddqa_new[i];
	}

	m_b_pca_ready = true;

	update_kinematics();

	return true;
}

void MySystem::init_variables_for_user_guided_simulation()
{
	int nL = pLinks.size();
	int nA = pCoordinatesActuator.size();

	// follower (following the mouse cursor)
	m_constr_follower_elem_type = ELM_NONE;
	m_constr_follower_idx_pbody = -1;	
	m_constr_follower_acc_target.SetZero();
	m_constr_follower_vel_target.SetZero();
	m_constr_follower_pos_target.SetZero();
	m_constr_follower_b_primary = false;

	// constraints on links for editing with IK or user-guided simulation
	m_constr_link_b_pbodies_selected.resize(nL);
	m_constr_link_b_coords_selected.resize(nL);
	m_constr_link_b_coords_primary.resize(nL);
	m_constr_link_type_coords_selected.resize(nL);
	m_constr_link_user_given_SE3_selected.resize(nL);
	m_constr_link_b_control_with_rtmocap.resize(nL);
	m_constr_link_sbj_idx_rtmc_control.resize(nL);
	m_constr_link_b_relative_translation_rtmocap.resize(nL);
	m_constr_link_idx_cube_control = -1;

	for (int i=0; i<nL; i++) {
		m_constr_link_b_pbodies_selected[i] = 0;//false;
		m_constr_link_b_coords_selected[i][0] = true;	// orientation-x on
		m_constr_link_b_coords_selected[i][1] = true;	// orientation-y on
		m_constr_link_b_coords_selected[i][2] = true;	// orientation-z on
		m_constr_link_b_coords_selected[i][3] = false;	// position-x on
		m_constr_link_b_coords_selected[i][4] = false;	// position-y on
		m_constr_link_b_coords_selected[i][5] = false;	// position-z on
		m_constr_link_user_given_SE3_selected[i] = SE3();
		for (int j=0; j<6; j++) { 
			m_constr_link_type_coords_selected[i][j] = LINK_CONSTR_TYPE_USER_POSE;
			m_constr_link_b_coords_primary[i][j] = true;
		}
		m_constr_link_b_control_with_rtmocap[i] = false;
		m_constr_link_sbj_idx_rtmc_control[i] = -1;
		m_constr_link_b_relative_translation_rtmocap[i] = true;
	}

	// constraints on joints for user-guided simulation
	m_constr_joint_b_selected.resize(nA);
	m_constr_joint_type.resize(nA);
	m_constr_joint_b_primary.resize(nA);
	m_constr_joint_user_given_q.resize(nA);
	m_constr_joint_user_given_dq.resize(nA);
	m_constr_joint_user_given_ddq.resize(nA);
	m_constr_joint_user_given_tau.resize(nA);

	for (int i=0; i<nA; i++) {
		m_constr_joint_b_selected[i] = 0;
		m_constr_joint_type[i] = JOINT_CONSTR_TYPE_USER_Q;
		m_constr_joint_b_primary[i] = 1;
		m_constr_joint_user_given_q[i] = 0;
		m_constr_joint_user_given_dq[i] = 0;
		m_constr_joint_user_given_ddq[i] = 0;
		m_constr_joint_user_given_tau[i] = 0;
	}

	// constraints on c.o.m. for user-guided simulation
	m_constr_com_b_selected = false;
	m_constr_com_b_coords_selected[0] = true;
	m_constr_com_b_coords_selected[1] = true;
	m_constr_com_b_coords_selected[2] = true;
	m_constr_com_b_primary[0] = false;
	m_constr_com_b_primary[1] = false;
	m_constr_com_b_primary[2] = false;
	m_constr_com_pos_target.SetZero();
	m_constr_com_sbj_idx_rtmc_control = -1;
	m_constr_com_b_relative_translation_rtmocap = true;

	// constraints on the momentum
	m_constr_momentum_b_selected = false;
	m_constr_momentum_b_coords_selected[0] = false;
	m_constr_momentum_b_coords_selected[1] = false;
	m_constr_momentum_b_coords_selected[2] = false;
	m_constr_momentum_b_coords_selected[3] = false;
	m_constr_momentum_b_coords_selected[4] = false;
	m_constr_momentum_b_coords_selected[5] = false;
	m_constr_momentum_b_primary[0] = false;
	m_constr_momentum_b_primary[1] = false;
	m_constr_momentum_b_primary[2] = false;
	m_constr_momentum_b_primary[3] = false;
	m_constr_momentum_b_primary[4] = false;
	m_constr_momentum_b_primary[5] = false;
	m_constr_momentum_pos_ref.SetZero();
	m_constr_momentum_vel_ref.SetZero();
	m_constr_momentum_H_target.SetZero();

	// virtual spring-damper coefficients for limiting joint displacements in torque-minimizing Cartesian control
	m_K_q_limit.resize(nA);
	m_C_q_limit.resize(nA);
	for (int i=0; i<nA; i++) {
		m_K_q_limit[i] = 0.2;
		m_C_q_limit[i] = 0.05;
	}

	// layer depth for limiting joint displacement in acceleration-minimizing Cartesian control
	m_layer_depth_q_limit.resize(nA);
	for (int i=0; i<nA; i++) {
		m_layer_depth_q_limit[i] = 10./180.*3.14159;
	}
}

void MySystem::init_collision()
{
	// initializes variables for the contact forces on the surface of the links
	for (size_t i=0; i<pLinks.size(); i++) {
		if ( pLinks[i]->bCheckCollision ) {
			for (size_t j=0; j<pLinks[i]->surface.fg.size(); j++) {
				pLinks[i]->surface.fg[j].SetZero();
				pLinks[i]->surface.bContact[j] = 0;
			}
			pLinks[i]->Fe2.SetZero();
		}
	}

	// initializes variables for the contact forces on the surface of the objects
	for (size_t i=0; i<pObjects.size(); i++) {
		if ( pObjects[i] != NULL && pObjects[i]->bCheckCollision ) {
			for (size_t j=0; j<pObjects[i]->surface.fg.size(); j++) {
				pObjects[i]->surface.fg[j].SetZero();
				pObjects[i]->surface.bContact[j] = 0;
			}
			pObjects[i]->Fe2.SetZero();
		}
	}
}

void MySystem::col_rb_surf(RigidBody *prbody, SurfaceFixedToRigidBody *psurf, RigidBody *prbody_surf, double d_lower, double d_upper, bool bapplycontactforce)
{
	bool bc, bContactTmp;
	double tmp_sd, sd;
	Vec3 tmp_contact_normal_vector;
	double d, vn, fn;
	vector<bool> b_overlap_surf_mesh_idx(psurf->getNumMesh());
	int cnt;

	// large scale test (prbody->bounding_box <--> psurf->tri[])
	// find which triangles in ground::surface are overlapping with prbody (update b_overlap_ground_surfac_mesh[])
	cnt = 0;
	for (int i=0; i<psurf->getNumMesh(); i++) {
		b_overlap_surf_mesh_idx[i] = false;
		if ( dcTriOBBOverlap(psurf->xg[psurf->mesh_index[i][0]].GetArray(), psurf->xg[psurf->mesh_index[i][1]].GetArray(), psurf->xg[psurf->mesh_index[i][2]].GetArray(),
			prbody->bounding_box.extents, prbody->bounding_box.position.GetArray(), prbody->bounding_box.orientation.GetArray()) )
		{
			b_overlap_surf_mesh_idx[i] = true;
			cnt++;
		}
	}

	if ( cnt == 0 ) return;

	// small scale test (prbody->surface.xg[] <--> psurf->tri[*])
	for (int i=0; i<prbody->surface.getNumNode(); i++) {

		// visit the overlapping triangles of the ground
		bContactTmp = false;
		sd = -1.0;
		for (int j=0; j<psurf->getNumMesh(); j++) {
			if ( b_overlap_surf_mesh_idx[j] ) {
				bc = dcPointTriangle(tmp_sd, tmp_contact_normal_vector, 
					prbody->surface.xg[i],
					psurf->xg[psurf->mesh_index[j][0]], psurf->xg[psurf->mesh_index[j][1]], psurf->xg[psurf->mesh_index[j][2]],
					d_lower, d_upper);
				if ( bc && tmp_sd > sd ) {
					bContactTmp = true;
					sd = tmp_sd;
					prbody->surface.bContact[i] = 1;
					prbody->surface.peneDepth[i] = -sd;
					prbody->surface.contactNormal[i] = tmp_contact_normal_vector;
				}
			}
		}
	}

	if ( bapplycontactforce ) {
		apply_contact_force(prbody, prbody_surf);
	}
}

void MySystem::apply_contact_force(RigidBody *prbody, RigidBody *prbody_surf)
{
	Vec3 tmp_contact_normal_vector, N, V, Vt, Fc, Pos_local, Fc_reaction_local;
	double d, vn, fn;

	for (int i=0; i<prbody->surface.getNumNode(); i++) {

		// updating contact force acting on the i-th surface node of prbody (the contact force will be transmitted to the object via prbody->update_Fe().)
		// *** contact & friction coefficient scaled *** <-- ??
		if ( prbody->surface.bContact[i] ) {

			d = prbody->surface.peneDepth[i];	// penetration depth, d>=0
			N = prbody->surface.contactNormal[i];
			V = prbody->surface.vg[i];

			if ( prbody_surf != NULL ) {
				Pos_local = Inv(prbody_surf->T_global) * prbody->surface.xg[i];	// position of the surface node viewed from prbody_surf->{body}
				V -= prbody_surf->getVelocityLinearGlobal(Pos_local);	// V becomes relative velocity of the surface node with respect to prbody_surf (still viewed from {global})
			}

			Vt = V - Inner(V, N) * N;
			vn = Inner(V, N); 
			//fn = (d>=0 ? K_rr * d - C_rr * vn : 0);
			if ( d0_nc_rr > 1E-6 ) {
				if ( d < d0_nc_rr ) {
					fn = 0.5 * K_rr / d0_nc_rr * d * d - C_rr * d / d0_nc_rr * vn;
				} else {
					fn = K_rr * (d - 0.5 * d0_nc_rr) - C_rr * vn;
				}
			} else {
				fn = K_rr * d - C_rr * vn;
			}

			Fc = fn * N;	// contact force along the normal direction
			if ( Norm(Vt) > 1E-6 ) {
				Vt.Normalize();
				Fc -= ( Mu_d_rr * fn ) * Vt;	// friction force along the tangential direction
			}

			prbody->surface.fg[i] += Fc;

			// applies reaction force to prbody_surf
			if ( prbody_surf != NULL ) {
				Fc_reaction_local = ~prbody_surf->getOrientationGlobal() * (-Fc);
				prbody_surf->Fe2 += dse3(Cross(Pos_local, Fc_reaction_local), Fc_reaction_local);
			}
		}
	}
}

void MySystem::col_rb_rb(RigidBody *prbodyA, RigidBody *prbodyB, double d_lower, double d_upper, bool bapplycontactforce)
{
	if ( dcOBBOverlap(prbodyA->bounding_box.extents, prbodyA->bounding_box.position.GetArray(), prbodyA->bounding_box.orientation.GetArray(),
					  prbodyB->bounding_box.extents, prbodyB->bounding_box.position.GetArray(), prbodyB->bounding_box.orientation.GetArray()) )
	{
		col_rb_surf(prbodyA, &(prbodyB->surface), prbodyB, d_lower, d_upper, bapplycontactforce);
	}
}

bool MySystem::check_collision(bool bapplycontactforce)
{
	init_collision();

	if ( !b_check_collision ) return true;

	// pLinks <--> ground
	for (size_t i=0; i<pLinks.size(); i++) {
		if ( pLinks[i]->bCheckCollision ) {
			col_rb_surf(pLinks[i], &(ground.surface), NULL, -0.05, 0.0, bapplycontactforce); 
		}
	}

	// pObjects <--> ground
	for (size_t i=0; i<pObjects.size(); i++) {
		if ( pObjects[i]->bCheckCollision ) {
			col_rb_surf(pObjects[i], &(ground.surface), NULL, -0.05, 0.0, bapplycontactforce); 
		}
	}

	// pLinks <--> pObjects
	for (size_t i=0; i<pLinks.size(); i++) {
		for (size_t j=0; j<pObjects.size(); j++) {
			if ( pLinks[i]->bCheckCollision && pObjects[j]->bCheckCollision ) {
				col_rb_rb(pLinks[i], pObjects[j], -0.05, 0.0, bapplycontactforce);
			}
		}
	}

	// pObjects <--> pObjects
	for (size_t i=0; i<pObjects.size(); i++) {
		for (size_t j=i+1; j<pObjects.size(); j++) {
			if ( pObjects[i]->bCheckCollision && pObjects[j]->bCheckCollision ) {
				col_rb_rb(pObjects[i], pObjects[j], -0.05, 0.0, bapplycontactforce);
			}
		}
	}

	return true;
}

static int s_cnt_div = 0;
bool MySystem::is_diverging()
{
	if ( FNorm(system.get_ddqp()) > 1E6 ) { s_cnt_div++; }
	if ( s_cnt_div > 1 ) {
		s_cnt_div = 0;
		return true;
	}
	return false;
}

void MySystem::update_kinematics()
{
	system.updateKinematics();

	com_sys = system.getPositionCOMGlobal();
	if ( b_rndr_show_sys_momentum ) {
		mom_sys_com = system.getMomentumCOM();
	}

	system_objects.updateKinematics();
}

bool MySystem::go_ahead(double t)
{
	if ( !integrate_me(t, step_size) ) return false;
	if ( is_diverging() ) return false;
	return true;
}

bool MySystem::integrate_me(double t, double h_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	if ( !calc_dyna(t) ) return false;

	for (iter_pcoord = system.pCoordinatesUnprescribed.begin(); iter_pcoord != system.pCoordinatesUnprescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->dq += (*iter_pcoord)->ddq * h_;
		(*iter_pcoord)->q += (*iter_pcoord)->dq * h_;		// semi-explicit integration: displacement is updated with the new velocity
	}

	for (iter_pcoord = system_objects.pCoordinatesUnprescribed.begin(); iter_pcoord != system_objects.pCoordinatesUnprescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->dq += (*iter_pcoord)->ddq * h_;
		(*iter_pcoord)->q += (*iter_pcoord)->dq * h_;		// semi-explicit integration: displacement is updated with the new velocity
	}

	return true;
}

bool MySystem::calc_dyna_without_contact()
{
	list<GCoordinate *>::iterator iter_pcoord;

	system.initBodyForcesAndJointTorques();
	system_objects.initBodyForcesAndJointTorques();

	for (size_t i=0; i<pCoordinatesNonActuator.size(); i++) {
		pCoordinatesNonActuator[i]->bPrescribed = false;
		pCoordinatesNonActuator[i]->tau = 0;	// the only torque acting on this coordinate will be a spring-damper force.
	}

	update_kinematics();

	//// check collision and update contact force
	//if ( !check_collision() ) return false;

	// add spring and damper on the actuating coordinates (this will be effective on the unprescribed coordinates only)
	if ( q_actuator_ref.size() == pCoordinatesActuator.size() && K_q_actuator.size() == pCoordinatesActuator.size() && C_dq_actuator.size() == pCoordinatesActuator.size() ) {
		for (size_t i=0; i<pCoordinatesActuator.size(); i++) {
			pCoordinatesActuator[i]->tau += -K_q_actuator[i] * (pCoordinatesActuator[i]->q - q_actuator_ref[i]);
			pCoordinatesActuator[i]->tau += -C_dq_actuator[i] * pCoordinatesActuator[i]->dq;
		}
	}

	// update articulated inertia and force
	system.fsBwdRecursion_b();
	system_objects.fsBwdRecursion_b();

	// update system.pCoordinates[]->(ddqp or taua)
	system.fsFwdRecursion_c();
	system_objects.fsFwdRecursion_c();

	return true;
}

bool MySystem::calc_dyna(double t, bool b_include_disturbance)
{
	list<GCoordinate *>::iterator iter_pcoord;

	system.initBodyForcesAndJointTorques();
	system_objects.initBodyForcesAndJointTorques();

	for (size_t i=0; i<pCoordinatesNonActuator.size(); i++) {
		pCoordinatesNonActuator[i]->bPrescribed = false;
		pCoordinatesNonActuator[i]->tau = 0;	// the only torque acting on this coordinate will be a spring-damper force.
	}

	update_kinematics();

	// unexpected disturbance
	if ( b_include_disturbance ) {
		set_unexpected_disturbance(t);
	}

	// check collision and update contact force
	if ( !check_collision() ) return false;

	// add spring and damper on the actuating coordinates (this will be effective on the unprescribed coordinates only)
	if ( q_actuator_ref.size() == pCoordinatesActuator.size() && K_q_actuator.size() == pCoordinatesActuator.size() && C_dq_actuator.size() == pCoordinatesActuator.size() ) {
		for (size_t i=0; i<pCoordinatesActuator.size(); i++) {
			pCoordinatesActuator[i]->tau += -K_q_actuator[i] * (pCoordinatesActuator[i]->q - q_actuator_ref[i]);
			pCoordinatesActuator[i]->tau += -C_dq_actuator[i] * pCoordinatesActuator[i]->dq;
		}
	}

	// update articulated inertia and force
	system.fsBwdRecursion_b();
	system_objects.fsBwdRecursion_b();

	// update system.pCoordinates[]->(ddqp or taua)
	system.fsFwdRecursion_c();
	system_objects.fsFwdRecursion_c();

	return true;
}

bool MySystem::render(bool b_update_kine_)
{
	if ( !b_rndr_show_system ) return true;

	GLdouble current_color[4];
	GLfloat current_line_width;

	glGetDoublev(GL_CURRENT_COLOR, current_color);
	glGetFloatv(GL_LINE_WIDTH, &current_line_width);

	float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	glMaterialfv(GL_FRONT, GL_SPECULAR, specReflection);
	glMateriali(GL_FRONT, GL_SHININESS, 96);

	double scale_mag_force = 0.05 * rndr_scale_force;
	double scale_mag_vel = 0.2 * rndr_scale_vel;
	double scale_mag_acc = 0.01 * rndr_scale_acc;

	double radius_arrow = 0.006 * rndr_scale_size;
	double radius_arrow_thin = 0.002 * rndr_scale_size;
	double radius_sphere_target_pos = 0.03 * rndr_scale_size;


	if ( b_update_kine_ ) update_kinematics();

	// ========= {global} ========
	glsubCoordinates(rndr_scale_size * 0.1, GLSUB_SOLID, 0.7);

	// ========= ground ================
	if ( b_rndr_show_ground ) {
		ground.render();		
	}

	// ========= object ==========
	for (size_t i=0; i<pObjects.size(); i++) {
		if ( pObjects[i] != NULL ) {

			// body frame
			if ( b_rndr_show_body_frame ) {
				glPushMatrix();
				glMultMatrixd(pObjects[i]->T_global.GetArray());
				glsubCoordinates(rndr_scale_size * 0.07, GLSUB_SOLID, 0.7);
				glPopMatrix();
			}

			// bounding box
			if ( b_rndr_show_bounding_box ) {
				pObjects[i]->b_rndr_bounding_box = true;
			} else {
				pObjects[i]->b_rndr_bounding_box = false;
			}

			// render object
			if ( b_rndr_show_body_surface ) {
				pObjects[i]->b_rndr_surface = true;
			} else {
				pObjects[i]->b_rndr_surface = false;
			}
			if ( b_rndr_show_body_surface_extended ) {
				pObjects[i]->b_rndr_surfaces_for_rendering_only = true;
			} else {
				pObjects[i]->b_rndr_surfaces_for_rendering_only = false;
			}
			pObjects[i]->render();
		}
	}

	// ========= skeleton ========================
	if ( b_rndr_show_skeleton ) {

		// pLinks
		if ( b_rndr_naming ) glPushName(ELM_LINK);
		for (size_t i=0; i<pLinks.size(); i++) {

			// bounding box
			if ( b_rndr_show_bounding_box ) {
				pLinks[i]->b_rndr_bounding_box = true;
			} else {
				pLinks[i]->b_rndr_bounding_box = false;
			}

			// body frame
			if ( b_rndr_show_body_frame ) {
				glPushMatrix();
				glMultMatrixd(pLinks[i]->T_global.GetArray());
				glsubCoordinates(rndr_scale_size * 0.07, GLSUB_SOLID, 0.7);
				glPopMatrix();
			}

			// render link
			if ( b_rndr_show_body_surface ) {
				pLinks[i]->b_rndr_surface = true;
			} else {
				pLinks[i]->b_rndr_surface = false;
			}
			if ( b_rndr_show_body_surface_extended ) {
				pLinks[i]->b_rndr_surfaces_for_rendering_only = true;
			} else {
				pLinks[i]->b_rndr_surfaces_for_rendering_only = false;
			}
			if ( b_rndr_naming ) glPushName(pLinks[i]->getID());
			pLinks[i]->render();
			if ( b_rndr_naming ) glPopName();

			glPushMatrix();
			Vec3 p_com = pLinks[i]->getPositionCOMGlobal();
			glTranslated(p_com[0], p_com[1], p_com[2]);
			// com of each link
			if ( b_rndr_show_body_com ) {
				glColor4d(0, 0, 1, 0.6);	// blue
				glsubSphere(rndr_scale_size * 0.015, GLSUB_SOLID);
			}
			// linear velocity at com
			if ( b_rndr_show_body_com_vel ) {
				glColor4d(1, 0.5, 0, 0.6);		// orange
				Vec3 v = pLinks[i]->getVelocityLinearGlobal(pLinks[i]->getPositionCOM());
				glsubArrow(scale_mag_vel * v, radius_arrow, GLSUB_SOLID);
			}    
			// linear acceleration at com
			if ( b_rndr_show_body_com_acc ) {
				glColor4d(0, 1, 1, 0.6);		// cyan
				Vec3 dv = pLinks[i]->getAccelerationLinearGlobal(pLinks[i]->getPositionCOM()) - GetV(ground.dV);
				glsubArrow(scale_mag_acc * dv, radius_arrow, GLSUB_SOLID);
			}            
			glPopMatrix();

		}
		if ( b_rndr_naming ) glPopName();

		if ( b_rndr_show_joint ) {

			// actuating joints
			if ( b_rndr_naming ) glPushName(ELM_JOINT);
			for (size_t i=0; i<pActuatingJoints.size(); i++) {

				if ( b_rndr_naming ) glPushName(pActuatingJoints[i]->getID());
				pActuatingJoints[i]->render();
				if ( b_rndr_naming ) glPopName();
			}
			if ( b_rndr_naming ) glPopName();

			// other joints
			for (size_t i=0; i<pNonActuatingJoints.size(); i++) {
				pNonActuatingJoints[i]->render();
			}

		}

	}

	// ======== contact force =================
	if ( b_rndr_show_contact_force ) {

		glColor4d(1, 0, 1, 0.6);		// magenta
		
		// contact forces on the skeleton
		if ( b_rndr_show_skeleton ) {
			for (size_t i=0; i<pLinks.size(); i++) {
				for (int j=0; j<pLinks[i]->surface.getNumNode(); j++) {
					glPushMatrix();
					glTranslated(pLinks[i]->surface.xg[j][0], pLinks[i]->surface.xg[j][1], pLinks[i]->surface.xg[j][2]);
					glsubArrow(scale_mag_force*pLinks[i]->surface.fg[j], radius_arrow_thin, GLSUB_SOLID);
					glPopMatrix();
				}
			}
		}
		
		// contact forces on the objects
		for (size_t i=0; i<pObjects.size(); i++) {
			for (int j=0; j<pObjects[i]->surface.getNumNode(); j++) {
				glPushMatrix();
				glTranslated(pObjects[i]->surface.xg[j][0], pObjects[i]->surface.xg[j][1], pObjects[i]->surface.xg[j][2]);
				glsubArrow(scale_mag_force*pObjects[i]->surface.fg[j], radius_arrow_thin, GLSUB_SOLID);
				glPopMatrix();
			}
		}

	}

	//// ======== contact force by self-collision ===========
	//if ( b_rndr_show_contact_force && b_rndr_show_dbody_ebody ) {
	//	glColor4d(1, 0, 1, 0.6);		// magenta
	//	for (size_t i=0; i<pDeformableBodies.size(); i++) {
	//		for (int j=0; j<pDeformableBodies[i]->ebody.getNumNode(); j++) {
	//			glPushMatrix();
	//			glTranslated(pDeformableBodies[i]->ebody.Nodes[j].xg[0], pDeformableBodies[i]->ebody.Nodes[j].xg[1], pDeformableBodies[i]->ebody.Nodes[j].xg[2]);
	//			glsubArrow(scale_mag_force*pDeformableBodies[i]->ebody.Nodes[j].contactForce, radius_arrow_thin, GLSUB_SOLID);
	//			glPopMatrix();
	//		}
	//	}
	//}

	// ======== center of mass and momentum ==========
	//Vec3 com_sys = system.getPositionCOMGlobal();
	//dse3 mom_sys_com = system.getMomentumCOM();

	glPushMatrix();
		glTranslated(com_sys[0], com_sys[1], com_sys[2]);
		// com
		if ( b_rndr_show_sys_com ) {
			glColor4d(rndr_color_com[0], rndr_color_com[1], rndr_color_com[2], 1);
			glLineWidth(rndr_line_width_com);
			if ( b_rndr_naming ) glPushName(ELM_COM);
			glsubSphere(rndr_scale_size * 0.06, GLSUB_SOLID);
			glPopName();
		}
		// momentum
		if ( b_rndr_show_sys_momentum ) {
			glsubCoordinates(rndr_scale_size * 0.1, GLSUB_SOLID, 0.6);
			glColor4d(1,0,0.5,1);
			glsubArrow((10.*scale_mag_force)*GetF(mom_sys_com), radius_arrow, GLSUB_SOLID);
			glColor4d(0.5,0,1,1);
			glsubArrowDoubleHead((50.*scale_mag_force)*GetM(mom_sys_com), radius_arrow, GLSUB_SOLID);
			//glsubSpiral3D((500.*scale_mag_force)*GetM(mom_sys_com), Vec3(0,1,0), radius_arrow*6, radius_arrow*9, radius_arrow, 32, 8, 1, 1, GLSUB_SOLID);
		}
	glPopMatrix();

	// ======== direct control =========
	glPushMatrix();
		// desired position
		glTranslated(m_desired_position[0], m_desired_position[1], m_desired_position[2]);
		glColor4d(1, 0, 0, 1);
		glLineWidth(rndr_line_width_com);
		glsubSphere(rndr_scale_size * 0.1, GLSUB_SOLID);
	glPopMatrix();


	// ======= restore previous setting ======
	glColor4dv(current_color);
	glLineWidth(current_line_width);

	return true;
}

void MySystem::renderLinkConstraints()
{
	double s = rndr_scale_size;

	// link constraints
	for (size_t i=0; i<pLinks.size(); i++) {

		if ( m_constr_link_b_pbodies_selected[i] ) {

			glPushMatrix();

			glMultMatrixd(pLinks[i]->T_global.GetArray());
			Vec3 x = pLinks[i]->getMarkerPosition();
			glTranslated(x[0], x[1], x[2]);

			// show black sphere if there is no constraints
			if ( !m_constr_link_b_coords_selected[i][0] && !m_constr_link_b_coords_selected[i][1] && !m_constr_link_b_coords_selected[i][2] 
			&& !m_constr_link_b_coords_selected[i][3] && !m_constr_link_b_coords_selected[i][4] && !m_constr_link_b_coords_selected[i][3] ) 
			{			 
				glColor4d(0,0,0,1);
				glsubSphere(s*0.02, GLSUB_SOLID);
			}

			// show position constraint
			if ( m_constr_link_b_coords_selected[i][3] || m_constr_link_b_coords_selected[i][4] || m_constr_link_b_coords_selected[i][3] ) {

				// set color
				// red = all selected and all primary
				// green = all selected and all secondary
				// yellow = mixed case
				if ( m_constr_link_b_coords_selected[i][3] && m_constr_link_b_coords_selected[i][4] && m_constr_link_b_coords_selected[i][5] ) {
					if ( m_constr_link_b_coords_primary[i][3] && m_constr_link_b_coords_primary[i][4] && m_constr_link_b_coords_primary[i][5] ) {
						glColor4d(1,0,0,1);	
					} else if ( !m_constr_link_b_coords_primary[i][3] && !m_constr_link_b_coords_primary[i][4] && !m_constr_link_b_coords_primary[i][5] ) {
						glColor4d(0,1,0,1);
					} else {
						glColor4d(1,1,0,1);
					}
				} else {
					glColor4d(1,1,0,1);	
				}

				glsubSphere(s*0.03, GLSUB_SOLID);
			}

			// show orientation constraint (current orientation)
			if ( m_constr_link_b_coords_selected[i][0] || m_constr_link_b_coords_selected[i][1] || m_constr_link_b_coords_selected[i][2] ) {

				// set color
				// red = all selected and all primary
				// green = all selected and all secondary
				// yellow = mixed case
				if ( m_constr_link_b_coords_selected[i][0] && m_constr_link_b_coords_selected[i][1] && m_constr_link_b_coords_selected[i][2] ) {
					if ( m_constr_link_b_coords_primary[i][0] && m_constr_link_b_coords_primary[i][1] && m_constr_link_b_coords_primary[i][2] ) {
						glColor4d(1,0,0,1);	
					} else if ( !m_constr_link_b_coords_primary[i][0] && !m_constr_link_b_coords_primary[i][1] && !m_constr_link_b_coords_primary[i][2] ) {
						glColor4d(0,1,0,1);
					} else {
						glColor4d(1,1,0,1);
					}
				} else {
					glColor4d(1,1,0,1);	
				}

				glsubFrameCube(s*0.07, s*0.007, GLSUB_SOLID);
			} 

			glPopMatrix();

			// show target position and orientation
			bool b_show_target_pos = false;
			bool b_show_target_ori = false;
			if ( m_constr_link_b_coords_selected[i][3] && m_constr_link_b_coords_selected[i][4] && m_constr_link_b_coords_selected[i][5] ) {
				b_show_target_pos = true;
			}
			if ( m_constr_link_b_coords_selected[i][0] && m_constr_link_b_coords_selected[i][1] && m_constr_link_b_coords_selected[i][2] ) {
				b_show_target_ori = true;
			}

			glPushMatrix();
			glColor4d(0,0,1,1);	// blue
			if ( b_show_target_pos && b_show_target_ori ) {
				glMultMatrixd(m_constr_link_user_given_SE3_selected[i].GetArray());
				glsubFrameCube(s*0.12, s*0.005, GLSUB_SOLID);
			} else if ( !b_show_target_pos && b_show_target_ori ) {
				glMultMatrixd(SE3(m_constr_link_user_given_SE3_selected[i].GetRotation(), pLinks[i]->getMarkerPositionGlobal()).GetArray());
				glsubFrameCube(s*0.12, s*0.005, GLSUB_SOLID);
			} else if ( b_show_target_pos && !b_show_target_ori ) {
				Vec3 xg = m_constr_link_user_given_SE3_selected[i].GetPosition();
				glTranslated(xg[0],xg[1],xg[2]);
				glsubSphere(s*0.03, GLSUB_SOLID);
			} else {
				;
			}
			glPopMatrix();

			//// show orientation constraint (target orientation)
			//if ( m_constr_link_type_coords_selected[i][0] == LINK_CONSTR_TYPE_USER_POSE && 
			//	m_constr_link_type_coords_selected[i][1] == LINK_CONSTR_TYPE_USER_POSE &&
			//	m_constr_link_type_coords_selected[i][2] == LINK_CONSTR_TYPE_USER_POSE ) {

			//	glPushMatrix();
			//	Vec3 xg;
			//	if ( b_show_target_pos ) {
			//		glMultMatrixd(m_constr_link_user_given_SE3_selected[i].GetArray());
			//	} else {
			//		xg = pLinks[i]->getMarkerPositionGlobal();
			//	}
			//	SO3 Rg = m_constr_link_user_given_SE3_selected[i].GetRotation();
			//	glMultMatrixd(SE3(Rg, xg).GetArray());

			//	// set color
			//	if ( i == m_constr_link_idx_cube_control ) {
			//		glColor4d(0,0,0,1);	// black if cube-controlled
			//	} else {
			//		glColor4d(0,0,1,1);	// blue
			//	}

			//	glsubFrameCube(s*0.12, s*0.005, GLSUB_SOLID);

			//	glPopMatrix();
			//}
		}
	}
}

void MySystem::renderJointConstraints()
{
	double s = rndr_scale_size;
	int idx;
	Vec3 x;

	for (size_t i=0; i<pCoordinatesActuator.size(); i++) {

		if ( m_constr_joint_b_selected[i] ) {

			idx = -1;
			for (size_t j=0; j<pActuatingJoints.size(); j++) {
				if ( find(pActuatingJoints[j]->pCoordinates.begin(), pActuatingJoints[j]->pCoordinates.end(), pCoordinatesActuator[i]) != pActuatingJoints[j]->pCoordinates.end() ) {
					idx = j;
					break;
				}
			}

			if ( idx >= 0 ) {
				x = pActuatingJoints[idx]->T_global.GetPosition();

				if ( m_constr_joint_b_primary[i] ) {
					glColor4d(1,0,0,1);
				} else {
					glColor4d(0,1,0,1);
				}

				glPushMatrix();
				glTranslated(x[0], x[1], x[2]);
				glLineWidth(4);
				glsubBox(s*0.08, s*0.08, s*0.08, GLSUB_WIRE, 1, 1, 1);
				//glsubTorus(Vec3(1,0,0), s*0.04, s*0.01, GLSUB_SOLID);
				glLineWidth(1);
				glPopMatrix();
			}
		}
	}
}

void MySystem::renderCOMConstraints()
{
	double s = rndr_scale_size;

	if ( m_constr_com_b_selected ) {
		// set color
		// red = all selected and all primary
		// green = all selected and all secondary
		// yellow = mixed case
		if ( m_constr_com_b_coords_selected[0] && m_constr_com_b_coords_selected[1] && m_constr_com_b_coords_selected[2] ) {
			if ( m_constr_com_b_primary[0] && m_constr_com_b_primary[1] && m_constr_com_b_primary[2] ) {
				glColor4d(1,0,0,1);	
			} else if ( !m_constr_com_b_primary[0] && !m_constr_com_b_primary[1] && !m_constr_com_b_primary[2] ) {
				glColor4d(0,1,0,1);
			} else {
				glColor4d(1,1,0,1);
			}
		} else {
			glColor4d(1,1,0,1);	
		}

		// show constraints on COM
		Vec3 com_sys = system.getPositionCOMGlobal();
		glPushMatrix();
		glTranslated(com_sys[0], com_sys[1], com_sys[2]);
		glsubSphere(s*0.05, GLSUB_SOLID);
		glPopMatrix();

		// show target position (blue)
		glColor4d(0,0,1,1);
		glPushMatrix();
		glTranslated(m_constr_com_pos_target[0], m_constr_com_pos_target[1], m_constr_com_pos_target[2]);
		glsubSphere(s*0.04, GLSUB_SOLID);
		glPopMatrix();
	}
}

void MySystem::renderFollower(bool b_show_follower_, bool b_show_target_pos_, bool b_show_target_vel_, bool b_show_target_acc_)
{
	Vec3 x(0,0,0);	

	// get the location of the follower
	switch ( m_constr_follower_elem_type ) {
		case ELM_LINK:
			if ( m_constr_follower_idx_pbody < 0 ) {
				cerr << "body index error!!!" << endl;
				return;
			}
			x = getFollowerLink()->getMarkerPositionGlobal();
			break;
		case ELM_COM:
			x = system.getPositionCOMGlobal();
			break;
		default:
			return;
	}

	glPushMatrix();
	glTranslated(x[0], x[1], x[2]);
	// sphere on the follower
	if ( b_show_follower_ ) {
		if ( m_constr_follower_b_primary ) {
			glColor4d(1,0,0,1);		// red for primary
		} else {
			glColor4d(0,1,0,1);		// green for secondary
		}
		double L = rndr_scale_size * 0.03;
		glsubSphere(L, GLSUB_SOLID);
	}
	// oragne arrow for the target velocity
	if ( b_show_target_vel_ ) {
		glColor4d(1,0.5,0,1); 
		glsubArrow(0.2*rndr_scale_vel*m_constr_follower_vel_target, 0.002, GLSUB_WIRE);
	}
	// green arrow for the target acceleration
	if ( b_show_target_acc_ ) {
		glColor4d(0,1,0,1);
		glsubArrow(0.002*rndr_scale_acc*m_constr_follower_acc_target, 0.002, GLSUB_WIRE);
	}
	glPopMatrix();

	// show the target position
	glPushMatrix();
	glTranslated(m_constr_follower_pos_target[0], m_constr_follower_pos_target[1], m_constr_follower_pos_target[2]);
	// blue sphere to the target position
	if ( b_show_target_pos_ ) {
		glColor4d(0,0,1,1);	
		double R = rndr_scale_size * 0.03;
		glsubSphere(R, GLSUB_SOLID);
	}
	glPopMatrix();
}

void MySystem::renderPositionOfTemporalRootJoint()
{
	glColor4d(0,0,0,1);
	glPushMatrix();
	glTranslated(pos_temporal_root_joint[0], pos_temporal_root_joint[1], pos_temporal_root_joint[2]);
	glsubSphere(rndr_scale_size * 0.05, GLSUB_WIRE);
	glPopMatrix();
}

/*
bool MySystem::init_fixed_surface_rendering()
{
	_idx_ref_link.resize(pDeformableBodies.size());
	_r0.resize(pDeformableBodies.size());

	for (size_t i=0; i<pDeformableBodies.size(); i++) {

		_idx_ref_link[i].resize(pDeformableBodies[i]->surf_embedded.Nodes.size());
		for (size_t j=0; j<pDeformableBodies[i]->surf_embedded.Nodes.size(); j++) {
			double d = 1E10;
			// find the nearest link from each surface nodes
			_idx_ref_link[i][j] = -1;
			for (int k=0; k<pLinks.size(); k++) {
				if ( Norm(pDeformableBodies[i]->surf_embedded.Nodes[j].x - pLinks[k]->getPositionCOMGlobal()) < d ) {
					d = Norm(pDeformableBodies[i]->surf_embedded.Nodes[j].x - pLinks[k]->getPositionCOMGlobal());	
					_idx_ref_link[i][j] = k;
				}
			}
			if ( _idx_ref_link[i][j] < 0 ) return false;
		}

		_r0[i].resize(pDeformableBodies[i]->surf_embedded.Nodes.size());
		for (size_t j=0; j<pDeformableBodies[i]->surf_embedded.Nodes.size(); j++) {
			_r0[i][j] = Inv(pLinks[_idx_ref_link[i][j]]->T_global) * pDeformableBodies[i]->surf_embedded.Nodes[j].x;
		}
	}

	return true;
}

void MySystem::update_fixed_surface_position()
{
	for (size_t i=0; i<pDeformableBodies.size(); i++) {
		for (size_t j=0; j<pDeformableBodies[i]->surf_embedded.Nodes.size(); j++) {
			pDeformableBodies[i]->surf_embedded.Nodes[j].x = pLinks[_idx_ref_link[i][j]]->T_global * _r0[i][j];
		}
	}
}
*/
/*
bool MySystem::build_surface_patch()
{
	int idx;
	Surface *psurf;

	surface_patch.resize(pLinks.size());
	for (size_t i=0; i<surface_patch.size(); i++) {
		surface_patch[i].pSurfNodes.clear();
		surface_patch[i].smesh_index.clear();
	}

	for (size_t i=0; i<pDeformableBodies.size(); i++) {
		psurf = &(pDeformableBodies[i]->ebody.surface);
		for ( size_t j=0; j<psurf->pSurfNodes.size(); j++ ) {
			idx = -1;
			for (int k=0; k<pLinks.size(); k++) {
				if ( psurf->pSurfNodes[j]->pParentBody == pLinks[k] ) { idx = k; break; }
			}
			if ( idx < 0 ) return false;
			surface_patch[idx].pSurfNodes.push_back(psurf->pSurfNodes[j]);
		}
	}

	for (size_t i=0; i<pDeformableBodies.size(); i++) {
		psurf = &(pDeformableBodies[i]->ebody.surface);
		for (size_t j=0; j<psurf->smesh_index.size(); j++) {
			for (int k=0; k<3; k++) {
				idx = -1;
				for (int m=0; m<pLinks.size(); m++) {
					if ( psurf->pSurfNodes[psurf->smesh_index[j][k]]->pParentBody == pLinks[m] ) { idx = m; break; }
				}
				if ( idx < 0 ) return false;
				for (int m=0; m<3; m++) {
					if ( surface_patch[idx].pSurfNodes.find(psurf->pSurfNodes[psurf->smesh_index[j][m]]) < 0 ) {
						surface_patch[idx].pSurfNodes.push_back(psurf->pSurfNodes[psurf->smesh_index[j][m]]);
					}
				}
			}
		}
	}

	for (size_t i=0; i<pDeformableBodies.size(); i++) {
		psurf = &(pDeformableBodies[i]->ebody.surface);
		for (size_t j=0; j<psurf->smesh_index.size(); j++) {
			for (int k=0; k<3; k++) {
				idx = -1;
				for (int m=0; m<pLinks.size(); m++) {
					if ( psurf->pSurfNodes[psurf->smesh_index[j][k]]->pParentBody == pLinks[m] ) { idx = m; break; }
				}
				if ( idx < 0 ) return false;
				surface_patch[idx].smesh_index.push_back( _triad<int> (
					surface_patch[idx].pSurfNodes.find(psurf->pSurfNodes[psurf->smesh_index[j][0]]),
					surface_patch[idx].pSurfNodes.find(psurf->pSurfNodes[psurf->smesh_index[j][1]]),
					surface_patch[idx].pSurfNodes.find(psurf->pSurfNodes[psurf->smesh_index[j][2]])	));
			}
		}
	}
}
*/

void MySystem::releaseConstraintsForUserGuidedSimulControl()
{
	int nL = pLinks.size();
	int nJ = pCoordinatesActuator.size();

	m_constr_follower_elem_type = ELM_NONE;
	m_constr_follower_idx_pbody = -1;

	for (int i=0; i<nL; i++) {
		m_constr_link_b_pbodies_selected[i] = 0;//false;
		for (int j=0; j<6; j++) {
			m_constr_link_b_coords_selected[i][j] = false;
		}
	}

	for (int i=0; i<nJ; i++) {
		m_constr_joint_b_selected[i] = 0;
	}

	m_constr_com_b_selected = false;
	for (int i=0; i<3; i++) {
		m_constr_com_b_coords_selected[i] = false;
	}

	m_constr_momentum_b_selected = false;
	for (int i=0; i<6; i++) {
		m_constr_momentum_b_coords_selected[i] = false;
	}
}

int MySystem::getIndexLink(RigidBody *pbody_)
{
	for (size_t i=0; i<pLinks.size(); i++) {
		if ( pbody_ == pLinks[i] ) return i;
	}

	return -1;
}

void MySystem::print_sys_info(ofstream *pfout)
{
	if ( pfout == NULL ) return;

	list<GBody*>::iterator iter_pbody;
	double total_mass = 0.0;
	for (iter_pbody = system.pBodies.begin(); iter_pbody != system.pBodies.end(); iter_pbody++) {
		total_mass += (*iter_pbody)->getMass();
	}

	*pfout << endl;
	*pfout << "//--------- MySystem Information ----------" << endl;
	*pfout << "//" << endl;
	*pfout << "// total mass = " << total_mass << " kg" << endl;
	*pfout << "//" << endl;
	*pfout << "// total DOF = " << system.getNumCoordinates() << endl;
	*pfout << "//" << endl;
	*pfout << "// C_particle_damper = " << C_particle_damper << endl;
	*pfout << "//" << endl;
	*pfout << "// K_dr = " << K_dr << endl;
	*pfout << "// C_dr = " << C_dr << endl;
	*pfout << "//" << endl;
	*pfout << "// Mu_s_dr = " << Mu_s_dr << endl;
	*pfout << "// Mu_d_dr = " << Mu_d_dr << endl;
	*pfout << "//" << endl;
	*pfout << "----------------------------------------" << endl;
	*pfout << endl;
}

void MySystem::print_sys_state(ofstream *pfout)
{
	list<GBody*>::iterator iter_pbody;
	list<GJoint*>::iterator iter_pjoint;
	list<GCoordinate*>::iterator iter_pcoord;
	GBody *prbody;
	GJoint *pjoint;

	if ( pfout == NULL ) return;

//	*pfout << system.getInfoStr() << endl;

	*pfout << "//----------------------------------------" << endl;
	*pfout << "//         Detailed System State          " << endl;
	*pfout << "//----------------------------------------" << endl;
	*pfout << endl;
	*pfout << "// ****** skeleton.pJoints ******" << endl;
	*pfout << endl;

	for (iter_pjoint = skeleton.pJoints.begin(); iter_pjoint != skeleton.pJoints.end(); iter_pjoint++) {
		pjoint = *iter_pjoint;
		*pfout << "// ------------------" << endl;
		*pfout << "// name of joint = " << pjoint->getName() << endl;
		*pfout << "// is prescribed? " << (pjoint->bPrescribed ? "yes" : "no") << endl;
		*pfout << "// q = [ ";
		for (iter_pcoord = pjoint->pCoordinates.begin(); iter_pcoord != pjoint->pCoordinates.end(); iter_pcoord++) {
			*pfout << (*iter_pcoord)->q << ", ";
		}
		*pfout << " ];" << endl;
		*pfout << "// dq = [ ";
		for (iter_pcoord = pjoint->pCoordinates.begin(); iter_pcoord != pjoint->pCoordinates.end(); iter_pcoord++) {
			*pfout << (*iter_pcoord)->dq << ", ";
		}
		*pfout << " ];" << endl;
		*pfout << "// ddq = [ ";
		for (iter_pcoord = pjoint->pCoordinates.begin(); iter_pcoord != pjoint->pCoordinates.end(); iter_pcoord++) {
			*pfout << (*iter_pcoord)->ddq << ", ";
		}
		*pfout << " ];" << endl;
		*pfout << "// tau = [ ";
		for (iter_pcoord = pjoint->pCoordinates.begin(); iter_pcoord != pjoint->pCoordinates.end(); iter_pcoord++) {
			*pfout << (*iter_pcoord)->tau << ", ";
		}
		*pfout << " ];" << endl;
		*pfout << endl;
	}

	*pfout << "// ****** skeleton.pBodies ******" << endl;
	*pfout << endl;

	for (iter_pbody = skeleton.pBodies.begin(); iter_pbody != skeleton.pBodies.end(); iter_pbody++) {
		prbody = (GBody*)(*iter_pbody);
		*pfout << "// name of body = " << prbody->getName() << endl;
		*pfout << "// base joint = " << prbody->pBaseJoint->getName() << endl;
		*pfout << "// Is the base joint prescribed? " << (prbody->pBaseJoint->isPrescribed() ? "yes" : "no") << endl;
		*pfout << "// I = " << prbody->I << endl;
		*pfout << "// Fe = " << prbody->Fe << endl;
		*pfout << "// T = " << prbody->T << endl;
		*pfout << "// V = " << prbody->V << endl;
		*pfout << "// dV = " << prbody->dV << endl;
		*pfout << "// F = " << prbody->F << endl;
		*pfout << "// aI = " << prbody->aI << endl;
		*pfout << "// aB = " << prbody->aB << endl;
		*pfout << "// eta = " << prbody->eta << endl;
		*pfout << "// Psi = " << prbody->Psi << endl;
		*pfout << "// Pi = " << prbody->Pi << endl;
		*pfout << "// beta = " << prbody->beta << endl;
		*pfout << endl;
	}

	*pfout << "//----------------------------------------" << endl;
}

static int cnt_frames_for_updating_command_input=0;

bool MySystem::stepSimulation()
{
	list<GCoordinate*>::iterator iter_pcoord;
	double h = step_size;

	if ( ++cnt_frames_for_updating_command_input > m_simul_num_skip_frames_for_updating_command_input ) {
		if ( !setCommandInputForCartesianControl() ) {
			cerr << "Error:: Failed in setting up command input." << endl;
		}
		cnt_frames_for_updating_command_input = 0;
	}

	// update system.pCoordinates[]->(ddqp and taua)
	calc_dyna(0);	

	//RMatrix ddqa1(pCoordinatesActuator.size(),1), ddqa2(pCoordinatesActuator.size(),1);
	//for (size_t i=0; i< pCoordinatesActuator.size(); i++) {
	//	ddqa1[i] = pCoordinatesActuator[i]->ddq;
	//}

	//set_actuator_prescribed(false);
	//calc_dyna();	
	//for (size_t i=0; i< pCoordinatesActuator.size(); i++) {
	//	ddqa2[i] = pCoordinatesActuator[i]->ddq;
	//}

	//fout << "t = " << m_simul_current_time << endl << "ddqa1 = " << ~ddqa1 << "ddqa2 = " << ~ddqa2 << endl;

	//for (iter_pcoord = system.pCoordinatesUnprescribed.begin(); iter_pcoord != system.pCoordinatesUnprescribed.end(); iter_pcoord++) {
	//	(*iter_pcoord)->dq += h * (*iter_pcoord)->ddq;
	//	(*iter_pcoord)->q += h * (*iter_pcoord)->dq;
	//}
	//set_actuator_prescribed(true);

	// update system state by integrating acceleration
	for (iter_pcoord = system.pCoordinates.begin(); iter_pcoord != system.pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->dq += h * (*iter_pcoord)->ddq;
		(*iter_pcoord)->q += h * (*iter_pcoord)->dq;
	}

	for (iter_pcoord = system_objects.pCoordinates.begin(); iter_pcoord != system_objects.pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->dq += h * (*iter_pcoord)->ddq;
		(*iter_pcoord)->q += h * (*iter_pcoord)->dq;
	}

	if ( is_diverging() ) { 
		cerr << "Error:: System diverging!" << endl;
		return false; 
	}

	// compensate joint-loop constraints
	if ( !compensateJointLoopConstraintError() ) { 
		cerr << "Error:: Failed in calling compensateJointLoopConstraintError()." << endl;
		return false; 
	}

	return true;
}

bool MySystem::setCommandInputForCartesianControl()
{
	// approximate contact model for planning of Cartesian control
	check_collision(false); // check collision without applying contact forces
	approx_contact_model(true);	// set approximate contact model by changing root joint

	// set command input on the actuating coordinates
	if ( !CartesianControlWithMinimalAcceleration() ) { 
		cerr << "Error:: Failed in calling CartesianControlWithMinimalAcceleration()." << endl;
		return false;
	}

	// restore the original system
	approx_contact_model(false);

	return true;
}

bool MySystem::CartesianControlWithMinimalAcceleration()
{
	int na = pCoordinatesActuator.size();
	RMatrix qa(na,1), dqa(na,1), ddqa(na,1);
	double h = step_size;

	// initialize
	bool b_setting_prev = b_actuators_prescribed;
	set_actuator_prescribed(true);
	for (int i=0; i<na; i++) {
		qa[i] = pCoordinatesActuator[i]->q;
		dqa[i] = pCoordinatesActuator[i]->dq;
	}
	ddqa.SetZero();

	// build constraints
	if ( !buildQP() ) {
		cerr << "Error::Failed in building QP problem!" << endl;
		return false;
	}

	// solve QP
	if ( !solveQP(ddqa, m_simul_alpha_acc) ) {
		cerr << "Error:: Failed in solving QP." << endl;
		return false;
	}

	// restore the coordinate setting
	set_actuator_prescribed(b_setting_prev);

	// update the command input
	if ( b_actuators_prescribed ) {
		for (int i=0; i<na; i++) {
			pCoordinatesActuator[i]->ddq = ddqa[i];
		}
	} else {
		RMatrix taua = m_simul_DtauaDx * ddqa + m_simul_taua0;

		for (int i=0; i<na; i++) {
			pCoordinatesActuator[i]->tau = 0.5*taua[i];

			//if ( pBodiesDistalOfpCoordinatesActuator[i] == NULL ) {
			//	fout << "null..." << endl;
			//} 
			//int idx = idxLocalOfpCoordinatesActuator[i];
			//GBodyRigid *pbody = pBodiesDistalOfpCoordinatesActuator[i];
			//RMatrix sJs = ~pbody->S * convert_to_RMatrix(pbody->aI) * pbody->S;
//			fout << "i = " << i << ", sJs = " << sJs(idx,idx) << endl;
//			pCoordinatesActuator[i]->tau = sJs(idx,idx) * ddqa[i];
			//double Kp = 1, Kv = 30; 
			//pCoordinatesActuator[i]->tau = Kp*h*(dqa[i]+h*ddqa[i]) + Kv*h*ddqa[i];	// taua = Kp*(qa_next - qa) + Kv*(dqa_next - dqa)
		}
	}

	return true;
}

bool MySystem::CartesianControlWithMinimalVelocity()
{
	RMatrix dqa_next, A0, Ap, As, C, b0, bp, bs, d;
	double h = step_size;
	int na = pCoordinatesActuator.size();

	// initialize
	bool b_setting_prev = b_actuators_prescribed;
	set_actuator_prescribed(true);
	dqa_next.SetZero(na, 1);

	// build constraints
	if ( !buildQP() ) {
		cerr << "Error:: Failed in building QP!" << endl;
		return false;
	}

	// reformulate the constraints on velocity
	if ( !reformQP_vel() ) {
		cerr << "Error:: Failed in reforming QP with respect to velocity!" << endl;
		return false;
	}

	// solve QP
	if ( !solveQP(dqa_next, h*m_simul_alpha_acc) ) {
		cerr << "Error:: Failed in solving QP." << endl;
		return false;
	}

	// restore the coordinate setting
	set_actuator_prescribed(b_setting_prev);

	// set acceleration for the active coordinates with dqa_desired
	for (size_t i=0; i<pCoordinatesActuator.size(); i++) {
		pCoordinatesActuator[i]->ddq = (1./h) * (dqa_next[i] - pCoordinatesActuator[i]->dq);
	}

	return true;
}

bool MySystem::buildQP()
{
	int i, j, nb, np, ns, cntb, cntp, cnts;
	bool b_link, b_com, b_joint, b_jloop, b_momentum;
	Vec3 pos_current, vel_current, acc_0;
	RMatrix A_tmp, b_tmp;
	list<GCoordinate*>::iterator iter_pcoord;

	// pos_command = desired target position
	Vec3 pos_command = m_desired_position;

	// update target for links, com and momentum
	vector<SE3> link_target_pose = m_constr_link_user_given_SE3_selected;
	Vec3 pos_com_target = m_constr_com_pos_target;

	// ** u = joint command input = pcoord_control[]->(ddq|tau)
	vector<RMatrix> DdVDu;	// DdVDu = derivatives of the link acceleration in {global} w.r.t. x = pcoord_control[]->(ddq|tau)

	// for joint limit and torque limit
	RMatrix ddqa0;			// ddqa0 = actuator acceleration when x = 0
	RMatrix taua0;			// taua0 = actuator torque when x = 0
	RMatrix DddqaDu;		// DddqaDu = derivatives of the actuator acceleration w.r.t. u
	RMatrix DtauaDu;		// DtauaDu = derivatives of the actuator torque w.r.t. u

	// for com control
	RMatrix DddxcDu;		// DddxcDu = derivatives of the acceleration of the center of mass w.r.t. u
	Vec3 xc_current, vc_current, ddxc_0;	// current position, velocity and acceleration(when u=0) of the center of mass of the skeleton
	Vec3 com_acc_target;	// target acceleration of the center of mass of the skeleton

	// for momentum control
	dse3 Hp_current;	// current momentum about p where p = m_constr_momentum_pos_ref
	RMatrix DdHpDu;		// DddHcDu = derivatives of the [time derivative of the system momentum about p] w.r.t. u
	dse3 dHp_0;			// time derivative of the system momentum about p when u=0

	RigidBody *pbody;
	RigidBody *pground = &(ground);

	vector<GCoordinate*> pcoord_control(pCoordinatesActuator);	

	// set u = 0
	for (i=0; i<pcoord_control.size(); i++) {
		if ( pcoord_control[i]->bPrescribed ) {
			pcoord_control[i]->ddq = 0;
		} else {
			pcoord_control[i]->tau = 0;
		}
	}

	int na = pcoord_control.size();

	// -------------------------------------------------------------------------
	// count the number of basic/primary/secondary constraints
	// -------------------------------------------------------------------------

	nb = np = ns = 0; 
	b_jloop = b_link = b_joint = b_com = b_momentum = false;

	// joint loops
	for (i=0; i<pJointLoops.size(); i++) {
		nb += pJointLoops[i]->getNumConstraints();
		b_jloop = true;
	}

	//// fixed contacts
	//for (i=0; i<contactPositions.size(); i++) {
	//	for (j=0; j<contactPositions[i].size(); j++) {
	//		nb += 3;
	//	}
	//}

	// constraints imposed on links
	for (i=0; i<pLinks.size(); i++) {
		if ( m_constr_link_b_pbodies_selected[i] ) {
			for (j=0; j<6; j++) {
				if ( m_constr_link_b_coords_selected[i][j] ) {
					if ( m_constr_link_b_coords_primary[i][j] ) 
						np++;
					else
						ns++;
				}
			}
			b_link = true;
		}
	}

	// constraints on actuating joints
	for (i=0; i<na; i++) {
		if ( m_constr_joint_b_selected[i] ) {
			if ( m_constr_joint_b_primary[i] ) {
				np++;
			} else {
				ns++;
			}
			b_joint = true;
		}
	}

	// constraints on C.O.M.
	if ( m_constr_com_b_selected ) {
		for (i=0; i<3; i++) {
			if ( m_constr_com_b_coords_selected[i] ) {
				if ( m_constr_com_b_primary[i] ) {
					np++;
				} else {
					ns++;
				}
				b_com = true;
			}
		}
	}

	// constraints on the follower link
	if ( m_constr_follower_elem_type == ELM_LINK && m_constr_follower_idx_pbody >= 0 ) {
		if ( m_constr_follower_b_primary ) {
			np += 3;
		} else {
			ns += 3;
		}
		b_link = true;
	}

	// constraints on the follower C.O.M.
	if ( m_constr_follower_elem_type == ELM_COM ) {
		if ( m_constr_follower_b_primary ) {
			np += 3;
		} else {
			ns += 3;
		}
		b_com = true;
	}

	// constraints on the momentum
	if ( m_constr_momentum_b_selected ) {
		for (int i=0; i<6; i++) {
			if ( m_constr_momentum_b_coords_selected[i] ) {
				if ( m_constr_momentum_b_primary[i] ) {
					np++;
				} else {
					ns++;
				}
			}
		}
		b_momentum = true;
	}


	// -------------------------------------------------------------------------
	// prepare stuffs for building constraints equations
	// -------------------------------------------------------------------------

	// calculate dynamics (taua, ddqp) with zero command(ddqa=0, taup=0)
	if ( !calc_dyna(0, false) ) return false; // without unexpected disturbance
//	if ( !calc_dyna_without_contact() ) return false;

	ddqa0.SetZero(na, 1); 
	taua0.SetZero(na, 1);
	for (int i=0; i<na; i++) {
		ddqa0[i] = pcoord_control[i]->ddq;
		taua0[i] = pcoord_control[i]->tau;
	}

	// calculate DdVDu, DdVDs, DddqDx, DddqaDu, DtauaDu

	// initialize
	DdVDu.resize(pLinks.size());
	for (i=0; i<pLinks.size(); i++) { 
		DdVDu[i].SetZero(6, na); 
	}
	DddqaDu.SetZero(na, na);
	DtauaDu.SetZero(na, na);

	// differentiation w.r.t. u
	for (i=0; i<na; i++) {

		// differentiating the equations of motion
		if ( pcoord_control[i]->bPrescribed ) {
			skeleton.setDeriv_Dddq(pcoord_control[i]);
		} else {
			skeleton.setDeriv_Dtau(pcoord_control[i]);
		}
		skeleton.diffDynamics();

		// DdVDu
		for (j=0; j<pLinks.size(); j++) {
			DdVDu[j].Push(0, i, RMatrix(6,1,pLinks[j]->DdVDp.GetArray()));
		}

		// DddqaDu, DtauaDu
		for (j=0; j<pcoord_control.size(); j++) {
			DddqaDu(j,i) = pcoord_control[i]->DddqDp;
			DtauaDu(j,i) = pcoord_control[j]->DtauDp;
		}
	}

	// calculate variables for C.O.M. constraints
	if ( b_com ) {
		xc_current = skeleton.getPositionCOMGlobal();
		vc_current = skeleton.getVelocityCOMGlobal();
		ddxc_0 = skeleton.getAccelerationCOMGlobal() - GetV(pground->dV);

		DddxcDu = Zeros(3, na);
		double mass_total = 0, mass_i;
		for (i=0; i<pLinks.size(); i++) {
			pbody = pLinks[i];
			SO3 R = pbody->getOrientationGlobal();
			Vec3 p = pbody->getPositionCOM();
			SE3 T(R, -(R*p));
			RMatrix DdVDu_tmp = Ad(T, DdVDu[i]);
			mass_i = pbody->getMass();
			DddxcDu += mass_i * DdVDu_tmp.Sub(3, 5, 0, na-1);
			mass_total += mass_i;
		}
		DddxcDu /= mass_total;
		
		//fout << "######################" << endl;
		//fout << "######################" << endl;		
		//fout << "######################" << endl;
		//fout << DddxcDu << endl;
		//fout << skeleton.getInfoStr() << endl;

	}

	// calculate variables for momentum constraints
	if ( b_momentum ) {
		Vec3 p = m_constr_momentum_pos_ref;
		Vec3 dp = m_constr_momentum_vel_ref;
		dse3 Hg = skeleton.getMomentumGlobal();
		Hp_current = dAd(SE3(p), Hg);

		dse3 tmp(0,0,0,0,0,0);
		for (i=0; i<pLinks.size(); i++) {
			pbody = pLinks[i];
			tmp += dAd(Inv(pbody->T_global), dad(pbody->V, pbody->getMomentum()));
		}
		dHp_0 = dAd(SE3(p), dad(se3(Vec3(0,0,0),dp), Hg) - tmp);
		
		DdHpDu = Zeros(6, na);
		RMatrix tmp2 = Zeros(6, na), tmp3(6, na);
		for (i=0; i<pLinks.size(); i++) {
			pbody = pLinks[i];
			set_Mult_Inertia_se3(tmp3.GetPtr(), pbody->I, DdVDu[i].GetPtr(), na); // tmp3 = I_i * D(dV_i)/Du
			tmp2 += dAd(Inv(pbody->T_global), tmp3);
		}
		DdHpDu = dAd(SE3(p), tmp2);
	}

	// calculate joint-loop Jacobian and its time derivative
	for (i=0; i<pJointLoops.size(); i++) {
		// local information of joints, such as S, was updated already by calling calc_dyna() above.
		pJointLoops[i]->update_J();		
		pJointLoops[i]->update_dJdt();
	}


	// -------------------------------------------------------------------------
	// -------------------------------------------------------------------------
	// build the constraints: 
	//           m_simul_A0 * x = m_simul_b0
	//           m_simul_Ap * x = m_simul_bp
	//           m_simul_As * x = m_simul_bs
	//           m_simul_C * x <= m_simul_d
	// where 
	//           x = (u, z)
	// -------------------------------------------------------------------------
	// -------------------------------------------------------------------------

	// ------------------------------
	// m_simul_C * x <= m_simul_d (hard constraints on the joint limit, torque limit constraints)
	// ------------------------------

	if ( m_b_simul_limit_joint_angle || m_b_simul_limit_joint_torque ) {

		m_simul_C.SetZero(2*na, na);
		m_simul_d.SetZero(2*na, 1); 
		for (int i=0; i<2*na; i++) { m_simul_d[i] = 1E20; }

		double ddqa_UL, ddqa_LL, beta = 1E-6;
		bool b_q_dangerous;
		for (int i=0; i<na; i++) {
			ddqa_UL = 1E20; ddqa_LL = -1E20;
			b_q_dangerous = false;
			if ( pcoord_control[i]->q > pcoord_control[i]->qUL - m_layer_depth_q_limit[i] && pcoord_control[i]->dq > 0 ) {
				ddqa_UL = - 0.5 * pcoord_control[i]->dq * pcoord_control[i]->dq / (fabs(pcoord_control[i]->qUL - pcoord_control[i]->q) + beta);
				b_q_dangerous = true;
			}
			if ( pcoord_control[i]->q < pcoord_control[i]->qLL + m_layer_depth_q_limit[i] && pcoord_control[i]->dq < 0 ) {
				ddqa_LL = 0.5 * pcoord_control[i]->dq * pcoord_control[i]->dq / (fabs(pcoord_control[i]->q - pcoord_control[i]->qLL) + beta);
				b_q_dangerous = true;
			}

			if ( m_b_simul_limit_joint_angle && m_b_simul_limit_joint_torque ) {
				// joint angle limit is prior to the torque limit
				if ( b_q_dangerous ) {
//					fout << "coordinate " << i << " is about to exceed its joint limit..." << endl;
					if ( b_actuators_prescribed ) {
						m_simul_C(i, i) = 1.0;
						m_simul_C(na+i, i) = -1.0;
						m_simul_d[i] = ddqa_UL;
						m_simul_d[na+i] = -ddqa_LL;
					} else {
						m_simul_C.Push(i, 0, DddqaDu.Sub(i, i, 0, na-1));
						m_simul_C.Push(na+i, 0, -DddqaDu.Sub(i, i, 0, na-1));
						m_simul_d[i] = ddqa_UL - ddqa0[i];
						m_simul_d[na+i] = -(ddqa_LL - ddqa0[i]);
					}
				} else {
					if ( b_actuators_prescribed ) {
						m_simul_C.Push(i, 0, DtauaDu.Sub(i, i, 0, na-1));
						m_simul_C.Push(na+i, 0, -DtauaDu.Sub(i, i, 0, na-1));
						m_simul_d[i] = pcoord_control[i]->tauUL - taua0[i];
						m_simul_d[na+i] = -(pcoord_control[i]->tauLL - taua0[i]);
					} else {
						m_simul_C(i, i) = 1.0;
						m_simul_C(na+i, i) = -1.0;
						m_simul_d[i] = pcoord_control[i]->tauUL;
						m_simul_d[na+i] = -pcoord_control[i]->tauLL;
					}
				}
			} else if ( m_b_simul_limit_joint_angle && !m_b_simul_limit_joint_torque ) {
				if ( b_q_dangerous ) {
//					fout << "coordinate " << i << " is about to exceed its joint limit..." << endl;
					if ( b_actuators_prescribed ) {
						m_simul_C(i, i) = 1.0;
						m_simul_C(na+i, i) = -1.0;
						m_simul_d[i] = ddqa_UL;
						m_simul_d[na+i] = -ddqa_LL;
					} else {
						m_simul_C.Push(i, 0, DddqaDu.Sub(i, i, 0, na-1));
						m_simul_C.Push(na+i, 0, -DddqaDu.Sub(i, i, 0, na-1));
						m_simul_d[i] = ddqa_UL - ddqa0[i];
						m_simul_d[na+i] = -(ddqa_LL - ddqa0[i]);
					}
				} 
			} else if ( !m_b_simul_limit_joint_angle && m_b_simul_limit_joint_torque ) {
				if ( b_actuators_prescribed ) {
					m_simul_C.Push(i, 0, DtauaDu.Sub(i, i, 0, na-1));
					m_simul_C.Push(na+i, 0, -DtauaDu.Sub(i, i, 0, na-1));
					m_simul_d[i] = pcoord_control[i]->tauUL - taua0[i];
					m_simul_d[na+i] = -(pcoord_control[i]->tauLL - taua0[i]);
				} else {
					m_simul_C(i, i) = 1.0;
					m_simul_C(na+i, i) = -1.0;
					m_simul_d[i] = pcoord_control[i]->tauUL;
					m_simul_d[na+i] = -pcoord_control[i]->tauLL;
				}
			} else {;}

		}

	} else {
		m_simul_C.SetZero(0, 0);
		m_simul_d.SetZero(0, 0);
	}

	// ------------------------------
	// m_simul_A0 * x = m_simul_b0 (joint loop constraints, contact constraints)
	// ------------------------------

	m_simul_A0.SetZero(nb, na);
	m_simul_b0.SetZero(nb, 1);
	cntb = 0;

	// constraints for joint-loops
	// J * ddqa = -dJdt * dq
	RMatrix b0_tmp;
	for (i=0; i<pJointLoops.size(); i++) {
		b0_tmp.SetZero(pJointLoops[i]->getNumConstraints(), 1);
		for (j=0; j<pcoord_control.size(); j++) {
			m_simul_A0.Push(cntb, j, pJointLoops[i]->get_J(pcoord_control[j]));
			b0_tmp -= pcoord_control[j]->dq * pJointLoops[i]->get_dJdt(pcoord_control[j]);
		}
		m_simul_b0.Push(cntb, 0, b0_tmp);
		cntb += pJointLoops[i]->getNumConstraints();
	}

	//// fixed contacts
	//for (i=0; i<contactPositions.size(); i++) {
	//	pbody = pLinks[i];
	//	SO3 R = pbody->getOrientationGlobal();
	//	for (j=0; j<contactPositions[i].size(); j++) {
	//		Vec3 p = contactPositions[i][j];
	//		SE3 T(R, -(R*p));
	//		RMatrix DdVDu_tmp = Ad(T, DdVDu[i]);	// derivative of the acceleration at p w.r.t. u
	//		Vec3 ag_0 = GetV(pbody->getAccelerationGlobal(p));// - pground->dV);	// current acceleration at p in {global}
	//		Vec3 ag_t = GetV(pground->dV);//Vec3(0,0,0);				// target acceleration at p in {global} = (0,0,0)
	//		m_simul_A0.Push(cntb, 0, DdVDu_tmp.Sub(3, 5, 0, na-1));
	//		m_simul_b0[cntb  ] = ag_t[0]-ag_0[0];
	//		m_simul_b0[cntb+1] = ag_t[1]-ag_0[1];
	//		m_simul_b0[cntb+2] = ag_t[2]-ag_0[2];
	//		cntb += 3;
	//	}
	//}

	// ------------------------------
	// m_simul_Ap * x = m_simul_bp, m_simul_As * x = m_simul_bs
	// ------------------------------

	m_simul_Ap.SetZero(np, na); m_simul_As.SetZero(ns, na);  
	m_simul_bp.SetZero(np, 1); m_simul_bs.SetZero(ns, 1); 
	cntp = cnts = 0;

	// constraints imposed on links
	for (i=0; i<pLinks.size(); i++) {

		if ( m_constr_link_b_pbodies_selected[i] ) {

			pbody = pLinks[i];

			SO3 R = pbody->getOrientationGlobal();					// link orientation in {global}
			Vec3 p = pbody->getMarkerPosition();					// marker position in {body}			
			Vec3 pg = pbody->getMarkerPositionGlobal();				// marker position in {global}
			se3 V = pbody->getVelocityGlobal(p);					// current velocity (at the marker) in {global}
			se3 dV = pbody->getAccelerationGlobal(p) - pground->dV;	// current acceleration (at the marker) in {global}

			// calculate derivative of the selected link acceleration (at the marker position) w.r.t. u and z
			SE3 T(R, -(R*p));
			RMatrix DdVDu_tmp = Ad(T, DdVDu[i]);
			// calculate target acceleration
			Vec3 dwt = m_simul_Kp_link_ori * ( R * Log(~R * link_target_pose[i].GetRotation()) )
						+ m_simul_Kv_link_ori * ( Vec3(0,0,0) - GetW(V) );
			Vec3 dvt = m_simul_Kp_link_pos * ( link_target_pose[i].GetPosition() - pg )
						+ m_simul_Kv_link_pos * ( Vec3(0,0,0) - GetV(V) );
			se3 dVt(dwt, dvt);

			se3 dV_target(0,0,0,0,0,0);

			for (j=0; j<6; j++) {
				if ( m_constr_link_b_coords_selected[i][j] ) {

					// identify target acceleration, dV_target
					switch ( m_constr_link_type_coords_selected[i][j] ) {
						case LINK_CONSTR_TYPE_ZERO_VEL:
							if ( j < 3 ) {
								dV_target[j] = m_simul_Kv_link_ori * ( 0 - V[j]);
							} else {
								dV_target[j] = m_simul_Kv_link_pos * ( 0 - V[j]);
							}
							break;
						case LINK_CONSTR_TYPE_ZERO_ACC:
							dV_target[j] = 0;
							break;
						case LINK_CONSTR_TYPE_USER_POSE:
							dV_target[j] = dVt[j];
							break;
					}

					if ( m_constr_link_b_coords_primary[i][j] ) {
						m_simul_Ap.Push(cntp, 0, DdVDu_tmp.Sub(j, j, 0, na-1));
						m_simul_bp[cntp] = dV_target[j] - dV[j];
						cntp++;
					} else {
						m_simul_As.Push(cnts, 0, DdVDu_tmp.Sub(j, j, 0, na-1));
						m_simul_bs[cnts] = dV_target[j] - dV[j];
						cnts++;
					}
				}
			}
		}
	}

	// constraints on actuating joints
	for (i=0; i<na; i++) {
		if ( m_constr_joint_b_selected[i] ) {
			double tmp = 0;
			switch ( m_constr_joint_type[i] ) {
				case JOINT_CONSTR_TYPE_USER_Q:
					tmp = m_simul_Kp_joint * (m_constr_joint_user_given_q[i] - pcoord_control[i]->q) + m_simul_Kv_joint * (0 - pcoord_control[i]->dq);
					break;
				case JOINT_CONSTR_TYPE_USER_DQ:
					tmp = m_simul_Kv_joint * (m_constr_joint_user_given_dq[i] - pcoord_control[i]->dq);
					break;
				case JOINT_CONSTR_TYPE_USER_DDQ:
					tmp = m_constr_joint_user_given_ddq[i];
					break;
				case JOINT_CONSTR_TYPE_USER_TAU:
					return false;
			}
			if ( !(pcoord_control[i]->bPrescribed) && pBodiesDistalOfpCoordinatesActuator[i] != NULL ) {
				int idx = idxLocalOfpCoordinatesActuator[i];
				double psi = pBodiesDistalOfpCoordinatesActuator[i]->Psi(idx, idx);	// the inverse of the diagonal term of the system mass matrix
				tmp *= (1./psi);
			}
			if ( m_constr_joint_b_primary[i] ) {
				m_simul_Ap(cntp, i) = 1;
				m_simul_bp[cntp] = tmp;
				cntp++;
			} else {
				m_simul_As(cnts, i) = 1;
				m_simul_bs[cnts] = tmp;
				cnts++;
			}
		}
	}

	// constraints on C.O.M. of the skeleton
	if ( m_constr_com_b_selected ) {
		Vec3 errp = pos_com_target - xc_current;
		Vec3 errv = Vec3(0,0,0) - vc_current;
		for (i=0; i<3; i++) {
			com_acc_target[i] = m_simul_Kp_com[i] * errp[i] + m_simul_Kv_com[i] * errv[i];
		}
		for (i=0; i<3; i++) {
			if ( m_constr_com_b_coords_selected[i] ) {
				if ( m_constr_com_b_primary[i] ) {
					m_simul_Ap.Push(cntp, 0, DddxcDu.Sub(i, i, 0, na-1));
					m_simul_bp[cntp] = com_acc_target[i] - ddxc_0[i];
					cntp++;

				} else {
					m_simul_As.Push(cnts, 0, DddxcDu.Sub(i, i, 0, na-1));
					m_simul_bs[cnts] = com_acc_target[i] - ddxc_0[i];
					cnts++;
				}
			}
		}
	}

	// constraints on the momentum
	if ( m_constr_momentum_b_selected ) {

		dse3 dHp_target;
		if ( m_constr_momentum_b_dH_target_given ) {
			dHp_target = m_constr_momentum_dH_target;
		} else {
			for (i=0; i<6; i++) {
				dHp_target[i] = m_simul_Kp_momentum[i] * (m_constr_momentum_H_target[i] - Hp_current[i]);
			}
		}

		for (i=0; i<6; i++) {
			if ( m_constr_momentum_b_coords_selected[i] ) {
				if ( m_constr_momentum_b_primary[i] ) {
					m_simul_Ap.Push(cntp, 0, DdHpDu.Sub(i, i, 0, na-1));
					m_simul_bp[cntp] = dHp_target[i] - dHp_0[i];
					cntp++;
				} else {
					m_simul_As.Push(cnts, 0, DdHpDu.Sub(i, i, 0, na-1));
					m_simul_bs[cnts] = dHp_target[i] - dHp_0[i];
					cnts++;
				}
			}
		}
	}

	// constraints on the follower link
	if ( m_constr_follower_elem_type == ELM_LINK && m_constr_follower_idx_pbody >= 0 ) {

		pbody = getFollowerLink();
		int idx_body = m_constr_follower_idx_pbody;	// index of the follower

		SO3 R = pbody->getOrientationGlobal();
		Vec3 p = pbody->getMarkerPosition();
		Vec3 pg = pbody->getMarkerPositionGlobal();
		Vec3 vg = GetV(pbody->getVelocityGlobal(pbody->getMarkerPosition()));
		Vec3 ag_0 = GetV(pbody->getAccelerationGlobal(pbody->getMarkerPosition()) - pground->dV);

		// calculate derivative of the follower's acceleration (at the marker position) w.r.t. u and z
		SE3 T(R, -(R*p));
		RMatrix DdVDu_tmp = Ad(T, DdVDu[idx_body]);
		
		//Vec3 p_ = getFollowerLink()->getMarkerPosition();
		//SO3 R(getFollowerLink()->T_global.GetRotation());
		//Vec3 w(GetW(getFollowerLink()->V)), v(GetV(getFollowerLink()->V)), dw(GetW(getFollowerLink()->dV)), dv(GetV(getFollowerLink()->dV));
		//Vec3 acc_0_2 = R*(Cross(w,Cross(w,p_)) + Cross(w,v) + Cross(dw,p_) + dv) - GetV(pground->dV);

		// set target acceleration
		m_constr_follower_pos_target = pos_command;
		m_constr_follower_vel_target.SetZero();
		m_constr_follower_acc_target = m_simul_Kp_mouse_drag * (m_constr_follower_pos_target - pg) + m_simul_Kv_mouse_drag * (m_constr_follower_vel_target - vg);

		// building constraint equations
		if ( m_constr_follower_b_primary ) {
			m_simul_Ap.Push(cntp, 0, DdVDu_tmp.Sub(3, 5, 0, na-1));
			m_simul_bp[cntp  ] = m_constr_follower_acc_target[0]-ag_0[0];
			m_simul_bp[cntp+1] = m_constr_follower_acc_target[1]-ag_0[1];
			m_simul_bp[cntp+2] = m_constr_follower_acc_target[2]-ag_0[2];
			cntp += 3;
		} else {
			m_simul_As.Push(cnts, 0, DdVDu_tmp.Sub(3, 5, 0, na-1));
			m_simul_bs[cnts  ] = m_constr_follower_acc_target[0]-ag_0[0];
			m_simul_bs[cnts+1] = m_constr_follower_acc_target[1]-ag_0[1];
			m_simul_bs[cnts+2] = m_constr_follower_acc_target[2]-ag_0[2];
			cnts += 3;
		}
	}

	// constraints on the follower C.O.M.
	if ( m_constr_follower_elem_type == ELM_COM ) {

		m_constr_follower_pos_target = pos_command;
		m_constr_follower_vel_target.SetZero();
		m_constr_follower_acc_target = m_simul_Kp_mouse_drag * (m_constr_follower_pos_target - xc_current) + m_simul_Kv_mouse_drag * (m_constr_follower_vel_target - vc_current);

		if ( m_constr_follower_b_primary ) {
			m_simul_Ap.Push(cntp, 0, DddxcDu);
			m_simul_bp[cntp  ] = m_constr_follower_acc_target[0] - ddxc_0[0];
			m_simul_bp[cntp+1] = m_constr_follower_acc_target[1] - ddxc_0[1];
			m_simul_bp[cntp+2] = m_constr_follower_acc_target[2] - ddxc_0[2];
			cntp += 3;
		} else {
			m_simul_As.Push(cnts, 0, DddxcDu);
			m_simul_bs[cnts  ] = m_constr_follower_acc_target[0] - ddxc_0[0];
			m_simul_bs[cnts+1] = m_constr_follower_acc_target[1] - ddxc_0[1];
			m_simul_bs[cnts+2] = m_constr_follower_acc_target[2] - ddxc_0[2];
			cnts += 3;
		}

	} 

	// restores byproducts for later use
	m_simul_taua0 = taua0;
	m_simul_ddqa0 = ddqa0;
	m_simul_DtauaDx = DtauaDu;
	m_simul_DddqaDx = DddqaDu;

	return true;
}

bool MySystem::checkQP()
{
	int na = pCoordinatesActuator.size();

	if ( m_simul_A0.RowSize() > 0 && m_simul_A0.ColSize() != na ) return false;
	if ( m_simul_Ap.RowSize() > 0 && m_simul_Ap.ColSize() != na ) return false;
	if ( m_simul_As.RowSize() > 0 && m_simul_As.ColSize() != na ) return false;
	if ( m_simul_C.RowSize() > 0 && m_simul_C.ColSize() != na ) return false;
	
	if ( m_simul_A0.RowSize() != m_simul_b0.RowSize() ) return false;
	if ( m_simul_Ap.RowSize() != m_simul_bp.RowSize() ) return false;
	if ( m_simul_As.RowSize() != m_simul_bs.RowSize() ) return false;
	if ( m_simul_C.RowSize() != m_simul_d.RowSize() ) return false;

	return true;
}

bool MySystem::solveQP(RMatrix &x_, double alpha_)
{
	bool b_weight = false;
	RMatrix iw;

	// size of variable x_
	int na = pCoordinatesActuator.size();

	// if there is no constraint, return true
	if ( m_simul_A0.RowSize() <= 0 && m_simul_Ap.RowSize() <= 0 && m_simul_As.RowSize() <= 0 && m_simul_C.RowSize() <= 0 ) return true;

	// check constraints
	if ( !checkQP() ) return false;

	// weight
	if ( m_b_simul_uneven_joint_weighting ) {
		iw.ReSize(na, 1);
		for (int i=0; i<na; i++) { iw[i] = m_simul_weight_qa[i]; }
		b_weight = true;
	}

	if ( !solve_A0x_b0_Apx_bp_Asx_bs_Cx_d(x_, m_simul_A0, m_simul_b0, m_simul_Ap, m_simul_bp, m_simul_As, m_simul_bs, m_simul_C, m_simul_d, alpha_) ) {
		cout << "Error:: Failed in solving QP problem!" << endl;
		cout << "--- Quadratic Programming ----------------------------------------------------" << endl;
		cout << "    minimize 0.5 * ( || Ap * x - bp ||^2 + || alpha ||^2 ) first and then" << endl;
		cout << "    minimize 0.5 * ( || Ax * x - bs ||^2 + || alpha ||^2 )" << endl;
		cout << "        s.t. C * x <= d" << endl;
		cout << "             A0 * x = b0" << endl;
		cout << "--------------------------------------------------------------------------" << endl;				 
		cout << "A0 = " << m_simul_A0 << "b0 = " << m_simul_b0 << "Ap = " << m_simul_Ap << "bp = " << m_simul_bp << "As = " << m_simul_As << "bs = " << m_simul_bs << "C = " << m_simul_C << "d = " << m_simul_d << endl;
		return false;
	}

	//fout << "x = " << x_ << endl;
	//fout << "m_simul_A0 = " << m_simul_A0 << endl;
	//fout << "m_simul_b0 = " << m_simul_b0 << endl;
	//fout << "m_simul_Ap = " << m_simul_Ap << endl;
	//fout << "m_simul_bp = " << m_simul_bp << endl;
	//fout << "m_simul_As = " << m_simul_As << endl;
	//fout << "m_simul_bs = " << m_simul_bs << endl;
	//fout << "------------" << endl;

	return true;
}

bool MySystem::reformQP_vel()
{
	int na = pCoordinatesActuator.size();
	double h = step_size;
	RMatrix dqa(na,1);
	for (int i=0; i<na; i++) {
		if ( !(pCoordinatesActuator[i]->bPrescribed) ) {
			set_actuator_prescribed(true);
			cout << "Warning: All actuators are set to be prescribed!" << endl;
			//return false;
		}
		dqa[i] = pCoordinatesActuator[i]->dq;
	}

	m_simul_b0 = h * m_simul_b0 + m_simul_A0 * dqa;
	m_simul_bp = h * m_simul_bp + m_simul_Ap * dqa;
	m_simul_bs = h * m_simul_bs + m_simul_As * dqa;
	m_simul_d = h * m_simul_d + m_simul_C * dqa;

	return true;
}

bool MySystem::compensateJointLoopConstraintError()
{
	int nc = 0;
	for (size_t i=0; i<pJointLoops.size(); i++) {
		nc += pJointLoops[i]->getNumConstraints();
	}
	if ( nc == 0 ) return true;

	int na = pCoordinatesActuator.size();

	RMatrix J(nc,na), C(nc,1), del_q(na,1), dq0(na,1), del_dq(na,1);
	J.SetZero();
	C.SetZero();

	// project q onto the closed-loop constraints
	int iter_cnt = 0;
	while (1)
	{
		int cnt = 0;
		for (size_t i=0; i<pJointLoops.size(); i++) {

			if ( !pJointLoops[i]->update_C() ) { cerr << "Error:: Failed in calling update_C() for " << i << "-th joint loop." << endl; return false; }
			if ( !pJointLoops[i]->update_J() ) { cerr << "Error:: Failed in calling update_J() for " << i << "-th joint loop." << endl; return false; }

			C.Push(cnt, 0, pJointLoops[i]->get_C());
			for (int j=0; j<na; j++) {
				J.Push(cnt, j, pJointLoops[i]->get_J(pCoordinatesActuator[j]));
			}
			
			cnt += pJointLoops[i]->getNumConstraints();
		}

		if ( FNorm(C) < 1E-4 ) break;
		//if ( FNorm(C) < 1E-6 ) break;

		solve_Ax_b_pInv(del_q, J, C, 1E-9);

		for (int i=0; i<na; i++) {
			pCoordinatesActuator[i]->q -= del_q[i];
		}

		if ( FNorm(del_q) < 1E-6 ) break;
		//if ( FNorm(del_q) < 1E-8 ) break;

		if ( iter_cnt++ > 30 ) {
			cerr << "Error:: Exceeded maximum iteration." << endl;
			return false;
		}
	}

	// project dq onto the (linearized) closed-loop constraints: J*dq = 0
	for (int i=0; i<na; i++) {
		dq0[i] = pCoordinatesActuator[i]->dq;
	}

	solve_Ax_b_pInv(del_dq, J, J*dq0, 1E-9);

	for (int i=0; i<na; i++) {
		pCoordinatesActuator[i]->dq -= del_dq[i];
	}

	return true;
}
