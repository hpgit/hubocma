#ifndef _MY_SYSTEM
#define _MY_SYSTEM

#include <fstream>
#include <vector>
#include <list>
#include "hexad.h"
#include "liegroup.h"
#include "sysdata.h"
#include "rigid_body.h"
#include "joint.h"
#include "gsystem_p.h"
#include "datatypedef.h"


enum IntegratorType {
	EXPLICIT_EULER,
	IMPLICIT_EULER,
	HYBRID_EULER,		// explicit integrator for skeletal passive cooradinates, and implicit one for the other passive coordinates.
};


//===================================================================
//			MySystem
//===================================================================
class MySystem
{
public:

	//////////////////////////////////////////////////////////////////////////////////////////
	// ---------------------------------------------------------------------------------------
	//           VARIABLES TO BE SET BY USER
	// ---------------------------------------------------------------------------------------
	//////////////////////////////////////////////////////////////////////////////////////////

	// --------------- system definition ------------------> build()

	// is created?
	bool b_created;

	// system
	GSystemP system;		// whole character system (skeleton + deformable body)
	GSystemP skeleton;		// skeleton only

	// ground
	RigidBody ground;

	// skeleton
	std::vector<RigidBody*> pLinks;									// pointers to real skeletal links
	std::vector<GJoint*> pActuatingJoints;							// pointers to actuating joints
	std::vector<GJoint*> pNonActuatingJoints;						// pointers to non-actuating joints in the skeleton --> this will be set automatically
	std::vector<GJoint*> pRootJoints;								// pointers to root joints

	std::vector<GCoordinate*> pCoordinatesActuator;					// pointers to coordinates of the active joints in the skeleton
	std::vector<GCoordinate*> pCoordinatesNonActuator;				// pointers to coordinates of the non-actuating joints in the skeleton
	std::vector<GCoordinate*> pCoordinatesRootJoint;				// pointers to root joint coordinates of the skeleton
	std::vector<GCoordinate*> pCoordinatesRootJointTranslational;	// pointers to translational root joint coordinates of the skeleton
	std::vector<GCoordinate*> pCoordinatesRootJointRotational;		// pointers to rotational root joint coordinates of the skeleton

	// joint loops
	std::vector<GConstraintJointLoop*> pJointLoops;

	// contact positions
	std::vector< std::vector<Vec3> > contactPositions;				// contactPosition[i][] = relative contact positions of pLinks[i] in {pLinks[i]}
	
	// objects
	std::vector<RigidBody*> pObjects;
	GSystemP system_objects;		// object system

	// com and momentum (to prevent crashes between threads)
	Vec3 com_sys;
	dse3 mom_sys_com;


	// ---------------- simulation -------------------------> set_properties()

	// integrator and step size
	IntegratorType integrator;
	double step_size;

	// actuator setting (prescribed or unprescribed)
	bool b_actuators_prescribed;

	// for the external dampers acting on the mass particles in the elastic bodies
	double C_particle_damper;				// damping coefficient = C_damper * mass

	// spring and damper on the skeletal joints (valid only for unprescribed simulation)
	std::vector<double> K_q_actuator, C_dq_actuator;
	std::vector<double> q_actuator_ref;			// reference pose for the spring

	// ---------------- contact ----------------------------> set_properties()

	// d(deformable bodies), o(object), g(ground)
	// e.g., 'dg' means "deformable bodies and ground", 'dd' means "self collision of deformable bodies"

	// switch for collision test
	bool b_check_collision;		// if b_check_collision == false, then all collision tests will be ignored.
	bool b_check_collision_dr;	// contact: pDeformableBodies <--> ground, pObjects
	bool b_check_collision_rr;	// contact: pLinks <--> pObjects <--> ground

	// coefficient for penalty force (linear part)
	double K_dr, C_dr;			// for deformable-rigid contact
	double K_dd, C_dd;			// for deformable-deformable contact
	double K_rr, C_rr;			// for rigid-rigid contact

	// coefficient for penalty force (quadratic part)
	double d0_nc_dr;			// penetration depth where the stiffness and damping become to K_dr and C_dr
	double d0_nc_dd;			// penetration depth where the stiffness and damping become to K_dd and C_dd
	double d0_nc_rr;			// penetration depth where the stiffness and damping become to K_rr and C_rr

	// coefficients in Coulomb's friction model: s(static), d(dynamic)
	double Mu_s_dr, Mu_d_dr;	// for deformable-rigid contact
	double Mu_s_dd, Mu_d_dd;	// for deformable-deformable contact
	double Mu_s_rr, Mu_d_rr;	// for rigid-rigid contact


	// --------------- rendering ---------------------------------> set_properties()

	// rendering options
	bool b_rndr_naming;						// naming for picking
	bool b_rndr_show_system;
	bool b_rndr_show_skeleton;
	bool b_rndr_show_body_surface;
	bool b_rndr_show_body_surface_extended;
	bool b_rndr_show_joint;
	bool b_rndr_show_body_frame;
	bool b_rndr_show_body_com;
	bool b_rndr_show_body_com_vel;
	bool b_rndr_show_body_com_acc;
	bool b_rndr_show_ground;
	bool b_rndr_show_contact_force;
	bool b_rndr_show_dbody_ebody;
	bool b_rndr_show_dbody_surf;
	bool b_rndr_show_dbody_surf_fake;
	bool b_rndr_show_bounding_box;
	bool b_rndr_show_sys_com;
	bool b_rndr_show_sys_momentum;
	bool b_rndr_show_constraints_for_user_guided_simul;
	bool b_rndr_hide_constraints_for_user_guided_simul_when_replay_running;
	bool b_rndr_show_position_of_temporal_root_joint;	// valid only when b_rndr_show_constraints_for_user_guided_simul == true
	bool b_rndr_hide_desired_acceleration_for_user_guided_simul;
	bool b_rndr_show_vnode_body_connection;				// show the reference bodies of the volume nodes
	double rndr_scale_force;
	double rndr_scale_vel;
	double rndr_scale_acc;
	double rndr_scale_size;

	Vec3 rndr_color_com, rndr_color_com_normal;
	float rndr_line_width_com, rndr_line_width_com_normal;

	// --------------- data-saving manager ---------------> set_data_to_be_saved()

	GSysData sysdata;		


	//////////////////////////////////////////////////////////////////////////////////////////
	// ---------------------------------------------------------------------------------------
	//           VARIABLES FOR SIMULATION CONTROL (set_variables_automatically())
	// ---------------------------------------------------------------------------------------
	//////////////////////////////////////////////////////////////////////////////////////////

	// ---------- simulation control by user-guidance or editing with IK ----------> init_variables_for_user_guided_simulation()

	// follower (following the mouse cursor)
	int m_constr_follower_elem_type;							// type of the follower (ELM_LINK or ELM_COM)
	int m_constr_follower_idx_pbody;
	bool m_constr_follower_b_primary;							// true if the constraint on the follower is primary
	Vec3 m_constr_follower_acc_target;							// target acceleration of the follower
	Vec3 m_constr_follower_vel_target;							// target velocity of the follower
	Vec3 m_constr_follower_pos_target;							// target position of the follower

	// constraints on links for editing with IK or user-guided simulation
	std::vector<int> m_constr_link_b_pbodies_selected;					// true if the corresponding body is selected to be constrained
	std::vector< _hexad<bool> > m_constr_link_b_coords_selected;	// true if the coordinate direction is constrained
	std::vector< _hexad<bool> > m_constr_link_b_coords_primary;		// true if the coordinate constraint is primary
	std::vector< _hexad<LinkConstraintType> > m_constr_link_type_coords_selected;	// constraint type along the coordinate direction
	std::vector<SE3> m_constr_link_user_given_SE3_selected;			// user-given target orientation and position of the link w.r.t. {global} (for LINK_CONSTR_TYPE_USER_POSE)
	std::vector<bool> m_constr_link_b_control_with_rtmocap;			// true if the corresponding body is controlled by real-time mocap
	std::vector<int> m_constr_link_sbj_idx_rtmc_control;			// indices of the real-time mocap subjects (defined in CShellView.h) controlling the links
	std::vector<bool> m_constr_link_b_relative_translation_rtmocap;

	int m_constr_link_idx_cube_control;								// index of the link whose target orientation is controlled by cube control

	// constraints on joints for user-guided simulation
	std::vector<int> m_constr_joint_b_selected;				// size(m_constr_joint_b_selected) == size(pCoordinatesActuator)
	std::vector<JointConstraintType> m_constr_joint_type;
	std::vector<int> m_constr_joint_b_primary;
	std::vector<double> m_constr_joint_user_given_q;		
	std::vector<double> m_constr_joint_user_given_dq;		
	std::vector<double> m_constr_joint_user_given_ddq;		
	std::vector<double> m_constr_joint_user_given_tau;	

	// constraints on c.o.m. for user-guided simulation
	bool m_constr_com_b_selected;								// true if com is selected to be constrained
	bool m_constr_com_b_coords_selected[3];						// true if (x,y,z) is constrained
	bool m_constr_com_b_primary[3];
	Vec3 m_constr_com_pos_target;								// user-given target position of c.o.m. w.r.t. {global}
	bool m_constr_com_b_control_with_rtmocap;					// true if com is controlled by real-time mocap
	int m_constr_com_sbj_idx_rtmc_control;						// index of the real-time mocap subject (defined in CShellView.h) controlling c.o.m.
	bool m_constr_com_b_relative_translation_rtmocap;

	// constraints on the momentum for user-guided simulation
	bool m_constr_momentum_b_selected;
	bool m_constr_momentum_b_coords_selected[6];
	bool m_constr_momentum_b_primary[6];
	Vec3 m_constr_momentum_pos_ref;								// position of the reference point in {global}
	Vec3 m_constr_momentum_vel_ref;								// velocity of the reference point in {global}
	dse3 m_constr_momentum_H_target;							// user-given target momentum (dH_target = K * (H_target - H_current))
	bool m_constr_momentum_b_dH_target_given;					// set true to directly specify the target dH
	dse3 m_constr_momentum_dH_target;

	// virtual spring-damper coefficients for limiting joint displacements in torque-minimizing Cartesian control
	std::vector<double> m_K_q_limit, m_C_q_limit;

	// layer depth for limiting joint displacement in acceleration-minimizing Cartesian control
	std::vector<double> m_layer_depth_q_limit;

	// low-dimensional subspace by PCA : qa = m_pca_qam + m_pca_Phi * m_pca_r where qa = pCoordinatesActuator[]->q
	// equivalent linear constraint equation: m_pca_A * qa = m_pca_b
	bool m_b_pca_ready;											// true if low-dimensional analysis is ready.
	RMatrix m_pca_qam, m_pca_Phi, m_pca_lambda;
	RMatrix m_pca_A, m_pca_b;								

	// reference pose set for style-based Cartesian control
	std::vector<RMatrix> m_ref_qa_style_based_control;

	// simulation type and user-guidance type when the simulation data saved
	int saved_simul_input_type;
	int saved_simul_user_guidance_type;

	// saved position of temporary root joint
	Vec3 pos_temporal_root_joint;
	
	//////////////////////////////////////////////////////////////////////////////////////////
	// ---------------------------------------------------------------------------------------
	//           
	// ---------------------------------------------------------------------------------------
	//////////////////////////////////////////////////////////////////////////////////////////

	// ----------------- derivative of dynamics ---------------

	// 'p' means system.pCoordinatesUnprescribed, 'a' means system.pCoordinatesPrescribed
	RMatrix DddqpDqa, DddqpDqp;			
	RMatrix DddqpDdqa, DddqpDdqp;
	RMatrix DddqpDddqa;
	RMatrix DtauaDqa, DtauaDqp;	
	RMatrix DtauaDdqa, DtauaDdqp;
	RMatrix DtauaDddqa;	
	// 'e' means pCoordinatesExplicit, 'i' means pCoordinatesImplicit
	RMatrix DddqiDqi, DddqiDdqi, DddqiDqe, DddqiDdqe;

	// coordinates set for hybrid integration (This will be set when set_actuator_prescribed() is called.)
	std::list<GCoordinate *> pCoordinatesExplicit, pCoordinatesImplicit;

	// ---------------- aux information on the skeleton ------------> set_variables_automatically()

	// pCoordinatesActuator[i] == pBodiesDistalOfpCoordinatesActuator[i]->pBaseJoint->pCoordinates[idxLocalOfpCoordinatesActuator[i]]
	std::vector<GBody*> pBodiesDistalOfpCoordinatesActuator;		// pointers to the distal links of the active joint coordinates
	std::vector<int> idxLocalOfpCoordinatesActuator;					// index of the active coordinate in their corresponding joint coordinates

	//// for self-collision test
	//std::vector<Surface> surface_patch;		// surface patches of deformable bodies corresponding to the bounding boxes of pLinks


public:
	MySystem();
	~MySystem();

public:
	bool create(const char *dirpath = NULL);

	virtual bool build(const char *dirpath = NULL) { return true; }
	virtual bool set_variables_automatically();
	virtual bool set_properties() { return true; }
	virtual bool set_data_to_be_saved();
	
	void set_actuator_prescribed(bool b_);
	void set_rootjoint_prescribed(bool b_);

	void save_state(const char *file_name);			// save system.pCoordinates[]->(q,dq,ddq,tau) to a binary file
	bool load_state(const char *file_name);			// load system.pCoordinates[]->(q,dq,ddq,tau) from a binary file

	void save_reference_pose(const char *file_name);// save m_ref_qa_style_based_control to a file (*.rfp)
	bool load_reference_pose(const char *file_name);// load m_ref_qa_style_based_control from a file (*.rfp)

	bool load_low_dim_pose_space(const char *file_name, int m_);	// load m_pca_qam, m_pca_lambda, m_pca_Phi where m = # of principal bases
	int calc_num_basis_low_dim_pose_space(const char *file_name, double score_);	// calculate the number of principal bases for the given score (0 < score_ <= 1)
	bool adjust_pose_to_low_dim_space(double *pdist_ = NULL);			// adjust current pose into the low-dimensional pose space and return the distance between the previous and new poses

	void update_kinematics();						// update global kinematic information including skeleton, volumetric meshes, and surfaces.
	void update_rigid_surface();					// update surface with an assumption that the nodes in the elastic body are fixed to their initial local positions with respect to their parent link.

	bool go_ahead(double t);						// go one step ahead 

public: // sub-functions
	void init_variables_for_user_guided_simulation();

	bool integrate_me(double t, double h_);					// mixed Euler integration

	virtual void set_unexpected_disturbance(double t_simul_current) {}

	bool calc_dyna(double t, bool b_include_disturbance = true);
	bool calc_dyna_without_contact();

	virtual void init_collision();
	virtual bool check_collision(bool bapplycontactforce = true);

	bool is_diverging();							// return true if simulation is diverging.

	// subfunctions for collision test
	// a rigid body must have an enclosing bounding box 
	void col_rb_surf(RigidBody *prbody, SurfaceFixedToRigidBody *psurf, RigidBody *prbody_surf = NULL, double d_lower = -0.05, double d_upper = 0.0, bool bapplycontactforce = true); // collision between a rigid body and a surface attached to prbody_surf (if prbody_surf == NULL, reaction force acting on the surface will not be considered.)
	void col_rb_rb(RigidBody *prbodyA, RigidBody *prbodyB, double d_lower = -0.05, double d_upper = 0.0, bool bapplycontactforce = true);	// collision between the two rigid bodies
	void apply_contact_force(RigidBody *prbody, RigidBody *prbody_surf = NULL); // apply contact force

public: 
	// for rendering
	virtual bool render(bool b_update_kine_);	// render the system
												// If b_update_kine_ == true, then call system.updateKinematics() is called before rendering. 
												// Warning! Updating kinematics is not recommended when dynamics simulation is running.

	virtual double getRadiusOfMinimalEnclosingSphere() { return 1; }
	virtual Vec3 getCenterOfMinimalEnclosingSphere() { return Vec3(0,0,0); }

	void renderLinkConstraints();
	void renderJointConstraints();
	void renderCOMConstraints();
	void renderFollower(bool b_show_follower_, bool b_show_target_pos_, bool b_show_target_vel_, bool b_show_target_acc_);
	void renderPositionOfTemporalRootJoint();

public:
	virtual void approx_contact_model(bool b_) {}
	virtual void auto_assist_motion_control(double t_simul_current) {}

	void releaseConstraintsForUserGuidedSimulControl();

	int getIndexLink(RigidBody* pbody_);
	RigidBody *getFollowerLink() { if ( m_constr_follower_idx_pbody < 0 ) { return NULL; } return pLinks[m_constr_follower_idx_pbody]; }

	// for printing system information
	void print_sys_info(std::ofstream *pfout);
	void print_sys_state(std::ofstream *pfout);

	// test function for developer
	virtual bool test_func() { return true; }

public:
	// user-guided simulation
	Vec3 m_desired_position;
	RMatrix m_simul_A0, m_simul_Ap, m_simul_As, m_simul_C, m_simul_b0, m_simul_bp, m_simul_bs, m_simul_d; // constraints: A0*x=b0, Ap*x=bp, As*x=bs, C*x<=d
	RMatrix m_simul_taua0, m_simul_ddqa0, m_simul_DtauaDx, m_simul_DddqaDx;

	double m_simul_Kp_mouse_drag, m_simul_Kv_mouse_drag;		// control gains for generating target acceleration
	double m_simul_Kp_link_pos, m_simul_Kv_link_pos;
	double m_simul_Kp_link_ori, m_simul_Kv_link_ori;
	double m_simul_Kp_joint, m_simul_Kv_joint;
	Vec3 m_simul_Kp_com, m_simul_Kv_com;						// control gains for x,y,z directions
	dse3 m_simul_Kp_momentum;
	double m_simul_alpha_acc, m_simul_alpha_trq, m_simul_alpha_vel;
	bool m_b_simul_limit_joint_angle;							// indicates whether joint angles are limited in user-guided simulation
	bool m_b_simul_limit_joint_torque;							// indicates whether joint torques are limited in user-guided simulation
	bool m_b_simul_uneven_joint_weighting;						// indicates whether uneven joint weighting will be used in user-guided simulation
	bool m_b_simul_automatic_weighting;							// set joint weighting automatically corresponding to the selected follower link
	std::vector<double> m_simul_weight_qa;						// weighting on active coordinates
	bool m_constr_b_primary_joint_angle_limit;					// true if the joint angle limit constraints are primary

	int m_simul_num_skip_frames_for_updating_command_input;

	bool stepSimulation();
	bool setCommandInputForCartesianControl();
	bool CartesianControlWithMinimalAcceleration();
	bool CartesianControlWithMinimalVelocity();
	bool buildQP();
	bool checkQP();
	bool solveQP(RMatrix &x_, double alpha_);
	bool reformQP_vel();
	bool compensateJointLoopConstraintError();
};

#endif

