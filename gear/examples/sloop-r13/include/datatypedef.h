#ifndef __DATA_TYPE_DEF_
#define __DATA_TYPE_DEF_

enum ElementType {
	ELM_NONE,					// not an element
	ELM_UNKNOWN,				// unknown element
	ELM_LINK,					// GBody
	ELM_JOINT,					// GJoint
	ELM_DBODY,					// GDeformableBody
		ELM_EBODY,				// GElasticBody
		ELM_SURF,				// GSurfaceEmbedded
			ELM_NODE_EBODY,		// NodePointMass
			ELM_NODE_SURF,		// NodeSurf
	ELM_COM,					// Center of Mass
	ELM_PLANE_MOUSE_DRAG,		// Mouse dragging plane
	ELM_CUBE_CONTROL,			// CUBE control
};

enum PoseEditingType {
	IK_NORMAL,
	IK_TRANSLATIONAL,
};

enum LinkConstraintType { 
	LINK_CONSTR_TYPE_ZERO_VEL,		// zero velocity
	LINK_CONSTR_TYPE_ZERO_ACC,		// zero acceleration
	LINK_CONSTR_TYPE_USER_POSE,		// user-given position/orientation
};

enum JointConstraintType {
	JOINT_CONSTR_TYPE_USER_Q,		// user-given displacement
	JOINT_CONSTR_TYPE_USER_DQ,		// user-given velocity
	JOINT_CONSTR_TYPE_USER_DDQ,		// user-given acceleration
	JOINT_CONSTR_TYPE_USER_TAU,		// user-given torque
};

enum SimulationInputType {
	SIM_INPUT_NONE = 0,
	SIM_INPUT_FILE_TRJ = 1,
	SIM_INPUT_FILE_BSTRJ = 2,
	SIM_INPUT_USER_GUIDANCE = 3,
};

enum SimUserGuidanceType {
	USER_GUIDE_CARTESIAN_ACCELERATION = 0,
	USER_GUIDE_CARTESIAN_VELOCITY = 1,
	USER_GUIDE_CARTESIAN_TORQUE = 2,
	USER_GUIDE_CARTESIAN_POSE = 3,
};

#endif