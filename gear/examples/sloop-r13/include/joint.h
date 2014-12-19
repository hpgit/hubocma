//================================================================================
//         JOINTS FOR MULTIBODY SKELETONS
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef __JOINT_
#define __JOINT_

#include "gear.h"
#include "glsub.h"
#include "surface_fixed_to_frame.h"

//=============================================================
//                 JointProperties
//=============================================================
class JointProperties
{
public:
	// rendering
	bool bRendering;
	bool bColorMapMode;
	glsubColorMapType typeColorMap;
	double maxTorqueForRendering;
	double color[3], alpha;
	glsubDrawType draw_type;			// 0(solid), 1(wire frame)

	bool bRenderSurface;				// set true to use surface in rendering
	SurfaceFixedToFrame surface;		// surface model

public:
	JointProperties();
	~JointProperties() {}

	// rendering
	void enableRendering(bool b) { bRendering = b; }
	void setColorMapMode(bool b, glsubColorMapType type, double max_trq) { bColorMapMode = b; typeColorMap = type; maxTorqueForRendering = fabs(max_trq); }
	void setColor(Vec3 c_) { color[0] = c_[0]; color[1] = c_[1]; color[2] = c_[2]; }
	void setAlphaBlending(double alpha_) { alpha = alpha_; }
	void setDrawType(glsubDrawType dtype_) { draw_type = dtype_; surface.draw_type = dtype_; }

	bool isEnabledRendering() { return bRendering; }
	Vec3 getColor() { return Vec3(color); }
	double getAlphaBlending() { return alpha; }

	// for loading surface
	bool loadSurfaceMesh(char *file_name_, Vec3 scale_ = Vec3(1,1,1), SE3 T0_ = SE3(), int index_base_ = 0);	
	bool addSurfaceMesh(char *file_name_, Vec3 scale_ = Vec3(1,1,1), SE3 T0_ = SE3(), int index_base_ = 0);	
	// loads/add surface mesh file
	//		sx_, sy_, sz_ = scale factors for the axes of {mesh file reference frame}
	//		T0_: {joint left} -> {mesh file reference frame}

};

//=============================================================
//                 RevoluteJoint
//=============================================================
class RevoluteJoint: public GJointRevolute, public JointProperties
{
public:
	double radius, height;	// dimension of cylinder

public:
	RevoluteJoint();
	~RevoluteJoint() {}

	void setRadius(double radius_) { radius = radius_; }
	void setHeight(double height_) { height = height_; }

	bool setCylinderSurface(double r_, double h_, int n_slice_ = 10);	// load a cylinderical surface mesh for JointProperties::surface
																		// r_ = radius, h_ = height

	void render();
};


#endif

