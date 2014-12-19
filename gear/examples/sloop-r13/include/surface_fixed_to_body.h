//================================================================================
//         SURFACE MESH
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _SURFACE_FIXED_TO_RIGIDBODY_
#define _SURFACE_FIXED_TO_RIGIDBODY_

#include <vector>
#include <string>
#include "gear.h"
#include "triad.h"
#include "glsub.h"



//=============================================================
//                 SurfaceFixedToRigidBody
//=============================================================
class SurfaceFixedToRigidBody
{
public:
	GBody *pBody;					// pointer to the master body of the surface

	// node information
	std::vector<Vec3> r0;					// x[i] = position of i-th node relative to pBody->{body} frame, constant vector
	std::vector<Vec3> xg;					// xg[i] = position of i-th node relative to {global} frame
	std::vector<Vec3> vg;					// vg[i] = velocity of i-th node relative to {global} frame
	std::vector<Vec3> fg;					// fg[i] = force acting on i-th node relative to {global} frame
	
	// contact information as master nodes
	std::vector<int> bContact;
	std::vector<double> peneDepth;
	std::vector<Vec3> contactNormal;
	
	// surface mesh information
	std::vector< _triad<int> > mesh_index;	// mesh_index[i][j] = corresponding node index of j-th node of i-th triangular surface mesh element.

	// rendering options
	bool b_render_mesh, b_render_node;
	double color[3], alpha;
	glsubDrawType draw_type;			// 0(solid), 1(wire frame)
	float line_width;

	// advanced rendering options
	bool b_render_highlighted_mesh_elem; // set true for rendering highlighted mesh element
	double color_highlighted_mesh_elem[3];
	std::vector<bool> b_highlighted_mesh_elem; // set true to highlight the i-th mesh element

	// name and material
	std::string str_name, str_material;

public:
	SurfaceFixedToRigidBody();
	~SurfaceFixedToRigidBody() {}

public:
	void setMasterBody(GBody *pbody_) { pBody = pbody_; }

	void clearSurface();

	void setName(char *name_) { str_name = name_; }
	void setMaterial(char *mtl_) { str_material = mtl_; }

	bool addSurface(std::vector<Vec3> r0_, std::vector< _triad<int> > mesh_index_);

	int getNumNode();					// return x.size()
	int getNumMesh();					// return mesh_index.size()
	
	bool update_xg();					// update xg
	bool update_vg();					// update vg
	bool transform_fg();				// transform fg to pBody->Fe

public:
	bool make_ready_for_analysis();

public:
	bool _check_mesh_index();

public:
	void setColor(Vec3 c_) { color[0] = c_[0]; color[1] = c_[1]; color[2] = c_[2]; }
	void setAlphaBlending(double alpha_) { alpha = alpha_; }
	void setDrawType(glsubDrawType dtype_) { draw_type = dtype_; }
	void setLineWidth(float width_) { line_width = width_; }

	Vec3 getColor() { return Vec3(color); }
	double getAlphaBlending() { return alpha; }
	float getLineWidth() { return line_width; }

	void render();
};


#endif

