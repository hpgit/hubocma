//================================================================================
//         SURFACE FIXED TO FRAME
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _SURFACE_FIXED_TO_FRAME
#define _SURFACE_FIXED_TO_FRAME

#include <vector>
#include <string>
#include "gear.h"
#include "triad.h"
#include "glsub.h"


//=============================================================
//                 SurfaceFixedToFrame
//=============================================================
class SurfaceFixedToFrame
{
public:
	SE3 *pFrame;						// pointer to a moving frame
	Vec3 *pFrameLinear;					// pointer to a linearly moving frame

	// node information
	std::vector<Vec3> r0;					// x[i] = position of i-th node relative to pBody->{body} frame, constant vector
	std::vector<Vec3> xg;					// xg[i] = position of i-th node relative to {global} frame
	
	// surface mesh information
	std::vector< _triad<int> > mesh_index;	// mesh_index[i][j] = corresponding node index of j-th node of i-th triangular surface mesh element.

	std::vector<std::string> strExtraInfo;		// extra information for obj export ('vt' and 'f' in obj file)

	// rendering options
	bool b_render_mesh, b_render_node;
	double color[3], alpha;
	glsubDrawType draw_type;			// 0(solid), 1(wire frame)
	float line_width;

	// name and material
	std::string str_name, str_material;

public:
	SurfaceFixedToFrame();
	~SurfaceFixedToFrame() {}

public:
	void setMasterFrame(SE3 *pT_) { pFrame = pT_; pFrameLinear = NULL; }
	void setMasterFrame(Vec3 *pV_) { pFrameLinear = pV_; pFrame = NULL; }

	void clearSurface();

	void setName(char *name_) { if ( name_ != NULL ) str_name = name_; }
	void setMaterial(char *mtl_) { if ( mtl_ != NULL ) str_material = mtl_; }

	bool addSurface(std::vector<Vec3> r0_, std::vector< _triad<int> > mesh_index_);

	bool loadExtraInfo(char *file_name_);				// load extra information file

	int getNumNode();					// return x.size()
	int getNumMesh();					// return mesh_index.size()
	
	bool update_xg();					// update xg

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

