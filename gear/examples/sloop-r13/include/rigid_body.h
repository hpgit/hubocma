//================================================================================
//         RIGID BONE FOR MULTIBODY SKELETONS
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GBODY_RIGID_OBJECT_
#define _GBODY_RIGID_OBJECT_

#include <vector>
#include "gear.h"
#include "surface_fixed_to_body.h"
#include "surface_fixed_to_frame.h"

//=============================================================
//                 RigidBody
//=============================================================
class RigidBody: public GBody
{
	struct BoundingBox
	{
		double extents[3];	// extents of the box (half-length)
		Vec3 position;		// global position of the box center
		SO3 orientation;	// global orientation of the box
	};

public:
	// surface for calculation (contact, friction)
	SurfaceFixedToRigidBody surface;

	// surface for rendering only
	std::vector<SurfaceFixedToFrame *> surfaces_for_rendering_only;

	// collision
	bool bCheckCollision;		// set true to check collision
	dse3 Fe2;					// contact force acting on the body from a deformable body

	// marker
	Vec3 pos_marker;			// marker position w.r.t. {body}

	// mouse-drag plane
	Vec3 pos_center_ref_plane;	// a point on the plane w.r.t. {body}
	Vec3 vec_normal_ref_plane;	// the normal vector of the plane w.r.t. {body}

	// oriented bounding box (OBB) (orientation of OBB is aligned to {body})
	BoundingBox bounding_box;		// bounding box (the global position and orientation will be automatically updated in update_T().)
	Vec3 pos_bounding_box_local;	// center of the bounding box w.r.t. {body}

	// rendering options
	bool b_rndr_ellipsoid;
	bool b_rndr_surface;
	bool b_rndr_surfaces_for_rendering_only;
	bool b_rndr_bounding_box;
	bool b_rndr_marker;			// set true to show the location of the marker
	double radius_marker;		// radius of marker
	double bounding_box_color[3];

public:
	RigidBody();
	~RigidBody();

public:
	void clearSurface();
	// clear all surfaces

	bool loadSurfaceMesh(const char *file_name_, Vec3 scale_ = Vec3(1,1,1), SE3 T0_ = SE3(), bool b_surface_for_calc_ = true, int index_base_ = 0, char *surf_name_ = NULL, char *mtl_name_ = NULL);
	bool addSurfaceMesh(const char *file_name_, Vec3 scale_ = Vec3(1,1,1), SE3 T0_ = SE3(), bool b_surface_for_calc_ = true, int index_base_ = 0, char *surf_name_ = NULL, char *mtl_name_ = NULL);	
	// loads/add surface mesh file
	//		sx_, sy_, sz_ = scale factors for the axes of {mesh file reference frame}
	//		T0_: {body} -> {mesh file reference frame}

	void setBoxSurface(double wx_, double wy_, double wz_, SE3 T_ref_ = SE3(), int div_x_ = 1, int div_y_ = 1, int div_z_ = 1, bool b_surface_for_calc_ = true);
	// see setBoxObject() for options

	void setCylinderSurface(double rb_, double rt_, double h_, SE3 T_ref_ = SE3(), int n_slice_ = 8, bool b_surface_for_calc_ = true);
	// see setCylinderObject() for options

	void addEllipsoidSurface(double rx_, double ry_, double rz_, SE3 T_ref_ = SE3(), int n_slice_ = 8, int n_stack_ = 8, bool b_surface_for_calc_ = true);

	void setBoxObject(double wx_, double wy_, double wz_, double mass_, SE3 T_ref_ = SE3(), bool b_msh_ = true, int div_x_ = 1, int div_y_ = 1, int div_z_ = 1, bool b_surface_for_calc_ = true);
	// sets up a box whose centroid will be located at the origin of {ref}
	// (wx_, wy_, wz_) = box width in x-y-z coordinates 
	// mass_ = mass of the rigid box
	// T_ref_ = {body} --> {ref}
	// The inertia matrix will be generated automatically based on the box size and mass under the assumption of uniform density.
	// If b_msh_ == true, a surface mesh for the box will be generated automatically. 
	// The mesh will be a triangular mesh with 8 vertices located at
	// {(wx_/2, wy_/2, wz_/2),...,(-wx_/2, -wy_/2, -wz_/2)} in {ref}.
	// To add surface nodes along the box edges, set div_x_, div_y_, div_z_ with more than 2.

	void setCylinderObject(double rb_, double rt_, double h_, double mass_, SE3 T_ref_ = SE3(), bool b_msh_ = false, int n_slice_ = 8, bool b_surface_for_calc_ = true);
	// sets up a solid cylinder(or conical frustum) whose base circle will be laid on the x-y plane of {ref}.
	// rb_ = base radius
	// rt_ = top radius
	// h_ = height of the cylinder
	// mass_ = mass of the rigid cylinder
	// T_ref_ = {body} --> {ref}
	// An approximated inertia matrix will be calculated from a cylinder which has the same height and volume.
	// If b_msh_ == true, a surface mesh for the cylinder will be generated automatically.
	// The mesh will be a triangular mesh with 18(=8+8+1+1) vertices located on the rims and centers of the base and top circles.

	void addEllipsoidObject(double density, double rx_, double ry_, double rz_, SE3 T_ref_ = SE3(), bool b_msh_ = true, int n_slice_ = 8, int n_stack_ = 8, bool b_surface_for_calc_ = true);



	void enableCollision(bool b_) { bCheckCollision = b_; }

	void setMarkerPosition(const Vec3 &p_) { pos_marker = p_; }
	void setMarkerPositionGlobal(const Vec3 &p_) { pos_marker = Inv(T_global) * p_; }

	Vec3 getMarkerPosition() { return pos_marker; }
	Vec3 getMarkerPositionGlobal() { return T_global * pos_marker; }

	void setReferencePlane(const Vec3 &p_, const Vec3 &n_) { pos_center_ref_plane = p_; vec_normal_ref_plane = n_; }
	void setReferencePlaneGlobal(const Vec3 &pg_, const Vec3 &ng_) { SE3 invT = Inv(T_global); pos_center_ref_plane = invT * pg_; vec_normal_ref_plane = invT.GetRotation() * ng_; }

	void getReferencePlane(Vec3 &p_, Vec3 n_) { p_ = getReferencePlanePosition(); n_ = getReferencePlaneNormalVector(); }
	void getReferencePlaneGlobal(Vec3 &pg_, Vec3 &ng_) { pg_ = getReferencePlanePositionGlobal(); ng_ = getReferencePlaneNormalVectorGlobal(); }

	Vec3 getReferencePlanePosition() { return pos_center_ref_plane; }
	Vec3 getReferencePlanePositionGlobal() { return T_global * pos_center_ref_plane; }

	Vec3 getReferencePlaneNormalVector() { return vec_normal_ref_plane; }
	Vec3 getReferencePlaneNormalVectorGlobal() { return T_global.GetRotation() * vec_normal_ref_plane; }

	void setBoundingBox(double Lx_, double Ly_, double Lz_, const Vec3 &pos_local_ = Vec3(0,0,0)) { pos_bounding_box_local = pos_local_; bounding_box.extents[0] = 0.5*Lx_; bounding_box.extents[1] = 0.5*Ly_; bounding_box.extents[2] = 0.5*Lz_; }
	void setBoundingBoxAutomatically(); // compute minimal bounding box based on RigidBody::surface information
	void setBoundingBoxExtents(double Lx_, double Ly_, double Lz_) { bounding_box.extents[0] = 0.5*Lx_; bounding_box.extents[1] = 0.5*Ly_; bounding_box.extents[2] = 0.5*Lz_; }
	void setBoundingBoxPosition(const Vec3 &pos_local_) { pos_bounding_box_local = pos_local_; }
	void setBoundingBoxColor(const Vec3 &c_) { bounding_box_color[0] = c_[0]; bounding_box_color[1] = c_[1]; bounding_box_color[2] = c_[2]; }

	void update_Fe();		// update Fe with m_pFveg and m_pFcg

	// rendering
	void SetRadiusMarker(double r_) { radius_marker = r_; }

	void removeAllSurfacesForRenderingOnly();

public: // virtual functions
	bool getReady();

	void update_T();
	void update_V();
	void update_F();
	void update_aB();
	void render();
};

#endif

