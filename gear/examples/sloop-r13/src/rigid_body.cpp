#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <vector>
#include <fstream>
#include <math.h>
#include "rigid_body.h"
#include "gear.h"

using namespace std;

#define _MAX_NUM_SURF_RNDR_ONLY 100

//=============================================================
//                 RigidBody
//=============================================================
RigidBody::RigidBody()
{
	surface.setMasterBody(this);

	bCheckCollision = false;
	Fe2.SetZero();
	pos_marker.SetZero();
	pos_center_ref_plane.SetZero();
	vec_normal_ref_plane = Vec3(1,0,0); //.SetZero();

	bounding_box.extents[0] = bounding_box.extents[1] = bounding_box.extents[2] = 0.0;
	pos_bounding_box_local.SetZero();

	b_rndr_ellipsoid = false;
	b_rndr_surface = true;
	b_rndr_surfaces_for_rendering_only = true;
	b_rndr_bounding_box = false;
	b_rndr_marker = true; //false;
	radius_marker = 0.001;
	bounding_box_color[0] = 0; bounding_box_color[1] = 0; bounding_box_color[2] = 1; // blue
}

RigidBody::~RigidBody()
{
	removeAllSurfacesForRenderingOnly();
}

void RigidBody::removeAllSurfacesForRenderingOnly()
{
	for (size_t i=0; i<surfaces_for_rendering_only.size(); i++) {
		delete surfaces_for_rendering_only[i];
	}
	surfaces_for_rendering_only.clear();
}

bool RigidBody::getReady()
{
	if ( !GBody::getReady() ) return false;

	// if the bounding box has not been set yet, set it automatically.
	if ( bounding_box.extents[0] == 0.0 && bounding_box.extents[1] == 0.0 && bounding_box.extents[2] == 0.0 ) {
		setBoundingBoxAutomatically();
	}

	return true;
}

void RigidBody::update_T()
{
	GBody::update_T();

	surface.update_xg();

	if ( b_rndr_surfaces_for_rendering_only ) {
		for (size_t i=0; i<surfaces_for_rendering_only.size(); i++) {
			surfaces_for_rendering_only[i]->update_xg();
		}
	}

	// update global position and orientation of the bounding box
	bounding_box.position = T_global * pos_bounding_box_local;
	bounding_box.orientation = T_global.GetRotation();
}

void RigidBody::update_V()
{
	GBody::update_V();

	surface.update_vg();
}

void RigidBody::update_Fe()
{
	SO3 Rt = ~getOrientationGlobal();
	Vec3 fg, f;

	if ( bCheckCollision ) {
		surface.transform_fg();
	}

	Fe += Fe2;
}

void RigidBody::update_F()
{
	update_Fe();
	GBody::update_F();
}

void RigidBody::update_aB()
{
	update_Fe();
	GBody::update_aB();
}

void RigidBody::clearSurface()
{
	surface.clearSurface();

	for (size_t i=0; i<surfaces_for_rendering_only.size(); i++) {
		surfaces_for_rendering_only[i]->clearSurface();
	}
}

bool RigidBody::loadSurfaceMesh(const char *file_name_, Vec3 scale_, SE3 T0_, bool b_surface_for_calc_, int index_base_, char *surf_name_, char *mtl_name_)
{
	if ( b_surface_for_calc_ ) {
		surface.clearSurface();
	} else {
		removeAllSurfacesForRenderingOnly();
	}
	return addSurfaceMesh(file_name_, scale_, T0_, b_surface_for_calc_, index_base_, surf_name_, mtl_name_);
}

bool RigidBody::addSurfaceMesh(const char *file_name_, Vec3 scale_, SE3 T0_, bool b_surface_for_calc_, int index_base_, char *surf_name_, char *mtl_name_)
{
	int i, n, m;
	std::vector<Vec3> r0;
	std::vector< _triad<int> > mesh_index;
	Vec3 x_tmp;
	ifstream fin;

	// open file
	fin.open(file_name_);
	if ( !fin.is_open() ) return false;

	// get the number of nodes and meshes
	fin >> n >> m;

	// get node positions from file
	r0.resize(n);
	for (i=0; i<n; i++) {
		// node position in {mesh file reference frame}
		fin >> x_tmp[0] >> x_tmp[1] >> x_tmp[2];

		// apply scale factors
		x_tmp[0] *= scale_[0];
		x_tmp[1] *= scale_[1];
		x_tmp[2] *= scale_[2];

		// position of nodes in {body}
		r0[i] = T0_ * x_tmp;
	}

	// get mesh index from file
	mesh_index.resize(m);
	for (i=0; i<m; i++) {
		fin >> mesh_index[i][0] >> mesh_index[i][1] >> mesh_index[i][2];
		mesh_index[i][0] -= index_base_;
		mesh_index[i][1] -= index_base_;
		mesh_index[i][2] -= index_base_;
	}

	// close file
	fin.close();

	if ( b_surface_for_calc_ ) {
		// add surface
		if ( !surface.addSurface(r0, mesh_index) ) return false;
		// update position of nodes in {global}
		if ( !surface.update_xg() ) return false;
		// update velocity of nodes in {global}
		if ( !surface.update_vg() ) return false;
	} else {
		SurfaceFixedToFrame *psurf = new SurfaceFixedToFrame;
		psurf->setMasterFrame(&T_global);
		if ( !psurf->addSurface(r0, mesh_index) ) return false;
		psurf->setName(surf_name_);
		psurf->setMaterial(mtl_name_);
		if ( !psurf->update_xg() ) return false;
		surfaces_for_rendering_only.push_back(psurf);
	}

	return true;
}

void RigidBody::setBoxSurface(double wx_, double wy_, double wz_, SE3 T_ref_, int div_x_, int div_y_, int div_z_, bool b_surface_for_calc_)
{
	std::vector<Vec3> p;
	std::vector<Vec3> pos0;
	std::vector< _triad<int> > mesh_index;

	// build surface
	int num_node_added = 0;
	if (div_x_ > 1) { num_node_added += 4 * (div_x_-1);	}
	if (div_y_ > 1) { num_node_added += 4 * (div_y_-1);	}
	if (div_z_ > 1) { num_node_added += 4 * (div_z_-1);	}

	p.resize(8+num_node_added);
	pos0.resize(8+num_node_added);
	mesh_index.resize(12);

	// node position in {ref}
	p[0] = Vec3(-wx_/2, -wy_/2, -wz_/2);
	p[1] = Vec3(wx_/2, -wy_/2, -wz_/2);
	p[2] = Vec3(wx_/2, wy_/2, -wz_/2);
	p[3] = Vec3(-wx_/2, wy_/2, -wz_/2);
	p[4] = Vec3(-wx_/2, -wy_/2, wz_/2);
	p[5] = Vec3(wx_/2, -wy_/2, wz_/2);
	p[6] = Vec3(wx_/2, wy_/2, wz_/2);
	p[7] = Vec3(-wx_/2, wy_/2, wz_/2);

	int cnt = 8;
	if ( div_x_ > 1) {
		for (int i=0; i<div_x_-1; i++) {
			p[cnt++] = p[0] + (double(i+1)/double(div_x_)) * (p[1]-p[0]);
			p[cnt++] = p[2] + (double(i+1)/double(div_x_)) * (p[3]-p[2]);
			p[cnt++] = p[4] + (double(i+1)/double(div_x_)) * (p[5]-p[4]);
			p[cnt++] = p[6] + (double(i+1)/double(div_x_)) * (p[7]-p[6]);
		}
	}
	if ( div_y_ > 1) {
		for (int i=0; i<div_y_-1; i++) {
			p[cnt++] = p[0] + (double(i+1)/double(div_y_)) * (p[3]-p[0]);
			p[cnt++] = p[1] + (double(i+1)/double(div_y_)) * (p[2]-p[1]);
			p[cnt++] = p[5] + (double(i+1)/double(div_y_)) * (p[6]-p[5]);
			p[cnt++] = p[4] + (double(i+1)/double(div_y_)) * (p[7]-p[4]);
		}
	}
	if ( div_z_ > 1) {
		for (int i=0; i<div_z_-1; i++) {
			p[cnt++] = p[0] + (double(i+1)/double(div_z_)) * (p[4]-p[0]);
			p[cnt++] = p[1] + (double(i+1)/double(div_z_)) * (p[5]-p[1]);
			p[cnt++] = p[2] + (double(i+1)/double(div_z_)) * (p[6]-p[2]);
			p[cnt++] = p[3] + (double(i+1)/double(div_z_)) * (p[7]-p[3]);
		}
	}

	// node position in {body}
	for (size_t i=0; i<pos0.size(); i++) {
		pos0[i] = T_ref_ * p[i];
	}

	// mesh index
	mesh_index[0] = _triad<int>(0, 1, 5);
	mesh_index[1] = _triad<int>(0, 5, 4);
	mesh_index[2] = _triad<int>(1, 6, 5);
	mesh_index[3] = _triad<int>(1, 2, 6);
	mesh_index[4] = _triad<int>(2, 3, 6);
	mesh_index[5] = _triad<int>(3, 7, 6);
	mesh_index[6] = _triad<int>(0, 4, 7);
	mesh_index[7] = _triad<int>(0, 7, 3);
	mesh_index[8] = _triad<int>(4, 5, 6);
	mesh_index[9] = _triad<int>(4, 6, 7);
	mesh_index[10] = _triad<int>(0, 2, 1);
	mesh_index[11] = _triad<int>(0, 3, 2);

	// set surface
	if ( b_surface_for_calc_ ) {
		surface.clearSurface();
		surface.addSurface(pos0, mesh_index);
	} else {
		removeAllSurfacesForRenderingOnly();
		SurfaceFixedToFrame *psurf = new SurfaceFixedToFrame;
		psurf->setMasterFrame(&T_global);
		psurf->addSurface(pos0, mesh_index);
		psurf->update_xg();
		surfaces_for_rendering_only.push_back(psurf);
	}
}

void RigidBody::setCylinderSurface(double rb_, double rt_, double h_, SE3 T_ref_, int n_slice_, bool b_surface_for_calc_)
{
	std::vector<Vec3> pos0;
	std::vector< _triad<int> > mesh_index;

	if ( n_slice_ >= 3 ) {
		
		// node position in {ref}
		vector<Vec3> pb(n_slice_), pt(n_slice_);
		Vec3 cb, ct;

		// node index
		vector<int> idx_pb(n_slice_), idx_pt(n_slice_);
		int idx_cb, idx_ct;

		double theta = 2. * 3.14159265358979 / double(n_slice_);
		double ang;

		for (int i=0; i<n_slice_; i++) {
			//ang = double(i)*theta;
			ang = (double(i)+0.5)*theta;
			pb[i] = Vec3(rb_ * cos(ang), rb_ * sin(ang), 0);
			pt[i] = Vec3(rt_ * cos(ang), rt_ * sin(ang), h_);
		}
		cb = Vec3(0, 0, 0);
		ct = Vec3(0, 0, h_);

		// resize pos0, mesh_index
		pos0.resize(2*n_slice_+2);
		mesh_index.resize(4*n_slice_);

		// set node position in {body}
		for (int i=0; i<n_slice_; i++) {
			pos0[i] = T_ref_ * pb[i];
			pos0[n_slice_+i] = T_ref_ * pt[i];
			idx_pb[i] = i;
			idx_pt[i] = n_slice_+i;
		}
		pos0[2*n_slice_] = T_ref_ * cb;
		pos0[2*n_slice_+1] = T_ref_ * ct;
		idx_cb = 2*n_slice_;
		idx_ct = 2*n_slice_+1;

		// set mesh index
		int last = n_slice_-1;

		for (int i=0; i<last; i++) {
			mesh_index[i] = _triad<int>(idx_pb[i], idx_pb[i+1], idx_pt[i]);
		}
		mesh_index[last] = _triad<int>(idx_pb[last], idx_pb[0], idx_pt[last]);

		for (int i=0; i<last; i++) {
			mesh_index[n_slice_+i] = _triad<int>(idx_pt[i], idx_pb[i+1], idx_pt[i+1]);
		}
		mesh_index[n_slice_+last] = _triad<int>(idx_pt[last], idx_pb[0], idx_pt[0]);
		
		for (int i=0; i<last; i++) {
			mesh_index[2*n_slice_+i] = _triad<int>(idx_pb[i], idx_cb, idx_pb[i+1]);
		}
		mesh_index[2*n_slice_+last] = _triad<int>(idx_pb[last], idx_cb, idx_pb[0]);

		for (int i=0; i<last; i++) {
			mesh_index[3*n_slice_+i] = _triad<int>(idx_pt[i], idx_pt[i+1], idx_ct);
		}
		mesh_index[3*n_slice_+last] = _triad<int>(idx_pt[last], idx_pt[0], idx_ct);

		// set surface
		if ( b_surface_for_calc_ ) {
			surface.clearSurface();
			surface.addSurface(pos0, mesh_index);
		} else {
			removeAllSurfacesForRenderingOnly();
			SurfaceFixedToFrame *psurf = new SurfaceFixedToFrame;
			psurf->setMasterFrame(&T_global);
			psurf->addSurface(pos0, mesh_index);
			psurf->update_xg();
			surfaces_for_rendering_only.push_back(psurf);
		}
	}
}

void RigidBody::addEllipsoidSurface(double rx_, double ry_, double rz_, SE3 T_ref_, int n_slice_, int n_stack_, bool b_surface_for_calc_)
{
	vector<Vec3> pos0;
	std::vector<Vec3> pos;
	std::vector< _triad<int> > mesh_index;

	if ( n_slice_ >= 3 ) {

		const double PI = 3.14159265358979;
		double tStep = (PI) / (double)n_slice_;
		double sStep = (PI) / (double)n_stack_;

		// node position in {ref}
		for(double t = -PI/2; t <= (PI/2)+.0001; t += tStep) {
			for(double s = -PI; s <= PI+.0001; s += sStep) {
				pos0.push_back(Vec3(rx_ * cos(t) * cos(s), ry_ * cos(t) * sin(s), rz_ * sin(t)));
				pos0.push_back(Vec3(rx_ * cos(t+tStep) * cos(s), ry_ * cos(t+tStep) * sin(s), rz_ * sin(t+tStep)));
			}
		}

		// node position in {body}
		pos.resize(pos0.size());
		for (size_t i=0; i<pos.size(); i++) {
			pos[i] = T_ref_ * pos0[i];
		}

		// set mesh index
		mesh_index.resize(pos.size()-2);
		for (size_t i=0; i<mesh_index.size(); i+=2) {
			mesh_index[i] = _triad<int>(i,i+1,i+2);
		}
		for (size_t i=1; i<mesh_index.size(); i+=2) {
			mesh_index[i] = _triad<int>(i+1,i,i+2);
		}

		// set surface
		if ( b_surface_for_calc_ ) {
			//surface.clearSurface();
			surface.addSurface(pos, mesh_index);
		} else {
			//removeAllSurfacesForRenderingOnly();
			SurfaceFixedToFrame *psurf = new SurfaceFixedToFrame;
			psurf->setMasterFrame(&T_global);
			psurf->addSurface(pos, mesh_index);
			psurf->update_xg();
			surfaces_for_rendering_only.push_back(psurf);
		}
	}
}

void RigidBody::addEllipsoidObject(double density, double rx_, double ry_, double rz_, SE3 T_ref_, bool b_msh_, int n_slice_, int n_stack_, bool b_surface_for_calc_)
{
	double m = density * 4./3. * 3.14159 * rx_ * ry_ * rz_;
	double Ixx = m * (ry_*ry_ + rz_*rz_) / 5.;
	double Iyy = m * (rz_*rz_ + rx_*rx_) / 5.;
	double Izz = m * (rx_*rx_ + ry_*ry_) / 5.;

	addMass(m, Ixx, Iyy, Izz, 0, 0, 0, T_ref_);

	if ( b_msh_ ) {
		addEllipsoidSurface(rx_, ry_, rz_, T_ref_, n_slice_, n_stack_, b_surface_for_calc_);
	}
}

void RigidBody::setBoxObject(double wx_, double wy_, double wz_, double mass_, SE3 T_ref_, bool b_msh_, int div_x_, int div_y_, int div_z_, bool b_surface_for_calc_)
{
	double Ixx, Iyy, Izz;
	std::vector<Vec3> p;
	std::vector<Vec3> pos0;
	std::vector< _triad<int> > mesh_index;

	if ( b_surface_for_calc_ ) {
		// calculate inertia
		Ixx = mass_ * (wy_*wy_ + wz_*wz_) / 12.;
		Iyy = mass_ * (wz_*wz_ + wx_*wx_) / 12.;
		Izz = mass_ * (wx_*wx_ + wy_*wy_) / 12.;

		// set mass and inertia
		setMass(mass_, Ixx, Iyy, Izz, 0, 0, 0, T_ref_);
	}

	// build surface
	if ( b_msh_ ) {

		int num_node_added = 0;
		if (div_x_ > 1) { num_node_added += 4 * (div_x_-1);	}
		if (div_y_ > 1) { num_node_added += 4 * (div_y_-1);	}
		if (div_z_ > 1) { num_node_added += 4 * (div_z_-1);	}

		p.resize(8+num_node_added);
		pos0.resize(8+num_node_added);
		mesh_index.resize(12);

		// node position in {ref}
		p[0] = Vec3(-wx_/2, -wy_/2, -wz_/2);
		p[1] = Vec3(wx_/2, -wy_/2, -wz_/2);
		p[2] = Vec3(wx_/2, wy_/2, -wz_/2);
		p[3] = Vec3(-wx_/2, wy_/2, -wz_/2);
		p[4] = Vec3(-wx_/2, -wy_/2, wz_/2);
		p[5] = Vec3(wx_/2, -wy_/2, wz_/2);
		p[6] = Vec3(wx_/2, wy_/2, wz_/2);
		p[7] = Vec3(-wx_/2, wy_/2, wz_/2);

		int cnt = 8;
		if ( div_x_ > 1) {
			for (int i=0; i<div_x_-1; i++) {
				p[cnt++] = p[0] + (double(i+1)/double(div_x_)) * (p[1]-p[0]);
				p[cnt++] = p[2] + (double(i+1)/double(div_x_)) * (p[3]-p[2]);
				p[cnt++] = p[4] + (double(i+1)/double(div_x_)) * (p[5]-p[4]);
				p[cnt++] = p[6] + (double(i+1)/double(div_x_)) * (p[7]-p[6]);
			}
		}
		if ( div_y_ > 1) {
			for (int i=0; i<div_y_-1; i++) {
				p[cnt++] = p[0] + (double(i+1)/double(div_y_)) * (p[3]-p[0]);
				p[cnt++] = p[1] + (double(i+1)/double(div_y_)) * (p[2]-p[1]);
				p[cnt++] = p[5] + (double(i+1)/double(div_y_)) * (p[6]-p[5]);
				p[cnt++] = p[4] + (double(i+1)/double(div_y_)) * (p[7]-p[4]);
			}
		}
		if ( div_z_ > 1) {
			for (int i=0; i<div_z_-1; i++) {
				p[cnt++] = p[0] + (double(i+1)/double(div_z_)) * (p[4]-p[0]);
				p[cnt++] = p[1] + (double(i+1)/double(div_z_)) * (p[5]-p[1]);
				p[cnt++] = p[2] + (double(i+1)/double(div_z_)) * (p[6]-p[2]);
				p[cnt++] = p[3] + (double(i+1)/double(div_z_)) * (p[7]-p[3]);
			}
		}

		// node position in {body}
		for (size_t i=0; i<pos0.size(); i++) {
			pos0[i] = T_ref_ * p[i];
		}

		// mesh index
		mesh_index[0] = _triad<int>(0, 1, 5);
		mesh_index[1] = _triad<int>(0, 5, 4);
		mesh_index[2] = _triad<int>(1, 6, 5);
		mesh_index[3] = _triad<int>(1, 2, 6);
		mesh_index[4] = _triad<int>(2, 3, 6);
		mesh_index[5] = _triad<int>(3, 7, 6);
		mesh_index[6] = _triad<int>(0, 4, 7);
		mesh_index[7] = _triad<int>(0, 7, 3);
		mesh_index[8] = _triad<int>(4, 5, 6);
		mesh_index[9] = _triad<int>(4, 6, 7);
		mesh_index[10] = _triad<int>(0, 2, 1);
		mesh_index[11] = _triad<int>(0, 3, 2);

		// set surface
		if ( b_surface_for_calc_ ) {
			surface.clearSurface();
			surface.addSurface(pos0, mesh_index);
		} else {
			removeAllSurfacesForRenderingOnly();
			SurfaceFixedToFrame *psurf = new SurfaceFixedToFrame;
			psurf->setMasterFrame(&T_global);
			psurf->addSurface(pos0, mesh_index);
			psurf->update_xg();
			surfaces_for_rendering_only.push_back(psurf);
		}
	}    
}

void RigidBody::setCylinderObject(double rb_, double rt_, double h_, double mass_, SE3 T_ref_, bool b_msh_, int n_slice_, bool b_surface_for_calc_)
{
	double Ixx, Iyy, Izz;
	double r0 = sqrt((rb_*rb_ + rb_*rt_ + rt_*rt_)/3.);	// radius of a cylinder which has the same volume
	SE3 T_com;											// {ref} --> {com}
	std::vector<Vec3> pos0;
	std::vector< _triad<int> > mesh_index;

	if ( b_surface_for_calc_ ) {
		Ixx = 1./12.*mass_*h_*h_ + 1./4.*mass_*r0*r0;
		Iyy = Ixx;
		Izz = 1./2.*mass_*r0*r0;

		T_com = SE3(Vec3(0, 0, (rb_+2.*rt_)/(3.*(rb_+rt_))*h_));

		setMass(mass_, Ixx, Iyy, Izz, 0, 0, 0, T_ref_ * T_com);
		//	setMass(mass_, Ixx, Iyy, Izz, 0, 0, 0, T_ref_);
	}

	if ( b_msh_ && n_slice_ >= 3 ) {

		// node position in {ref}
		vector<Vec3> pb(n_slice_), pt(n_slice_);
		Vec3 cb, ct;

		// node index
		vector<int> idx_pb(n_slice_), idx_pt(n_slice_);
		int idx_cb, idx_ct;

		double theta = 2. * 3.14159265358979 / double(n_slice_);
		double ang;

		for (int i=0; i<n_slice_; i++) {
			//ang = double(i)*theta;
			ang = (double(i)+0.5)*theta;
			pb[i] = Vec3(rb_ * cos(ang), rb_ * sin(ang), 0);
			pt[i] = Vec3(rt_ * cos(ang), rt_ * sin(ang), h_);
		}
		cb = Vec3(0, 0, 0);
		ct = Vec3(0, 0, h_);

		// resize pos0, mesh_index
		pos0.resize(2*n_slice_+2);
		mesh_index.resize(4*n_slice_);

		// set node position in {body}
		for (int i=0; i<n_slice_; i++) {
			pos0[i] = T_ref_ * pb[i];
			pos0[n_slice_+i] = T_ref_ * pt[i];
			idx_pb[i] = i;
			idx_pt[i] = n_slice_+i;
		}
		pos0[2*n_slice_] = T_ref_ * cb;
		pos0[2*n_slice_+1] = T_ref_ * ct;
		idx_cb = 2*n_slice_;
		idx_ct = 2*n_slice_+1;

		// set mesh index
		int last = n_slice_-1;

		for (int i=0; i<last; i++) {
			mesh_index[i] = _triad<int>(idx_pb[i], idx_pb[i+1], idx_pt[i]);
		}
		mesh_index[last] = _triad<int>(idx_pb[last], idx_pb[0], idx_pt[last]);

		for (int i=0; i<last; i++) {
			mesh_index[n_slice_+i] = _triad<int>(idx_pt[i], idx_pb[i+1], idx_pt[i+1]);
		}
		mesh_index[n_slice_+last] = _triad<int>(idx_pt[last], idx_pb[0], idx_pt[0]);

		for (int i=0; i<last; i++) {
			mesh_index[2*n_slice_+i] = _triad<int>(idx_pb[i], idx_cb, idx_pb[i+1]);
		}
		mesh_index[2*n_slice_+last] = _triad<int>(idx_pb[last], idx_cb, idx_pb[0]);

		for (int i=0; i<last; i++) {
			mesh_index[3*n_slice_+i] = _triad<int>(idx_pt[i], idx_pt[i+1], idx_ct);
		}
		mesh_index[3*n_slice_+last] = _triad<int>(idx_pt[last], idx_pt[0], idx_ct);

		// set surface
		if ( b_surface_for_calc_ ) {
			surface.clearSurface();
			surface.addSurface(pos0, mesh_index);
		} else {
			removeAllSurfacesForRenderingOnly();
			SurfaceFixedToFrame *psurf = new SurfaceFixedToFrame;
			psurf->setMasterFrame(&T_global);
			psurf->addSurface(pos0, mesh_index);
			psurf->update_xg();
			surfaces_for_rendering_only.push_back(psurf);
		}
	}
}

void RigidBody::setBoundingBoxAutomatically()
{
	const double inf = 1E10;
	double maxX = -inf, minX = inf, maxY = -inf, minY = inf, maxZ = -inf, minZ = inf;
	double *p;

	bounding_box.extents[0] = bounding_box.extents[1] = bounding_box.extents[2] = 0.0;
	pos_bounding_box_local.SetZero();

	if ( surface.getNumNode() > 0 ) {
		for (int i=0; i<surface.getNumNode(); i++) {
			p = surface.r0[i].GetArray();
			if ( p[0] > maxX ) maxX = p[0];
			if ( p[0] < minX ) minX = p[0];
			if ( p[1] > maxY ) maxY = p[1];
			if ( p[1] < minY ) minY = p[1];
			if ( p[2] > maxZ ) maxZ = p[2];
			if ( p[2] < minZ ) minZ = p[2];
		}
		pos_bounding_box_local = Vec3(0.5*(minX+maxX),0.5*(minY+maxY),0.5*(minZ+maxZ));
		bounding_box.extents[0] = 0.5*(maxX-minX);
		bounding_box.extents[1] = 0.5*(maxY-minY);
		bounding_box.extents[2] = 0.5*(maxZ-minZ);
	}
}

void RigidBody::render()
{
	GLdouble previous_color[4], previous_line_width;
	glGetDoublev(GL_CURRENT_COLOR, previous_color);
	glGetDoublev(GL_LINE_WIDTH, &previous_line_width);

	if ( b_rndr_surface ) {
		surface.render();
	}

	if ( b_rndr_surfaces_for_rendering_only ) {
		for (size_t i=0; i<surfaces_for_rendering_only.size(); i++) {
			surfaces_for_rendering_only[i]->render();
		}
	}

	if ( b_rndr_marker ) {
		Vec3 p_marker = getMarkerPositionGlobal();
		glPushMatrix();
		glTranslated(p_marker[0], p_marker[1], p_marker[2]);
		glColor4d(1, 0.5, 0, 0.6);	// orange marker
		glsubSphere(radius_marker, GLSUB_SOLID);
		glPopMatrix();
	}

	if ( b_rndr_bounding_box ) {
		glColor4d(bounding_box_color[0],bounding_box_color[1],bounding_box_color[2],1);
		glPushMatrix();
		glMultMatrixd(T_global.GetArray());
		glTranslated(pos_bounding_box_local[0],pos_bounding_box_local[1],pos_bounding_box_local[2]);
		glBegin(GL_LINE_LOOP);
		glVertex3d(-bounding_box.extents[0], -bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], -bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(-bounding_box.extents[0], bounding_box.extents[1], -bounding_box.extents[2]);
		glEnd();
		glBegin(GL_LINE_LOOP);
		glVertex3d(-bounding_box.extents[0], -bounding_box.extents[1], bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], -bounding_box.extents[1], bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], bounding_box.extents[1], bounding_box.extents[2]);
		glVertex3d(-bounding_box.extents[0], bounding_box.extents[1], bounding_box.extents[2]);
		glEnd();
		glBegin(GL_LINES);
		glVertex3d(-bounding_box.extents[0], -bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(-bounding_box.extents[0], -bounding_box.extents[1], bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], -bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], -bounding_box.extents[1], bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(bounding_box.extents[0], bounding_box.extents[1], bounding_box.extents[2]);
		glVertex3d(-bounding_box.extents[0], bounding_box.extents[1], -bounding_box.extents[2]);
		glVertex3d(-bounding_box.extents[0], bounding_box.extents[1], bounding_box.extents[2]);
		glEnd();
		glPopMatrix();
	}

	glColor4dv(previous_color);
	glLineWidth((GLfloat)previous_line_width);
}
