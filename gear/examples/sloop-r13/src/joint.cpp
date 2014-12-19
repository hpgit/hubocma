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
#include "joint.h"
#include "liegroup.h"
#include "rmatrix3j.h"
#include "glsub.h"

using namespace std;

static GLUquadricObj *qobj = gluNewQuadric();

//=============================================================
//                 JointProperties
//=============================================================

JointProperties::JointProperties()
{
	 bRendering = true;
	 bColorMapMode = false;
	 typeColorMap = GLSUB_COLOR_MAP_AUTUMN_INVERSE;
	 maxTorqueForRendering = 1.0;
	 color[0] = 0.5; color[1] = 0.5; color[2] = 0.0; alpha = 1.0; 
	 draw_type = GLSUB_SOLID;
	 bRenderSurface = false;
}

bool JointProperties::loadSurfaceMesh(char *file_name_, Vec3 scale_, SE3 T0_, int index_base_)
{
	surface.clearSurface();
	return addSurfaceMesh(file_name_, scale_, T0_, index_base_);
}

bool JointProperties::addSurfaceMesh(char *file_name_, Vec3 scale_, SE3 T0_, int index_base_)
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

		// position of nodes in {joint left}
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

	// add surface
	if ( !surface.addSurface(r0, mesh_index) ) return false;

	// update position of nodes in {global}
	if ( !surface.update_xg() ) return false;

	// render this surface instead of the default shape
	bRenderSurface = true;

	return true;
}

//=============================================================
//                 RevoluteJoint
//=============================================================

RevoluteJoint::RevoluteJoint()
{
	radius = 0.015;
	height = 0.04;
	surface.setMasterFrame(&T_global);
}

bool RevoluteJoint::setCylinderSurface(double r_, double h_, int n_slice_)
{
	std::vector<Vec3> pos0;
	std::vector< _triad<int> > mesh_index;

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
		pb[i] = Vec3(r_ * cos(ang), r_ * sin(ang), 0);
		pt[i] = Vec3(r_ * cos(ang), r_ * sin(ang), h_);
	}
	cb = Vec3(0, 0, 0);
	ct = Vec3(0, 0, h_);

	// resize pos0, mesh_index
	pos0.resize(2*n_slice_+2);
	mesh_index.resize(4*n_slice_);

	// T: SE(3) {joint left} --> {ref}
	// z-axis of {ref} coincides with the rotational axis of the revolute joint
	SE3 T;
	Vec3 z(0,0,1);
	axis.Normalize();

	if ( Norm(axis-z) < 1E-6 )		// if axis = z
		T.SetIdentity();	// identity
	else if ( Norm(axis+z) < 1E-6 )	// else if axis = -z
		T = SE3(SO3(1,0,0,0,-1,0,0,0,-1),Vec3(0,0,0));
	else
	{
		Vec3 m = Cross(z,axis);
		double theta = asin(Norm(m));
		if ( Inner(z,axis) < 0 ) theta = 3.14159265358979 - theta;

		m.Normalize();
		T = SE3(Exp(m*theta), Vec3(0,0,0));
	}

	T *= SE3(Vec3(0, 0, -h_/2.));

	// set node position in {joint left}
	for (int i=0; i<n_slice_; i++) {
		pos0[i] = T * pb[i];
		pos0[n_slice_+i] = T * pt[i];
		idx_pb[i] = i;
		idx_pt[i] = n_slice_+i;
	}
	pos0[2*n_slice_] = T * cb;
	pos0[2*n_slice_+1] = T * ct;
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
	surface.clearSurface();
	if ( !surface.addSurface(pos0, mesh_index) ) return false;

	// update position of nodes in {global}
	if ( !surface.update_xg() ) return false;

	bRenderSurface = true;

	return true;
}

void RevoluteJoint::render()
{
	if ( !isEnabledRendering() ) return;

	GLdouble color_prev[4];

	glGetDoublev(GL_CURRENT_COLOR, color_prev);

	Vec3 tmp_color(color);
	if ( bColorMapMode ) {
		tmp_color = glsubGetColorWithColorMap(fabs(coordinate.tau/maxTorqueForRendering), typeColorMap);
	}

	if ( bRenderSurface ) {
		surface.setColor(tmp_color);
		surface.setAlphaBlending(alpha);
		surface.update_xg();
		surface.render();
	} else {
		switch ( draw_type ) {
			case GLSUB_SOLID:
				gluQuadricDrawStyle(qobj, GLU_FILL);
				break;
			case GLSUB_WIRE:
				gluQuadricDrawStyle(qobj, GLU_LINE);
				break;
		}

		glsubColorVec(tmp_color, alpha);
		glPushMatrix();
		glMultMatrixd(T_global.GetArray());
		glsubSetZAxis(getAxis());
		glTranslated(0, 0, -0.5*height);
		gluCylinder(qobj, radius, radius, height, 10, 6);
		glTranslated(0, 0, height);
		gluDisk(qobj, 0, radius, 10, 6);
		glTranslated(0, 0, -height);
		glsubSetZAxis(Vec3(0,0,-1));
		gluDisk(qobj, 0, radius, 10, 6);
		glPopMatrix();
	}

	glColor4dv(color_prev);
}

////=============================================================
////                 UniversalJoint
////=============================================================
//UniversalJoint::UniversalJoint()
//{
//	color[0] = 0.5; color[1] = 0.5; color[2] = 1.0; alpha = 1.0; 
//	radius = 0.02;
//	height = 0.04;
//	surface.setMasterFrame(&T_global);
//}
//
//void UniversalJoint::render()
//{
//	if ( !isEnabledRendering() ) return;
//
//	GLdouble color_prev[4];
//
//	glGetDoublev(GL_CURRENT_COLOR, color_prev);
//
//	Vec3 tmp_color(color);
//	if ( bColorMapMode ) {
//		tmp_color = glsubGetColorWithColorMap(0.5*(fabs(coordinates[0].tau/maxTorqueForRendering)+fabs(coordinates[1].tau/maxTorqueForRendering)), typeColorMap);
//	}
//
//	if ( bRenderSurface ) {
//		surface.setColor(tmp_color);
//		surface.setAlphaBlending(alpha);
//		surface.update_xg();
//		surface.render();
//	} else {
//		glsubColorVec(tmp_color, alpha);
//		glPushMatrix();
//		glMultMatrixd(T_global.GetArray());
//		glsubSphere(radius, draw_type);
//		glPopMatrix();
//	}
//
//	glColor4dv(color_prev);
//}
//
////=============================================================
////                 TranslationalJoint
////=============================================================
//TranslationalJoint::TranslationalJoint()
//{
//	length = 0.02;
//	surface.setMasterFrame(&T_global);
//}
//
//void TranslationalJoint::render()
//{
//	if ( !isEnabledRendering() ) return;
//
//	GLdouble color_prev[4];
//
//	glGetDoublev(GL_CURRENT_COLOR, color_prev);
//
//	Vec3 tmp_color(color);
//	if ( bColorMapMode ) {
//		RMatrix tau(getDOF(),1);
//		get_tau(tau.GetPtr());
//		tmp_color = glsubGetColorWithColorMap(fabs(FNorm(tau)/maxTorqueForRendering), typeColorMap);
//	}
//
//	if ( bRenderSurface ) {
//		surface.update_xg();
//		surface.setColor(tmp_color);
//		surface.setAlphaBlending(alpha);
//		surface.render();
//	} else {
//		glsubColorVec(tmp_color, alpha);
//		glPushMatrix();
//		glMultMatrixd(T_global.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glMultMatrixd(T.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glPopMatrix();
//	}
//
//	glColor4dv(color_prev);
//}
//
////=============================================================
////                 SphericalJoint
////=============================================================
//SphericalJoint::SphericalJoint()
//{
//	radius = 0.02;
//	surface.setMasterFrame(&T_global);
//}
//
//void SphericalJoint::render()
//{
//	if ( !isEnabledRendering() ) return;
//
//	GLdouble color_prev[4];
//
//	glGetDoublev(GL_CURRENT_COLOR, color_prev);
//
//	Vec3 tmp_color(color);
//	if ( bColorMapMode ) {
//		RMatrix tau(getDOF(),1);
//		get_tau(tau.GetPtr());
//		tmp_color = glsubGetColorWithColorMap(fabs(FNorm(tau)/maxTorqueForRendering), typeColorMap);
//	}
//
//	if ( bRenderSurface ) {
//		surface.update_xg();
//		surface.setColor(tmp_color);
//		surface.setAlphaBlending(alpha);
//		surface.render();
//	} else {
//		glsubColorVec(tmp_color, alpha);
//		glPushMatrix();
//		glMultMatrixd(T_global.GetArray());
//		glsubSphere(radius, draw_type);
//		glPopMatrix();
//	}
//
//	glColor4dv(color_prev);
//}
//
////=============================================================
////                 FreeJointF
////=============================================================
//
//FreeJointF::FreeJointF()
//{
//	length = 0.02;
//	surface.setMasterFrame(&T_global);
//}
//
//void FreeJointF::render()
//{
//	if ( !isEnabledRendering() ) return;
//
//	GLdouble color_prev[4];
//
//	glGetDoublev(GL_CURRENT_COLOR, color_prev);
//
//	Vec3 tmp_color(color);
//	if ( bColorMapMode ) {
//		tmp_color = glsubGetColorWithColorMap(fabs(FNorm(get_tau())/maxTorqueForRendering), typeColorMap);
//	}
//
//	if ( bRenderSurface ) {
//		surface.update_xg();
//		surface.setColor(tmp_color);
//		surface.setAlphaBlending(alpha);
//		surface.render();
//	} else {
//		glsubColorVec(tmp_color, alpha);
//		glPushMatrix();
//		glMultMatrixd(T_global.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glMultMatrixd(T.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glPopMatrix();
//	}
//
//	glColor4dv(color_prev);
//}
//
////=============================================================
////                 FreeJoint
////=============================================================
//
//FreeJoint::FreeJoint()
//{
//	compose(&translational_joint, &spherical_joint);
//	jointType = GJOINT_FREE; ////// cautious!!
//	length = 0.02;
//	surface.setMasterFrame(&T_global);
//}
//
//void FreeJoint::setFixedCoordinateChartForRotation(CoordinateChartForSphericalJoint cc_)
//{
//	spherical_joint.setFixedCoordinateChart(cc_);
//}
//
//void FreeJoint::resetCoordinateChartForRotation()
//{
//	spherical_joint.resetCoordinateChart();
//}
//
//void FreeJoint::render()
//{
//	if ( !isEnabledRendering() ) return;
//
//	GLdouble color_prev[4];
//
//	glGetDoublev(GL_CURRENT_COLOR, color_prev);
//
//	Vec3 tmp_color(color);
//	if ( bColorMapMode ) {
//		RMatrix tau(getDOF(),1);
//		get_tau(tau.GetPtr());
//		tmp_color = glsubGetColorWithColorMap(fabs(FNorm(tau)/maxTorqueForRendering), typeColorMap);
//	}
//
//	if ( bRenderSurface ) {
//		surface.update_xg();
//		surface.setColor(tmp_color);
//		surface.setAlphaBlending(alpha);
//		surface.render();
//	} else {
//		glsubColorVec(tmp_color, alpha);
//		glPushMatrix();
//		glMultMatrixd(T_global.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glMultMatrixd(T.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glPopMatrix();
//	}
//
//	glColor4dv(color_prev);
//}
//
////=============================================================
////                 FixedJoint
////=============================================================
//
//FixedJoint::FixedJoint()
//{
//	color[0] = color[1] = color[2] = 0.0;
//	length = 0.02;
//	surface.setMasterFrame(&T_global);
//}
//
//void FixedJoint::render()
//{
//	if ( !isEnabledRendering() ) return;
//
//	GLdouble color_prev[4];
//
//	glGetDoublev(GL_CURRENT_COLOR, color_prev);
//
//	Vec3 tmp_color(color);
//	if ( bColorMapMode ) {
//		RMatrix tau(getDOF(),1);
//		get_tau(tau.GetPtr());
//		tmp_color = glsubGetColorWithColorMap(fabs(FNorm(tau)/maxTorqueForRendering), typeColorMap);
//	}
//
//	if ( bRenderSurface ) {
//		surface.update_xg();
//		surface.setColor(tmp_color);
//		surface.setAlphaBlending(alpha);
//		surface.render();
//	} else {
//		glsubColorVec(tmp_color, alpha);
//		glPushMatrix();
//		glMultMatrixd(T_global.GetArray());
//		glsubBox(length, length, length, draw_type);
//		glPopMatrix();
//	}
//
//	glColor4dv(color_prev);
//}