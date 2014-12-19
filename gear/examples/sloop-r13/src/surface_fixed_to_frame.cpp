#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <vector>
#include <string>
#include <fstream>
#include "surface_fixed_to_frame.h"
#include "glsub.h"

using namespace std;

//=============================================================
//                 SurfaceFixedToFrame
//=============================================================
SurfaceFixedToFrame::SurfaceFixedToFrame()
{
	pFrame = NULL;
	pFrameLinear = NULL;

	b_render_mesh = true;
	b_render_node = false;
	//setColor(Vec3(0,0,1));
    alpha = 1.0;
	draw_type = GLSUB_WIRE;
	line_width = 1.0;

	str_name = "";
	str_material = "";
}

void SurfaceFixedToFrame::clearSurface()
{
	r0.clear();
	xg.clear();

	str_name = "";
	str_material = "";
}

bool SurfaceFixedToFrame::addSurface(std::vector<Vec3> r0_, std::vector< _triad<int> > mesh_index_)
{
	int idx0 = getNumNode();	// the number of current nodes
	int idx1 = getNumMesh();	// the number of current mesh elements

	std::vector<Vec3> r0_old(r0);
	std::vector< _triad<int> > mesh_index_old(mesh_index);

	// resize arrays for node and mesh index
	r0.resize(idx0+r0_.size());
	mesh_index.resize(idx1+mesh_index_.size());

	// copy the previous nodes and mesh index
	for (int i=0; i<idx0; i++) {
		r0[i] = r0_old[i];
	}
	for (int i=0; i<idx1; i++) {
		mesh_index[i] = mesh_index_old[i];
	}

	// push back the new nodes and mesh index
	for (int i=0; i<(int)r0_.size(); i++) {
		r0[idx0+i] = r0_[i];
	}
	for (int i=0; i<(int)mesh_index_.size(); i++) {
		mesh_index[idx1+i] = _triad<int>(mesh_index_[i][0]+idx0, mesh_index_[i][1]+idx0, mesh_index_[i][2]+idx0);
	}

	// check surface
	if ( !_check_mesh_index() ) return false;

	// memory allocation
	int n = r0.size();

	xg.resize(n);

	for (int i=0; i<n; i++) {
		xg[i].SetZero();
	}

	return true;
}

bool SurfaceFixedToFrame::loadExtraInfo(char *file_name_)
{
	string str;
	ifstream fin;

	strExtraInfo.clear();

	fin.open(file_name_);
	
	if ( !fin.is_open() ) return false;

	getline(fin,str);
	while ( fin ) {
		strExtraInfo.push_back(str);
		getline(fin,str);
	}

	return true;
}

int SurfaceFixedToFrame::getNumNode()
{
	return r0.size();
}

int SurfaceFixedToFrame::getNumMesh()
{
	return mesh_index.size();
}

bool SurfaceFixedToFrame::update_xg()
{
	if ( pFrame == NULL && pFrameLinear == NULL ) return false;

	if ( pFrame != NULL ) {
		for (size_t i=0; i<r0.size(); i++) {
			xg[i] = (*pFrame) * r0[i];
		}
	}

	if ( pFrameLinear != NULL ) {
		for (size_t i=0; i<r0.size(); i++) {
			xg[i] = (*pFrameLinear) + r0[i];
		}
	}

	return true;
}

bool SurfaceFixedToFrame::make_ready_for_analysis()
{
	if ( pFrame == NULL && pFrameLinear == NULL ) return false;
	if ( !_check_mesh_index() ) return false;

	return true;
}

bool SurfaceFixedToFrame::_check_mesh_index()
{
	// check mesh
	for (size_t i=0; i<mesh_index.size(); i++) {
		// index check
		for (int j=0; j<3; j++) {
			if ( mesh_index[i][j] < 0 || mesh_index[i][j] >= (int)r0.size() ) return false;
		}
		// node duplicity check
		if ( mesh_index[i][0] == mesh_index[i][1] || mesh_index[i][0] == mesh_index[i][2] || mesh_index[i][1] == mesh_index[i][2]  ) {
			return false;
		}
	}

	return true;
}

void SurfaceFixedToFrame::render()
{
	GLint previous_polygonmode[2];
	GLdouble previous_color[4];
	GLfloat previous_line_width;
	glGetIntegerv(GL_POLYGON_MODE, previous_polygonmode);
	glGetDoublev(GL_CURRENT_COLOR, previous_color);
	glGetFloatv(GL_LINE_WIDTH, &previous_line_width);

	glColor4d(color[0], color[1], color[2], alpha);
	glLineWidth(line_width);

	switch (draw_type) {
	case GLSUB_SOLID:
		glPolygonMode(GL_FRONT,GL_FILL);
		break;
	case GLSUB_WIRE:
		glPolygonMode(GL_FRONT,GL_LINE);
		break;
	}

	if ( b_render_mesh ) {
		glPushMatrix();
		for (int i=0; i<getNumMesh(); i++) {
			glBegin(GL_TRIANGLES);
			Vec3 x1 = xg[mesh_index[i][0]];
			Vec3 x2 = xg[mesh_index[i][1]];
			Vec3 x3 = xg[mesh_index[i][2]];
			Vec3 n = Cross(x2-x1, x3-x1);
			n.Normalize();
			glNormal3d(n[0], n[1], n[2]);
			glVertex3dv(x1.GetArray());
			glVertex3dv(x2.GetArray());
			glVertex3dv(x3.GetArray());
			glEnd();
		}
		glPopMatrix();
	}

	if ( b_render_node ) {
		for (int i=0; i<getNumNode(); i++) {
			glPushMatrix();
			glTranslated(xg[i][0], xg[i][1], xg[i][2]);
			glsubSphere(0.005, draw_type);
			glPopMatrix();
		}
	}

	glPolygonMode(previous_polygonmode[0], previous_polygonmode[1]);
	glColor4dv(previous_color);
	glLineWidth(previous_line_width);
}


