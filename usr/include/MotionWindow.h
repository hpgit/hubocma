#pragma once

#ifndef WIN32
	#define WIN32
#endif

#include <iostream>
#include <FL/Fl.H>
//#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <FL/Fl_Gl_Window.H>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Camera
{
	// viewing -z direction, right-handed coordinate
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Affine3d se3;
	Eigen::Affine3d transforming_se3;
	double distance;
	int transforming;

	Camera();
	
	void camera_lookat();
	
	void setDistance(double d);
	void setPosOnSphereAfterScreenRotate(int mousedx, int mousedy, int screenw, int screenh);
	void addPosAfterScreenShift(int mousedx, int mousedy, int screenh, double fovyRad);

	void setTransforming();
	void unsetTransforming();

	void invalidate();

private:
	Eigen::Vector3d eye;
	Eigen::Vector3d view;
	Eigen::Vector3d obj;
	Eigen::Vector3d up;
	void vecInvalidate(Eigen::Affine3d &_se3);
};


class MotionGLWindow : public Fl_Gl_Window
{
public:
	MotionGLWindow(int x, int y, int w, int h, const char *l = 0);

	int press[4];
	int mousePrevX;
	int mousePrevY;

	virtual int handle(int event);
	virtual void draw();
	virtual void _draw(); // need to be implementes in child class

private:
	Camera camera;

	void draw_ground();
	void initialize_gl();
	void init_gl_color();
	void setupFloor();
	void draw_axis();
};
