#ifndef _GLSUB_
#define _GLSUB_

#include <vector>
#include "liegroup.h"

#define GLSUB_N_SLICE	10
#define GLSUB_N_STACK	8
#define GLSUB_N_RING	4

enum glsubColorMapType { GLSUB_COLOR_MAP_JET, GLSUB_COLOR_MAP_AUTUMN, GLSUB_COLOR_MAP_AUTUMN_INVERSE };
enum glsubDrawType { GLSUB_SOLID, GLSUB_WIRE };

// find a point which lies on the normal plane to axis and the distance from origin to the point is radius.
Vec3 glsubFindNormal(Vec3 axis, double radius);	
Vec3 glsubFindNormal(Vec3 axis, double radius, Vec3 ref);	// ref should not be parallel to axis

// rotate current frame to fit z-axis with axis
void glsubSetZAxis(Vec3 axis);
void glsubSetZAxis(Vec3 axis_z, Vec3 axis_x_ref);
SO3 glsubGetRotationToSetZAxis(Vec3 axis_z, Vec3 axis_x_ref);

// set color
void glsubColorVec(Vec3 color_, double alpha_ = 1.0);

// get color from the color map
Vec3 glsubGetColorWithColorMap(double t, glsubColorMapType cmtype);	// 0 <= t <= 1

// line/curve
void glsubCircle(Vec3 n, double rad, int slice = GLSUB_N_SLICE);
void glsubScrew(se3 S, double theta, int slice = GLSUB_N_SLICE);
void glsubScrew(Vec3 w, Vec3 q, Vec3 p0, double h, double theta, int slice = GLSUB_N_SLICE); // p(x) = q + Exp(w*x)*(p0-q) + h*x*w, 0<=x<=theta, h = pitch (translation per rotation(rad))
void glsubSpiral(Vec3 n, double theta, double radius, int slice = GLSUB_N_SLICE);
void glsubSpiral3D(Vec3 w, Vec3 ref, double radius_sprial_0, double radius_sprial_2pi, double radius_cylinder, int slice_sprial = GLSUB_N_SLICE, int slice_cylinder = GLSUB_N_SLICE, int stack_cylinder = 1, int ring_cylinder = 1, glsubDrawType draw_type = GLSUB_SOLID);

// plane
void glsubGrid(double dx, double dy, int nx, int ny);
void glsubPlane(double wx, double wy, int slicex = GLSUB_N_SLICE, int slicey = GLSUB_N_SLICE, double *color4d_0 = NULL, double *color4d_1 = NULL, double *color4d_2 = NULL, glsubDrawType draw_type = GLSUB_SOLID);

// surface of a 3D object
void glsubDisk(Vec3 n, double radius, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int ring = GLSUB_N_RING);
void glsubDisk(Vec3 n, double radius_inner, double radius_outer, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int ring = GLSUB_N_RING);
void glsubCylinder(Vec3 n, double radius_base, double radius_top, double height, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int stack = GLSUB_N_STACK, int ring = GLSUB_N_RING);
void glsubTorus(Vec3 n, double radius_outer, double radius_inner, glsubDrawType draw_type, int slice = GLSUB_N_SLICE); 
void glsubSphere(double radius, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int stack = GLSUB_N_STACK);
void glsubEllipsoid(double radius_x, double radius_y, double radius_z, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int stack = GLSUB_N_STACK);
void glsubBox(double dx, double dy, double dz, glsubDrawType draw_type, int stacks_x = GLSUB_N_STACK, int stacks_y = GLSUB_N_STACK, int stacks_z = GLSUB_N_STACK);
void glsubTriangle(Vec3 p0, Vec3 p1, Vec3 p2, glsubDrawType draw_type);
void glsubTriangle2(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 in, glsubDrawType draw_type);
void glsubTetrahedron(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, glsubDrawType draw_type);
void glsubPolygon(std::vector<Vec3> p, Vec3 normal, glsubDrawType draw_type);
void glsubExtrusion(std::vector<Vec3> p, Vec3 e, glsubDrawType draw_type);

// arrow
void glsubArrow(Vec3 n, double length1, double length2, double radius1, double radius2, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int stack = GLSUB_N_STACK, int ring = GLSUB_N_RING);
void glsubArrowDoubleHead(Vec3 n, double length1, double length2, double radius1, double radius2, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int stack = GLSUB_N_STACK, int ring = GLSUB_N_RING);
void glsubArrow(Vec3 f, double radius, glsubDrawType draw_type);
void glsubArrowDoubleHead(Vec3 m, double radius, glsubDrawType draw_type);

// xyz coordinates
void glsubCoordinates(double length, glsubDrawType draw_type, double alpha = 1.0);

// etc
void glsubBodyLine(std::vector<Vec3> p, glsubDrawType draw_type);
void glsubBodyCylinder(std::vector<Vec3> p, double radius_, glsubDrawType draw_type, int slice = GLSUB_N_SLICE, int stack = GLSUB_N_STACK, int ring = GLSUB_N_RING);
void glsubGear(double inner_radius, double outer_radius, double width, int teeth, double tooth_depth, glsubDrawType draw_type);

void glsubFrameCube(double length, double radius, glsubDrawType draw_type);
void glsubSolidCube(double length, double alpha = 1.0, Vec3 color_x = Vec3(1,0,0), Vec3 color_y = Vec3(0,1,0), Vec3 color_z = Vec3(0,0,1));

//void glsubSTL(_polytope *stl, glsubDrawType draw_type);

#endif