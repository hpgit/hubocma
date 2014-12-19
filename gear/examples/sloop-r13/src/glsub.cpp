#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <GL/freeglut.h>
#include <math.h>
#include "glsub.h"
#include "liegroup.h"

#define TWO_PI 6.28318530717959f
#define PI 3.14159265358979f
#define EPS 1E-6f

static GLUquadricObj *qobj = gluNewQuadric();

static const double _glsub_index_color_map_jet[64][3] = { // from colormap(jet) in Matlab
	{					0,                  0,   0.56250000000000	  },
	{					0,                  0,   0.62500000000000     },
	{					0,                  0,   0.68750000000000     },
	{					0,                  0,   0.75000000000000     },
	{					0,                  0,   0.81250000000000     },
	{					0,                  0,   0.87500000000000     },
	{					0,                  0,   0.93750000000000     },
	{					0,                  0,   1.00000000000000     },
	{					0,   0.06250000000000,   1.00000000000000     },
	{					0,   0.12500000000000,   1.00000000000000     },
	{					0,   0.18750000000000,   1.00000000000000     },
	{					0,   0.25000000000000,   1.00000000000000     },
	{					0,   0.31250000000000,   1.00000000000000     },
	{					0,   0.37500000000000,   1.00000000000000     },
	{					0,   0.43750000000000,   1.00000000000000     },
	{					0,   0.50000000000000,   1.00000000000000     },
	{					0,   0.56250000000000,   1.00000000000000     },
	{					0,   0.62500000000000,   1.00000000000000     },
	{					0,   0.68750000000000,   1.00000000000000     },
	{					0,   0.75000000000000,   1.00000000000000     },
	{					0,   0.81250000000000,   1.00000000000000     },
	{					0,   0.87500000000000,   1.00000000000000     },
	{					0,   0.93750000000000,   1.00000000000000     },
	{					0,   1.00000000000000,   1.00000000000000     },
	{	 0.06250000000000,   1.00000000000000,   0.93750000000000     },
	{	 0.12500000000000,   1.00000000000000,   0.87500000000000     },
	{	 0.18750000000000,   1.00000000000000,   0.81250000000000     },
	{	 0.25000000000000,   1.00000000000000,   0.75000000000000     },
	{	 0.31250000000000,   1.00000000000000,   0.68750000000000     },
	{	 0.37500000000000,   1.00000000000000,   0.62500000000000     },
	{	 0.43750000000000,   1.00000000000000,   0.56250000000000     },
	{	 0.50000000000000,   1.00000000000000,   0.50000000000000     },
	{	 0.56250000000000,   1.00000000000000,   0.43750000000000     },
	{	 0.62500000000000,   1.00000000000000,   0.37500000000000     },
	{	 0.68750000000000,   1.00000000000000,   0.31250000000000     },
	{	 0.75000000000000,   1.00000000000000,   0.25000000000000     },
	{	 0.81250000000000,   1.00000000000000,   0.18750000000000     },
	{	 0.87500000000000,   1.00000000000000,   0.12500000000000     },
	{	 0.93750000000000,   1.00000000000000,   0.06250000000000     },
	{	 1.00000000000000,   1.00000000000000,                  0     },
	{	 1.00000000000000,   0.93750000000000,                  0     },
	{	 1.00000000000000,   0.87500000000000,                  0     },
	{	 1.00000000000000,   0.81250000000000,                  0     },  
	{	 1.00000000000000,   0.75000000000000,                  0     },
	{	 1.00000000000000,   0.68750000000000,                  0     },
	{	 1.00000000000000,   0.62500000000000,                  0     },
	{	 1.00000000000000,   0.56250000000000,                  0     },
	{	 1.00000000000000,   0.50000000000000,                  0     },
	{	 1.00000000000000,   0.43750000000000,                  0     },
	{	 1.00000000000000,   0.37500000000000,                  0     },
	{	 1.00000000000000,   0.31250000000000,                  0     },
	{	 1.00000000000000,   0.25000000000000,                  0     },
	{	 1.00000000000000,   0.18750000000000,                  0     },
	{	 1.00000000000000,   0.12500000000000,                  0     },
	{	 1.00000000000000,   0.06250000000000,                  0     },
	{	 1.00000000000000,                  0,                  0     },
	{	 0.93750000000000,                  0,                  0     },
	{	 0.87500000000000,                  0,                  0     },
	{	 0.81250000000000,                  0,                  0     },
	{	 0.75000000000000,                  0,                  0     },
	{	 0.68750000000000,                  0,                  0     },
	{	 0.62500000000000,                  0,                  0     },
	{	 0.56250000000000,                  0,                  0     },
	{	 0.50000000000000,                  0,                  0     }};  

// find a vector which is normal to axis and whose length is radius.
Vec3 glsubFindNormal(Vec3 axis, double radius)
{
	Vec3 m, a;

	axis.Normalize();
	
	// arbitrary m which is not collinear to axis
	if ( fabs(axis[2]) > EPS ) m = Vec3(1, 1, -(axis[0]+axis[1])/axis[2]);
	else m = Vec3(0, 0, 1);
	
	// a is normal to axis
	a = m - Inner(m,axis)*axis;
	
	return (radius/Norm(a))*a;
}

// find a vector which is normal to axis and whose length is radius.
Vec3 glsubFindNormal(Vec3 axis, double radius, Vec3 ref)
{
	axis.Normalize();

	// a is normal to axis
	Vec3 a = ref - Inner(ref,axis)*axis;

	return (radius/Norm(a))*a;
}

void glsubSetZAxis(Vec3 axis)
{
	SE3 T;
	Vec3 z(0,0,1);
	axis.Normalize();

	if ( Norm(axis-z) < EPS )		// if axis = z
		T.SetIdentity();	// identity
	else if ( Norm(axis+z) < EPS )	// else if axis = -z
		T = SE3(SO3(1,0,0,0,-1,0,0,0,-1),Vec3(0,0,0));
	else
	{
		Vec3 m = Cross(z,axis);
		double theta = asin(Norm(m));
		if ( Inner(z,axis) < 0 ) theta = PI - theta;

		m.Normalize();
		T = SE3(Exp(m*theta), Vec3(0,0,0));
	}

	glMultMatrixd(T.GetArray());
}

void glsubSetZAxis(Vec3 axis_z, Vec3 axis_x_ref)
{
	SO3 R;
	Vec3 z(0,0,1);
	axis_z.Normalize();

	if ( Norm(axis_z-z) < EPS )		// if axis_z = z
		R.SetIdentity();	// identity
	else if ( Norm(axis_z+z) < EPS )	// else if axis_z = -z
		R = SO3(1,0,0,0,-1,0,0,0,-1);
	else
	{
		Vec3 m = Cross(z,axis_z);
		double theta = asin(Norm(m));
		if ( Inner(z,axis_z) < 0 ) theta = PI - theta;

		m.Normalize();
		R = Exp(m*theta);
	}

	axis_x_ref = R * axis_x_ref;			// reference vector viewed in {T}
	axis_x_ref = axis_x_ref - Inner(axis_x_ref, z) * z; // components of the reference vector normal to z
	double phi = asin(Norm(Cross(Vec3(1,0,0), axis_x_ref)));

	R = R * Exp(z*phi);

	glMultMatrixd(SE3(R).GetArray());
}

SO3 glsubGetRotationToSetZAxis(Vec3 axis_z, Vec3 axis_x_ref)
{
	SO3 R;
	Vec3 z(0,0,1), x(1,0,0);
	axis_z.Normalize();

	if ( Norm(axis_z-z) < EPS )		// if axis_z = z
		R.SetIdentity();	// identity
	else if ( Norm(axis_z+z) < EPS )	// else if axis_z = -z
		R = SO3(1,0,0,0,-1,0,0,0,-1);
	else
	{
		Vec3 m = Cross(z,axis_z);
		double theta = asin(Norm(m));
		if ( Inner(z,axis_z) < 0 ) theta = PI - theta;

		m.Normalize();
		R = Exp(m*theta);
	}

	Vec3 axis_x_ref_2 = ~R * axis_x_ref;						// reference vector viewed in the rotated frame
	axis_x_ref_2 = axis_x_ref_2 - Inner(axis_x_ref_2, z) * z;	// components of the reference vector normal to z
	axis_x_ref_2.Normalize();

	if ( Norm(axis_x_ref_2-x) < EPS ) {
		;
	} else if ( Norm(axis_x_ref_2+x) < EPS ) {
		R = R * Exp(z*PI);
	} else {
		Vec3 zz = Cross(x, axis_x_ref_2);
		double phi = asin(Norm(zz));
		if ( Inner(x, axis_x_ref_2) < 0 ) phi = PI - phi;
		zz.Normalize();
		R = R * Exp(zz*phi);
	}

	return R;
}

void glsubColorVec(Vec3 color_, double alpha_) 
{ 
	glColor4d(color_[0], color_[1], color_[2], alpha_); 
}

Vec3 glsubGetColorWithColorMap(double t, glsubColorMapType cmtype)
{
	double r, g, b;
	double d, ratio;
	int idx;

	if ( t < 0 ) t = 0;
	if ( t > 1 ) t = 1;

	switch ( cmtype ) {

		case GLSUB_COLOR_MAP_JET:
			d = t/63;
			idx = int(floor(d));
			ratio = d-double(idx);
			r = (1-ratio) * _glsub_index_color_map_jet[idx][0] + ratio * _glsub_index_color_map_jet[idx+1][0];
			g = (1-ratio) * _glsub_index_color_map_jet[idx][1] + ratio * _glsub_index_color_map_jet[idx+1][1];
			b = (1-ratio) * _glsub_index_color_map_jet[idx][2] + ratio * _glsub_index_color_map_jet[idx+1][2];
			break;

		case GLSUB_COLOR_MAP_AUTUMN:
			r = 1;
			g = t;
			b = 0;
			break;

		case GLSUB_COLOR_MAP_AUTUMN_INVERSE:
			r = 1;
			g = 1-t;
			b = 0;
			break;

		default:
			r = g = b = 0;
	}

	return Vec3(r,g,b);
}

void glsubCircle(Vec3 n, double radius, int slice)
{
	double theta;
	Vec3 x, x0;

	n.Normalize();
	x0 = glsubFindNormal(n, radius);

	glBegin(GL_LINE_LOOP);
	for ( int i = 0; i < slice; i++ )
	{
		theta = (TWO_PI * (double)i) / (double)slice;
		x = Exp(n * theta) * x0;
		glVertex3d(x[0], x[1], x[2]);
	}
	glEnd();
}

void glsubScrew(se3 S, double theta, int slice)
{
	Vec3 w = GetW(S);
	Vec3 v = GetV(S);
	
	if ( Norm(w) > EPS )
	{
		Vec3 q = 1/(Norm(w)*Norm(w)) * Cross(w,v);
		double h = Inner(w,v) / ( Norm(w)*Norm(w) );	// pitch

		glsubScrew(w, q, Vec3(0,0,0), h, theta, slice);
	}
	else	// pure translation
	{
		if ( Norm(v) > EPS )
		{
			Vec3 p = theta * v;
			glBegin(GL_LINES);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(p[0], p[1], p[2]);
			glEnd();
		}
	}

}

void glsubScrew(Vec3 w, Vec3 q, Vec3 p0, double h, double theta, int slice) 
{
	int num = int( (Norm(w)*fabs(theta)) / (TWO_PI/slice) );	// number of total slice
	double sgn_theta = ( theta == 0 ? 0.0 : ( theta > 0.0 ? 1.0 : -1.0 ) );
	double ww = Norm(w);
	
	double x = 0.0;
	Vec3 p;

	glBegin(GL_LINE_STRIP);
		for (int i=0; i<=num; i++)
		{
			p = q + Exp(w*x)*(p0-q) + h*x*w;
			glVertex3d(p[0], p[1], p[2]);
			x += (TWO_PI/slice/ww)*sgn_theta;
		}
		p = q + Exp(w*theta)*(p0-q) + h*theta*w;
		glVertex3d(p[0], p[1], p[2]);
	glEnd();
}

// r = a * q
void glsubSpiral(Vec3 n, double theta, double radius, int slice)
{
	Vec3 m, x, p;

	n.Normalize();

	x = glsubFindNormal(n, 1.0);

	int num = int( theta / (TWO_PI/slice) );	// total number of slice
	double q = 0.0;
	double r = 0.0;
	double a = radius/theta;
	
	glBegin(GL_LINE_STRIP);
	for ( int i = 0; i <= num; i++ )
	{
		p = Exp(n * q) * (r * x);
		glVertex3d(p[0], p[1], p[2]);
		q += TWO_PI/slice;
		r = a * q;
	}
	p = Exp(n * theta) * (radius * x);
	glVertex3d(p[0], p[1], p[2]);
	glEnd();
}

void glsubSpiral3D(Vec3 w, Vec3 ref, double radius_sprial_0, double radius_sprial_2pi, double radius_cylinder, int slice_sprial, int slice_cylinder, int stack_cylinder, int ring_cylinder, glsubDrawType draw_type)
{
	// radius_sprial(t) = radius_sprial_0 + b * t
	// where b = (radius_sprial_2pi - radius_sprial_0) / (2 * pi)

	double theta, b, h, t, height;

	theta = Norm(w);
	b = (radius_sprial_2pi - radius_sprial_0) / (2 * PI);
	h = 2 * PI / double(slice_sprial);

	if ( theta < 1E-6 ) return;

	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}

	Vec3 n(w), p1, p2, pf;
	n.Normalize();
	p1 = glsubFindNormal(n, radius_sprial_0, ref);

	for (t = h; t < theta; t += h) {
		p2 = Exp(h * n) * p1;
		p2.Normalize();
		p2 *= radius_sprial_0 + b * t;
		height = Norm(p2-p1);

		glPushMatrix();
		glTranslated(p1[0], p1[1], p1[2]);
		glsubSetZAxis(p2-p1);
		glsubDisk(Vec3(0,0,-1), radius_cylinder, draw_type, slice_cylinder, ring_cylinder);
		gluCylinder(qobj, radius_cylinder, radius_cylinder, height, slice_cylinder, stack_cylinder);
		glTranslated(0, 0, height);
		glsubDisk(Vec3(0,0,1), radius_cylinder, draw_type, slice_cylinder, ring_cylinder);
		glPopMatrix();

		p1 = p2;
	}

	p2 = Exp(h * n) * p1;
	p2.Normalize();
	p2 *= radius_sprial_0 + b * t;
	pf = p1 + ((h-t+theta)/h) * (p2 - p1);
	height = Norm(pf-p1);

	glPushMatrix();
	glTranslated(p1[0], p1[1], p1[2]);
	glsubSetZAxis(p2-p1);
	glsubDisk(Vec3(0,0,-1), radius_cylinder, draw_type, slice_cylinder, ring_cylinder);
	gluCylinder(qobj, radius_cylinder, radius_cylinder, height, slice_cylinder, stack_cylinder);
	glTranslated(0, 0, height);
	glsubDisk(Vec3(0,0,1), radius_cylinder, draw_type, slice_cylinder, ring_cylinder);
	glPopMatrix();
	
}

void glsubDisk(Vec3 n, double radius, glsubDrawType draw_type, int slice, int ring)
{
	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}
	
	n.Normalize();
	
	glPushMatrix();
		glsubSetZAxis(n);
		gluDisk(qobj, 0.0, radius, slice, ring);
	glPopMatrix();	
}

void glsubDisk(Vec3 n, double radius_inner, double radius_outer, glsubDrawType draw_type, int slice, int ring)
{
	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}

	n.Normalize();

	glPushMatrix();
	glsubSetZAxis(n);
	//gluDisk(qobj, radius_inner, radius_outer, slice, ring);
	glPopMatrix();	
}

void glsubCylinder(Vec3 n, double radius_base, double radius_top, double height, glsubDrawType draw_type, int slice, int stack, int ring)
{
	if ( Norm(n) < EPS || height < EPS ) return;

	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}
	
	Vec3 z(0,0,1);
	n.Normalize();

	glPushMatrix();
	glsubSetZAxis(-n);
	gluDisk(qobj, 0.0, radius_base, slice, ring);
	glsubSetZAxis(Vec3(0,0,-1));
	gluCylinder(qobj, radius_base, radius_top, height, slice, stack);
	glTranslated(0, 0, height);
	gluDisk(qobj, 0.0, radius_top, slice, ring);
	glPopMatrix();	
}

void glsubTorus(Vec3 n, double radius_outer, double radius_inner, glsubDrawType draw_type, int slice)
{
	n.Normalize();
	
	glPushMatrix();
		glsubSetZAxis(n);
		if ( draw_type == 0 ) {
			glutSolidTorus(radius_inner, radius_outer, slice, slice);
		} else {
			glutWireTorus(radius_inner, radius_outer, slice, slice);
		}
	glPopMatrix();	
}

void glsubSphere(double radius, glsubDrawType draw_type, int slices, int stacks)
{
	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}

	gluSphere(qobj, radius, slices, stacks);
}

void DrawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, float fA, float fB, float fC)
{
	float tStep = (PI) / (float)uiSlices;
	float sStep = (PI) / (float)uiStacks;
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for(float t = -PI/2; t <= (PI/2)+.0001; t += tStep)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for(float s = -PI; s <= PI+.0001; s += sStep)
		{
			glVertex3d(fA * cos(t) * cos(s), fB * cos(t) * sin(s), fC * sin(t));
			glVertex3d(fA * cos(t+tStep) * cos(s), fB * cos(t+tStep) * sin(s), fC * sin(t+tStep));
		}
		glEnd();
	}
}

void glsubEllipsoid(double radius_x, double radius_y, double radius_z, glsubDrawType draw_type, int slices, int stacks)
{
	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}

	glPushMatrix();
	glScaled(radius_x, radius_y, radius_z);
	gluSphere(qobj, 1.0, slices, stacks);
	glPopMatrix();

	//DrawEllipsoid(stacks, slices, radius_x, radius_y, radius_z);
}

void glsubBox(double dx, double dy, double dz, glsubDrawType draw_type, int stacks_x, int stacks_y, int stacks_z)
{
	if ( draw_type == 0 )
	{
		glBegin(GL_QUADS);
			glNormal3d(1, 0, 0);
			glVertex3d(dx/2, -dy/2, dz/2);
			glVertex3d(dx/2, -dy/2, -dz/2);
			glVertex3d(dx/2, dy/2, -dz/2);
			glVertex3d(dx/2, dy/2, dz/2);
		glEnd();
		glBegin(GL_QUADS);
			glNormal3d(0, 1, 0);
			glVertex3d(dx/2, dy/2, dz/2);
			glVertex3d(dx/2, dy/2, -dz/2);
			glVertex3d(-dx/2, dy/2, -dz/2);
			glVertex3d(-dx/2, dy/2, dz/2);
		glEnd();
		glBegin(GL_QUADS);
			glNormal3d(-1, 0, 0);
			glVertex3d(-dx/2, dy/2, -dz/2);
			glVertex3d(-dx/2, -dy/2, -dz/2);
			glVertex3d(-dx/2, -dy/2, dz/2);
			glVertex3d(-dx/2, dy/2, dz/2);
		glEnd();
		glBegin(GL_QUADS);
			glNormal3d(0, -1, 0);
			glVertex3d(-dx/2, -dy/2, dz/2);
			glVertex3d(-dx/2, -dy/2, -dz/2);
			glVertex3d(dx/2, -dy/2, -dz/2);
			glVertex3d(dx/2, -dy/2, dz/2);
		glEnd();
		glBegin(GL_QUADS);
			glNormal3d(0, 0, 1);
			glVertex3d(dx/2, -dy/2, dz/2);
			glVertex3d(dx/2, dy/2, dz/2);
			glVertex3d(-dx/2, dy/2, dz/2);
			glVertex3d(-dx/2, -dy/2, dz/2);
		glEnd();
		glBegin(GL_QUADS);
			glNormal3d(0, 0, -1);
			glVertex3d(dx/2, -dy/2, -dz/2);
			glVertex3d(-dx/2, -dy/2, -dz/2);
			glVertex3d(-dx/2, dy/2, -dz/2);
			glVertex3d(dx/2, dy/2, -dz/2);
		glEnd();
	}
	else if ( draw_type == 1 )
	{
		glBegin(GL_LINE_LOOP);
			glVertex3d(dx/2, -dy/2, dz/2);
			glVertex3d(dx/2, dy/2, dz/2);
			glVertex3d(-dx/2, dy/2, dz/2);
			glVertex3d(-dx/2, -dy/2, dz/2);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex3d(dx/2, -dy/2, -dz/2);
			glVertex3d(-dx/2, -dy/2, -dz/2);
			glVertex3d(-dx/2, dy/2, -dz/2);
			glVertex3d(dx/2, dy/2, -dz/2);
		glEnd();
		glBegin(GL_LINES);
			glVertex3d(dx/2, -dy/2, dz/2);
			glVertex3d(dx/2, -dy/2, -dz/2);
			glVertex3d(dx/2, dy/2, dz/2);
			glVertex3d(dx/2, dy/2, -dz/2);
			glVertex3d(-dx/2, dy/2, dz/2);
			glVertex3d(-dx/2, dy/2, -dz/2);
			glVertex3d(-dx/2, -dy/2, dz/2);
			glVertex3d(-dx/2, -dy/2, -dz/2);
		glEnd();
		for (int i=1; i<stacks_x; i++) {
			double xx = -dx/2+double(i)*dx/double(stacks_x);
			glBegin(GL_LINE_LOOP);
				glVertex3d(xx, dy/2, dz/2);
				glVertex3d(xx, -dy/2, dz/2);
				glVertex3d(xx, -dy/2, -dz/2);
				glVertex3d(xx, dy/2, -dz/2);
			glEnd();
		}
		for (int i=1; i<stacks_y; i++) {
			double yy = -dy/2+double(i)*dy/double(stacks_y);
			glBegin(GL_LINE_LOOP);
				glVertex3d(dx/2, yy, dz/2);
				glVertex3d(-dx/2, yy, dz/2);
				glVertex3d(-dx/2, yy, -dz/2);
				glVertex3d(dx/2, yy, -dz/2);
			glEnd();
		}
		for (int i=1; i<stacks_z; i++) {
			double zz = -dz/2+double(i)*dz/double(stacks_z);
			glBegin(GL_LINE_LOOP);
				glVertex3d(dx/2, dy/2, zz);
				glVertex3d(-dx/2, -dy/2, zz);
				glVertex3d(-dx/2, -dy/2, zz);
				glVertex3d(dx/2, dy/2, zz);
			glEnd();
		}
	}
	else
		;
}

void glsubFrameCube(double length, double radius, glsubDrawType draw_type)
{
	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}
	
	double L = length;
	double R = radius;
	int slice = 10;
	int stack = 8;

	glPushMatrix();

	glTranslated(0.5*L,0.5*L,-0.5*L);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(-L,0,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(0,-L,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(L,0,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(-0.5*L,0.5*L,0.5*L);
	
	glRotated(90,1,0,0);
	
	glTranslated(0.5*L,0.5*L,-0.5*L);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(-L,0,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(0,-L,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(L,0,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(-0.5*L,0.5*L,0.5*L);
	
	glRotated(90,0,1,0);
	
	glTranslated(0.5*L,0.5*L,-0.5*L);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(-L,0,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(0,-L,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(L,0,0);
	gluCylinder(qobj, R, R, L, slice, stack);
	glTranslated(-0.5*L,0.5*L,0.5*L);
	
	glPopMatrix();
}

void glsubSolidCube(double length, double alpha, Vec3 color_x, Vec3 color_y, Vec3 color_z)
{
	GLdouble previous_color[4];

	glGetDoublev(GL_CURRENT_COLOR, previous_color);

	double L = length;

	glColor4d(color_x[0], color_x[1], color_x[2], alpha);
	glBegin(GL_QUADS);
	glNormal3d(1, 0, 0);
	glVertex3d(L/2, -L/2, L/2);
	glVertex3d(L/2, -L/2, -L/2);
	glVertex3d(L/2, L/2, -L/2);
	glVertex3d(L/2, L/2, L/2);
	glEnd();

	glColor4d(color_y[0], color_y[1], color_y[2], alpha);
	glBegin(GL_QUADS);
	glNormal3d(0, 1, 0);
	glVertex3d(L/2, L/2, L/2);
	glVertex3d(L/2, L/2, -L/2);
	glVertex3d(-L/2, L/2, -L/2);
	glVertex3d(-L/2, L/2, L/2);
	glEnd();

	glColor4d(color_x[0], color_x[1], color_x[2], alpha);
	glBegin(GL_QUADS);
	glNormal3d(-1, 0, 0);
	glVertex3d(-L/2, L/2, -L/2);
	glVertex3d(-L/2, -L/2, -L/2);
	glVertex3d(-L/2, -L/2, L/2);
	glVertex3d(-L/2, L/2, L/2);
	glEnd();

	glColor4d(color_y[0], color_y[1], color_y[2], alpha);
	glBegin(GL_QUADS);
	glNormal3d(0, -1, 0);
	glVertex3d(-L/2, -L/2, L/2);
	glVertex3d(-L/2, -L/2, -L/2);
	glVertex3d(L/2, -L/2, -L/2);
	glVertex3d(L/2, -L/2, L/2);
	glEnd();

	glColor4d(color_z[0], color_z[1], color_z[2], alpha);
	glBegin(GL_QUADS);
	glNormal3d(0, 0, 1);
	glVertex3d(L/2, -L/2, L/2);
	glVertex3d(L/2, L/2, L/2);
	glVertex3d(-L/2, L/2, L/2);
	glVertex3d(-L/2, -L/2, L/2);
	glEnd();

	glColor4d(color_z[0], color_z[1], color_z[2], alpha);
	glBegin(GL_QUADS);
	glNormal3d(0, 0, -1);
	glVertex3d(L/2, -L/2, -L/2);
	glVertex3d(-L/2, -L/2, -L/2);
	glVertex3d(-L/2, L/2, -L/2);
	glVertex3d(L/2, L/2, -L/2);
	glEnd();

	glColor4dv(previous_color);
}

void glsubTriangle(Vec3 p0, Vec3 p1, Vec3 p2, glsubDrawType draw_type)
{
	if ( draw_type == 0 ) {
		glBegin(GL_TRIANGLES);
			Vec3 n = Cross(p1-p0, p2-p0);
			n.Normalize();
			glNormal3dv(n.GetArray());
			glVertex3dv(p0.GetArray());
			glVertex3dv(p1.GetArray());
			glVertex3dv(p2.GetArray());
		glEnd();
	} else if ( draw_type == 1 ) {
		glBegin(GL_LINE_LOOP);
			glVertex3dv(p0.GetArray());
			glVertex3dv(p1.GetArray());
			glVertex3dv(p2.GetArray());
		glEnd();
	} else {
	}
}

void glsubTriangle2(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 in, glsubDrawType draw_type)
{
	Vec3 n = Cross(p1-p0, p2-p0);
	n.Normalize();
	if ( Inner(n, in-p0) > 0 ) {
		glsubTriangle(p0, p2, p1, draw_type);
	} else {
		glsubTriangle(p0, p1, p2, draw_type);
	}
}

void glsubTetrahedron(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, glsubDrawType draw_type)
{
	glsubTriangle2(p0, p1, p2, p3, draw_type);
	glsubTriangle2(p0, p2, p3, p1, draw_type);
	glsubTriangle2(p0, p1, p3, p2, draw_type);
	glsubTriangle2(p1, p2, p3, p0, draw_type);
}

void glsubPolygon(std::vector<Vec3> p, Vec3 normal, glsubDrawType draw_type)
{
	int i, idx0, idx1, idx2;
	int n = p.size();
	if ( draw_type == 0 )
	{
		Vec3 v;
		for (i=0; i<n-1; i++)
		{
			idx0 = i; idx1 = i+1; idx2 = n-i-1;
			glBegin(GL_TRIANGLES);
				v = Cross(p[idx1]-p[idx0], p[idx2]-p[idx0]);
				if ( Inner(v, normal) < 0.0 ) v = -v;
				v.Normalize();
				glNormal3d(v[0], v[1], v[2]);
				glVertex3d(p[idx0][0],p[idx0][1],p[idx0][2]);
				glVertex3d(p[idx1][0],p[idx1][1],p[idx1][2]);
				glVertex3d(p[idx2][0],p[idx2][1],p[idx2][2]);
			glEnd();
		}
	}
	else if ( draw_type == 1 )
	{
		for (i=0; i<n-1; i++)
		{
			idx0 = i; idx1 = i+1; idx2 = n-i-1;
			glBegin(GL_LINE_LOOP);
				glVertex3d(p[idx0][0],p[idx0][1],p[idx0][2]);
				glVertex3d(p[idx1][0],p[idx1][1],p[idx1][2]);
				glVertex3d(p[idx2][0],p[idx2][1],p[idx2][2]);
			glEnd();
		}
	}
	else
		;
}

void glsubExtrusion(std::vector<Vec3> p, Vec3 e, glsubDrawType draw_type)
{
	int i, n, idx0, idx1;
	n = p.size();
	std::vector<Vec3> p2(n);
	for (i=0; i<n; i++) p2[i] = p[i]+e;
	
	glsubPolygon(p, -e, draw_type);
	glsubPolygon(p2, e, draw_type);
	
	if ( draw_type == 0 )
	{
		Vec3 v;
		for (i=0; i<n-1; i++)
		{
			idx0 = i; idx1 = i+1;
			glBegin(GL_QUADS);
				v = Cross(p[idx1]-p[idx0], e);
				v.Normalize();
				glNormal3d(v[0], v[1], v[2]);
				glVertex3d(p[idx0][0],p[idx0][1],p[idx0][2]);
				glVertex3d(p[idx1][0],p[idx1][1],p[idx1][2]);
				glVertex3d(p2[idx1][0],p2[idx1][1],p2[idx1][2]);
				glVertex3d(p2[idx0][0],p2[idx0][1],p2[idx0][2]);
			glEnd();
		}
		glBegin(GL_QUADS);
			v = Cross(p[n-1]-p[0], e);
			v.Normalize();
			glNormal3d(v[0], v[1], v[2]);
			glVertex3d(p[0][0],p[0][1],p[0][2]);
			glVertex3d(p[n-1][0],p[n-1][1],p[n-1][2]);
			glVertex3d(p2[n-1][0],p2[n-1][1],p2[n-1][2]);
			glVertex3d(p2[0][0],p2[0][1],p2[0][2]);
		glEnd();

	}
	else if ( draw_type == 1 )
	{
		for (i=0; i<n-1; i++)
		{
			idx0 = i; idx1 = i+1;
			glBegin(GL_LINE_LOOP);
				glVertex3d(p[idx0][0],p[idx0][1],p[idx0][2]);
				glVertex3d(p[idx1][0],p[idx1][1],p[idx1][2]);
				glVertex3d(p2[idx1][0],p2[idx1][1],p2[idx1][2]);
				glVertex3d(p2[idx0][0],p2[idx0][1],p2[idx0][2]);
			glEnd();
		}
		glBegin(GL_LINE_LOOP);
			glVertex3d(p[0][0],p[0][1],p[0][2]);
			glVertex3d(p[n-1][0],p[n-1][1],p[n-1][2]);
			glVertex3d(p2[n-1][0],p2[n-1][1],p2[n-1][2]);
			glVertex3d(p2[0][0],p2[0][1],p2[0][2]);
		glEnd();
	}
	else
		;
}

void glsubGrid(double dx, double dy, int nx, int ny)
{
	int i;
	double x1, x2;
	double y1, y2;

	glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(dx*(nx+1), 0.0, 0.0);
	glEnd();

	glBegin(GL_LINES);
			glVertex3d(0.0, 0.0, 0.0);
			glVertex3d(0.0, dy*(ny+1), 0.0);
	glEnd();

	for (i = 0 ; i < 2*nx+1 ; i++)
	{
		x2 = dx*i -dx*nx;
		y1 = -dy*ny;
		y2 = dy*ny;
		glBegin(GL_LINES);
			glVertex3d(x2, y1, 0.0);
			glVertex3d(x2, y2, 0.0);
		glEnd();
	}	
	
	for (i = 0 ; i < 2*ny+1 ; i++)
	{
		x1 = -dx*nx; 
		x2 = dx*nx; 
		y2 = dy*i - dy*ny;
		glBegin(GL_LINES);
			glVertex3d(x1, y2, 0.0);
			glVertex3d(x2, y2, 0.0);
		glEnd();
	}	
}

void glsubPlane(double wx, double wy, int slicex, int slicey, double *color4d_0, double *color4d_1, double *color4d_2, glsubDrawType draw_type)
{
	double x, y, xL, xU, yL, yU, dx, dy;
	bool bx, by;

	if ( wx < 0 || wy < 0 || slicex < 1 || slicey < 1 ) return;

	xU = 0.5*wx; xL = -xU; dx = wx/double(slicex);
	yU = 0.5*wy; yL = -yU; dy = wy/double(slicey);

	GLint previous_polygonmode[2];
	GLdouble previous_color[4];
	GLfloat previous_line_width;
	glGetIntegerv(GL_POLYGON_MODE, previous_polygonmode);
	glGetDoublev(GL_CURRENT_COLOR, previous_color);
	glGetFloatv(GL_LINE_WIDTH, &previous_line_width);

	// draw grids
	if ( color4d_0 != NULL ) {
		glColor4dv(color4d_0);		// color0 for grid rendering
		for (int i=0; i<slicex+1; i++) {
			x = xL + dx * double(i);
			glBegin(GL_LINES);
			glVertex3d(x, yL, 0.);
			glVertex3d(x, yU, 0.);
			glEnd();
		}
		for (int i=0; i<slicey+1; i++) {
			y = yL + dy * double(i);
			glBegin(GL_LINES);
			glVertex3d(xL, y, 0.);
			glVertex3d(xU, y, 0.);
			glEnd();
		}
	}

	switch (draw_type) {
	case GLSUB_SOLID:
		glPolygonMode(GL_FRONT,GL_FILL);
		break;
	case GLSUB_WIRE:
		glPolygonMode(GL_FRONT,GL_LINE);
		break;
	}

	// draw checked pattern
	bx = by = true;
	for (int i=0; i<slicex; i++) {
		for (int j=0; j<slicey; j++) {
			if ( bx == by && color4d_1 != NULL ) {
				glColor4dv(color4d_1);
				x = xL + dx * double(i);
				y = yL + dy * double(j);
				glBegin(GL_QUADS);
				glNormal3d(0, 0, 1);
				glVertex3d(x, y, 0);
				glVertex3d(x+dx, y, 0);
				glVertex3d(x+dx, y+dy, 0);
				glVertex3d(x, y+dy, 0);
				glEnd();
			} else if ( bx != by && color4d_2 != NULL ) {
				glColor4dv(color4d_2);
				x = xL + dx * double(i);
				y = yL + dy * double(j);
				glBegin(GL_QUADS);
				glNormal3d(0, 0, 1);
				glVertex3d(x, y, 0);
				glVertex3d(x+dx, y, 0);
				glVertex3d(x+dx, y+dy, 0);
				glVertex3d(x, y+dy, 0);
				glEnd();
			} else {
				;
			}
			by ? by=false : by=true;
		}
		bx ? bx=false : bx=true;
	}

	glPolygonMode(previous_polygonmode[0], previous_polygonmode[1]);
	glColor4dv(previous_color);
	glLineWidth(previous_line_width);
}

void glsubArrow(Vec3 n, double length1, double length2, double radius1, double radius2, glsubDrawType draw_type, int slice, int stack, int ring)
{
	if ( Norm(n) < EPS ) return;

	switch ( draw_type ) {
		case GLSUB_SOLID:
			gluQuadricDrawStyle(qobj, GLU_FILL);
			break;
		case GLSUB_WIRE:
			gluQuadricDrawStyle(qobj, GLU_LINE);
			break;
	}
	
	n.Normalize();
	glPushMatrix();
	glsubSetZAxis(n);
	gluCylinder(qobj, radius1, radius1, length1, slice, stack);
	glTranslated(0, 0, length1);
	//gluDisk(qobj, 0, radius1, slice, ring);
	gluCylinder(qobj, radius2, 0, length2, slice, stack);
	glsubSetZAxis(Vec3(0,0,-1));
	gluDisk(qobj, 0, radius2, slice, ring);
	glTranslated(0, 0, length1);
	gluDisk(qobj, 0, radius1, slice, ring);
	glPopMatrix();
}

void glsubArrowDoubleHead(Vec3 n, double length1, double length2, double radius1, double radius2, glsubDrawType draw_type, int slice, int stack, int ring)
{
	if ( Norm(n) < EPS ) return;
	glPushMatrix();
	n.Normalize();
	glsubCylinder(n, radius1, radius1, length1, draw_type, slice, stack, ring);
	glTranslated(length1*n[0], length1*n[1], length1*n[2]);
	glsubCylinder(n, radius2, 0, length2, draw_type, slice, stack, ring);
	glTranslated(length2/2*n[0], length2/2*n[1], length2/2*n[2]);
	glsubCylinder(n, radius2, 0, length2, draw_type, slice, stack, ring);
	glPopMatrix();
}

void glsubBodyLine(std::vector<Vec3> p, glsubDrawType draw_type)
{
	int i;
	int n = p.size();
	
	// mid point
	Vec3 p0(0.0);
	for (i=0; i<n; i++) p0 += p[i];
	p0 *= (1/double(n));

	p.push_back(p[0]);
	for (i=0; i<n; i++)
	{	
		if ( draw_type == 0 )
		{
			glBegin(GL_POLYGON);
				glVertex3d(p0[0],p0[1],p0[2]);
				glVertex3d(p[i][0],p[i][1],p[i][2]);
				glVertex3d(p[i+1][0],p[i+1][1],p[i+1][2]);
			glEnd();
		}
		else if ( draw_type == 1 )
		{
			glBegin(GL_LINE_LOOP);
				glVertex3d(p0[0],p0[1],p0[2]);
				glVertex3d(p[i][0],p[i][1],p[i][2]);
				glVertex3d(p[i+1][0],p[i+1][1],p[i+1][2]);
			glEnd();
		}
		else
			;
	}
}

void glsubBodyCylinder(std::vector<Vec3> p, double radius_, glsubDrawType draw_type, int slice_, int stack_, int ring_)
{
	int i, j;
	int n = p.size();
	
	// mid point
	Vec3 p0(0.0);
	for (i=0; i<n; i++) p0 += p[i];
	p0 *= (1/double(n));

	for (i=0; i<n; i++)
	{	
		for (j=i+1; j<n; j++)
		{
			glPushMatrix();
				glTranslated(p[i][0], p[i][1], p[i][2]);
				glsubCylinder(p[j]-p[i], radius_, radius_, Norm(p[j]-p[i]), draw_type, slice_, stack_, ring_);
			glPopMatrix();
		}
	}
}

void glsubGear(double inner_radius, double outer_radius, double width, int teeth, double tooth_depth, glsubDrawType draw_type)
{
// 3-D gear wheels.  This program is in the public domain.
// Brian Paul
// Conversion to GLUT by Mark J. Kilgard 

	int i;
	double r0, r1, r2;
	double angle, da;
	double u, v, len;

	r0 = inner_radius;
	r1 = outer_radius - tooth_depth / 2.0;
	r2 = outer_radius + tooth_depth / 2.0;

	da = TWO_PI / teeth / 4.0;

	glShadeModel(GL_FLAT);

	glNormal3d(0.0, 0.0, 1.0);

	if ( draw_type == 0 )
	{
		/* draw front face */
		glBegin(GL_QUAD_STRIP);
		for (i = 0; i <= teeth; i++) {
			angle = i * TWO_PI / teeth;
			glVertex3d(r0 * cos(angle), r0 * sin(angle), width * 0.5);
			glVertex3d(r1 * cos(angle), r1 * sin(angle), width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), width * 0.5);
		}
		glEnd();

		/* draw front sides of teeth */
		glBegin(GL_QUADS);
		da = TWO_PI / teeth / 4.0;
		for (i = 0; i < teeth; i++) {
			angle = i * TWO_PI / teeth;

			glVertex3d(r1 * cos(angle), r1 * sin(angle), width * 0.5);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), width * 0.5);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), width * 0.5);
		}
		glEnd();

		glNormal3d(0.0, 0.0, -1.0);

		/* draw back face */
		glBegin(GL_QUAD_STRIP);
		for (i = 0; i <= teeth; i++) {
			angle = i * TWO_PI / teeth;
			glVertex3d(r1 * cos(angle), r1 * sin(angle), -width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), -width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), -width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), -width * 0.5);
		}
		glEnd();

		/* draw back sides of teeth */
		glBegin(GL_QUADS);
		da = TWO_PI / teeth / 4.0;
		for (i = 0; i < teeth; i++) {
			angle = i * TWO_PI / teeth;

			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), -width * 0.5);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), -width * 0.5);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), -width * 0.5);
			glVertex3d(r1 * cos(angle), r1 * sin(angle), -width * 0.5);
		}
		glEnd();

		/* draw outward faces of teeth */
		glBegin(GL_QUAD_STRIP);
		for (i = 0; i < teeth; i++) {
			angle = i * TWO_PI / teeth;

			glVertex3d(r1 * cos(angle), r1 * sin(angle), width * 0.5);
			glVertex3d(r1 * cos(angle), r1 * sin(angle), -width * 0.5);
			u = r2 * cos(angle + da) - r1 * cos(angle);
			v = r2 * sin(angle + da) - r1 * sin(angle);
			len = sqrt(u * u + v * v);
			u /= len;
			v /= len;
			glNormal3d(v, -u, 0.0);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), width * 0.5);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), -width * 0.5);
			glNormal3d(cos(angle), sin(angle), 0.0);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), width * 0.5);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), -width * 0.5);
			u = r1 * cos(angle + 3 * da) - r2 * cos(angle + 2 * da);
			v = r1 * sin(angle + 3 * da) - r2 * sin(angle + 2 * da);
			glNormal3d(v, -u, 0.0);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), -width * 0.5);
			glNormal3d(cos(angle), sin(angle), 0.0);
		}

		glVertex3d(r1 * cos(0.0), r1 * sin(0.0), width * 0.5);
		glVertex3d(r1 * cos(0.0), r1 * sin(0.0), -width * 0.5);

		glEnd();

		glShadeModel(GL_SMOOTH);

		/* draw inside radius cylinder */
		glBegin(GL_QUAD_STRIP);
		for (i = 0; i <= teeth; i++) {
			angle = i * TWO_PI / teeth;
			glNormal3d(-cos(angle), -sin(angle), 0.0);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), -width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), width * 0.5);
		}
		glEnd();
	}
	else if ( draw_type == 1 )
	{
		// draw front face
		glBegin(GL_LINE_STRIP);
		for (i = 0; i <= teeth; i++) {
			angle = i * TWO_PI / teeth;
			glVertex3d(r0 * cos(angle), r0 * sin(angle), width * 0.5);
			glVertex3d(r1 * cos(angle), r1 * sin(angle), width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), width * 0.5);
		}
		glEnd();

		// draw front sides of teeth 
		glBegin(GL_LINE_STRIP);
		da = TWO_PI / teeth / 4.0;
		for (i = 0; i < teeth; i++) {
			angle = i * TWO_PI / teeth;

			glVertex3d(r1 * cos(angle), r1 * sin(angle), width * 0.5);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), width * 0.5);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), width * 0.5);
		}
		glEnd();

		glNormal3d(0.0, 0.0, -1.0);

		// draw back face 
		glBegin(GL_LINE_STRIP);
		for (i = 0; i <= teeth; i++) {
			angle = i * TWO_PI / teeth;
			glVertex3d(r1 * cos(angle), r1 * sin(angle), -width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), -width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), -width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), -width * 0.5);
		}
		glEnd();

		// draw back sides of teeth
		glBegin(GL_LINE_STRIP);
		da = TWO_PI / teeth / 4.0;
		for (i = 0; i < teeth; i++) {
			angle = i * TWO_PI / teeth;

			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), -width * 0.5);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), -width * 0.5);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), -width * 0.5);
			glVertex3d(r1 * cos(angle), r1 * sin(angle), -width * 0.5);
		}
		glEnd();

		// draw outward faces of teeth 
		glBegin(GL_LINE_STRIP);
		for (i = 0; i < teeth; i++) {
			angle = i * TWO_PI / teeth;

			glVertex3d(r1 * cos(angle), r1 * sin(angle), width * 0.5);
			glVertex3d(r1 * cos(angle), r1 * sin(angle), -width * 0.5);
			u = r2 * cos(angle + da) - r1 * cos(angle);
			v = r2 * sin(angle + da) - r1 * sin(angle);
			len = sqrt(u * u + v * v);
			u /= len;
			v /= len;
			glNormal3d(v, -u, 0.0);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), width * 0.5);
			glVertex3d(r2 * cos(angle + da), r2 * sin(angle + da), -width * 0.5);
			glNormal3d(cos(angle), sin(angle), 0.0);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), width * 0.5);
			glVertex3d(r2 * cos(angle + 2 * da), r2 * sin(angle + 2 * da), -width * 0.5);
			u = r1 * cos(angle + 3 * da) - r2 * cos(angle + 2 * da);
			v = r1 * sin(angle + 3 * da) - r2 * sin(angle + 2 * da);
			glNormal3d(v, -u, 0.0);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), width * 0.5);
			glVertex3d(r1 * cos(angle + 3 * da), r1 * sin(angle + 3 * da), -width * 0.5);
			glNormal3d(cos(angle), sin(angle), 0.0);
		}

		glVertex3d(r1 * cos(0.0), r1 * sin(0.0), width * 0.5);
		glVertex3d(r1 * cos(0.0), r1 * sin(0.0), -width * 0.5);

		glEnd();

		glShadeModel(GL_SMOOTH);

		// draw inside radius cylinder 
		glBegin(GL_LINE_STRIP);
		for (i = 0; i <= teeth; i++) {
			angle = i * TWO_PI / teeth;
			glNormal3d(-cos(angle), -sin(angle), 0.0);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), -width * 0.5);
			glVertex3d(r0 * cos(angle), r0 * sin(angle), width * 0.5);
		}
		glEnd();
	}
	else
		;
}

void glsubCoordinates(double length, glsubDrawType draw_type, double alpha)
{
	double previous_color[4];
	glGetDoublev(GL_CURRENT_COLOR, previous_color); 

	glColor4d(1,0,0,alpha);
	glsubArrow(Vec3(1,0,0), length*0.7, length*0.3, length/20*1, length/20*2, draw_type, 10, 1, 2);
	glColor4d(0,1,0,alpha);
	glsubArrow(Vec3(0,1,0), length*0.7, length*0.3, length/20*1, length/20*2, draw_type, 10, 1, 2);
	glColor4d(0,0,1,alpha);
	glsubArrow(Vec3(0,0,1), length*0.7, length*0.3, length/20*1, length/20*2, draw_type, 10, 1, 2);

	glColor4dv(previous_color);
}

void glsubArrow(Vec3 f, double radius, glsubDrawType draw_type)
{
	double length = Norm(f);
	glsubArrow(f, length*0.70, length*0.30, radius, 2.0*radius, draw_type, 10, 1, 2);
}

void glsubArrowDoubleHead(Vec3 m, double radius, glsubDrawType draw_type)
{
	double length = Norm(m); 
	glsubArrowDoubleHead(m, length*0.85, length*0.1, radius, radius*2, draw_type, 10, 1, 2);
}
/*
void glsubSTL(_polytope *stl, glsubDrawType draw_type)
{
	int i, j;

	glVertexPointer(3, GL_FLOAT, 0, stl->v);
	glNormalPointer(GL_FLOAT, 0, stl->normal);
	
	glPushMatrix();
	glMultMatrixd( stl->m_T.GetArray() );
	glScalef( (float)stl->m_fScale, (float)stl->m_fScale, (float)stl->m_fScale); 

	if ( draw_type == 1 )
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);	
	else if ( draw_type == 0 )
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	else
		return;

	for ( i = 0; i < stl->num_polygon; i++ )
	{
		glBegin(GL_POLYGON);
		for ( j = 0; j < stl->f[i].num_vertex; j++ ) 
		{
			glArrayElement(stl->f[i].idx[j]);
			glNormal3d(stl->normal[stl->f[i].idx[j]].x, stl->normal[stl->f[i].idx[j]].y, stl->normal[stl->f[i].idx[j]].z);
			glVertex3d(stl->v[stl->f[i].idx[j]].x, stl->v[stl->f[i].idx[j]].y, stl->v[stl->f[i].idx[j]].z);
		}
		glEnd();
	}

	glPopMatrix();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
}
*/

