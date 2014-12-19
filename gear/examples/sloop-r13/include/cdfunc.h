//================================================================================
//         FUNCTIONS FOR CONTACT DETECTION
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _FUNCTIONS_FOR_CONTACT_DETECTION_
#define _FUNCTIONS_FOR_CONTACT_DETECTION_

#include "liegroup.h"


bool dcPointTriangle(double &d, Vec3 &n, const Vec3 &x, const Vec3 &t0, const Vec3 &t1, const Vec3 &t2, double d_lower, double d_upper = 0.0);
// ======================================================================================
// bool dcPointTriangle(d, n, x, t0, t1, t2, d_lower, d_upper)
// --------------------------------------------------------------------------------------
// Detects whether a point which is located at x is in contact with a triangle plane defined by {t0, t1, t2}.
// outputs: 
//		return = true if the test point is in contact.
//		d = signed distance between the point and the triangle plane. (d > 0 if the point is located above the plane.)
//		n = normal vector to the triangle plane. The right hand rule for {t0, t1, t2} will be applied to determine n.
// inputs:
//		x = The location of the test point
//		{t0, t1, t2} = Triplet defining the triangle plane
//		[d_lower, d_upper] = The range of d for contact. ex) [-0.1, 0.0]
// ======================================================================================

bool detectCollision(double &pd, double &s, double &alpha, double &beta, Vec3 &x_collision, Vec3 &n, const Vec3 &x_present, const Vec3 &x_previous, const Vec3 &t0, const Vec3 &t1, const Vec3 &t2);
// ======================================================================================
// bool detectCollision(pd, s, x_collision, n, x_present, x_previous, t0, t1, t2)
// --------------------------------------------------------------------------------------
// Detects whether the line trajectory(from x_previous to x_present) collides to the inside of the triangle plane defined by {t0, t1, t2}.
// outputs: 
//		return = true if collision happens
//		pd = penetration depth
//		s = distance ratio: (x_collision, x_previous) / (x_present, x_previous)
//		(alpha, beta) = local 2-dimensional coordinates of x_collision, alpha >= 0, beta >= 0, alpha+beta <= 1
//		x_collision = the intersectional point of the line trajectory and the plane
//		n = normal vector to the triangle plane. The right hand rule for {t0, t1, t2} will be applied to determine n.
// inputs:
//		x_present = The present position of the test point
//		x_previous = The previous position of the test point
//		{t0, t1, t2} = Triplet defining the triangle plane
// ======================================================================================

bool dcBox2d(double *x, double *y, double *X, double *Y);
// ======================================================================================
// bool dcBox2d(double *x, double *y, double *X, double *Y)
// --------------------------------------------------------------------------------------
// Detects whether the two plane boxes(rectangles or parallelograms) are colliding with each other. 
// output:
//		return = true if collision happens.
// inputs:
//		(x[i], y[i]), (X[i], Y[i]) are the corner points of the two boxes in 2D. (i=0,1,2,3)
//		The index ordering of the points should be clock or counter-clock wise.
//			
//			  (x[0],y[0])  ---------------- (x[3],y[3])         (x[0],y[0])	----------------- (x[1],y[1])
//						  /				 /               or     		   / 	           /
//						 /			    /	                     		  /		          /
//			(x[1],y[1])	---------------- (x[2],y[2])         (x[3],y[3]) ----------------- (x[2],y[2])

bool dcOBBOverlap(double *Ea, double *Pa, double *Ra, double *Eb, double *Pb, double *Rb);
// ======================================================================================
// bool dcOBBOverlap(double *Ea, double *Pa, double *Ra, double *Eb, double *Pb, double *Ra)
// --------------------------------------------------------------------------------------
// Detects whether two Oriented Bounding Boxes (OBBs) are colliding with each other. 
// output:
//		return = true if collision happens.
// inputs:
//		OBB A = (Ea, Pa, Ra), OBB B = (Eb, Pb, Rb) where
//		(Ea[0],Ea[1],Ea[2]) = extents
//		(Pa[0],Pa[1],Pa[2]) = position
//		[Ra[0] Ra[3] Ra[6]]
//		[Ra[1] Ra[4] Ra[7]] = orientation
//		[Ra[2] Ra[5] Ra[8]]

bool dcTriOBBOverlap(double *v0, double *v1, double *v2, double *Ea, double *Pa, double *Ra);
// ======================================================================================
// bool dcTriOBBOverlap(double *v0, double *v1, double *v2, double *Ea, double *Pa, double *Ra)
// --------------------------------------------------------------------------------------
// Detects whether a triangle and an Oriented Bounding Box (OBB) are colliding with each other. 
// output:
//		return = true if collision happens.
// inputs:
//		triangle = (v0, v1, v2) where vi = double[3] = i-th vertex position of the triangle
//		OBB A = (Ea, Pa, Ra) where
//		(Ea[0],Ea[1],Ea[2]) = extents
//		(Pa[0],Pa[1],Pa[2]) = position
//		[Ra[0] Ra[3] Ra[6]]
//		[Ra[1] Ra[4] Ra[7]] = orientation
//		[Ra[2] Ra[5] Ra[8]]

bool dcPointOBBOverlap(double *v, double *Ea, double *Pa, double *Ra);
// ======================================================================================
// bool dcPointOBBOverlap(double *v, double *Ea, double *Pa, double *Ra)
// --------------------------------------------------------------------------------------
// Detects whether a point and an Oriented Bounding Box (OBB) are colliding with each other. 
// output:
//		return = true if collision happens.
// inputs:
//		v = position of the point
//		OBB A = (Ea, Pa, Ra) where
//		(Ea[0],Ea[1],Ea[2]) = extents
//		(Pa[0],Pa[1],Pa[2]) = position
//		[Ra[0] Ra[3] Ra[6]]
//		[Ra[1] Ra[4] Ra[7]] = orientation
//		[Ra[2] Ra[5] Ra[8]]


#endif

