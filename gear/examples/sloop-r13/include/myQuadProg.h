#ifndef __MYQUADPROG_
#define __MYQUADPROG_

#include "rmatrix3j.h"

// ** maximum matrix dimension limited!! ** (see QuadProg.h)

bool solve_myquadprog(RMatrix &x, const RMatrix &G, const RMatrix &h, const RMatrix &C, const RMatrix &d);
// solve quadratic programming:
//          minimize   0.5*x'*G*x + h'*x
//              s.t.   C*x <= d

bool solve_myquadprog(RMatrix &x, const RMatrix &G, const RMatrix &h, const RMatrix &Ci, const RMatrix &di, const RMatrix &Ce, const RMatrix &de);
// solve quadratic programming:
//          minimize   0.5*x'*G*x + h'*x
//              s.t.   Ci*x <= di
//					   Ce*x = de

bool solve_Ax_b_Cx_d(RMatrix &x, const RMatrix &A, const RMatrix &b, const RMatrix C, const RMatrix &d, double alpha);
// solve quadratic programming:
//			minimize  0.5 * ( || Ax-b ||^2 + alpha * || x ||^2 )
//			    s.t.  C*x <= d

bool solve_Apx_bp_Asx_bs_Cx_d(RMatrix &x, const RMatrix &Ap, const RMatrix &bp, const RMatrix &As, const RMatrix &bs, const RMatrix C, const RMatrix &d, double alpha);
// solve quadratic programming:
//			minimize 0.5 * ( || Ap x - bp ||^2 + alpha * || x ||^2 ) first and then
//			minimize 0.5 * ( || Ax x - bs ||^2 + alpha * || x ||^2 )
//				s.t. C*x <= d

bool solve_A0x_b0_Apx_bp_Asx_bs_Cx_d(RMatrix &x, const RMatrix &A0, const RMatrix &b0, const RMatrix &Ap, const RMatrix &bp, const RMatrix &As, const RMatrix &bs, const RMatrix C, const RMatrix &d, double alpha);
// solve 
//			minimize 0.5 * ( || Ap x - bp ||^2 + alpha * || x ||^2 ) first and then
//			minimize 0.5 * ( || Ax x - bs ||^2 + alpha * || x ||^2 )
//				s.t. C*x <= d 
//					 A0*x = b0

#endif