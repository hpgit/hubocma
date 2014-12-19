#include "myQuadProg.h"
#include "QuadProg.h"
#include <stdexcept>
#include <fstream>

using namespace std;

bool solve_myquadprog(RMatrix &x_, const RMatrix &G_, const RMatrix &h_, const RMatrix &C_, const RMatrix &d_)
{
	if ( G_.RowSize() != G_.ColSize() || G_.RowSize() != h_.RowSize() ) return false;
	if ( C_.RowSize() != d_.RowSize() ) return false;

	int n = G_.RowSize();
	int m = C_.RowSize();

	if ( n == 0 ) { x_.SetZero(0, 0); return true; }

	if ( C_.ColSize() != n && m != 0 ) return false;
	if ( n > MATRIX_DIM || m > MATRIX_DIM ) return false;

	double (*G)[MATRIX_DIM] = new double[n][MATRIX_DIM];
	double (*Ct)[MATRIX_DIM] = new double[n][MATRIX_DIM];
	double *g0 = new double[n];
	double *c0 = new double[m];

	for (int i=0; i<n; i++) { for (int j=0; j<n; j++) { G[i][j] = G_(i,j); } }
	for (int i=0; i<n; i++) { for (int j=0; j<m; j++) { Ct[i][j] = -C_(j,i); } }
	for (int i=0; i<n; i++) { g0[i] = h_[i]; }
	for (int i=0; i<m; i++) { c0[i] = d_[i]; }

	x_.ReNew(n,1);

	double J = solve_quadprog(G, g0, n, NULL, NULL, 0, Ct, c0, m, x_.GetPtr());

	delete [] G;
	delete [] Ct;
	delete [] g0;
	delete [] c0;

	return true;
}

bool solve_myquadprog(RMatrix &x_, const RMatrix &G_, const RMatrix &h_, const RMatrix &Ci_, const RMatrix &di_, const RMatrix &Ce_, const RMatrix &de_)
{
	if ( G_.RowSize() != G_.ColSize() || G_.RowSize() != h_.RowSize() ) return false;
	if ( Ci_.RowSize() != di_.RowSize() ) return false;

	int n = G_.RowSize();
	int mi = Ci_.RowSize();
	int me = Ce_.RowSize();

	if ( n == 0 ) { x_.SetZero(0, 0); return true; }

	if ( mi != 0 && Ci_.ColSize() != n ) return false;
	if ( me != 0 && Ce_.ColSize() != n ) return false;
	if ( n > MATRIX_DIM || mi > MATRIX_DIM || me > MATRIX_DIM ) return false;

	double (*G)[MATRIX_DIM] = new double[n][MATRIX_DIM];
	double (*Cit)[MATRIX_DIM] = new double[n][MATRIX_DIM];
	double (*Cet)[MATRIX_DIM] = new double[n][MATRIX_DIM];
	double *g0 = new double[n];
	double *ci0 = new double[mi];
	double *ce0 = new double[me];

	for (int i=0; i<n; i++) { for (int j=0; j<n; j++) { G[i][j] = G_(i,j); } }
	for (int i=0; i<n; i++) { for (int j=0; j<mi; j++) { Cit[i][j] = -Ci_(j,i); } }	// Be cautious!!
	for (int i=0; i<n; i++) { for (int j=0; j<me; j++) { Cet[i][j] = Ce_(j,i); } }
	for (int i=0; i<n; i++) { g0[i] = h_[i]; }
	for (int i=0; i<mi; i++) { ci0[i] = di_[i]; }
	for (int i=0; i<me; i++) { ce0[i] = -de_[i]; }	// Be cautious!!

	x_.ReNew(n,1);

	double J = solve_quadprog(G, g0, n, Cet, ce0, me, Cit, ci0, mi, x_.GetPtr());

	delete [] G;
	delete [] Cit;
	delete [] Cet;
	delete [] g0;
	delete [] ci0;
	delete [] ce0;

	return true;
}

bool solve_Ax_b_Cx_d(RMatrix &x, const RMatrix &A, const RMatrix &b, const RMatrix C, const RMatrix &d, double alpha)
{
	return solve_myquadprog(x, ~A*A + alpha*Eye(A.ColSize()), -(~A*b), C, d);
}

bool solve_Apx_bp_Asx_bs_Cx_d(RMatrix &x, const RMatrix &Ap, const RMatrix &bp, const RMatrix &As, const RMatrix &bs, const RMatrix C, const RMatrix &d, double alpha)
{
	if ( Ap.RowSize() > 0 ) {
		RMatrix xp, Np, ys;
		if ( !solve_Ax_b_Cx_d(xp, Ap, bp, C, d, alpha) ) return false;
		Np = Nullspace(Ap);
		if ( As.RowSize() > 0 && Np.ColSize() > 0 ) {
			if ( !solve_Ax_b_Cx_d(ys, As*Np, bs-As*xp, C*Np, d-C*xp, alpha) ) return false;
			x = xp + Np*ys;
		} else {
			x = xp;
		}
	} else {
		if ( As.RowSize() > 0 ) {
			if ( !solve_Ax_b_Cx_d(x, As, bs, C, d, alpha) ) return false;
		} else {
			if ( C.RowSize() > 0 ) {
				x.SetZero(C.ColSize(), 1);
			} else {
				;	// no action when Ap.RowSize() = As.ColSize() = C.RowSize() = 0.
			}
		}
	}
	return true;
}

bool solve_A0x_b0_Apx_bp_Asx_bs_Cx_d(RMatrix &x, const RMatrix &A0, const RMatrix &b0, const RMatrix &Ap, const RMatrix &bp, const RMatrix &As, const RMatrix &bs, const RMatrix C, const RMatrix &d, double alpha)
{
	if ( A0.RowSize() > 0 ) {
		RMatrix x0, N0, y;
		solve_Ax_b_pInv(x0, A0, b0, N0);
		if ( N0.ColSize() <= 0 ) { x = x0; return true; }
		if ( !solve_Apx_bp_Asx_bs_Cx_d(y, Ap*N0, bp-Ap*x0, As*N0, bs-As*x0, C*N0, d-C*x0, alpha) ) return false;
		x = x0 + N0*y;
	} else {
		if ( !solve_Apx_bp_Asx_bs_Cx_d(x, Ap, bp, As, bs, C, d, alpha) ) return false;
	}
	return true;
}
