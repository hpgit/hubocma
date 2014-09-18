#include "HpMotionMath.h"

//HuboVPBody.cpp
Vec3 vectorToVec3(const Eigen::Vector3d &v)
{
	return Vec3(v.data());
}
Eigen::Vector3d Vec3Tovector(const Vec3 &v)
{
	return Eigen::Vector3d(v[0], v[1], v[2]);
}
Eigen::Matrix3d vectorToSkewMat(const Eigen::Vector3d &v)
{
	Eigen::Matrix3d m;
	m.setZero();
	m(1, 0) = v(3);
	m(2, 0) = -v(2);
	m(2, 1) = v(1);
	m(0, 1) = -v(3);
	m(0, 2) = v(2);
	m(1, 2) = -v(1);

	return m;
}

//HuboMotionData.cpp
Eigen::Quaterniond quaterMinus(Eigen::Quaterniond &q2, Eigen::Quaterniond &q1)
{
	Eigen::Quaterniond q;
	q.w() = q2.w() - q1.w();
	q.x() = q2.x() - q1.x();
	q.y() = q2.y() - q1.y();
	q.z() = q2.z() - q1.z();

	return q;
}

Eigen::Quaterniond quaterRealMult(Eigen::Quaterniond &q, double a)
{
	Eigen::Quaterniond newq;
	newq.w() = q.w() *a;
	newq.x() = q.x() *a;
	newq.y() = q.y() *a;
	newq.z() = q.z() *a;
	return newq;
}

//HuboVpController.cpp
void matrixStackUptoDown(Eigen::MatrixXd &a, Eigen::MatrixXd &b)
{
	// a b | [a]
	//       [b]
	assert(a.cols() == b.cols());
	a.conservativeResize(a.rows()+b.rows(), a.cols());
	a.block(a.rows(), 0, b.rows(), b.cols()) = b;
}

//IKSolver.cpp
Eigen::Vector3d ln(Eigen::Quaterniond &q)
{
	double sc = q.vec().norm();
    double theta = atan2(sc, q.w());
	if(sc > DBL_EPSILON)
        sc = theta / sc;
    else  sc = 1.0 ;
	return sc*q.vec();
}

Eigen::Vector3d diffQuat(Eigen::Quaterniond &q1, Eigen::Quaterniond &q2)
{
	Eigen::Quaterniond q = q2.inverse() * q1;
	return ln(q);
}

Eigen::Vector3d qToEulerZYX(Eigen::Quaterniond &q)
{
	Eigen::Matrix3d m = q.toRotationMatrix();
	//Eigen::Matrix3d m = q.toRotationMatrix().transpose();
	double zang, xang, yang;
	zang = atan2(m(1, 0), m(0, 0));
	xang = atan2(m(2, 1), m(2, 2));
	if (abs(cos(zang)) < DBL_EPSILON)
		yang = atan2(-m(2, 0), m(1, 0) / sin(zang));
	else
		yang = atan2(-m(2, 0), m(0, 0) / cos(zang));


	return Vector3d(zang, yang, xang);
}

#define GOLD 1.61803398874989484820
#define GLIMIT 100.0
#define TINY 1.0e-20
#define ITMAX 100
#define CGOLD 0.3819660
#define ZEPS 1.0e-10

#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#define ABS(x)   ( ((x)>0.0) ? (x) :-(x) )
#define ACOS(x)  ( ((x)>1.0) ? (0) : ( ((x)<-1.0) ? (M_PI) : (acos(x)) ) )
#define ASIN(x)  ( ((x)>1.0) ? (M_PI/2.0) : ( ((x)<-1.0) ? (-M_PI/2.0) : (asin(x)) ) )
#define SQR(x)   ( (x)*(x) )
#define SHIFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b) ((b) >= 0.0 ? abs(a) : -abs(a))

#define TOL 2.0e-4

static int    ncom;
static double (IKSolver::*nrfunc) (Eigen::VectorXd &);
static Eigen::VectorXd pcom, xicom;
IKSolver *ik;

double f1dim(double x)
{
	static Eigen::VectorXd xt; xt.resize(ncom);

	for (int j=0; j<ncom; j++)
		xt[j] = pcom[j] + x * xicom[j];
	double f = ik->computePenalty(xt);

	return f;
}

void error( char* msg )
{
	std::cout << "ERROR : " << msg << std::endl;
//	assert( FALSE );
}

void mnbrak(double& ax, double& bx, double& cx, double& fa, double&fb, double& fc, double (*func) (double))
{
	double ulim, u, r, q, fu, dum;

	fa = (*func) (ax);
	fb = (*func) (bx);
	if (fb > fa)
	{
		SHIFT(dum, ax, bx, dum)
		SHIFT(dum, fb, fa, dum)
	}
	cx = (bx) + GOLD * (bx - ax);
	fc = (*func) (cx);
	
	while (fb > fc)
	{
		r = (bx - ax) * (fb - fc);
		q = (bx - cx) * (fb - fa);
		u = bx - ((bx - cx) * q - (bx - ax) * r) /
			(2.0 * SIGN(MAX(fabs(q - r), TINY), q - r));
		ulim = bx + GLIMIT * (cx - bx);
		
		if ((bx - u) * (u - cx) > 0.0)
		{
			fu = (*func) (u);
			if (fu < fc)
			{
				ax = bx;
				bx = u;
				fa = fb;
				fb = fu;
				return;
			}
			else if (fu > fb)
			{
				cx = u;
				fc = fu;
				return;
			}
			
			u = cx + GOLD * (cx - bx);
			fu = (*func) (u);
		}
		else if ((cx - u) * (u - ulim) > 0.0)
		{
			fu = (*func) (u);
			if (fu < fc)
			{
				SHIFT(bx, cx, u, cx + GOLD * (cx - bx))
				SHIFT(fb, fc, fu, (*func) (u))
			}
		}
		else if ((u - ulim) * (ulim - cx) >= 0.0)
		{
			u = ulim;
			fu = (*func) (u);
		}
		else
		{
			u = cx + GOLD * (cx - bx);
			fu = (*func) (u);
		}
		
		SHIFT(ax, bx, cx, u)
		SHIFT(fa, fb, fc, fu)
	}
}

double brent(double ax, double bx, double cx, double (*f) (double), double tol, double& xmin)
{
	int iter;
	double a, b, d, etemp, fu, fv, fw, fx, p, q, r, tol1, tol2, u, v, w, x, xm;
	double e = 0.0;

	a = (ax < cx ? ax : cx);
	b = (ax > cx ? ax : cx);
	x = w = v = bx;
	fw = fv = fx = (*f) (x);
	for (iter = 1; iter <= ITMAX; iter++)
	{
		xm = 0.5 * (a + b);
		tol2 = 2.0 * (tol1 = tol * fabs(x) + ZEPS);
		
		if (fabs(x - xm) <= (tol2 - 0.5 * (b - a)))
		{
			xmin = x;
			return fx;
		}
		
		if (fabs(e) > tol1)
		{
			r = (x - w) * (fx - fv);
			q = (x - v) * (fx - fw);
			p = (x - v) * q - (x - w) * r;
			q = 2.0 * (q - r);
			if (q > 0.0)
				p = -p;
			q = fabs(q);
			etemp = e;
			e = d;
			if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
				d = CGOLD * (e = (x >= xm ? a - x : b - x));
			else
			{
				d = p / q;
				u = x + d;
				if (u - a < tol2 || b - u < tol2)
					d = SIGN(tol1, xm - x);
			}
		}
		else
		{
			d = CGOLD * (e = (x >= xm ? a - x : b - x));
		}
		
		u = (fabs(d) >= tol1 ? x + d : x + SIGN(tol1, d));
		fu = (*f) (u);
		
		if (fu <= fx)
		{
			if (u >= x)	a = x;
				   else b = x;
			SHIFT(v, w, x, u)
			SHIFT(fv, fw, fx, fu)
		}
		else
		{
			if (u < x) a = u;
				  else b = u;
				  
			if (fu <= fw || w == x)
			{
				v = w;
				w = u;
				fv = fw;
				fw = fu;
			}
			else if (fu <= fv || v == x || v == w)
			{
				v = u;
				fv = fu;
			}
		}
	}
	
	error("Too many iterations in brent");
	xmin = x;
	return fx;
}

void linmin( Eigen::VectorXd &p, Eigen::VectorXd &xi, int n, double &fret)
{
	double xx, xmin, fx, fb, fa, bx, ax;

	ncom = n;
	pcom.resize(n);
	xicom.resize(n);

	for (int j=0; j<n; j++)
	{
		pcom[j] = p[j];
		xicom[j] = xi[j];
	}
  
	ax = 0.0;
	xx = 1.0;
	mnbrak(ax, xx, bx, fa, fx, fb, f1dim);
	fret = brent(ax, xx, bx, f1dim, TOL, xmin);
  
	for (int j=0; j<n; j++)
	{
		xi[j] *= xmin;
		p[j] += xi[j];
	}
}

#undef ITMAX
#define ITMAX 200

void gradient_descent(Eigen::VectorXd &p, int n, double ftol, int &iter, double &fret, IKSolver *_ik)
{
	double fp;
	static Eigen::VectorXd xi; xi.resize(n);
	ik = _ik;

	for (iter=0; iter<ITMAX; iter++)
	{
		fp = ik->computePenaltyGradient(p, xi);

		linmin(p, xi, n, fret);
		if (2.0 * abs(fret - fp) <= ftol * (abs(fret) + abs(fp) + DBL_EPSILON))
   	    	return;
	}

	error("Too many iterations in gradient_descent");
}
/*
void frprmn(Eigen::VectorXd &p, int n, double ftol, int &iter, double &fret,
			double (IKSolver::*func)  (Eigen::VectorXd &),
			double (IKSolver::*dfunc) (Eigen::VectorXd &,Eigen::VectorXd&))
{
	double gg, gam, fp, dgg;

	static Eigen::VectorXd g;   g.resize(n);
	static Eigen::VectorXd h;   h.resize(n);
	static Eigen::VectorXd xi; xi.resize(n);
  
	fp = (ik->*dfunc) (p, xi);

	for (int j=0; j<n; j++)
	{
		g[j] = -xi[j];
		xi[j] = h[j] = g[j];
	}

	for (iter=0; iter<ITMAX; iter++)
	{
		linmin(p, xi, n, fret, func);
		if (2.0 * fabs(fret - fp) <= ftol * (fabs(fret) + fabs(fp) + DBL_EPSILON)) return;
		
		fp = (ik->*dfunc) (p, xi);
		dgg = gg = 0.0;

		for (int j=0; j<n; j++)
		{
			gg += g[j] * g[j];
			dgg += (xi[j] + g[j]) * xi[j];
		}

		if (gg == 0.0) return;

		gam = dgg / gg;

		for (int j=0; j<n; j++)
		{
			g[j] = -xi[j];
			xi[j] = h[j] = g[j] + gam * h[j];
		}
	}

	error("Too many iterations in frprmn");
}
*/
#undef ITMAX
