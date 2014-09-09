#ifndef _JOINT_H_
#define _JOINT_H_

#include "Motion.h"
#include <UniformBspline.h>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>

//#define NOOBJ
#ifndef NOOBJ
//#include <Windows.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#endif

enum DOF {Xt, Yt, Zt, Xr, Yr, Zr};

class Joint{
protected:
	//static int frame;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Joint()	: name("")
	{
		init();
	};

	Joint(string _name) : name(_name)
	{
		init();
	};

	Joint(string _name, char* filename) : name(_name)
	{ 
		init();
		hasObjects = importOBJ(filename);
	};

	~Joint();

	string name;

	//obj variables
	int hasObjects;
	int onOBJ;
	int m_iNumFaces;
	
	std::vector<int> m_vNumIndex;
	std::vector<int> m_vIndexVertices;
	std::vector<int> m_vIndexTexture;
	std::vector<int> m_vIndexNormal;

	std::vector<float> m_vVertices;
	std::vector<float> m_vNormal;
	std::vector<float> m_vTexture;

	std::vector<float> points;
	std::vector<float> texture;
	std::vector<float> normal;
	

	
	//Kinematics variables
	float IKweight;
	//std::vector<Motion*> IKmotion;
	
	Vector3d offsetFromParent;
	
	Joint* parent;
	std::vector<Joint*> children;
	
	std::vector<DOF> dof; // saved in local coordinate order
	std::vector< std::pair<double,double> > constraints;
	std::vector<Motion*> motions;
	std::vector<double> angles;
	//UniformBspline angleBspline;
	double angleOffset;
	
	
	int hasBB;
	Vector3d BBmax;
	Vector3d BBmin;
	Vector3d BBpoints[8];
	Vector3d BBsizev;
	Vector3d BSpos;
	double BSrad;
	double BBvol;

	int canRotate;
	int isConstraint; // constraint in IK


	//variables for constraints
	Vector3d constraintAxis;
	double constraintAngle[2];


	//variables for dynamics
	double childBodyMass;
	Vector3d childBodyCom;
	Eigen::Matrix3d childBodyInertia;

	


	//methods
	void init();
	int importOBJ(char* filename);
	void drawHierarchical(int frame);
	//void drawHierarchical();

	Vector3d getGlobalPosition(int frame);
	Vector3d getGlobalPosition(int frame, double t);
	Vector3d getGlobalBoundingBoxPosition(int frame);
	Vector3d getGlobalBoundingBoxPosition(int frame, double t);
	Vector3d getGlobalComPosition(int frame);
	Vector3d getGlobalComPosition(int frame, double t);
	Quaterniond getGlobalOrientation(int frame);
	Quaterniond getGlobalOrientation(int frame, double t);
	Vector3d getGlobalRotationAxis(int frame);
	Vector3d getGlobalRotationAxis(int frame, double t);

	Quaterniond getRotation(int frame);
	Quaterniond getRotation(int firstFrame, double t);
	Vector3d getTranslation(int frame);
	Vector3d getTranslation(int firstFrame, double t);

	void setOBJon(int on);
	void setIKweight(float w);
	void setOffset(Vector3d &offset);
	double getAngle(int frame);
	double getAngle(int firstFrame, double t);
	double getAngleInTime(double frame); // can have float frame number
	double getAngleRateInTime(double frame); // can have float frame number
	double getAngleSecondDerivateInTime(double frame); // can have float frame number
	void setAngle(int _frame, double radian);
	void setTranslation(int _frame, Vector3d &translation);
	void setRotation(int _frame, Quaterniond &rotation);

	void makeAngleSpline();

	void setParent(Joint* _parent);
	void setChild(Joint* _child);
	
	
	int isAncestor(Joint* joint);
	int isDescendant(Joint* joint);

	void makeBB();
	void drawBB();


	void setConstraintAngle(double min, double max);
	void setConstraintAxis(string &axis);


	friend int splitRotation(std::vector<Joint*> &srcJoints, 
							std::vector<Joint*> &destJoints, 
							std::vector<DOF> &dofInLocalCoordOrder);

	friend double boundingBoxDistanceBetweenJoint(Joint *joint1, Joint *joint2, int frame); // distance between two boxes
	friend int isCollideBoundingSphere(Joint *joint1, Joint *joint2, int frame); // check if bounding sphere collide
};

#endif
