#include "Joint.h"
#include <fstream>
#include <iostream>
#include <float.h>

//int Joint::frame;

void Joint::init()
{
	hasObjects = 0;
	onOBJ = 0;
	IKweight = 1.0;
	parent = NULL;
	offsetFromParent.setZero();
	m_iNumFaces = 0;
	childBodyMass = 1.0;
	constraintAxis.setZero();
	childBodyCom.setZero();
	childBodyInertia.setIdentity();
	hasBB = 0;
	canRotate = 0;
	Vector3d v0(0,0,0);
	Quaterniond q0(1,0,0,0);
	Quaterniond q(1, 0, 0, 0);
	motions.push_back(
		new Motion(v0, q0)
		);
}
Joint::~Joint()
{
	if(motions.size())
		for(int i = 0; i<motions.size(); i++)
			delete motions.at(i);
}

static void getFaceIndex(Joint &vOBJPart, char *pData)
{
	int iStartTexture = 0;
	int iStartNormal = 0;
	char buf[255];
	const int iDataLength = strlen(pData);

	int iCountSeperator =0;

	strcpy(buf, pData);

	for(int i=0; i < iDataLength && iCountSeperator < 2; i++)
	{
		if(buf[i] == '/')
		{
			iCountSeperator++;
			if(iCountSeperator == 1)
				iStartTexture = i+1;
			else if(iCountSeperator == 2)
				iStartNormal = i+1;
			buf[i] = NULL;
		}
		
	}

	if(iCountSeperator == 0)
	{
		vOBJPart.m_vIndexVertices.push_back(atoi(&buf[0])-1);
	}
	else if(iCountSeperator == 1)
	{
		vOBJPart.m_vIndexVertices.push_back(atoi(&buf[0])-1);
		vOBJPart.m_vIndexTexture.push_back(atoi(&buf[iStartTexture])-1);
	}
	else if(iCountSeperator == 2)
	{
		vOBJPart.m_vIndexVertices.push_back(atoi(&buf[0])-1);
		if(iStartNormal != iStartTexture +1)
			vOBJPart.m_vIndexTexture.push_back(atoi(&buf[iStartTexture])-1);
		vOBJPart.m_vIndexNormal.push_back(atoi(&buf[iStartNormal])-1);
	}

}

int Joint::importOBJ(char* filename)
{
	std::ifstream fin(filename);

	char buf[255];
	char *token;
	char *context;

	int iNumGroup = 0;
	int iNumFaceSize = 0;

	
	while(!fin.eof())
	{
		fin.getline(buf, 254);
		if(buf[0] == '#')
		{
			//comment
		}
		else if(buf[0] == 'o')
		{
			//object name
		}
		else if(buf[0] == 'g')
		{
			//group name
		}

		else if(buf[0] == 'v')
		{
			token = strtok(buf, " ");
			token = strtok(NULL, " ");
			//vertex
			if(buf[1] == 'n')
			{
				
				while(token != NULL)
				{
					m_vNormal.push_back(atof(token));
					token = strtok(NULL, " ");
				}
			}
			else if(buf[1] == 't')
			{
				while(token != NULL)
				{
					m_vTexture.push_back(atof(token));
					token = strtok(NULL, " ");
				}
			}
			else 
			{
				while(token != NULL)
				{
					m_vVertices.push_back(atof(token));
					token = strtok(NULL, " ");
				}
			}
		}
		else if(buf[0] == 'f')
		{
			//faces
			token = strtok(buf, " ");
			token = strtok(NULL," ");
			while(token !=NULL)
			{
				getFaceIndex(*this, token);
				iNumFaceSize++;
				token= strtok(NULL," ");
			}

			m_vNumIndex.push_back(iNumFaceSize);
			m_iNumFaces++;
			iNumFaceSize = 0;

		}
		else if(buf[0] == 's')
		{
			//smooth shading
		}
		else if(buf[0] == 'm')
		{
			//mtllib
		}
		else if(buf[0] == 'u')
		{
			//usemtl
		}
		else if(buf[0] == 'p')
		{
			//point
		}
		else if(buf[0] == 'l')
		{
			//line
		}
	}

	fin.close();
	hasObjects = 1;

	int VertexSize = m_vIndexVertices.size();
	int NormalSize = m_vIndexNormal.size();
	int TextureSize = m_vIndexTexture.size();

	for(int i=0; i<VertexSize; i++)
	{
		points.push_back(m_vVertices[m_vIndexVertices[i]*3]);
		points.push_back(m_vVertices[m_vIndexVertices[i]*3+1]);
		points.push_back(m_vVertices[m_vIndexVertices[i]*3+2]);
	}

	if(m_vIndexNormal.size() > 0)
	{
		for(int i=0; i<NormalSize; i++)
		{
			Vector3d v_normal( 
				m_vNormal[m_vIndexNormal[i]*3], 
				m_vNormal[m_vIndexNormal[i]*3+1], 
				m_vNormal[m_vIndexNormal[i]*3+2]);
			v_normal.normalize();
			normal.push_back(v_normal.x());
			normal.push_back(v_normal.y());
			normal.push_back(v_normal.z());
		}
	}

	if(0 && m_vIndexTexture.size() > 0)
	{
		for(int i=0; i<TextureSize; i++)
			texture[i] = m_vIndexTexture[i];
	}

	makeBB();

	return 1;
}

#ifndef NOOBJ
void Joint::drawHierarchical(int frame)
{
	glPushMatrix();

    if(onOBJ)
    //if(false)
	{
        Vector3d translation = getGlobalPosition(frame);
		Quaterniond rotation = getGlobalOrientation(frame);
		//Vector3d translation = offsetFromParent + getTranslation(frame);
		//Quaterniond rotation = getRotation(frame);
		//double rotateAngle = getAngle(frame)*180/M_PI;
		//Vector3d rotateAxis = rotation.vec().normalized();
		
		//glTranslatef(translation[0], translation[1], translation[2]);
		Eigen::Affine3d a;
		a.setIdentity();
		//a.translate(translation);
		a.rotate(rotation).pretranslate(translation);
		//if(rotation.vec().norm()>DBL_EPSILON)
			glMultMatrixd(a.data());
			//glRotated(rotateAngle, rotateAxis[0], rotateAxis[1], rotateAxis[2]);
		//if(constraintAxis.length() >= 0.5)
		//	glRotated(rotateAngle, constraintAxis[0], constraintAxis[1], constraintAxis[2]);
		if(hasObjects)
		{
			glDisable(GL_LIGHTING);
			glColor3f(0.8,0.8,0.8);
			//drawBB();
			glEnable(GL_LIGHTING);

			int VertexSize = m_vIndexVertices.size();
			glVertexPointer(3, GL_FLOAT, 0, points.data());
			glNormalPointer(GL_FLOAT, 0, normal.data());

			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			{
				glColor3f(0.6,0.6,0.6);
				glDrawArrays(GL_TRIANGLES, 0, VertexSize);
			}
			glDisableClientState(GL_NORMAL_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);
		}
		else
		{
			glColor3f(1.0, 0, 0);
			glBegin(GL_POINTS);
				glVertex3d(0,0,0);
			glEnd();
		}
	}
	else
	{
		glDisable(GL_LIGHTING);
		Vector3d vv = getGlobalPosition(frame);

		glColor3f(1,0,0);
		glPointSize(5.0);

		glBegin(GL_POINTS);
			glVertex3f(vv.x(),vv.y(),vv.z());
		glEnd();
		
		if(parent != NULL)
		{
			Vector3d v = parent->getGlobalPosition(frame);
			glBegin(GL_LINES);
				glColor3f(1,0,0);
				glVertex3dv(v.data());
				glVertex3dv(vv.data());
			glEnd();
		}

		glPointSize(1.0);
		glEnable(GL_LIGHTING);
	}
	glPopMatrix();

	for(int i=0; i<children.size() ; i++)
		children.at(i)->drawHierarchical(frame);
	
}


void Joint::drawBoxHierarchical(int frame)
{
	glPushMatrix();
	{
        Vector3d translation = getGlobalPosition(frame);
		Quaterniond rotation = getGlobalOrientation(frame);
		Eigen::Affine3d a;
		a.setIdentity();
		a.rotate(rotation).pretranslate(translation);
		glMultMatrixd(a.data());

		glEnable(GL_LIGHTING);
		drawBBRigid();
		glDisable(GL_LIGHTING);
	}
	glPopMatrix();

	for(int i=0; i<children.size() ; i++)
		children.at(i)->drawBoxHierarchical(frame);
}
#endif

Vector3d Joint::getGlobalPosition(int frame)
{
	if(parent == NULL)
		return motions.at(frame)->getTranslation() + offsetFromParent;

	Quaterniond q = parent->getGlobalOrientation(frame);
	Quaterniond q_position(0, offsetFromParent.x(), offsetFromParent.y(), offsetFromParent.z());

	return parent->getGlobalPosition(frame) + (q*q_position*q.inverse()).vec();
}

Vector3d Joint::getGlobalPosition(int frame, double t)
{
	if(parent == NULL)
		return getTranslation(frame, t) + offsetFromParent;

	Quaterniond q = parent->getGlobalOrientation(frame, t);
	Quaterniond q_position(0, offsetFromParent.x(), offsetFromParent.y(), offsetFromParent.z());

	return parent->getGlobalPosition(frame, t) + (q*q_position*q.inverse()).vec();
}

Vector3d Joint::getGlobalBoundingBoxPosition(int frame)
{
	Quaterniond q = getGlobalOrientation(frame);
	Quaterniond q_position(0, BSpos.x(), BSpos.y(), BSpos.z());

	return getGlobalPosition(frame) + (q*q_position*q.inverse()).vec();
}

Vector3d Joint::getGlobalBoundingBoxPosition(int frame, double t)
{
	Quaterniond q = getGlobalOrientation(frame, t);
	Quaterniond q_position(0, BSpos.x(), BSpos.y(), BSpos.z());

	return getGlobalPosition(frame, t) + (q*q_position*q.inverse()).vec();
}

Vector3d Joint::getGlobalComPosition(int frame)
{
	Quaterniond q = getGlobalOrientation(frame);
	Quaterniond q_position(0, childBodyCom.x(), childBodyCom.y(), childBodyCom.z());

	return getGlobalPosition(frame) + (q*q_position*q.inverse()).vec();
}

Vector3d Joint::getGlobalComPosition(int frame, double t)
{
	Quaterniond q = getGlobalOrientation(frame, t);
	Quaterniond q_position(0, childBodyCom.x(), childBodyCom.y(), childBodyCom.z());

	return getGlobalPosition(frame, t) + (q*q_position*q.inverse()).vec();
}

Quaterniond Joint::getGlobalOrientation(int _frame)
{
	if(parent == NULL)
		return motions.at(_frame)->getRotation();

	return parent->getGlobalOrientation(_frame) * motions.at(_frame)->getRotation();
}

Quaterniond Joint::getGlobalOrientation(int _frame, double t)
{
	if(parent == NULL)
		return getRotation(_frame, t);

	return parent->getGlobalOrientation(_frame, t) * getRotation(_frame, t);
}

Vector3d Joint::getGlobalRotationAxis(int frame)
{
	if(parent == NULL)
		return constraintAxis;
	Quaterniond q = parent->getGlobalOrientation(frame);
	Quaterniond q_vector(0, constraintAxis.x(), constraintAxis.y(), constraintAxis.z());
	return (q*q_vector*q.inverse()).vec();
}

Vector3d Joint::getGlobalRotationAxis(int frame, double t)
{
	if(parent == NULL)
		return constraintAxis;
	Quaterniond q = parent->getGlobalOrientation(frame, t);
	Quaterniond q_vector(0, constraintAxis.x(), constraintAxis.y(), constraintAxis.z());
	return (q*q_vector*q.inverse()).vec();
}

Quaterniond Joint::getRotation(int frame)
{
	return motions.at(frame)->getRotation();
}

Quaterniond Joint::getRotation(int firstFrame, double t)
{
	if (firstFrame == motions.size() - 1)
		return motions.at(firstFrame)->getRotation();

	Quaterniond f = motions.at(firstFrame)->getRotation();	
	Quaterniond l = motions.at(firstFrame+1)->getRotation();	

	return f.slerp(t, l);
}

Vector3d Joint::getTranslation(int frame)
{
	return motions.at(frame)->getTranslation();
}

Vector3d Joint::getTranslation(int firstFrame, double t)
{
	if (firstFrame == motions.size() - 1)
		return motions.at(firstFrame)->getTranslation();

	return t*motions.at(firstFrame)->getTranslation() 
		+ (1-t)*motions.at(firstFrame+1)->getTranslation();
}

void Joint::setOBJon(int on)
{
	onOBJ = on;
}

void Joint::setIKweight(float w)
{
	assert(w >= 0);
	IKweight = w;
}

void Joint::setOffset(Vector3d &offset)
{
	offsetFromParent = offset;
}

double Joint::getAngle(int _frame)
{
	Quaterniond q;
	q = motions.at(_frame)->getRotation();
	return 2 * atan2(q.vec().dot(constraintAxis), q.w());
}

double Joint::getAngle(int firstFrame, double t)
{
	if (firstFrame <= angles.size() - 2)
		return angles.at(firstFrame)*(1 - t) + angles.at(firstFrame + 1)*t;
	else
		return angles.back();
	//Quaterniond q = getRotation(firstFrame, lastFrame, t);
	//return 2 * atan2(q.vec().dot(constraintAxis), q.w());

	//Quaterniond q1 = getRotation(firstFrame);
	//Quaterniond q2 = getRotation(lastFrame);
	//double firstAngle = 2 * atan2(q1.vec().dot(constraintAxis), q1.w());
	//double lastAngle = 2 * atan2(q2.vec().dot(constraintAxis), q2.w());
	//return firstAngle + (lastAngle - firstAngle)*t;

	//return angleBspline.getValue(firstFrame + t)(0);
}

double Joint::getAngleInTime(double frame)
{
	int iframe = (int)frame;
	double phase = frame - iframe;
	if (iframe <= angles.size() - 2)
		return angles.at(iframe)*(1 - phase) + angles.at(iframe + 1)*phase;
	else
		return angles.back();


	//return angleBspline.getValue(frame)(0);
}

double Joint::getAngleRateInTime(double frame)
{
	// difference of two adjacent frame
	int iframe = (int)frame;
	if (iframe <= angles.size() - 2)
		return angles.at(iframe + 1) - angles.at(iframe);
	else
		return angles.at(iframe) - angles.at(iframe-1);

	//return angleBspline.getDifferentialValue(frame)(0);
}

double Joint::getAngleSecondDerivateInTime(double frame)
{
	// difference of adjacent of difference of two adjacent frame 
	int iframe = (int)frame;
	if (iframe == angles.size() - 1)
		return angles.at(iframe) - 2 * angles.at(iframe - 1) + angles.at(iframe - 2);
	else if (iframe == angles.size() - 2)
		return angles.at(iframe + 1) - 2 * angles.at(iframe) + angles.at(iframe - 1);
	else
		return angles.at(iframe + 2) - 2 * angles.at(iframe + 1) + angles.at(iframe);

	//return angleBspline.getSecondDerivationValue(frame)(0);
}

void Joint::setAngle(int _frame, double radian)
{
	double radover2 = radian /2 ;
	Vector3d v = sin(radover2) * constraintAxis;
	Quaterniond q = Quaterniond(cos(radover2), v.x(), v.y(), v.z());
	motions.at(_frame)->setRotation(q);
	angles.at(_frame) = radian;
}

void Joint::setTranslation(int _frame, Vector3d &translation)
{
	motions.at(_frame)->setTranslation(translation);
}

void Joint::setRotation(int _frame, Quaterniond &rotation)
{
	motions.at(_frame)->setRotation(rotation);
}

/*
void Joint::makeAngleSpline()
{
	angleBspline.clear();
	Eigen::VectorXd angle, bsplineControlPoint;
	Eigen::Vector3d firstCompo(1, -2, 1);
	Eigen::Vector3d secondCompo(1, 4, 1);
	Eigen::MatrixXd naturalToBspline;
	int motionSize = motions.size();
	angle.resize(motionSize+2);
	angle.setZero();
	bsplineControlPoint.resize(motionSize+2);

	for (int i = 0; i < motionSize; i++)
		angle(i + 1) = getAngle(i);
	
	naturalToBspline.resize(motionSize + 2, motionSize + 2);
	naturalToBspline.block(0, 0, 1, 3) = firstCompo.transpose();
	naturalToBspline.block(motionSize+1, motionSize-1, 1, 3) = firstCompo.transpose();

	for (int i = 0; i < motionSize; i++)
		naturalToBspline.block(i+1, i, 1, 3) = secondCompo.transpose();

	bsplineControlPoint = 6 * naturalToBspline.inverse()*angle;

	for (int i = 0; i < bsplineControlPoint.size(); i++)
	{
		Eigen::VectorXd v;
		v.resize(1);
		v(0) = bsplineControlPoint(i);
		angleBspline.setControlPoint(i-1, v);
	}
}
*/

void Joint::setParent(Joint* _parent)
{
	this->parent = _parent;
	for(unsigned i=0; i<parent->children.size(); i++)
		if(_parent->children.at(i) == this)
			return;
	_parent->children.push_back(this);
}

void Joint::setChild(Joint* _child)
{
	_child->parent = this;
	for(unsigned i=0; i<children.size(); i++)
		if(children.at(i) == _child)
			return;
	children.push_back(_child);
}

int Joint::isDescendant(Joint *joint)
{
	// Is *this pure(not contain *this) descendant of *joint?
	if(parent == NULL)
		return 0;
	else if(parent == joint)
		return 1;
	return parent->isDescendant(joint);
}

int Joint::isAncestor(Joint* joint)
{
	// Is *this pure(not contain *this) ancestor of *joint?
	return joint->isDescendant(this);
}

void Joint::makeBB()
{
	//bounding box part
	BBmax = Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);
	BBmin = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	for (int i = 0; i < points.size(); i++)
	{
		if (points.at(i) > BBmax(i%3))
			BBmax(i%3) = points.at(i);
		if (points.at(i) < BBmin(i%3))
			BBmin(i%3) = points.at(i);
	}

	hasBB=1;

	BBpoints[0] = Vector3d(BBmax.x(), BBmax.y(), BBmax.z());
	BBpoints[1] = Vector3d(BBmax.x(), BBmax.y(), BBmin.z());
	BBpoints[2] = Vector3d(BBmax.x(), BBmin.y(), BBmax.z());
	BBpoints[3] = Vector3d(BBmax.x(), BBmin.y(), BBmin.z());

	BBpoints[4] = Vector3d(BBmin.x(), BBmax.y(), BBmax.z());
	BBpoints[5] = Vector3d(BBmin.x(), BBmax.y(), BBmin.z());
	BBpoints[6] = Vector3d(BBmin.x(), BBmin.y(), BBmax.z());
	BBpoints[7] = Vector3d(BBmin.x(), BBmin.y(), BBmin.z());

	BBsizev = BBmax - BBmin;
	BBvol = BBsizev.x()*BBsizev.y()*BBsizev.z();

	//bounding sphere part
	BSpos = (BBmax + BBmin)/2;
	BSrad = (BBmax - BSpos).norm();
}

void Joint::drawBB()
{
	glColor3f(.8f,.5f,.5f);
	glBegin(GL_LINES);
	glVertex3dv(BBpoints[0].data());
	glVertex3dv(BBpoints[1].data());
	glVertex3dv(BBpoints[0].data());
	glVertex3dv(BBpoints[2].data());
	glVertex3dv(BBpoints[0].data());
	glVertex3dv(BBpoints[4].data());

	glVertex3dv(BBpoints[3].data());
	glVertex3dv(BBpoints[1].data());
	glVertex3dv(BBpoints[3].data());
	glVertex3dv(BBpoints[2].data());
	glVertex3dv(BBpoints[3].data());
	glVertex3dv(BBpoints[7].data());

	glVertex3dv(BBpoints[5].data());
	glVertex3dv(BBpoints[4].data());
	glVertex3dv(BBpoints[5].data());
	glVertex3dv(BBpoints[7].data());
	glVertex3dv(BBpoints[5].data());
	glVertex3dv(BBpoints[1].data());

	glVertex3dv(BBpoints[6].data());
	glVertex3dv(BBpoints[4].data());
	glVertex3dv(BBpoints[6].data());
	glVertex3dv(BBpoints[7].data());
	glVertex3dv(BBpoints[6].data());
	glVertex3dv(BBpoints[2].data());
	glEnd();
}
void Joint::drawBBRigid()
{
	glColor3f(.5f, .5f, .5f);
	glBegin(GL_QUADS);
	glNormal3d(1, 0, 0);
	glVertex3dv(BBpoints[0].data());
	glNormal3d(1, 0, 0);
	glVertex3dv(BBpoints[2].data());
	glNormal3d(1, 0, 0);
	glVertex3dv(BBpoints[3].data());
	glNormal3d(1, 0, 0);
	glVertex3dv(BBpoints[1].data());

	glNormal3d(0, 1, 0);
	glVertex3dv(BBpoints[0].data());
	glNormal3d(0, 1, 0);
	glVertex3dv(BBpoints[1].data());
	glNormal3d(0, 1, 0);
	glVertex3dv(BBpoints[5].data());
	glNormal3d(0, 1, 0);
	glVertex3dv(BBpoints[4].data());

	glNormal3d(0, 0, 1);
	glVertex3dv(BBpoints[0].data());
	glNormal3d(0, 0, 1);
	glVertex3dv(BBpoints[4].data());
	glNormal3d(0, 0, 1);
	glVertex3dv(BBpoints[6].data());
	glNormal3d(0, 0, 1);
	glVertex3dv(BBpoints[2].data());

	glNormal3d(0, -1, 0);
	glVertex3dv(BBpoints[2].data());
	glNormal3d(0, -1, 0);
	glVertex3dv(BBpoints[6].data());
	glNormal3d(0, -1, 0);
	glVertex3dv(BBpoints[7].data());
	glNormal3d(0, -1, 0);
	glVertex3dv(BBpoints[3].data());

	glNormal3d(0, 0, -1);
	glVertex3dv(BBpoints[1].data());
	glNormal3d(0, 0, -1);
	glVertex3dv(BBpoints[3].data());
	glNormal3d(0, 0, -1);
	glVertex3dv(BBpoints[7].data());
	glNormal3d(0, 0, -1);
	glVertex3dv(BBpoints[5].data());

	glNormal3d(-1, 0, 0);
	glVertex3dv(BBpoints[4].data());
	glNormal3d(-1, 0, 0);
	glVertex3dv(BBpoints[5].data());
	glNormal3d(-1, 0, 0);
	glVertex3dv(BBpoints[7].data());
	glNormal3d(-1, 0, 0);
	glVertex3dv(BBpoints[6].data());
	glEnd();
}


void Joint::setConstraintAngle(double min, double max)
{
	this->constraintAngle[0] = min;
	this->constraintAngle[1] = max;
}
void Joint::setConstraintAxis(string &Axis)
{
	if(Axis == "X")
		this->constraintAxis.x() = 1;
	else if(Axis == "Y")
		this->constraintAxis.y() = 1;
	else if(Axis == "Z")
		this->constraintAxis.z() = 1;
}

double distance(Joint *j1, Joint *j2, int frame)
{
	if( !(j1->hasBB && j2->hasBB))
		return DBL_EPSILON;
	// TODO:

	return 0;
}

int isCollideBoundingSphere(Joint *j1, Joint *j2, int frame)
{
	Quaterniond q1 = j1->parent->getGlobalOrientation(frame);
	Quaterniond q2 = j2->parent->getGlobalOrientation(frame);

	Quaterniond v1( 0, j1->BSpos.x(), j1->BSpos.y(), j1->BSpos.z() );
	Quaterniond v2( 0, j2->BSpos.x(), j2->BSpos.y(), j2->BSpos.z() );

	if( (  (q1*v1*q1.inverse()).vec() - (q2*v2*q2.inverse()).vec()
			+ j1->getGlobalPosition(frame) - j2->getGlobalPosition(frame)).norm() 
		- j1->BSrad - j1->BSrad) 
		return 1;
	return 0;
}
