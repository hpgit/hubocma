#include <list>
#include <vector>
#include <algorithm>
#include "gcoordinate.h"
#include "gjoint.h"
#include "gsystem_p.h"

using namespace std;


//=============================================================
//                 GSystemP
//=============================================================

void GSystemP::get_qa(double *qa_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		qa_[i++] = (*iter_pcoord)->q;
	}
}

void GSystemP::get_dqa(double *dqa_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		dqa_[i++] = (*iter_pcoord)->dq;
	}
}

void GSystemP::get_ddqa(double *ddqa_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		ddqa_[i++] = (*iter_pcoord)->ddq;
	}
}

void GSystemP::get_taua(double *taua_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		taua_[i++] = (*iter_pcoord)->tau;
	}
}

void GSystemP::get_DtauaDp(double *DtauaDp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		DtauaDp_[i++] = (*iter_pcoord)->DtauDp;
	}
}

void GSystemP::get_qp(double *qp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		qp_[i++] = (*iter_pcoord)->q;
	}
}

void GSystemP::get_dqp(double *dqp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		dqp_[i++] = (*iter_pcoord)->dq;
	}
}

void GSystemP::get_ddqp(double *ddqp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		ddqp_[i++] = (*iter_pcoord)->ddq;
	}
}

void GSystemP::get_taup(double *taup_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		taup_[i++] = (*iter_pcoord)->tau;
	}
}

void GSystemP::get_DddqpDp(double *DddqpDp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		DddqpDp_[i++] = (*iter_pcoord)->DddqDp;
	}
}

void GSystemP::set_qa(double *qa_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->q = qa_[i++];
	}
}

void GSystemP::set_dqa(double *dqa_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->dq = dqa_[i++];
	}
}

void GSystemP::set_ddqa(double *ddqa_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->ddq = ddqa_[i++];
	}
}

void GSystemP::set_taua(double *taua_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesPrescribed.begin(); iter_pcoord != pCoordinatesPrescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->tau = taua_[i++];
	}
}

void GSystemP::set_qp(double *qp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->q = qp_[i++];
	}
}

void GSystemP::set_dqp(double *dqp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->dq = dqp_[i++];
	}
}

void GSystemP::set_ddqp(double *ddqp_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->ddq = ddqp_[i++];
	}
}

void GSystemP::set_taup(double *taup_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinatesUnprescribed.begin(); iter_pcoord != pCoordinatesUnprescribed.end(); iter_pcoord++) {
		(*iter_pcoord)->tau = taup_[i++];
	}
}


void GSystemP::set_q_selective(double *q_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( (*iter_pcoord)->bPrescribed ) { (*iter_pcoord)->q = q_[i]; }
	}
}

void GSystemP::set_dq_selective(double *dq_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( (*iter_pcoord)->bPrescribed ) { (*iter_pcoord)->dq = dq_[i]; }
	}
}

void GSystemP::set_ddq_selective(double *ddq_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( (*iter_pcoord)->bPrescribed ) { (*iter_pcoord)->ddq = ddq_[i]; }
	}
}

void GSystemP::set_tau_selective(double *tau_)
{
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		if ( !((*iter_pcoord)->bPrescribed) ) { (*iter_pcoord)->tau = tau_[i]; }
	}
}

bool GSystemP::buildSystem(GBody *pGround_, bool b_scan_fwd_joint_loop_)
{
	if ( !GSystem::buildSystem(pGround_, b_scan_fwd_joint_loop_) ) return false;
	_scanCoordinatesPrescribed();
	if ( !_findClosedJointLoopConstraints() ) return false; 
	return true;
}

bool GSystemP::buildSystemWith(GBody *pGround_, std::vector<GBody *> pFirstBodiesIn_, bool b_scan_fwd_joint_loop_)
{
	if ( !GSystem::buildSystemWith(pGround_, pFirstBodiesIn_, b_scan_fwd_joint_loop_) ) return false;
	_scanCoordinatesPrescribed();
	if ( !_findClosedJointLoopConstraints() ) return false; 
	return true;
}

bool GSystemP::buildSystemWithout(GBody *pGround_, std::vector<GBody *> pFirstBodiesOut_, bool b_scan_fwd_joint_loop_)
{
	if ( !GSystem::buildSystemWithout(pGround_, pFirstBodiesOut_, b_scan_fwd_joint_loop_) ) return false;
	_scanCoordinatesPrescribed();
	if ( !_findClosedJointLoopConstraints() ) return false; 
	return true;
}

void GSystemP::_scanCoordinatesPrescribed()
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;

	pCoordinatesPrescribed.clear();
	pCoordinatesUnprescribed.clear();
	idxPrescribed.clear();
	idxUnprescribed.clear();

	for (i=0, iter_pcoord = pCoordinates.begin(); iter_pcoord != pCoordinates.end(); i++, iter_pcoord++) {
		if ( (*iter_pcoord)->bPrescribed ) {
			pCoordinatesPrescribed.push_back(*iter_pcoord);
			idxPrescribed.push_back(i);
		} else {
			pCoordinatesUnprescribed.push_back(*iter_pcoord);
			idxUnprescribed.push_back(i);
		}
	}
}

bool GSystemP::_findClosedJointLoopConstraints()
{
	list<GJoint *> ploopjoints, pcutjoints;
	list<GJoint *>::iterator iter_pjoint;
	list<GJoint *>::iterator iter_pcutjoint;
	list<GCoordinate *>::iterator iter_pcoord;
	list<GConstraintJointLoop>::iterator iter_constr_jointloop;

	// find cut joints
	for (iter_pjoint = pJoints.begin(); iter_pjoint != pJoints.end(); iter_pjoint++) {
		if ( (*iter_pjoint)->isCut() ) pcutjoints.push_back(*iter_pjoint);
	}

	// set pCutCoordinates
	pCutCoordinates.clear();
	for (iter_pcutjoint = pcutjoints.begin(); iter_pcutjoint != pcutjoints.end(); iter_pcutjoint++) {
		for (iter_pcoord = (*iter_pcutjoint)->pCoordinates.begin(); iter_pcoord != (*iter_pcutjoint)->pCoordinates.end(); iter_pcoord++) {
			pCutCoordinates.push_back(*iter_pcoord);
		}
	}

	// set closedJointLoopConstraints
	closedJointLoopConstraints.resize(pcutjoints.size());	// number of joint loops = number of cut joints

	for (iter_constr_jointloop = closedJointLoopConstraints.begin(), iter_pcutjoint = pcutjoints.begin(); iter_constr_jointloop != closedJointLoopConstraints.end(); iter_constr_jointloop++, iter_pcutjoint++) {
		// find joint loop for each cut joint
		if ( !_findClosedJointLoop(*iter_pcutjoint, ploopjoints) ) return false;

		// set joint loop constraints 
		(*iter_constr_jointloop).setJoints(ploopjoints);
		(*iter_constr_jointloop).setT(SE3());	// identity

//		if ( !addConstraint(&(*iter_constr_jointloop)) ) return false;
	}

	return true;
}


bool GSystemP::_findClosedJointLoop(GJoint *pCutJoint_, list<GJoint *> &loopjoints_)
{
	if ( !_findJointLoop(pCutJoint_, loopjoints_) ) return false;
	if ( (*loopjoints_.begin())->pLeftBody != pCutJoint_->pRightBody ) return false;
	return true;
}


bool GSystemP::_findJointLoop(GJoint *pEndJoint_, list<GJoint *> &loopJoints_)
{
	GJoint *pjoint;
	list<GJoint *>::iterator iter_pjoint;

	if ( pEndJoint_ == NULL ) return false;

	loopJoints_.clear();

	pjoint = pEndJoint_;

	while (1) {

		// select a joint among the joints belongs to pjoint->pLeftBody
		for (iter_pjoint = pjoint->pLeftBody->pJoints.begin(); iter_pjoint != pjoint->pLeftBody->pJoints.end(); iter_pjoint++) {
			if ( *iter_pjoint != NULL && !(*iter_pjoint)->isCut() && (*iter_pjoint)->pRightBody == pjoint->pLeftBody ) {
				loopJoints_.push_front(*iter_pjoint);
				break;
			}
		}
		if ( iter_pjoint == pjoint->pLeftBody->pJoints.end() ) { return false; }

		// recursion will be ended when reached the end joint or the ground
		if ( (*iter_pjoint)->pLeftBody == pEndJoint_->pRightBody ) { break; }
		if ( (*iter_pjoint)->pLeftBody == pGround ) { break; }

		// repeat selecting a joint
		pjoint = *iter_pjoint;
	}

	loopJoints_.push_back(pEndJoint_);

	return true;
}