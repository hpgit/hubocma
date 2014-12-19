// -------------------------------------------------------------------------------
// Copyright (c) 2012, Junggon Kim
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met: 
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer. 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution. 
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// -------------------------------------------------------------------------------

#include <algorithm>
#include "gsystem_constrained.h"
#include "gbody.h"
#include "gjoint.h"
#include "gcoordinate.h"
#include "gconstraint.h"
#include "gconstraint_jointloop.h"

//=============================================================
//                 GSystemConstrained
//=============================================================

GSystemConstrained::GSystemConstrained()
{
	C.SetZero(0,0);
	J.SetZero(0,0);
	Ju.SetZero(0,0);
	Jv.SetZero(0,0);
	dJdt.SetZero(0,0);

	tolerance = (gReal)1E-6;
	max_iter_num = 50;
}

bool GSystemConstrained::buildSystem(GBody *pGround_)
{
	pConstraints.clear();
	pCoordinatesIndependent.clear();
	pCoordinatesDependent.clear();

	if ( !GSystem::buildSystem(pGround_) ) return false;

	if ( !_findClosedJointLoopConstraints() ) return false; 

	return true;
}


bool GSystemConstrained::addConstraint(GConstraint *pConstraint_)
{
	if ( pConstraint_ == NULL ) return false;
	if ( find(pConstraints.begin(), pConstraints.end(), pConstraint_) != pConstraints.end() ) return false;

	pConstraints.push_back(pConstraint_);

	return true;
}


bool GSystemConstrained::removeConstraint(GConstraint *pConstraint_)
{
	if ( pConstraint_ == NULL ) return false;
	if ( find(pConstraints.begin(), pConstraints.end(), pConstraint_) == pConstraints.end() ) return false;

	pConstraints.remove(pConstraint_);

	return true;
}


void GSystemConstrained::removeAllConstraints()
{
	pConstraints.clear();
}


gReal GSystemConstrained::setTolerance(gReal tolerance_)
{
	gReal old = tolerance;
	tolerance = tolerance_;
	return old;
}


int GSystemConstrained::setMaximumIterationNumber(int max_iter_num_)
{
	int old = max_iter_num;
	max_iter_num = max_iter_num_;
	return old;
}

bool GSystemConstrained::setIndependentCoordinates(std::list<GCoordinate *> pIndependentCoordinates_)
{
	std::list<GCoordinate *>::iterator iter_coord;

	// pIndependentCoordinates_ should be a subset of pCoordinates.
	for (iter_coord = pIndependentCoordinates_.begin(); iter_coord != pIndependentCoordinates_.end(); iter_coord++) {
		if ( find(pCoordinates.begin(), pCoordinates.end(), *iter_coord) == pCoordinates.end() ) return false;
	}

	// set independent coordinates
	pCoordinatesIndependent = pIndependentCoordinates_;

	// set dependent coordinates
	for (iter_coord = pCoordinates.begin(); iter_coord != pCoordinates.end(); iter_coord++) {
		if ( find(pCoordinatesIndependent.begin(), pCoordinatesIndependent.end(), *iter_coord) == pCoordinatesIndependent.end() ) {
			pCoordinatesDependent.push_back(*iter_coord);
		}
	}

	// just gReal check if the sum of independent and dependent coordinates is same to total coordinates
	if ( getNumCoordinatesIndependent() + getNumCoordinatesDependent() != getNumCoordinates() ) return false;

	return true;
}


bool GSystemConstrained::updateDependentCoordinates()
{
	if ( _getNC() == 0 ) return true;

	// update displacement
	if ( !_update_qv() ) return false;

	// update Jacobian
	if ( !_update_Ju_Jv() ) return false;

	// update velocity
	if ( !_update_dqv() ) return false;

	// update DotJacobian
	if ( !_update_dJdt() ) return false;

	// update acceleration
	if ( !_update_ddqv() ) return false;

	return true;
}

 
bool GSystemConstrained::_findClosedJointLoopConstraints()
{
	std::list<GJoint *> ploopjoints, pcutjoints;
	std::list<GJoint *>::iterator iter_pjoint;
	std::list<GJoint *>::iterator iter_pcutjoint;
	std::list<GCoordinate *>::iterator iter_pcoord;
	std::list<GConstraintJointLoop>::iterator iter_constr_jointloop;

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

		if ( !addConstraint(&(*iter_constr_jointloop)) ) return false;
	}

	return true;
}

 
bool GSystemConstrained::_findClosedJointLoop(GJoint *pCutJoint_, std::list<GJoint *> &loopjoints_)
{
	if ( !_findJointLoop(pCutJoint_, loopjoints_) ) return false;
	if ( (*loopjoints_.begin())->pLeftBody != pCutJoint_->pRightBody ) return false;
	return true;
}

 
bool GSystemConstrained::_findJointLoop(GJoint *pEndJoint_, std::list<GJoint *> &loopJoints_)
{
	GJoint *pjoint;
	std::list<GJoint *>::iterator iter_pjoint;

	if ( pEndJoint_ == NULL ) return false;

	loopJoints_.clear();

	pjoint = pEndJoint_;

	while (1) {

		for (iter_pjoint = pjoint->pLeftBody->pJoints.begin(); iter_pjoint != pjoint->pLeftBody->pJoints.end(); iter_pjoint++) {
			if ( *iter_pjoint != NULL && !(*iter_pjoint)->isCut() && (*iter_pjoint)->pRightBody == pjoint->pLeftBody ) {
				loopJoints_.push_front(*iter_pjoint);
				break;
			}
		}

		if ( iter_pjoint == pjoint->pLeftBody->pJoints.end() ) { return false; }

		if ( (*iter_pjoint)->pLeftBody == pEndJoint_->pRightBody ) { break; }
		if ( (*iter_pjoint)->pLeftBody == pGround ) { break; }
	}

	loopJoints_.push_back(pEndJoint_);

	return true;
}

int GSystemConstrained::_getNC()
{
	int num;
	std::list<GConstraint *>::iterator iter_pconstraint;

	num = 0;
	for (iter_pconstraint = pConstraints.begin(); iter_pconstraint != pConstraints.end(); iter_pconstraint++) {
		num += (*iter_pconstraint)->constrNum;
	}

	return num;
}


bool GSystemConstrained::_update_qv()
{
	// _update_qv() ---------------------------------------------------------------------------------
	// Updates displacement of dependent coordinates using Newton-Raphson iteration method.
	// ----------------------------------------------------------------------------------------------

	int i, cnt = 0;
	static RMatrix del_q;
	std::list<GCoordinate *>::iterator iter_pcoord;
	
	del_q.ReNew(getNumCoordinatesDependent(), 1);

	while (1)
	{
		if ( !_update_C() ) return false;
		if ( !_update_Jv() ) return false;
		
		if ( !SolveAxEqualB(Jv, del_q, C) ) return false;

		if ( FNorm(del_q) < tolerance ) break;

		for (iter_pcoord = pCoordinatesDependent.begin(), i=0; iter_pcoord != pCoordinatesDependent.end(); iter_pcoord++, i++) {
			(*iter_pcoord)->q -= del_q[i];
		}

		if ( cnt++ > max_iter_num ) return false;
	}

	return true;
}


bool GSystemConstrained::_update_dqv()
{
	// _update_dqv() ---------------------------------------------------------------------------------
	// Updates velocity of dependent coordinates.
	// Prerequisite: _update_Ju_Jv()
	// ----------------------------------------------------------------------------------------------

	int i;
	static RMatrix dqu, dqv;
	std::list<GCoordinate *>::iterator iter_pcoord;

	dqv.SetZero(getNumCoordinatesDependent(), 1);
	dqu.SetZero(getNumCoordinatesIndependent(), 1);

	for (iter_pcoord = pCoordinatesIndependent.begin(), i=0; iter_pcoord != pCoordinatesIndependent.end(); iter_pcoord++, i++) {
		dqu[i] = (*iter_pcoord)->dq;
	}
	
	//dqv = Jv % ( Ju * dqu );
	if ( !SolveAxEqualB(Jv, dqv, Ju * dqu) ) return false;

	for (iter_pcoord = pCoordinatesDependent.begin(), i=0; iter_pcoord != pCoordinatesDependent.end(); iter_pcoord++, i++) {
		(*iter_pcoord)->dq = -dqv[i];
	}
	
	return true;
}


bool GSystemConstrained::_update_ddqv()
{
	// _update_ddqv() ---------------------------------------------------------------------------------
	// Updates acceleration of dependent coordinates.
	// Prerequisite: _update_Ju_Jv(), _update_dJdt()
	// ----------------------------------------------------------------------------------------------

	int i;
	static RMatrix dq, ddqu, ddqv;
	std::list<GCoordinate *>::iterator iter_pcoord;

	dq.SetZero(getNumCoordinates(), 1);
	ddqv.SetZero(getNumCoordinatesDependent(), 1);
	ddqu.SetZero(getNumCoordinatesIndependent(), 1);

	for (iter_pcoord = pCoordinatesIndependent.begin(), i=0; iter_pcoord != pCoordinatesIndependent.end(); iter_pcoord++, i++) {
		ddqu[i] = (*iter_pcoord)->ddq;
	}

	for (iter_pcoord = pCoordinates.begin(), i=0; iter_pcoord != pCoordinates.end(); iter_pcoord++, i++) {
		dq[i] = (*iter_pcoord)->dq;
	}

	// ddqv = Jv % ( Ju * ddqu + dJdt * dq );
	if ( !SolveAxEqualB(Jv, ddqv, Ju * ddqu + dJdt * dq) ) return false;

	for (iter_pcoord = pCoordinatesDependent.begin(), i=0; iter_pcoord != pCoordinatesDependent.end(); iter_pcoord++, i++) {
		(*iter_pcoord)->ddq = -ddqv[i];
	}

	return true;
}


bool GSystemConstrained::_update_C()
{
	int r;
	std::list<GConstraint *>::iterator iter_pconstr;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_C() ) return false;
	}

	C.SetZero(_getNC(), 1);

	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		(*iter_pconstr)->get_C(&C[r]); //C.Push(r, 0, (*iter_pconstr)->get_C());
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}


bool GSystemConstrained::_update_J()
{
	int j, r, c;
	std::list<GConstraint *>::iterator iter_pconstr;
	std::list<GCoordinate *>::iterator iter_pcoord, iter_pcoord_all;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_J() ) return false;
	}

	J.SetZero(_getNC(), getNumCoordinates());
	
	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		for (iter_pcoord = (*iter_pconstr)->pCoordinates.begin(), j=0; iter_pcoord != (*iter_pconstr)->pCoordinates.end(); iter_pcoord++, j++) {
			for (iter_pcoord_all = pCoordinates.begin(), c=0; iter_pcoord_all != pCoordinates.end(); iter_pcoord_all++, c++) {
				if ( (*iter_pcoord_all) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&J[r+c*J.RowSize()], j); // J.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
		}
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}


bool GSystemConstrained::_update_Ju()
{
	int j, r, c;
	std::list<GConstraint *>::iterator iter_pconstr;
	std::list<GCoordinate *>::iterator iter_pcoord, iter_pcoord_u;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_J() ) return false;
	}

	Ju.SetZero(_getNC(), getNumCoordinatesIndependent());
	
	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		for (iter_pcoord = (*iter_pconstr)->pCoordinates.begin(), j=0; iter_pcoord != (*iter_pconstr)->pCoordinates.end(); iter_pcoord++, j++) {
			for (iter_pcoord_u = pCoordinatesIndependent.begin(), c=0; iter_pcoord_u != pCoordinatesIndependent.end(); iter_pcoord_u++, c++) {
				if ( (*iter_pcoord_u) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&Ju[r+c*Ju.RowSize()], j); // Ju.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
		}
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}


bool GSystemConstrained::_update_Jv()
{
	int j, r, c;
	std::list<GConstraint *>::iterator iter_pconstr;
	std::list<GCoordinate *>::iterator iter_pcoord, iter_pcoord_v;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_J() ) return false;
	}

	Jv.SetZero(_getNC(), getNumCoordinatesDependent());
	
	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		for (iter_pcoord = (*iter_pconstr)->pCoordinates.begin(), j=0; iter_pcoord != (*iter_pconstr)->pCoordinates.end(); iter_pcoord++, j++) {
			for (iter_pcoord_v = pCoordinatesDependent.begin(), c=0; iter_pcoord_v != pCoordinatesDependent.end(); iter_pcoord_v++, c++) {
				if ( (*iter_pcoord_v) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&Jv[r+c*Jv.RowSize()], j); // Jv.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
		}
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}


bool GSystemConstrained::_update_Ju_Jv()
{
	int j, r, c;
	std::list<GConstraint *>::iterator iter_pconstr;
	std::list<GCoordinate *>::iterator iter_pcoord, iter_pcoord_u, iter_pcoord_v;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_J() ) return false;
	}

	Ju.SetZero(_getNC(), getNumCoordinatesIndependent());
	Jv.SetZero(_getNC(), getNumCoordinatesDependent());
	
	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		for (iter_pcoord = (*iter_pconstr)->pCoordinates.begin(), j=0; iter_pcoord != (*iter_pconstr)->pCoordinates.end(); iter_pcoord++, j++) {
			for (iter_pcoord_u = pCoordinatesIndependent.begin(), c=0; iter_pcoord_u != pCoordinatesIndependent.end(); iter_pcoord_u++, c++) {
				if ( (*iter_pcoord_u) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&Ju[r+c*Ju.RowSize()], j); // Ju.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
			for (iter_pcoord_v = pCoordinatesDependent.begin(), c=0; iter_pcoord_v != pCoordinatesDependent.end(); iter_pcoord_v++, c++) {
				if ( (*iter_pcoord_v) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&Jv[r+c*Jv.RowSize()], j); // Jv.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
		}
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}


bool GSystemConstrained::_update_J_Ju_Jv()
{
	int j, r, c;
	std::list<GConstraint *>::iterator iter_pconstr;
	std::list<GCoordinate *>::iterator iter_pcoord, iter_pcoord_all, iter_pcoord_u, iter_pcoord_v;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_J() ) return false;
	}

	J.SetZero(_getNC(), getNumCoordinates());
	Ju.SetZero(_getNC(), getNumCoordinatesIndependent());
	Jv.SetZero(_getNC(), getNumCoordinatesDependent());
	
	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		for (iter_pcoord = (*iter_pconstr)->pCoordinates.begin(), j=0; iter_pcoord != (*iter_pconstr)->pCoordinates.end(); iter_pcoord++, j++) {
			for (iter_pcoord_all = pCoordinates.begin(), c=0; iter_pcoord_all != pCoordinates.end(); iter_pcoord_all++, c++) {
				if ( (*iter_pcoord_all) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&J[r+c*J.RowSize()], j); // J.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
			for (iter_pcoord_u = pCoordinatesIndependent.begin(), c=0; iter_pcoord_u != pCoordinatesIndependent.end(); iter_pcoord_u++, c++) {
				if ( (*iter_pcoord_u) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&Ju[r+c*Ju.RowSize()], j); // Ju.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
			for (iter_pcoord_v = pCoordinatesDependent.begin(), c=0; iter_pcoord_v != pCoordinatesDependent.end(); iter_pcoord_v++, c++) {
				if ( (*iter_pcoord_v) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_J(&Jv[r+c*Jv.RowSize()], j); // Jv.Push(r, c, (*iter_pconstr)->get_J(j));
					break;
				}
			}
		}
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}


bool GSystemConstrained::_update_dJdt()
{
	int j, r, c;
	std::list<GConstraint *>::iterator iter_pconstr;
	std::list<GCoordinate *>::iterator iter_pcoord, iter_pcoord_all;

	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		if ( !(*iter_pconstr)->update_dJdt() ) return false;
	}

	dJdt.SetZero(_getNC(), getNumCoordinates());
	
	r = 0;
	for (iter_pconstr = pConstraints.begin(); iter_pconstr != pConstraints.end(); iter_pconstr++) {
		for (iter_pcoord = (*iter_pconstr)->pCoordinates.begin(), j=0; iter_pcoord != (*iter_pconstr)->pCoordinates.end(); iter_pcoord++, j++) {
			for (iter_pcoord_all = pCoordinates.begin(), c=0; iter_pcoord_all != pCoordinates.end(); iter_pcoord_all++, c++) {
				if ( (*iter_pcoord_all) == (*iter_pcoord) ) {
					(*iter_pconstr)->get_dJdt(&dJdt[r+c*dJdt.RowSize()], j); // dJdt.Push(r, c, (*iter_pconstr)->get_dJdt(j));
					break;
				}
			}
		}
		r += (*iter_pconstr)->constrNum;
	}

	return true;
}

// calculate equivalent independent coordinates force: (tau_u, tau_v) --> tau_u_equivalent
bool GSystemConstrained::_calculateEquivalentIndependentCoordinatesForce(std::list<gReal> &tauu)
{
	int i, nu, nv;
	static RMatrix tau_u, tau_v;
	std::list<GCoordinate *>::iterator iter_pcoord;
	
	nu = getNumCoordinatesIndependent();
	nv = getNumCoordinatesDependent();

	tauu.clear();
	
	if ( nv == 0 ) {
		for (iter_pcoord = pCoordinatesIndependent.begin(); iter_pcoord != pCoordinatesIndependent.end(); iter_pcoord++) {
			tauu.push_back((*iter_pcoord)->tau);
		}
		return true;
	}

	tau_u.ReNew(nu, 1);
	tau_v.ReNew(nv, 1);
	
	for (iter_pcoord = pCoordinatesIndependent.begin(), i=0; iter_pcoord != pCoordinatesIndependent.end(); iter_pcoord++, i++) {
		tau_u[i] = (*iter_pcoord)->tau;
	}
	for (iter_pcoord = pCoordinatesDependent.begin(), i=0; iter_pcoord != pCoordinatesDependent.end(); iter_pcoord++, i++) {
		tau_v[i] = (*iter_pcoord)->tau;
	}

	// tau_u = tau_u - Ju^T Jv^{-T} tau_v
	tau_u -= Ju ^ ( Jv & tau_v );

	for (iter_pcoord = pCoordinatesIndependent.begin(), i=0; iter_pcoord != pCoordinatesIndependent.end(); iter_pcoord++, i++) {
		tauu.push_back(tau_u[i]);
	}

	return true;
}
