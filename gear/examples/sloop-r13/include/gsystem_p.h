//================================================================================
//         GEOMETRIC MULTIBODY SYSTEM TEMPLATE
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GMBS_MULTI_BODY_SYSTEM_P_
#define _GMBS_MULTI_BODY_SYSTEM_P_

#include <list>
#include <vector>
#include "gear.h"


//=============================================================
//                 GSystemP
//=============================================================
class GSystemP: public GSystem
{
public:	
	// prescribed/unprescribed coordinates
	std::list<GCoordinate *>	pCoordinatesPrescribed, pCoordinatesUnprescribed;
	std::list<int>				idxPrescribed, idxUnprescribed;		
											// pCoordinatesPrescribed[] = collection of all prescribed coordinates
											// pCoordinatesUnprescribed[] = collection of all unprescribed coordinates
											// (pCoordinatesPrescribed[], pCoordinatesUnprescribed[]) = pCoordinates[]
											// idxPrescribed = indexes of pCoordinatesPrescribed in pCoordinates
											// idxUnprescribed = indexes of pCoordinatesUnprescribed in pCoordinates

	// joint loop constraints
	std::list<GConstraintJointLoop> closedJointLoopConstraints;	// embedded closed joint-loop constraints
	std::list<GCoordinate *> pCutCoordinates;					// pointer to coordinates of cut-joints

public:
	GSystemP() {}
	~GSystemP() {}

public:
	virtual bool buildSystem(GBody *pGround_, bool b_scan_fwd_joint_loop_ = true);
	virtual bool buildSystemWith(GBody *pGround_, std::vector<GBody *> pFirstBodiesIn_, bool b_scan_fwd_joint_loop_ = true);
	virtual bool buildSystemWithout(GBody *pGround_, std::vector<GBody *> pFirstBodiesOut_, bool b_scan_fwd_joint_loop_ = true);

	// sub-functions for buildSystem()
	bool _findClosedJointLoopConstraints();
	bool _findClosedJointLoop(GJoint *pCutJoint_, std::list<GJoint *> &loopjoints_);
	bool _findJointLoop(GJoint *pEndJoint_, std::list<GJoint *> &loopJoints_);

	int getNumCoordinatesPrescribed() { return int(pCoordinatesPrescribed.size()); }
	int getNumCoordinatesUnprescribed() { return int(pCoordinatesUnprescribed.size()); }

public:
	void get_qa(double *qa_);
	void get_dqa(double *dqa_);
	void get_ddqa(double *ddqa_);
	void get_taua(double *taua_);
	void get_DtauaDp(double *DtauaDp_);

	void get_qp(double *qp_);
	void get_dqp(double *dqp_);
	void get_ddqp(double *ddqp_);
	void get_taup(double *taup_);
	void get_DddqpDp(double *DddqpDp_);

	void set_qa(double *qa_);
	void set_dqa(double *dqa_);
	void set_ddqa(double *ddqa_);
	void set_taua(double *taua_);

	void set_qp(double *qp_);
	void set_dqp(double *dqp_);
	void set_ddqp(double *ddqp_);
	void set_taup(double *taup_);

	RMatrix get_qa() { RMatrix re(getNumCoordinatesPrescribed(), 1); get_qa(re.GetPtr()); return re; }
	RMatrix get_dqa() { RMatrix re(getNumCoordinatesPrescribed(), 1); get_dqa(re.GetPtr()); return re; }
	RMatrix get_ddqa() { RMatrix re(getNumCoordinatesPrescribed(), 1); get_ddqa(re.GetPtr()); return re; }
	RMatrix get_taua() { RMatrix re(getNumCoordinatesPrescribed(), 1); get_taua(re.GetPtr()); return re; }
	RMatrix get_DtauaDp() { RMatrix re(getNumCoordinatesPrescribed(), 1); get_DtauaDp(re.GetPtr()); return re; }

	RMatrix get_qp() { RMatrix re(getNumCoordinatesUnprescribed(), 1); get_qp(re.GetPtr()); return re; }
	RMatrix get_dqp() { RMatrix re(getNumCoordinatesUnprescribed(), 1); get_dqp(re.GetPtr()); return re; }
	RMatrix get_ddqp() { RMatrix re(getNumCoordinatesUnprescribed(), 1); get_ddqp(re.GetPtr()); return re; }
	RMatrix get_taup() { RMatrix re(getNumCoordinatesUnprescribed(), 1); get_taup(re.GetPtr()); return re; }
	RMatrix get_DddqpDp() { RMatrix re(getNumCoordinatesUnprescribed(), 1); get_DddqpDp(re.GetPtr()); return re; }

	bool set_qa(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesPrescribed() ) return false; set_qa(in_.GetPtr()); return true; }
	bool set_dqa(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesPrescribed() ) return false; set_dqa(in_.GetPtr()); return true; }
	bool set_ddqa(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesPrescribed() ) return false; set_ddqa(in_.GetPtr()); return true; }
	bool set_taua(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesPrescribed() ) return false; set_taua(in_.GetPtr()); return true; }

	bool set_qp(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesUnprescribed() ) return false; set_qp(in_.GetPtr()); return true; }
	bool set_dqp(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesUnprescribed() ) return false; set_dqp(in_.GetPtr()); return true; }
	bool set_ddqp(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesUnprescribed() ) return false; set_ddqp(in_.GetPtr()); return true; }
	bool set_taup(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinatesUnprescribed() ) return false; set_taup(in_.GetPtr()); return true; }

	void set_q_selective(double *q_);		// if pCoordinates[i]->bPrescribed == true, then pCoordinates[i]->q = q_[i]. size(q_) = pCoordinates.size()
	void set_dq_selective(double *dq_);		// if pCoordinates[i]->bPrescribed == true, then pCoordinates[i]->dq = dq_[i]. size(dq_) = pCoordinates.size()
	void set_ddq_selective(double *ddq_);	// if pCoordinates[i]->bPrescribed == true, then pCoordinates[i]->ddq = ddq_[i]. size(ddq_) = pCoordinates.size()
	void set_tau_selective(double *tau_);	// if pCoordinates[i]->bPrescribed == false, then pCoordinates[i]->tau = tau_[i]. size(tau_) = pCoordinates.size()

	bool set_q_selective(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_q_selective(in_.GetPtr()); return true; }
	bool set_dq_selective(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_dq_selective(in_.GetPtr()); return true; }
	bool set_ddq_selective(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_ddq_selective(in_.GetPtr()); return true; }
	bool set_tau_selective(RMatrix &in_) { if ( in_.RowSize() * in_.ColSize() != getNumCoordinates() ) return false; set_tau_selective(in_.GetPtr()); return true; }

public:	
	// sub-functions for buildSystem()
	void _scanCoordinatesPrescribed();			// scan pCoordinatesPrescribed and pCoordinatesUnprescribed from pCoordinates and update idxPrescribed and idxUnprescribed.

};



#endif

