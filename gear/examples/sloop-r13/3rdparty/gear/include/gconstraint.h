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

//================================================================================
//         GConstraint: base class for constraints
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _GEAR_CONSTRAINT_
#define _GEAR_CONSTRAINT_

#include <vector>
#include "gelement.h"
#include "gcoordinate.h"
#include "liegroup.h"
#include "rmatrix3j.h"



//=============================================================
//                 GConstraint
//=============================================================
class GConstraint: public GElementWithCoordinates
{
public:
	int constrNum;		// the number of constraints

	RMatrix C;			// C = C(q,t) where q = pCoordinates[]->q, size(C) = (m,1)
	RMatrix J;			// J = dC/dq, size(J) = (m,n)
	RMatrix dJdt;		// dJdt = dJ/dt, size(dJdt) = (m,n)
						// where m = constrNum, n = size(pCoordinates)

public:
	GConstraint();
	~GConstraint() {}

public:
	int getNumConstraints() { return constrNum; }

	void get_C(gReal *re);

	void get_J(gReal *re);
	void get_J(gReal *re, int i);
	void get_J(gReal *re, GCoordinate *pcoord);
	void get_J(gReal *re, std::vector<int> indices);
	void get_J(gReal *re, std::vector<GCoordinate *> pcoords);

	void get_dJdt(gReal *re);
	void get_dJdt(gReal *re, int i);
	void get_dJdt(gReal *re, GCoordinate *pcoord);
	void get_dJdt(gReal *re, std::vector<int> indices);
	void get_dJdt(gReal *re, std::vector<GCoordinate *> pcoords);

	const gReal * getPtr_C();
	
	const gReal * getPtr_J();
	const gReal * getPtr_J(int i);
	const gReal * getPtr_J(GCoordinate *pcoord);

	const gReal * getPtr_dJdt();
	const gReal * getPtr_dJdt(int i);
	const gReal * getPtr_dJdt(GCoordinate *pcoord);

	RMatrix get_C();

	RMatrix get_J();
	RMatrix get_J(int i);
	RMatrix get_J(GCoordinate *pcoord);
	RMatrix get_J(std::vector<int> i);
	RMatrix get_J(std::vector<GCoordinate*> pcoord);
	
	RMatrix get_dJdt();
	RMatrix get_dJdt(int i);
	RMatrix get_dJdt(GCoordinate *pcoord);
	RMatrix get_dJdt(std::vector<int> i);
	RMatrix get_dJdt(std::vector<GCoordinate*> pcoord);

public: // virtual functions
	virtual bool update_C() = 0;
	virtual bool update_J() = 0;
	virtual bool update_dJdt() = 0;
};

inline void GConstraint::get_C(gReal *re) { matSet(re, C.GetPtr(), constrNum); }

inline void GConstraint::get_J(gReal *re) { matSet(re, J.GetPtr(), constrNum*getNumCoordinates()); }
inline void GConstraint::get_J(gReal *re, int i) { if ( i>=0 && i<getNumCoordinates()) { matSet(re, &J[constrNum*i], constrNum); } else { matSet_zero(re, constrNum); } }
inline void GConstraint::get_J(gReal *re, GCoordinate *pcoord) { get_J(re, getIndexOfCoordinate(pcoord)); }
inline void GConstraint::get_J(gReal *re, std::vector<int> indices) { for (int i=0; i<(int)indices.size(); i++) { get_J(&re[constrNum*i], indices[i]); } }
inline void GConstraint::get_J(gReal *re, std::vector<GCoordinate *> pcoords) { for (int i=0; i<(int)pcoords.size(); i++) { get_J(&re[constrNum*i], pcoords[i]); } }

inline void GConstraint::get_dJdt(gReal *re) { matSet(re, dJdt.GetPtr(), constrNum*getNumCoordinates()); }
inline void GConstraint::get_dJdt(gReal *re, int i) { if ( i>=0 && i<getNumCoordinates()) { matSet(re, &dJdt[constrNum*i], constrNum); } else { matSet_zero(re, constrNum); } }
inline void GConstraint::get_dJdt(gReal *re, GCoordinate *pcoord) { get_dJdt(re, getIndexOfCoordinate(pcoord)); }
inline void GConstraint::get_dJdt(gReal *re, std::vector<int> indices) { for (int i=0; i<(int)indices.size(); i++) { get_dJdt(&re[constrNum*i], indices[i]); } }
inline void GConstraint::get_dJdt(gReal *re, std::vector<GCoordinate *> pcoords) { for (int i=0; i<(int)pcoords.size(); i++) { get_dJdt(&re[constrNum*i], pcoords[i]); } }

inline const gReal * GConstraint::getPtr_C() { return C.GetPtr(); }

inline const gReal * GConstraint::getPtr_J() { return J.GetPtr(); }
inline const gReal * GConstraint::getPtr_J(int i) { if ( i>=0 && i<getNumCoordinates()) { return &J[constrNum*i]; } else { return NULL; } }
inline const gReal * GConstraint::getPtr_J(GCoordinate *pcoord) { return getPtr_J(getIndexOfCoordinate(pcoord)); }

inline const gReal * GConstraint::getPtr_dJdt() { return dJdt.GetPtr(); }
inline const gReal * GConstraint::getPtr_dJdt(int i) { if ( i>=0 && i<getNumCoordinates()) { return &dJdt[constrNum*i]; } else { return NULL; } }
inline const gReal * GConstraint::getPtr_dJdt(GCoordinate *pcoord) { return getPtr_dJdt(getIndexOfCoordinate(pcoord)); }

inline RMatrix GConstraint::get_C() { return C; }

inline RMatrix GConstraint::get_J() { return J; }
inline RMatrix GConstraint::get_J(int i) { if ( i>=0 && i<getNumCoordinates()) { return RMatrix(constrNum, 1, &J[constrNum*i]); } else { return Zeros(constrNum, 1); } }
inline RMatrix GConstraint::get_J(GCoordinate *pcoord) { return get_J(getIndexOfCoordinate(pcoord)); }

inline RMatrix GConstraint::get_dJdt() { return dJdt; }
inline RMatrix GConstraint::get_dJdt(int i) { if ( i>=0 && i<getNumCoordinates()) { return RMatrix(constrNum, 1, &dJdt[constrNum*i]); } else { return Zeros(constrNum, 1); } }
inline RMatrix GConstraint::get_dJdt(GCoordinate *pcoord) { return get_dJdt(getIndexOfCoordinate(pcoord)); }


#endif

