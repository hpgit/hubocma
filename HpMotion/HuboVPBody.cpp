#include "HuboVPBody.h"
#include "HpMotionMath.h"

HuboVPBody::HuboVPBody()
{
	grfKs = 75000.;
	grfDs = 100.;
	//grfKs = 2000.;
	//grfDs = 700.;
	//grfDs = 2 * sqrt(grfKs);
	//grfKs = 800.;
	//grfDs = 50.;
	mu = 1.;
}

HuboVPBody::~HuboVPBody()
{
	for(int i=0; i<bodies.size(); i++)
		delete bodies.at(i);
	for(int i=0; i<bodyGeoms.size(); i++)
		delete bodyGeoms.at(i);
	for(int i=0; i<joints.size(); i++)
		delete joints.at(i);
}

void HuboVPBody::ignoreVpHuboBodyCollision(vpWorld *pWorld)
{
	for(int i=0; i<bodies.size(); i++)
		for(int j=0; j<i; j++)
			pWorld->IgnoreCollision(bodies.at(i), bodies.at(j));
			
}

void HuboVPBody::ignoreVpGroundBodyCollision(vpWorld *pWorld, vpBody *pGround)
{
	for(int i=0; i<bodies.size(); i++)
		pWorld->IgnoreCollision(bodies.at(i), pGround);
}

void HuboVPBody::initBody()
{
	for(int i=0; i<bodies.size(); i++)
		delete bodies.at(i);
	bodies.clear();
	collideCheckBodies.clear();

	Hip = new vpBody;
	Torso = new vpBody;
	Head = new vpBody;
	for(int i=0; i<2; i++)
	{
		ShoulderP[i] = new vpBody;
		ShoulderR[i] = new vpBody;
		UpperArm[i] = new vpBody;
		Elbow[i] = new vpBody;
		Wrist[i] = new vpBody;
		Hand[i] = new vpBody;
		PelvisY[i] = new vpBody;
		PelvisR[i] = new vpBody;
		UpperLeg[i] = new vpBody;
		LowerLeg[i] = new vpBody;
		Ankle[i] = new vpBody;
		Foot[i] = new vpBody;
	}
	
	bodies.push_back(Hip);
	bodies.push_back(Torso);
	bodies.push_back(Head);
	collideCheckBodies.push_back(Hip);
	collideCheckBodies.push_back(Torso);
	collideCheckBodies.push_back(Head);
	for(int i=0; i<2; i++)
	{
		bodies.push_back(ShoulderP[i]);
		bodies.push_back(ShoulderR[i]);
		bodies.push_back(UpperArm[i]);
		bodies.push_back(Elbow[i]);
		bodies.push_back(Wrist[i]);
		bodies.push_back(Hand[i]);
		bodies.push_back(PelvisY[i]);
		bodies.push_back(PelvisR[i]);
		bodies.push_back(UpperLeg[i]);
		bodies.push_back(LowerLeg[i]);
		bodies.push_back(Ankle[i]);
		bodies.push_back(Foot[i]);
		collideCheckBodies.push_back(ShoulderP[i]);
		collideCheckBodies.push_back(ShoulderR[i]);
		collideCheckBodies.push_back(UpperArm[i]);
		collideCheckBodies.push_back(Elbow[i]);
		collideCheckBodies.push_back(Wrist[i]);
		collideCheckBodies.push_back(Hand[i]);
		collideCheckBodies.push_back(PelvisY[i]);
		collideCheckBodies.push_back(PelvisR[i]);
		collideCheckBodies.push_back(UpperLeg[i]);
		//collideCheckBodies.push_back(LowerLeg[i]);
		//collideCheckBodies.push_back(Ankle[i]);
		collideCheckBodies.push_back(Foot[i]);
	}
	for(int i=0; i<bodyGeoms.size(); i++)
		delete bodyGeoms.at(i);
	bodyGeoms.clear();
}

void HuboVPBody::initJoint()
{
	for(int i=0; i<joints.size(); i++)
		delete joints.at(i);
	WST = new vpRJoint;
	NKY = new vpRJoint;
	RSP = new vpRJoint;
	RSR = new vpRJoint;
	RSY = new vpRJoint;
	REB = new vpRJoint;
	RWY = new vpRJoint;
	RWP = new vpRJoint;
	RHY = new vpRJoint;
	RHR = new vpRJoint;
	RHP = new vpRJoint;
	RKN = new vpRJoint;
	RAP = new vpRJoint;
	RAR = new vpRJoint;
	LSP = new vpRJoint;
	LSR = new vpRJoint;
	LSY = new vpRJoint;
	LEB = new vpRJoint;
	LWY = new vpRJoint;
	LWP = new vpRJoint;
	LHY = new vpRJoint;
	LHR = new vpRJoint;
	LHP = new vpRJoint;
	LKN = new vpRJoint;
	LAP = new vpRJoint;
	LAR = new vpRJoint;

	this->joints.clear();

	this->joints.push_back(WST);

	this->joints.push_back(RSP);
	this->joints.push_back(RSR);
	this->joints.push_back(RSY);
	this->joints.push_back(REB);
	this->joints.push_back(RWY);
	this->joints.push_back(RWP);

	this->joints.push_back(NKY);

	this->joints.push_back(RHY);
	this->joints.push_back(RHR);
	this->joints.push_back(RHP);
	this->joints.push_back(RKN);
	this->joints.push_back(RAP);
	this->joints.push_back(RAR);

	this->joints.push_back(LHY);
	this->joints.push_back(LHR);
	this->joints.push_back(LHP);
	this->joints.push_back(LKN);
	this->joints.push_back(LAP);
	this->joints.push_back(LAR);

	this->joints.push_back(LSP);
	this->joints.push_back(LSR);
	this->joints.push_back(LSY);
	this->joints.push_back(LEB);
	this->joints.push_back(LWY);
	this->joints.push_back(LWP);

}

void HuboVPBody::setInitBodyJoint(
	vpBody *pBody, vpRJoint *pJoint, string jointName, vpBody *pParentBody, 
	double elasticity, double damping
	)
{
	Joint *huboJoint = pHuboMotion->jointMap[jointName];

	if (pParentBody != NULL)
	{
		pParentBody->SetJoint(pJoint, vectorToVec3(huboJoint->offsetFromParent-huboJoint->parent->BSpos));
		pBody->SetJoint(pJoint, -vectorToVec3(huboJoint->BSpos));
		pJoint->SetElasticity(elasticity);
		pJoint->SetDamping(damping);
		pJoint->SetAxis(vectorToVec3(huboJoint->constraintAxis));
	}

	bodyGeoms.push_back(new vpBox(vectorToVec3(huboJoint->BBsizev)));
	//pBody->AddGeometry(bodyGeoms.back(), vectorToVec3(huboJoint->BSpos));
	pBody->AddGeometry(bodyGeoms.back());
	pBody->SetInertia(
		BoxInertia(
		huboJoint->childBodyMass / huboJoint->BBvol,
		vectorToVec3(huboJoint->BBsizev/2)
		));
}

void HuboVPBody::initHybridDynamics(bool floatingBase)
{
	if (floatingBase == true)
		Hip->SetHybridDynamicsType(VP::DYNAMIC);
	else
		Hip->SetHybridDynamicsType(VP::KINEMATIC);

	for(int i=0; i<joints.size(); i++)
		joints.at(i)->SetHybridDynamicsType(VP::KINEMATIC);
}

void HuboVPBody::solveHybridDynamics()
{
	Hip->GetSystem()->HybridDynamics();	
}

void HuboVPBody::create(vpWorld *pWorld, HuboMotionData *pHuboImporter)
{
	this->pHuboMotion = pHuboImporter;
	std::map<string, Joint*> &huboJointMap = pHuboImporter->jointMap;
	//pIKSolver = new IKSolver(pHuboImporter);

	scalar Hubo_elasticity = 1;
	scalar Hubo_damping = 2;
		
	vpMaterial::GetDefaultMaterial()->SetRestitution(0.01);
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(1);
	vpMaterial::GetDefaultMaterial()->SetStaticFriction(1);

	initJoint();
	initBody();

	pWorld->AddBody(Hip);

	setInitBodyJoint(Hip, NULL, "Hip", NULL, Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Torso, WST, "WST", Hip, Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Head, NKY, "NKY", Torso, Hubo_elasticity, Hubo_damping);

	setInitBodyJoint(ShoulderP[RIGHT], RSP, "RSP", Torso, Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(ShoulderR[RIGHT], RSR, "RSR", ShoulderP[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(UpperArm[RIGHT], RSY, "RSY", ShoulderR[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Elbow[RIGHT], REB, "REB", UpperArm[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Wrist[RIGHT], RWY, "RWY", Elbow[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Hand[RIGHT], RWP, "RWP", Wrist[RIGHT], Hubo_elasticity, Hubo_damping);

	setInitBodyJoint(PelvisY[RIGHT], RHY, "RHY", Hip, Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(PelvisR[RIGHT], RHR, "RHR", PelvisY[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(UpperLeg[RIGHT], RHP, "RHP", PelvisR[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(LowerLeg[RIGHT], RKN, "RKN", UpperLeg[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Ankle[RIGHT], RAP, "RAP", LowerLeg[RIGHT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Foot[RIGHT], RAR, "RAR", Ankle[RIGHT], Hubo_elasticity, Hubo_damping);

	setInitBodyJoint(ShoulderP[LEFT], LSP, "LSP", Torso, Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(ShoulderR[LEFT], LSR, "LSR", ShoulderP[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(UpperArm[LEFT], LSY, "LSY", ShoulderR[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Elbow[LEFT], LEB, "LEB", UpperArm[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Wrist[LEFT], LWY, "LWY", Elbow[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Hand[LEFT], LWP, "LWP", Wrist[LEFT], Hubo_elasticity, Hubo_damping);

	setInitBodyJoint(PelvisY[LEFT], LHY, "LHY", Hip, Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(PelvisR[LEFT], LHR, "LHR", PelvisY[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(UpperLeg[LEFT], LHP, "LHP", PelvisR[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(LowerLeg[LEFT], LKN, "LKN", UpperLeg[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Ankle[LEFT], LAP, "LAP", LowerLeg[LEFT], Hubo_elasticity, Hubo_damping);
	setInitBodyJoint(Foot[LEFT], LAR, "LAR", Ankle[LEFT], Hubo_elasticity, Hubo_damping);

	mass = 0;
	for(int i=0; i<bodies.size(); i++)
		mass += bodies[i]->GetInertia().GetMass();

	vptohuboJointmap.clear();
	vptohuboJointmap[WST] = huboJointMap["WST"];
	vptohuboJointmap[NKY] = huboJointMap["NKY"];
	vptohuboJointmap[RSP] = huboJointMap["RSP"];
	vptohuboJointmap[RSR] = huboJointMap["RSR"];
	vptohuboJointmap[RSY] = huboJointMap["RSY"];
	vptohuboJointmap[REB] = huboJointMap["REB"];
	vptohuboJointmap[RWY] = huboJointMap["RWY"];
	vptohuboJointmap[RWP] = huboJointMap["RWP"];
	vptohuboJointmap[RHY] = huboJointMap["RHY"];
	vptohuboJointmap[RHR] = huboJointMap["RHR"];
	vptohuboJointmap[RHP] = huboJointMap["RHP"];
	vptohuboJointmap[RKN] = huboJointMap["RKN"];
	vptohuboJointmap[RAP] = huboJointMap["RAP"];
	vptohuboJointmap[RAR] = huboJointMap["RAR"];
	vptohuboJointmap[LSP] = huboJointMap["LSP"];
	vptohuboJointmap[LSR] = huboJointMap["LSR"];
	vptohuboJointmap[LSY] = huboJointMap["LSY"];
	vptohuboJointmap[LEB] = huboJointMap["LEB"];
	vptohuboJointmap[LWY] = huboJointMap["LWY"];
	vptohuboJointmap[LWP] = huboJointMap["LWP"];
	vptohuboJointmap[LHY] = huboJointMap["LHY"];
	vptohuboJointmap[LHR] = huboJointMap["LHR"];
	vptohuboJointmap[LHP] = huboJointMap["LHP"];
	vptohuboJointmap[LKN] = huboJointMap["LKN"];
	vptohuboJointmap[LAP] = huboJointMap["LAP"];
	vptohuboJointmap[LAR] = huboJointMap["LAR"];

	vpHuboJointToVpmap.clear();
	vpHuboJointToVpmap[huboJointMap["Hip"]]=NULL;
	vpHuboJointToVpmap[huboJointMap["WST"]]=WST;
	vpHuboJointToVpmap[huboJointMap["NKY"]]=NKY;
	vpHuboJointToVpmap[huboJointMap["RSP"]]=RSP;
	vpHuboJointToVpmap[huboJointMap["RSR"]]=RSR;
	vpHuboJointToVpmap[huboJointMap["RSY"]]=RSY;
	vpHuboJointToVpmap[huboJointMap["REB"]]=REB;
	vpHuboJointToVpmap[huboJointMap["RWY"]]=RWY;
	vpHuboJointToVpmap[huboJointMap["RWP"]]=RWP;
	vpHuboJointToVpmap[huboJointMap["RHY"]]=RHY;
	vpHuboJointToVpmap[huboJointMap["RHR"]]=RHR;
	vpHuboJointToVpmap[huboJointMap["RHP"]]=RHP;
	vpHuboJointToVpmap[huboJointMap["RKN"]]=RKN;
	vpHuboJointToVpmap[huboJointMap["RAP"]]=RAP;
	vpHuboJointToVpmap[huboJointMap["RAR"]]=RAR;
	vpHuboJointToVpmap[huboJointMap["LSP"]]=LSP;
	vpHuboJointToVpmap[huboJointMap["LSR"]]=LSR;
	vpHuboJointToVpmap[huboJointMap["LSY"]]=LSY;
	vpHuboJointToVpmap[huboJointMap["LEB"]]=LEB;
	vpHuboJointToVpmap[huboJointMap["LWY"]]=LWY;
	vpHuboJointToVpmap[huboJointMap["LWP"]]=LWP;
	vpHuboJointToVpmap[huboJointMap["LHY"]]=LHY;
	vpHuboJointToVpmap[huboJointMap["LHR"]]=LHR;
	vpHuboJointToVpmap[huboJointMap["LHP"]]=LHP;
	vpHuboJointToVpmap[huboJointMap["LKN"]]=LKN;
	vpHuboJointToVpmap[huboJointMap["LAP"]]=LAP;
	vpHuboJointToVpmap[huboJointMap["LAR"]]=LAR;
	
	vpBodytohuboParentJointmap.clear();
	vpBodytohuboParentJointmap[Hip] = huboJointMap["Hip"];
	vpBodytohuboParentJointmap[Torso] = huboJointMap["WST"];
	vpBodytohuboParentJointmap[Head] = huboJointMap["NKY"];

	vpBodytohuboParentJointmap[ShoulderP[RIGHT]] = huboJointMap["RSP"];
	vpBodytohuboParentJointmap[ShoulderR[RIGHT]] = huboJointMap["RSR"];
	vpBodytohuboParentJointmap[UpperArm[RIGHT]] = huboJointMap["RSY"];
	vpBodytohuboParentJointmap[Elbow[RIGHT]] = huboJointMap["REB"];
	vpBodytohuboParentJointmap[Wrist[RIGHT]] = huboJointMap["RWY"];
	vpBodytohuboParentJointmap[Hand[RIGHT]] = huboJointMap["RWP"];
	
	vpBodytohuboParentJointmap[PelvisY[RIGHT]] = huboJointMap["RHY"];
	vpBodytohuboParentJointmap[PelvisR[RIGHT]] = huboJointMap["RHR"];
	vpBodytohuboParentJointmap[UpperLeg[RIGHT]] = huboJointMap["RHP"];
	vpBodytohuboParentJointmap[LowerLeg[RIGHT]] = huboJointMap["RKN"];
	vpBodytohuboParentJointmap[Ankle[RIGHT]] = huboJointMap["RAP"];
	vpBodytohuboParentJointmap[Foot[RIGHT]] = huboJointMap["RAR"];

	vpBodytohuboParentJointmap[ShoulderP[LEFT]] = huboJointMap["LSP"];
	vpBodytohuboParentJointmap[ShoulderR[LEFT]] = huboJointMap["LSR"];
	vpBodytohuboParentJointmap[UpperArm[LEFT]] = huboJointMap["LSY"];
	vpBodytohuboParentJointmap[Elbow[LEFT]] = huboJointMap["LEB"];
	vpBodytohuboParentJointmap[Wrist[LEFT]] = huboJointMap["LWY"];
	vpBodytohuboParentJointmap[Hand[LEFT]] = huboJointMap["LWP"];
	
	vpBodytohuboParentJointmap[PelvisY[LEFT]] = huboJointMap["LHY"];
	vpBodytohuboParentJointmap[PelvisR[LEFT]] = huboJointMap["LHR"];
	vpBodytohuboParentJointmap[UpperLeg[LEFT]] = huboJointMap["LHP"];
	vpBodytohuboParentJointmap[LowerLeg[LEFT]] = huboJointMap["LKN"];
	vpBodytohuboParentJointmap[Ankle[LEFT]] = huboJointMap["LAP"];
	vpBodytohuboParentJointmap[Foot[LEFT]] = huboJointMap["LAR"];

	vpJointtoBodymap.clear();
	vpJointtoBodymap[WST] = Torso;
	vpJointtoBodymap[NKY] = Head;
	vpJointtoBodymap[RSP] = ShoulderP[RIGHT];
	vpJointtoBodymap[RSR] = ShoulderR[RIGHT];
	vpJointtoBodymap[RSY] = UpperArm[RIGHT];
	vpJointtoBodymap[REB] = Elbow[RIGHT];
	vpJointtoBodymap[RWY] = Wrist[RIGHT];
	vpJointtoBodymap[RWP] = Hand[RIGHT];
	vpJointtoBodymap[RHY] = PelvisY[RIGHT];
	vpJointtoBodymap[RHR] = PelvisR[RIGHT];
	vpJointtoBodymap[RHP] = UpperLeg[RIGHT];
	vpJointtoBodymap[RKN] = LowerLeg[RIGHT];
	vpJointtoBodymap[RAP] = Ankle[RIGHT];
	vpJointtoBodymap[RAR] = Foot[RIGHT];
	vpJointtoBodymap[LSP] = ShoulderP[LEFT];
	vpJointtoBodymap[LSR] = ShoulderR[LEFT];
	vpJointtoBodymap[LSY] = UpperArm[LEFT];
	vpJointtoBodymap[LEB] = Elbow[LEFT];
	vpJointtoBodymap[LWY] = Wrist[LEFT];
	vpJointtoBodymap[LWP] = Hand[LEFT];
	vpJointtoBodymap[LHY] = PelvisY[LEFT];
	vpJointtoBodymap[LHR] = PelvisR[LEFT];
	vpJointtoBodymap[LHP] = UpperLeg[LEFT];
	vpJointtoBodymap[LKN] = LowerLeg[LEFT];
	vpJointtoBodymap[LAP] = Ankle[LEFT];
	vpJointtoBodymap[LAR] = Foot[LEFT];

	vpBodytoParentBody.clear();
	vpBodytoParentBody[Hip]					= NULL;
	vpBodytoParentBody[Torso]				= Hip;
	vpBodytoParentBody[Head]				= Torso;
	vpBodytoParentBody[ShoulderP[RIGHT]]	= Torso;
	vpBodytoParentBody[ShoulderR[RIGHT]]	= ShoulderP[RIGHT];
	vpBodytoParentBody[UpperArm[RIGHT]]		= ShoulderR[RIGHT];
	vpBodytoParentBody[Elbow[RIGHT]]		= UpperArm[RIGHT];
	vpBodytoParentBody[Wrist[RIGHT]]		= Elbow[RIGHT];
	vpBodytoParentBody[Hand[RIGHT]]			= Wrist[RIGHT];
	vpBodytoParentBody[PelvisY[RIGHT]]		= Hip;
	vpBodytoParentBody[PelvisR[RIGHT]]		= PelvisY[RIGHT];
	vpBodytoParentBody[UpperLeg[RIGHT]]		= PelvisR[RIGHT];
	vpBodytoParentBody[LowerLeg[RIGHT]]		= UpperLeg[RIGHT];
	vpBodytoParentBody[Ankle[RIGHT]]		= LowerLeg[RIGHT];
	vpBodytoParentBody[Foot[RIGHT]]			= Ankle[RIGHT];
	vpBodytoParentBody[ShoulderP[LEFT]]		= Torso;
	vpBodytoParentBody[ShoulderR[LEFT]]		= ShoulderP[LEFT];
	vpBodytoParentBody[UpperArm[LEFT]]		= ShoulderR[LEFT];
	vpBodytoParentBody[Elbow[LEFT]]			= UpperArm[LEFT];
	vpBodytoParentBody[Wrist[LEFT]]			= Elbow[LEFT];
	vpBodytoParentBody[Hand[LEFT]]			= Wrist[LEFT];
	vpBodytoParentBody[PelvisY[LEFT]]		= Hip;
	vpBodytoParentBody[PelvisR[LEFT]]		= PelvisY[LEFT];
	vpBodytoParentBody[UpperLeg[LEFT]]		= PelvisR[LEFT];
	vpBodytoParentBody[LowerLeg[LEFT]]		= UpperLeg[LEFT];
	vpBodytoParentBody[Ankle[LEFT]]			= LowerLeg[LEFT];
	vpBodytoParentBody[Foot[LEFT]]			= Ankle[LEFT];

	vpBodytoJointmap.clear();
	vpBodytoJointmap[Hip]				= NULL;
	vpBodytoJointmap[Torso]				= WST;
	vpBodytoJointmap[Head]				= NKY;
	vpBodytoJointmap[ShoulderP[RIGHT]]	= RSP;
	vpBodytoJointmap[ShoulderR[RIGHT]]	= RSR;
	vpBodytoJointmap[UpperArm[RIGHT]]	= RSY;
	vpBodytoJointmap[Elbow[RIGHT]]		= REB;
	vpBodytoJointmap[Wrist[RIGHT]]		= RWY;
	vpBodytoJointmap[Hand[RIGHT]]		= RWP;
	vpBodytoJointmap[PelvisY[RIGHT]]	= RHY;
	vpBodytoJointmap[PelvisR[RIGHT]]	= RHR;
	vpBodytoJointmap[UpperLeg[RIGHT]]	= RHP;
	vpBodytoJointmap[LowerLeg[RIGHT]]	= RKN;
	vpBodytoJointmap[Ankle[RIGHT]]		= RAP;
	vpBodytoJointmap[Foot[RIGHT]]		= RAR;
	vpBodytoJointmap[ShoulderP[LEFT]]	= LSP;
	vpBodytoJointmap[ShoulderR[LEFT]]	= LSR;
	vpBodytoJointmap[UpperArm[LEFT]]	= LSY;
	vpBodytoJointmap[Elbow[LEFT]]		= LEB;
	vpBodytoJointmap[Wrist[LEFT]]		= LWY;
	vpBodytoJointmap[Hand[LEFT]]		= LWP;
	vpBodytoJointmap[PelvisY[LEFT]]		= LHY;
	vpBodytoJointmap[PelvisR[LEFT]]		= LHR;
	vpBodytoJointmap[UpperLeg[LEFT]]	= LHP;
	vpBodytoJointmap[LowerLeg[LEFT]]	= LKN;
	vpBodytoJointmap[Ankle[LEFT]]		= LAP;
	vpBodytoJointmap[Foot[LEFT]]		= LAR;

}

void HuboVPBody::stepAhead(vpWorld *pWorld, vpBody *pGround)
{
	std::vector<vpBody*>checkBodies;
	//checkBodies.push_back(Foot[0]);
	//checkBodies.push_back(Foot[1]);
	//calcPenaltyForce(
	//	pWorld, pGround, checkBodies, collideBodies, positions, positionsLocal, forces,
	//	grfKs, grfDs, mu
	//	);

	
	//calcPenaltyForce(
	//	pWorld, pGround, bodies, collideBodies, positions, positionsLocal, forces, 
	//	grfKs, grfDs, mu
	//	);
	//	
	//applyPenaltyForce(collideBodies,positionsLocal, forces);


	applyPenaltyForce();
	pWorld->StepAhead();
	calcPenaltyForce(pWorld, pGround, collideCheckBodies, grfKs, grfDs, mu);
	//calcPenaltyForce(pWorld, pGround, bodies, grfKs, grfDs, mu);
	//Eigen::MatrixXd J;
	//getJacobian(J, 1);
}

void HuboVPBody::drawBodyBoundingBox(vpBody *body)
{
	Vec3 v = body->GetFrame().GetPosition();
	Vec3 s = ((vpBox*)body->GetGeometry(0))->GetHalfSize();
	double data[16], data_temp[16];

	glPushMatrix();
	body->GetGeometry(0)->GetGlobalFrame().ToArray(data_temp);
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			data[4*i+j] = data_temp[4*j+i];
	glMultMatrixd(data_temp);
	glBegin(GL_LINES);
	{
		glColor3f(1.0, .5, .5);
		glVertex3d(+s[0], +s[1], +s[2]);
		glVertex3d(-s[0], +s[1], +s[2]);

		glVertex3d(+s[0], +s[1], +s[2]);
		glVertex3d(+s[0], -s[1], +s[2]);

		glVertex3d(+s[0], +s[1], +s[2]);
		glVertex3d(+s[0], +s[1], -s[2]);

		glVertex3d(-s[0], -s[1], +s[2]);
		glVertex3d(+s[0], -s[1], +s[2]);

		glVertex3d(-s[0], -s[1], +s[2]);
		glVertex3d(-s[0], +s[1], +s[2]);

		glVertex3d(-s[0], -s[1], +s[2]);
		glVertex3d(-s[0], -s[1], -s[2]);
		
		glVertex3d(-s[0], +s[1], -s[2]);
		glVertex3d(+s[0], +s[1], -s[2]);

		glVertex3d(-s[0], +s[1], -s[2]);
		glVertex3d(-s[0], -s[1], -s[2]);

		glVertex3d(-s[0], +s[1], -s[2]);
		glVertex3d(-s[0], +s[1], +s[2]);

		glVertex3d(+s[0], -s[1], -s[2]);
		glVertex3d(-s[0], -s[1], -s[2]);

		glVertex3d(+s[0], -s[1], -s[2]);
		glVertex3d(+s[0], +s[1], -s[2]);

		glVertex3d(+s[0], -s[1], -s[2]);
		glVertex3d(+s[0], -s[1], +s[2]);

		/*
		glVertex3d(v[0]+s[0], v[1]+s[1], v[2]+s[2]);
		glVertex3d(v[0]-s[0], v[1]+s[1], v[2]+s[2]);

		glVertex3d(v[0]+s[0], v[1]+s[1], v[2]+s[2]);
		glVertex3d(v[0]+s[0], v[1]-s[1], v[2]+s[2]);

		glVertex3d(v[0]+s[0], v[1]+s[1], v[2]+s[2]);
		glVertex3d(v[0]+s[0], v[1]+s[1], v[2]-s[2]);

		glVertex3d(v[0]-s[0], v[1]-s[1], v[2]+s[2]);
		glVertex3d(v[0]+s[0], v[1]-s[1], v[2]+s[2]);

		glVertex3d(v[0]-s[0], v[1]-s[1], v[2]+s[2]);
		glVertex3d(v[0]-s[0], v[1]+s[1], v[2]+s[2]);

		glVertex3d(v[0]-s[0], v[1]-s[1], v[2]+s[2]);
		glVertex3d(v[0]-s[0], v[1]-s[1], v[2]-s[2]);
		
		glVertex3d(v[0]+s[0], v[1]-s[1], v[2]+s[2]);
		glVertex3d(v[0]-s[0], v[1]-s[1], v[2]+s[2]);

		glVertex3d(v[0]+s[0], v[1]-s[1], v[2]+s[2]);
		glVertex3d(v[0]+s[0], v[1]+s[1], v[2]+s[2]);

		glVertex3d(v[0]+s[0], v[1]-s[1], v[2]+s[2]);
		glVertex3d(v[0]+s[0], v[1]-s[1], v[2]-s[2]);

		glVertex3d(v[0]-s[0], v[1]+s[1], v[2]+s[2]);
		glVertex3d(v[0]+s[0], v[1]+s[1], v[2]+s[2]);
		
		glVertex3d(v[0]-s[0], v[1]+s[1], v[2]+s[2]);
		glVertex3d(v[0]-s[0], v[1]-s[1], v[2]+s[2]);

		glVertex3d(v[0]-s[0], v[1]+s[1], v[2]+s[2]);
		glVertex3d(v[0]-s[0], v[1]+s[1], v[2]-s[2]);
		*/
	}
	glEnd();
	glPopMatrix();
}

void HuboVPBody::drawAllBoundingBox()
{
	for(int i=0; i<bodies.size(); i++)
		drawBodyBoundingBox(bodies.at(i));
}

void HuboVPBody::getAllJointTorque(Eigen::VectorXd &torque)
{
	torque.resize(joints.size());
	for (int i = 0; i < joints.size(); i++)
		torque(i) = joints.at(i)->GetTorque();
}

void HuboVPBody::applyAllJointValueVptoHubo()
{
	const int frame = pHuboMotion->getCurrentFrame();
	std::map<string, Joint*> &hubojoints = pHuboMotion->jointMap;

	Eigen::Affine3d m;
	double mat[16];
	Vector3d bspos = hubojoints["Hip"]->BSpos;
	Quaterniond q;
	Quaterniond q_position(0, bspos.x(), bspos.y(), bspos.z());
	
	Hip->GetFrame().ToArray<double>(mat);
	for(int i=0; i<16; i++)
		m(i%4, i/4) = mat[i];
	q = Quaterniond(m.rotation());
	hubojoints["Hip"]->setRotation(frame, q);
	Vector3d trans = m.translation() - (q*q_position*q.inverse()).vec();
	hubojoints["Hip"]->setTranslation(frame,  trans);

	for (int i = 0; i < joints.size(); i++)
		vptohuboJointmap[joints.at(i)]->setAngle(frame, joints.at(i)->GetAngle());
}
void HuboVPBody::applyAddAllBodyForce(Eigen::VectorXd &force)
{
	for (int i = 0; i < bodies.size(); i++)
		bodies.at(i)->ApplyGlobalForce(Vec3(force(3*i), force(3*i+1), force(3*i+2)), Vec3(0, 0, 0));
}

void HuboVPBody::applyAllJointTorque(Eigen::VectorXd &torque)
{
	for(int i=0; i<joints.size(); i++)
		joints.at(i)->SetTorque(torque(i));
}

void HuboVPBody::applyRootJointDofAccel(Vector3d &accHip, Vector3d &angAccHip)
{
//	setJointAccelerationGlobal(0, dofaccs[0].slice(0,3));
//	void setJointAccelerationGlobal( int index, const object& acc );
//		setBodyAccelerationGlobal(index, pyVec3_2_Vec3(acc), &(Inv(_boneTs[index]).GetPosition()));
//			
//		void VpModel::setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal)
//		{
//		
//		static se3 genAcc;
//		genAcc = _nodes[index]->body.GetGenAcceleration();
//		genAcc[3] = acc[0];
//		genAcc[4] = acc[1];
//		genAcc[5] = acc[2];
//		
//		_nodes[index]->body.SetGenAcceleration(genAcc);
//		}

	se3 genAcc = Hip->GetGenAcceleration();
	genAcc[3] = accHip(0);
	genAcc[4] = accHip(1);
	genAcc[5] = accHip(2);
	Hip->SetGenAcceleration(genAcc);

//	//setJointAngAccelerationGlobal(0, dofaccs[0].slice(3,6));
//	setJointAngAccelerationLocal(0, dofaccs[0].slice(3,6));
//	
//	void VpControlModel::setJointAngAccelerationLocal( int index, const object& angacc )
//	{
//		if(index == 0)
//		{
//			static se3 genAccBodyLocal, genAccJointLocal;
//			
//			genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();
//	
//			genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
//			genAccJointLocal[0] = XD(angacc[0]);
//			genAccJointLocal[1] = XD(angacc[1]);
//			genAccJointLocal[2] = XD(angacc[2]); 
//	
//			genAccBodyLocal = Ad(Inv(_boneTs[index]), genAccJointLocal);;
//			_nodes[index]->body.SetGenAccelerationLocal(genAccBodyLocal);
//		}
//		else
//			_nodes[index]->joint.SetAcceleration(pyVec3_2_Vec3(angacc));
//	}
	
	se3 genAccBodyLocal, genAccJointLocal;
	genAccBodyLocal = Hip->GetGenAccelerationLocal();
	// _boneTs : joint transformation -> body transformation
	Vec3 hipJointToBody = vectorToVec3(pHuboMotion->jointMap["Hip"]->BSpos);
	
	genAccJointLocal = InvAd(Inv(hipJointToBody), genAccBodyLocal);

	genAccJointLocal[0] = angAccHip[0];
	genAccJointLocal[1] = angAccHip[1];
	genAccJointLocal[2] = angAccHip[2];

	genAccBodyLocal = Ad(Inv(hipJointToBody), genAccJointLocal);
	Hip->SetGenAccelerationLocal(genAccBodyLocal);
}

void HuboVPBody::applyAllJointDofAccel(Eigen::VectorXd &acc)
{
	for (int i = 0; i < joints.size(); i++)
		joints.at(i)->SetAcceleration(acc(i));
}

void HuboVPBody::applyAllJointTorque(
	double tWST, double tNKY, 
	double tRSP, double tRSR, double tRSY, double tREB, double tRWY, double tRWP, 
	double tRHY, double tRHR, double tRHP, double tRKN, double tRAP, double tRAR,
	double tLSP, double tLSR, double tLSY, double tLEB, double tLWY, double tLWP,
	double tLHY, double tLHR, double tLHP, double tLKN, double tLAP, double tLAR 
	)
{
	WST->SetTorque(tWST);
	NKY->SetTorque(tNKY);
	RSP->SetTorque(tRSP);
	RSR->SetTorque(tRSR);
	RSY->SetTorque(tRSY);
	REB->SetTorque(tREB);
	RWY->SetTorque(tRWY);
	RWP->SetTorque(tRWP);
	RHY->SetTorque(tRHY);
	RHR->SetTorque(tRHR);
	RHP->SetTorque(tRHP);
	RKN->SetTorque(tRKN);
	RAP->SetTorque(tRAP);
	RAR->SetTorque(tRAR);
	LSP->SetTorque(tLSP);
	LSR->SetTorque(tLSR);
	LSY->SetTorque(tLSY);
	LEB->SetTorque(tLEB);
	LWY->SetTorque(tLWY);
	LWP->SetTorque(tLWP);
	LHY->SetTorque(tLHY);
	LHR->SetTorque(tLHR);
	LHP->SetTorque(tLHP);
	LKN->SetTorque(tLKN);
	LAP->SetTorque(tLAP);
	LAR->SetTorque(tLAR);	
}

void HuboVPBody::setInitialHuboHipFromMotion(HuboMotionData *refer)
{
	Eigen::Vector3d pos = refer->getHipJointGlobalPositionInTime(0) + Eigen::Vector3d(0, 0.03, 0);
	Eigen::Quaterniond ori = refer->getHipJointGlobalOrientationInTime(0);
	Eigen::Affine3d m;
	m.setIdentity();
	m.rotate(ori);
	//m.pretranslate(pos);
	m.pretranslate(pos);
	
	double mat[16];
	for(int i=0; i<16; i++)
		mat[i] = m(i%4, i/4);

	SE3 T(mat);
	Hip->SetFrame(T);
}
void HuboVPBody::setInitialHuboAngleFromMotion(HuboMotionData *refer)
{
	Eigen::VectorXd rAngles;
	refer->getAllAngleInHuboMotion(0, rAngles);
	for (int i = 0; i < joints.size(); i++)
		joints.at(i)->SetAngle(rAngles(i));
}
void HuboVPBody::setInitialHuboAngleRateFromMotion(HuboMotionData *refer)
{
	Eigen::VectorXd rVels;
	refer->getAllAngleRateInHuboMotion(0, rVels);
	for (int i = 0; i < joints.size(); i++)
		joints.at(i)->SetVelocity(rVels(0));
}

vpRJoint *HuboVPBody::getParentJoint(vpRJoint *joint)
{
	if (joint == NULL)
		return NULL;
	if (vptohuboJointmap[joint]->parent == NULL)
		return NULL;
	return vpHuboJointToVpmap[vptohuboJointmap[joint]->parent];
}

void HuboVPBody::getHuboLimit(int minOrMax, std::vector<double> &bound)
{
	bound.clear();
	for(int i=0; i<joints.size(); i++)
		bound.push_back(vptohuboJointmap[joints.at(i)]->constraintAngle[minOrMax]);
}

static int getFootSupVertices(
	vpBox *pBox, 
	std::vector<Vec3> &verticesLocal, 
	std::vector<Vec3> &verticesGlobal
	)
{
	Vec3 size = pBox->GetHalfSize();
	SE3 frame = pBox->GetGlobalFrame();
	const double groundHeight = 0.00;
	Vec3 temp;
	{
		temp = Vec3(size[0], -size[1], size[2]);
		if ((frame*temp)[1] < groundHeight)
		{
			verticesLocal.push_back(temp);
			verticesGlobal.push_back(frame * temp);
		}
		temp = Vec3(size[0], -size[1], -size[2]);
		if ((frame*temp)[1] < groundHeight)
		{
			verticesLocal.push_back(temp);
			verticesGlobal.push_back(frame * temp);
		}
		temp = Vec3(-size[0], -size[1], size[2]);
		if ((frame*temp)[1] < groundHeight)
		{
			verticesLocal.push_back(temp);
			verticesGlobal.push_back(frame * temp);
		}
		temp = Vec3(-size[0], -size[1], -size[2]);
		if ((frame*temp)[1] < groundHeight)
		{
			verticesLocal.push_back(temp);
			verticesGlobal.push_back(frame * temp);
		}
	}
	if (verticesGlobal.size() == 0)
		return 0;
	else return 1;
}

Vector3d HuboVPBody::getSupportRegionCenter()
{
	int num = 0;
	std::vector<Vec3>verticesLocal;
	std::vector<Vec3>verticesGlobal;
	getFootSupVertices((vpBox*)Foot[RIGHT]->GetGeometry(0), verticesLocal, verticesGlobal);
	getFootSupVertices((vpBox*)Foot[LEFT]->GetGeometry(0), verticesLocal, verticesGlobal);
	
	Vector3d sup(0, 0, 0);

	for (int i = 0; i < verticesGlobal.size(); i++)
	{
		sup += Vec3Tovector(verticesGlobal.at(i));
		num++;
	}
	if (num == 0)
		return sup;
	return sup / num;
	
	//
	/*
	Vector3d rightfoot = Vec3Tovector(Foot[RIGHT]->GetFrame().GetPosition());
	Vector3d leftfoot = Vec3Tovector(Foot[LEFT]->GetFrame().GetPosition());
	leftfoot.y() = 0;
	rightfoot.y() = 0;

	

	return  (rightfoot + leftfoot)/2 ;
	*/
}

Vector3d HuboVPBody::getVpHipRotationalJointPosition()
{
	double mat[16];
	Eigen::Affine3d m;
	m.setIdentity();

	Hip->GetFrame().ToArray(mat);
	for(int i=0; i<16; i++)
		m(i%4, i/4) = mat[i];
	Quaterniond q = Quaterniond(m.rotation());
	Vector3d bspos = pHuboMotion->jointMap["Hip"]->BSpos;
	Quaterniond q_position(0, bspos.x(), bspos.y(), bspos.z());
	return m.translation() - (q*q_position*q.inverse()).vec();
}

Vector3d HuboVPBody::getVpJointPosition(vpRJoint *joint)
{
	double mat[16];
	Eigen::Affine3d m;
	m.setIdentity();

	vpJointtoBodymap[joint]->GetFrame().ToArray(mat);
	for(int i=0; i<16; i++)
		m(i%4, i/4) = mat[i];
	Quaterniond q = Quaterniond(m.rotation());
	Vector3d bspos = vptohuboJointmap[joint]->BSpos;
	Quaterniond q_position(0, bspos.x(), bspos.y(), bspos.z());
	return m.translation() - (q*q_position*q.inverse()).vec();
}

Vector3d HuboVPBody::getVpJointAxis(vpRJoint *joint)
{
	double mat[16];
	Eigen::Affine3d m;
	m.setIdentity();

	vpJointtoBodymap[joint]->GetFrame().ToArray(mat);
	for(int i=0; i<16; i++)
		m(i%4, i/4) = mat[i];
	Quaterniond q = Quaterniond(m.rotation());
	Vector3d axis = vptohuboJointmap[joint]->constraintAxis;
	Quaterniond q_position(0, axis.x(), axis.y(), axis.z());
	return (q*q_position*q.inverse()).vec().normalized();

}

Vector3d HuboVPBody::getCOMposition()
{
	Vec3 v(.0, .0, .0);
	for(int i=0; i<bodies.size(); i++)
		v = v + bodies[i]->GetInertia().GetMass() * bodies[i]->GetFrame().GetPosition();
	
	return Vec3Tovector(v)/mass;
}

Vec3 HuboVPBody::getCOM()
{
	Vec3 v(.0, .0, .0);
	for(int i=0; i<bodies.size(); i++)
		v = v + bodies[i]->GetInertia().GetMass() * bodies[i]->GetFrame().GetPosition();
	
	return Vec3(v[0]/mass, v[1]/mass, v[2]/mass);
}

Vector3d HuboVPBody::getCOMvelocity()
{
	Vec3 v(.0, .0, .0);
	for (int i = 0; i < bodies.size(); i++)
		v = v + bodies[i]->GetInertia().GetMass() * bodies[i]->GetLinVelocity(Vec3(0, 0, 0));;

	return Vec3Tovector(v)/mass;
}

Vector3d HuboVPBody::getCOPposition(vpWorld *pWorld, vpBody *pGround)
{
	return Vec3Tovector(getCOP(pWorld, pGround));
}

Vec3 HuboVPBody::getCOMLinvel()
{
	Vec3 v(.0, .0, .0);
	for (int i = 0; i < bodies.size(); i++)
		v = v + bodies[i]->GetInertia().GetMass() * bodies[i]->GetLinVelocity(Vec3(0, 0, 0));;

	return Vec3(v[0]/mass, v[1]/mass, v[2]/mass);
}

Vec3 HuboVPBody::getCOP(vpWorld *pWorld, vpBody *pGround)
{
	//TODO:
	//calc correct COP
	//temporally, only two foot are considered

	//calcPenaltyForce(
	//	pWorld, pGround, bodies, 
	//	grfKs, grfDs, mu
	//	);

	double sumForce = 0;

	for (int j = 0; j < forces.size(); j++)
	{
		if (collideBodies.at(j) == Foot[0] || collideBodies.at(j) == Foot[1])
			sumForce += forces.at(j)[1];
	}

	Vec3 cop(0, 0, 0);

	for (int j = 0; j < forces.size(); j++)
	{
		if (collideBodies.at(j) == Foot[0] || collideBodies.at(j) == Foot[1])
			cop += forces.at(j)[1] * collideBodies.at(j)->GetFrame().GetPosition();
	}

	if (sumForce < DBL_EPSILON)
		return Vec3(0, -1, 0);
	return Vec3(cop[0]/sumForce, 0, cop[2]/sumForce);


	//temporally, only two foot are considered
	//std::vector<vpBody*>collideBodies;
	//std::vector<Vec3>positions;
	//std::vector<Vec3>positionsLocal;
	//std::vector<Vec3>forces;
	//std::vector <vpBody*> bodies;
	//bodies.push_back(Foot[0]);
	//bodies.push_back(Foot[1]);

	//calcPenaltyForce(
	//	pWorld, pGround, bodies, collideBodies, positions, positionsLocal, forces,
	//	grfKs, grfDs, mu
	//	);

	//double sumForce = 0;

	//for (int j = 0; j < forces.size(); j++)
	//	sumForce += forces.at(j)[1];

	//Vec3 cop(0, 0, 0);

	//for (int j = 0; j < positions.size(); j++)
	//	cop += forces.at(j)[1] * positions.at(j);

	//if (sumForce < DBL_EPSILON)
	//	return Vec3(0, -1, 0);
	//return Vec3(cop[0]/sumForce, 0, cop[2]/sumForce);

}
int HuboVPBody::getMainContactFoot(vpWorld *pWorld, vpBody *pGround, double &leftRate, double &rightRate)
{
	//TODO:
	//calc correct COP
	//temporally, only two foot are considered
	//calcPenaltyForce(
	//	pWorld, pGround, bodies,
	//	grfKs, grfDs, mu
	//	);

	int sumForce[2] = {0, 0};

	//TODO:
	//number of contact points or sum of force?
	for(int i=0; i<collideBodies.size(); i++)
	{
		if(collideBodies.at(i) == Foot[RIGHT])
			sumForce[RIGHT] += forces.at(i)[1];
			//sumForce[RIGHT] += 1;
		else if(collideBodies.at(i) == Foot[LEFT])
			sumForce[LEFT] += forces.at(i)[1];
			//sumForce[LEFT] += 1;
	}
	leftRate = 0;
	rightRate = 0;
	if(sumForce[LEFT]+sumForce[RIGHT] > 0)
	{
		leftRate = (double)sumForce[LEFT]/(sumForce[LEFT]+sumForce[RIGHT]);
		rightRate = (double)sumForce[RIGHT]/(sumForce[LEFT]+sumForce[RIGHT]);
	}
	if (sumForce[LEFT] >0 && sumForce[RIGHT] >0 )
		return 2;

	else if (sumForce[LEFT] > 0)
		return LEFT;
	else if (sumForce[RIGHT] > 0)
		return RIGHT;
	return -1;
}

dse3 HuboVPBody::getBodyContactForcesToEachCom(vpBody *body)
{
	int idx = -1;
	// search body
	for (int i = 0; i<collideBodies.size(); i++)
	{
		if (body == collideBodies.at(i))
		{
			idx = i;
			break;
		}
	}
	Vec3 p = body->GetFrame().GetPosition();
	

	Vec3 m(0, 0, 0), f(0, 0, 0);
	for (int i = idx; i< collideBodies.size() && body == collideBodies.at(i); i++)
	{
		m += Cross(forces.at(i), positions.at(i));
		f += forces.at(i);
	}

	return dse3(Axis(m), f);
}

Vector3d HuboVPBody::getHipDirection()
{
	return Vec3Tovector(Hip->GetFrame()*Vec3(0,0,1));
}
void HuboVPBody::getHuboHipState(Eigen::Vector3d &Pos, Eigen::Quaterniond &Ori, Eigen::Vector3d &Vel, Eigen::Vector3d &AngVel)
{
	Pos = Vec3Tovector(Hip->GetFrame().GetPosition());
	Vel = Vec3Tovector(Hip->GetLinVelocity(Vec3(0,0,0)));
	Ori = getOrientation(Hip);
	AngVel = Vec3Tovector(Hip->GetAngVelocity());
}

Quaterniond HuboVPBody::getOrientation(vpBody *pBody)
{
	double mat[16];
	Eigen::Affine3d m;
	pBody->GetFrame().ToArray(mat);
	for (int i = 0; i < 16; i++)
		m(i%4, i/4) = mat[i];

	return Quaterniond(m.rotation());
}
void HuboVPBody::getBodyState(vpBody *pBody, Eigen::Vector3d &pos, Eigen::Quaterniond &ori, Eigen::Vector3d &vel, Eigen::Vector3d &angVel)
{
	//TODO:
}
void HuboVPBody::getFootSole(int LEFTorRIGHT, Eigen::Vector3d &pos, Eigen::Quaterniond &ori)
{
	const int lr = LEFTorRIGHT;
	vpRJoint *ankleJoint = (lr == RIGHT) ? RAR : LAR;
	ori = getOrientation(Foot[lr]);
	pos = getVpJointPosition(ankleJoint)
			+ (ori*Eigen::Quaterniond(0, 0, -0.046, 0)*ori.inverse()).vec()
			;
}

void HuboVPBody::getAllAngle(Eigen::VectorXd &angles)
{
	angles.resize(26);
	
	for(int i=0; i<joints.size(); i++)
		angles(i) = joints.at(i)->GetAngle();
}

void HuboVPBody::getAllAngularVelocity(Eigen::VectorXd &angVel)
{
	angVel.resize(26);
	for(int i=0; i<joints.size(); i++)
		angVel(i) = joints.at(i)->GetVelocity();
}
void HuboVPBody::getAllAngularAcc(Eigen::VectorXd &angacc)
{
	angacc.resize(26);
	for(int i=0; i<joints.size(); i++)
		angacc(i) = joints.at(i)->GetAcceleration();
}

void HuboVPBody::setAllAngularAcc(Eigen::VectorXd &angacc)
{
	for(int i=0; i<joints.size(); i++)
		joints.at(i)->SetAcceleration(angacc(i));
}

void HuboVPBody::setAllJointTorque(Eigen::VectorXd &torque)
{
	for(int i=0; i<joints.size(); i++)
		joints.at(i)->SetTorque(torque(i));
}
void HuboVPBody::getFootPoints(int LEFTorRIGHT, std::vector<Vector3d, aligned_allocator<Vector3d> > &points)
{
	char type;
	scalar size[3];
	
	points.clear();

	Foot[LEFTorRIGHT]->GetGeometry(0)->GetShape(&type, &size[0]);
	size[0]/=2;
	size[1]/=2;
	size[2]/=2;
	
	Vec3 pos(0);
	/*
	if(LEFTorRIGHT == LEFT)
		pos = vectorToVec3(pHuboImporter->jointMap["LAR"]->BSpos);
	else
		pos = vectorToVec3(pHuboImporter->jointMap["RAR"]->BSpos);
		*/
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3(-size[0], -size[1],  size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3( size[0], -size[1],  size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3( size[0], -size[1], -size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3(-size[0], -size[1], -size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3(-size[0],  size[1],  size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3( size[0],  size[1],  size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3( size[0],  size[1], -size[2]))) );
	points.push_back(Vec3Tovector(Foot[LEFTorRIGHT]->GetFrame() * ( pos + Vec3(-size[0],  size[1], -size[2]))) );
}

Vector3d HuboVPBody::getFootPenaltyPosition(int LEFTorRIGHT, double &retmomentsum)
{
	std::vector<Vector3d, aligned_allocator<Vector3d> > footpoints;
	Vector3d p;
	char type;
	scalar boxsize[3];
	double moment[4];
	double momentsum = 0;

	getFootPoints(LEFTorRIGHT, footpoints);
	Foot[LEFTorRIGHT]->GetGeometry(0)->GetShape(&type, &boxsize[0]);
	boxsize[0]/=2;
	boxsize[1]/=2;
	boxsize[2]/=2;

	if(LEFTorRIGHT == LEFT)
		p = pHuboMotion->jointMap["LAR"]->BSpos;
	else
		p = pHuboMotion->jointMap["RAR"]->BSpos;

	for(int i=0; i<4; i++)
	{
		moment[i] = 0;
		if(footpoints[i].y() >= 0)
			continue;
		moment[i] = abs(footpoints[i].y());
		momentsum += moment[i];
	}
	retmomentsum = momentsum;
	if(momentsum < DBL_EPSILON)
		return Vector3d(0,0,0);

	p  += moment[0] * Vector3d(-boxsize[0], -boxsize[1],  boxsize[2])
		+ moment[1] * Vector3d( boxsize[0], -boxsize[1],  boxsize[2])
		+ moment[2] * Vector3d( boxsize[0], -boxsize[1], -boxsize[2])
		+ moment[3] * Vector3d(-boxsize[0], -boxsize[1], -boxsize[2]);
	p /= momentsum;

	return p;
}

static void getVertices(
	vpBox *pBox, 
	std::vector<Vec3> &verticesLocal, 
	std::vector<Vec3> &verticesGlobal
	)
{
	Vec3 size = pBox->GetHalfSize();
	SE3 frame = pBox->GetGlobalFrame();
	Vec3 temp;
	{
		temp = Vec3(size[0], size[1], size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(size[0], size[1], -size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(size[0], -size[1], size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(size[0], -size[1], -size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(-size[0], size[1], size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(-size[0], size[1], -size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(-size[0], -size[1], size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
		temp = Vec3(-size[0], -size[1], -size[2]);
		verticesLocal.push_back(temp);
		verticesGlobal.push_back(frame * temp);
	}
}

void HuboVPBody::calcPenaltyForce(
	vpWorld *pWorld, vpBody* pGround,
	std::vector<vpBody*> &checkBodies,
	double ks, double ds, double mu
	)
{
	vpBody* pBody;
	vpBox* pBox;
	Vec3 position, velocity, force, positionLocal;

	std::vector<Vec3> verticesLocal;
	std::vector<Vec3> verticesGlobal; 

	collideBodies.clear();
	positions.clear();
	positionsLocal.clear();
	forces.clear();

	for (int i = 0; i < checkBodies.size(); ++i)
	{
		pBody = checkBodies.at(i);

		for(int j=0; j < pBody->GetNumGeometry(); ++j)
		{
			pBox = (vpBox*)pBody->GetGeometry(j);

			verticesLocal.clear();
			verticesGlobal.clear();
			getVertices(pBox, verticesLocal, verticesGlobal);

			for(int k=0; k<verticesLocal.size(); ++k)
			{
				positionLocal = verticesLocal[k];
				position = verticesGlobal[k];
				velocity = pBody->GetLinVelocity(positionLocal);

				bool penentrated = _calcPenaltyForce(pWorld, pBody, position, velocity, force, ks, ds, mu);
				if(penentrated)
				{
					collideBodies.push_back(checkBodies.at(i));
					forces.push_back(force);
					positions.push_back(position);
					positionsLocal.push_back(positionLocal);
				}
			}
		}
	}
}

bool HuboVPBody::_calcPenaltyForce(
	vpWorld *pWorld,
	const vpBody *pBody, const Vec3 &position, const Vec3 &velocity, 
	Vec3 &force, double ks, double ds, double mu
	) 
{
	static Vec3 vRelVel, vNormalRelVel, vTangentialRelVel;
	static scalar normalRelVel, tangentialRelVel;
	static const Vec3 vNormal(0,1,0);
	static Vec3 vNormalForce, vFrictionForce;
	scalar normalForce=0., frictionForce=0.;
	double _lockingVel = 0.05;
	double _planeHeight = 0.0;

	vRelVel = velocity;
	normalRelVel = Inner(vRelVel, vNormal);
	vNormalRelVel = normalRelVel * vNormal;
	vTangentialRelVel = vRelVel - vNormalRelVel;
	tangentialRelVel = Norm(vTangentialRelVel);

	double timestep = pWorld->GetTimeStep();
	
	Vec3 nextPosition = position + velocity * timestep;

//	if (position[1] > _planeHeight)
//		return false;

	if (position[1] > _planeHeight && nextPosition[1] > _planeHeight)
		return false;

	else
	{
		if (position[1] > _planeHeight)
		{
			double reachTime = (_planeHeight - position[1]) / velocity[1];
			normalForce = ks * velocity[1] * (timestep - reachTime*reachTime/timestep) / 2 ;
			normalForce -= ds * velocity[1] * (1 - reachTime/timestep);
		}
		else if (nextPosition[1] > _planeHeight)
		{
			double escapeTime = (_planeHeight - position[1]) / velocity[1];
			normalForce = ks * velocity[1] * (escapeTime*escapeTime) / (2*timestep) ;
			normalForce -= ds*velocity[1] * (escapeTime/timestep);
		}
		else
		{
			// normal reaction force
			normalForce = ks*(_planeHeight - (position[1] + nextPosition[1]) / 2);
			normalForce -= ds*velocity[1];
		}

		if (normalForce < 0.) normalForce = 0.;
		vNormalForce = normalForce * vNormal;

		// tangential reaction force
		frictionForce = mu * normalForce;

		/*
		if (tangentialRelVel < _lockingVel)
			frictionForce *= tangentialRelVel / _lockingVel;
			*/

		vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);

		force = vNormalForce + vFrictionForce;

		return true;
	}
}


// @return ( bodyIDs, positions, postionLocals, forces)
void HuboVPBody::calcPenaltyForceBoxGround(
	vpWorld *pWorld, vpBody* pGround, 
	std::vector<vpBody*> &checkBodies,
	double ks, double ds, double mu
	)
{
	//bp::list bodyIDs, positions, forces, positionLocals;
	//static numeric::array O_Vec3(make_tuple(0.,0.,0.));
	vpBody* pBody;
	vpBox* pBox;
	char type;
	scalar data[3];
	Vec3 position, velocity, force, positionLocal;

	// if you want consider multiple contact objects,
	// make list for boxSize and boxFrame
	pGround->GetGeometry(0)->GetShape(&type, data);
	Vec3 boxSize(data);
	SE3 boxFrame = pGround->GetGeometry(0)->GetGlobalFrame();

	std::vector<Vec3> verticesLocal;
	std::vector<Vec3> verticesGlobal; 

	collideBodies.clear();
	positions.clear();
	positionsLocal.clear();
	forces.clear();

	for(int i=0; i<checkBodies.size(); ++i)
	{
		pBody = checkBodies.at(i);

		for(int j=0; j<pBody->GetNumGeometry(); ++j)
		{
			pBox = (vpBox*)pBody->GetGeometry(j);

			verticesGlobal.clear();
			verticesLocal.clear();
			getVertices(pBox, verticesLocal, verticesGlobal);

			for(int k=0; k<verticesLocal.size(); ++k)
			{
				positionLocal = verticesLocal[k];
				position = verticesGlobal[k];
				velocity = pBody->GetLinVelocity(positionLocal);

				bool penentrated = _calcPenaltyForceBoxGround(boxSize, boxFrame, pBody, position, velocity, force, ks, ds, mu);
				if(penentrated)
				{
					collideBodies.push_back(checkBodies.at(i));
					forces.push_back(force);
					positions.push_back(position);
					positionsLocal.push_back(positionLocal);
				}
			}
		}
	}

	return ;
}

bool HuboVPBody::_calcPenaltyForceBoxGround( 
	const Vec3& boxSize, const SE3& boxFrame, const vpBody* pBody, 
	const Vec3& position, const Vec3& velocity, 
	Vec3& force, double Ks, double Ds, double mu )
{
	static Vec3 vRelVel, vNormalRelVel, vTangentialRelVel;
	static scalar normalRelVel, tangentialRelVel;
	static const Vec3 vNormal(0,1,0);
	static Vec3 vNormalForce, vFrictionForce;
	scalar normalForce=0., frictionForce=0.;

	//
	double _lockingVel = 0.05;

	static Vec3 position_moved, velocity_moved, force_moved;
	static const Vec3 vZero(0,0,0);
	SE3 boxR;
	scalar boxHeight;

	boxHeight = boxSize[1]/2.;
	position_moved = Inv(boxFrame) * position;
	boxR = boxFrame; boxR.SetPosition(vZero);
	velocity_moved = Inv(boxR) * velocity;


//	vRelVel = velocity;
	vRelVel = velocity_moved;
	normalRelVel = Inner(vRelVel, vNormal);
	vNormalRelVel = normalRelVel * vNormal;
	vTangentialRelVel = vRelVel - vNormalRelVel;
	tangentialRelVel = Norm(vTangentialRelVel);

//	if(position[1] > _planeHeight)
	if(position_moved[1] > boxHeight || 
		position_moved[0] < -boxSize[0]/2. || position_moved[0] > boxSize[0]/2. ||
		position_moved[2] < -boxSize[2]/2. || position_moved[2] > boxSize[2]/2.)
		return false;
	else
	{
		// normal reaction force
//		normalForce = Ks*(_planeHeight - position[1]);
//		normalForce -= Ds*velocity[1];
		normalForce = Ks*(boxHeight - position_moved[1]);
		normalForce -= Ds*velocity_moved[1];
		if(normalForce<0.) normalForce = 0.;
		vNormalForce = normalForce * vNormal;
		
		// tangential reaction force
		frictionForce = mu * normalForce;

		if(tangentialRelVel < _lockingVel) 
			frictionForce *= tangentialRelVel/_lockingVel;

		vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);

		force_moved = vNormalForce + vFrictionForce;
		force = boxFrame * force_moved;

		return true;
	}
}


void HuboVPBody::applyPenaltyForce()
{
	//TODO:
	// is it right?
	// positions"Local" ?
	for(int i=0; i<collideBodies.size(); i++)
		collideBodies.at(i)->ApplyGlobalForce(forces.at(i), positionsLocal.at(i));
}
void HuboVPBody::getBodyJacobian(vpBody *body, Eigen::MatrixXd &J)
{
	// 6dof root, 26 joints
	// 6dof root = 3 dof for position, 3 dof for orientation

	const int jointssize = joints.size();

	J.resize(6, jointssize+6);
	J.setZero();

	// root translation
	J.topLeftCorner(6, 6).setIdentity();

	// root orientation
	Eigen::Vector3d r;

	for(int i=0; i < 3; i+=3)
	{
		r = Vec3Tovector(body->GetFrame().GetPosition())-getVpHipRotationalJointPosition();
		J.block(0, 3, 3, 1) = Eigen::Vector3d::UnitX().cross(r);
		J.block(0, 4, 3, 1) = Eigen::Vector3d::UnitY().cross(r);
		J.block(0, 5, 3, 1) = Eigen::Vector3d::UnitZ().cross(r);
	}

	// joint part

	Eigen::Vector3d rcom, rk, w, v; 
	// rcom : CoM of link
	// rk : position of joint which is considered
	// w : an axis of the joint 

	rcom = Vec3Tovector(body->GetFrame().GetPosition());

	for(int j=0; j<jointssize; j++)
	{
		// linear velocity part
		if( vpBodytohuboParentJointmap[body]->isDescendant(vptohuboJointmap[joints[j]]) 
			|| vpBodytoJointmap[body] == joints[j] )
		{
			rk = getVpJointPosition(joints[j]);
			w = getVpJointAxis(joints[j]);
			v = w.cross(rcom - rk);

			J.block(0, j+6, 3, 1) = v;
			J.block(3, j+6, 3, 1) = w;
		}
	}
}
void HuboVPBody::getBodyDifferentialJacobian(vpBody *body, Eigen::MatrixXd &dJ)
{
	const int jointssize = joints.size();

	dJ.resize(6, 32);
	dJ.setZero();

	Vec3 rcom = body->GetFrame().GetPosition(); 

	// root orientation
	{
		vpRJoint *parentJoint = NULL, *joint  = NULL;
		Vec3 rk, wb, v, wp, w, sumWcrossRk, rKpforSum, rKcforSum, wKforSum;
		for (int j = 0; j < 3; j++)
		{
			wb = Vec3(0, 0, 0);
			wb[j] = 1;

			wp = Hip->GetAngVelocity();
			w = Cross(wp, wb);

			rk = vectorToVec3(getVpHipRotationalJointPosition());
			sumWcrossRk = Vec3(0, 0, 0);
			
			if (body != Hip)
			{
				joint = vpBodytoJointmap[body];

				sumWcrossRk += Cross(
					vpBodytoParentBody[body]->GetAngVelocity(),
					rcom - vectorToVec3(getVpJointPosition(joint))
					);

				for (joint = vpBodytoJointmap[body], parentJoint = getParentJoint(joint);
					parentJoint != NULL;
					joint = parentJoint, parentJoint = getParentJoint(parentJoint)
					)
				{
					// <sum k=j to n> { w(k+1)<cross>P(k+1, k) }
					rKcforSum = vectorToVec3(getVpJointPosition(joint));
					rKpforSum = vectorToVec3(getVpJointPosition(parentJoint));
					sumWcrossRk += Cross(
						//parentJoint->GetVelocity() * vectorToVec3(getVpJointAxis(parentJoint)),
						vpJointtoBodymap[parentJoint]->GetAngVelocity(),
						rKcforSum - rKpforSum);
					rKpforSum = rKcforSum;
				}

				sumWcrossRk += Cross(
					Hip->GetAngVelocity()[j] * wb,
					vectorToVec3(getVpJointPosition(joint) - getVpHipRotationalJointPosition())
					);
				
			}
			else
			{
				sumWcrossRk += Cross(
					Hip->GetAngVelocity()[j] * wb,
					Hip->GetFrame().GetPosition() - vectorToVec3(getVpHipRotationalJointPosition())
					);
			}
			//	# Z(j)<cross>(<sum k=j to n>w(k+1)<cross>P(k+1, k)) + (w(j)<cross>Z(j))<cross>P(n+1, j)

			int internal=1;
			//TODO:
			//w(j) == 0 for root
			if(internal)
				v = Cross(wb, sumWcrossRk);
			else
				v = Cross(wb, sumWcrossRk)
					+ Cross(w, Hip->GetFrame().GetPosition() - vectorToVec3(getVpHipRotationalJointPosition()));

			dJ(0, 3+j) = v[0];
			dJ(1, 3+j) = v[1];
			dJ(2, 3+j) = v[2];

			// # dZ(j) = w(j)<cross>Z(j)
			// w(j) == 0 for root
			if(!internal)
			{
				dJ(3, j + 3) = w[0];
				dJ(4, j + 3) = w[1];
				dJ(5, j + 3) = w[2];
			}
		}
	}

	// other joint part
	for(int j=0; j<jointssize; j++)
	{
		Vec3 rk, wb, v, wp, w, sumWcrossRk, rKpforSum, rKcforSum, wKforSum;
		vpRJoint *parentJoint = NULL, *joint  = NULL;
		// rcom : com of link
		// rk : position of joint of basis
		// rKpforSum, rKcforSum : variables that calculating sumWcrossRk, position of each joint
		// wKforSum : variables that calculating sumWcrossRk, angular velocity of each body
		// wb : normalized angular velocity of joint : Z(j) 
		// wp : angular velocity of parent link : w(j)
		//sumWcrossRk : <sum k=j to n> {w(k+1)<cross>P(k+1, k)}
		
		// linear acceleration part
		{
			if( vpBodytohuboParentJointmap[body]->isDescendant(vptohuboJointmap[joints[j]]) 
				|| vpBodytoJointmap[body] == joints[j] )
			{
				
//				# Z(j)<cross>(<sum k=j to n>w(k+1)<cross>P(k+1, k)) + (w(j)<cross>Z(j))<cross>P(n+1, j)
//				instanteneousAcceleration_colth_dof = np.cross(axis_colth_dof, get_dP_effector_from_joint2(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
//				          + np.cross(np.cross(parentLinkAngVel_jth_joint, axis_colth_dof), effectorPositions[e]-jointPosition_jth_joint)

//				def get_dP_effector_from_joint2(jointIndex, jointPositions, linkAngVels, effectorJointMask, effectorPosition, internalJointsOnly=False):
//				    jointIndexes = jointMask_2_jointIndexesDownward(effectorJointMask, jointIndex)
//				
//				    dP = mm.v3(0,0,0)
//				    for i in range(len(jointIndexes)):
//				        index = jointIndexes[i]
//				        jointPosition = jointPositions[index]
//				        childPosition = jointPositions[jointIndexes[i+1]] if i < len(jointIndexes)-1 else effectorPosition
//				        linkAngVel = linkAngVels[index] if internalJointsOnly==False else linkAngVels[index+1]
//				        dP += np.cross(linkAngVel, childPosition - jointPosition)
//				    return dP 

//				# dZ(j) = w(j)<cross>Z(j)
				wb = vectorToVec3(getVpJointAxis(joints[j]));
				//wb = vectorToVec3(vptohuboJointmap[joints[j]]->constraintAxis);
				wb.Normalize();
				wp = vpBodytoParentBody[vpJointtoBodymap[joints[j]]]->GetAngVelocity();
				w = Cross(wp, wb);

				rk = vectorToVec3(getVpJointPosition(joints[j]));
				sumWcrossRk = Vec3(0,0,0);

				joint = vpBodytoJointmap[body];

				sumWcrossRk += Cross(
					vpBodytoParentBody[body]->GetAngVelocity(),
					//rcom - vectorToVec3(getVpJointPosition(joint))
					rcom - rk
					);

				for (joint = vpBodytoJointmap[body], parentJoint = getParentJoint(joint);
					joint != joints[j];
					joint = parentJoint, parentJoint = getParentJoint(parentJoint)
					)
				{
					// <sum k=j to n> { w(k+1)<cross>P(k+1, k) }
					rKcforSum = vectorToVec3(getVpJointPosition(joint));
					rKpforSum = vectorToVec3(getVpJointPosition(parentJoint));
					sumWcrossRk += Cross(
						vpJointtoBodymap[parentJoint]->GetAngVelocity(),
						rKcforSum - rKpforSum);
				}
				
//				# Z(j)<cross>(<sum k=j to n>w(k+1)<cross>P(k+1, k)) + (w(j)<cross>Z(j))<cross>P(n+1, j)
				v = Cross(wb, sumWcrossRk) + Cross(w, rcom-rk);

				dJ(0, j+6) = v[0];
				dJ(1, j+6) = v[1];
				dJ(2, j+6) = v[2];

				dJ(3, j+6) = w[0];
				dJ(4, j+6) = w[1];
				dJ(5, j+6) = w[2];
			}
		}
	}
}

void HuboVPBody::getFootSupJacobian(Eigen::MatrixXd &fullJ, Eigen::MatrixXd &J, int RIGHTorLEFT)
{
	/*
	std::vector<Vec3>verticesLocal;
	std::vector<Vec3>verticesGlobal;
	int contactRightFoot = getFootSupVertices((vpBox*)Foot[RIGHT]->GetGeometry(0), verticesLocal, verticesGlobal);
	int contactLeftFoot = getFootSupVertices((vpBox*)Foot[LEFT]->GetGeometry(0), verticesLocal, verticesGlobal);
	//TODO:
	int contactFootSize = contactRightFoot + contactLeftFoot;

	// 6dof root, 26 joints
	// 6dof root = 3 dof for position, 3 dof for orientation

	const int jointssize = joints.size();

	J.resize(6*contactFootSize, jointssize+6); 
	J.setZero();

	//TODO:
	if (contactFootSize == 2)
	{
		J.block(0, 0, 3, 32) = fullJ.block(eRFoot * 3, 0, 3, 32);
		J.block(3, 0, 3, 32) = fullJ.block(eLFoot * 3, 0, 3, 32);
		J.block(6, 0, 3, 32) = fullJ.block(3*bodies.size() + eRFoot * 3, 0, 3, 32);
		J.block(9, 0, 3, 32) = fullJ.block(3*bodies.size() + eLFoot * 3, 0, 3, 32);
	}
	else
	*/
	J.resize(6, 32);
	J.setZero();
	if (RIGHTorLEFT == RIGHT)
	{
		J.block(0, 0, 3, 32) = fullJ.block(eRFoot * 3, 0, 3, 32);
		J.block(3, 0, 3, 32) = fullJ.block(3*bodies.size() + eRFoot * 3, 0, 3, 32);
	}
	else if (RIGHTorLEFT == LEFT)
	{
		J.block(0, 0, 3, 32) = fullJ.block(eLFoot * 3, 0, 3, 32);
		J.block(3, 0, 3, 32) = fullJ.block(3*bodies.size() + eLFoot * 3, 0, 3, 32);
	}
}

void HuboVPBody::getDifferentialFootSupJacobian(Eigen::MatrixXd &fulldJ, Eigen::MatrixXd &dJ, int RIGHTorLEFT)
{
	/*
	std::vector<Vec3>verticesLocal;
	std::vector<Vec3>verticesGlobal;

	int contactRightFoot = getFootSupVertices((vpBox*)Foot[RIGHT]->GetGeometry(0), verticesLocal, verticesGlobal);
	int contactLeftFoot = getFootSupVertices((vpBox*)Foot[LEFT]->GetGeometry(0), verticesLocal, verticesGlobal);
	//TODO:
	int contactFootSize = contactRightFoot + contactLeftFoot;
	int footOffset = 0;
	if (contactLeftFoot == 1 && contactRightFoot == 0)
		footOffset = 1;
		*/

	//const int bodiessize = bodies.size();
	const int jointssize = joints.size();

	dJ.resize(6, 32);
	dJ.setZero();

	//TODO:
	if (RIGHTorLEFT == RIGHT)
	{
		dJ.block(0, 0, 3, 32) = fulldJ.block(eRFoot * 3, 0, 3, 32);
		dJ.block(3, 0, 3, 32) = fulldJ.block(3*bodies.size() + eRFoot * 3, 0, 3, 32);
	}
	else if (RIGHTorLEFT == LEFT)
	{
		dJ.block(0, 0, 3, 32) = fulldJ.block(eLFoot * 3, 0, 3, 32);
		dJ.block(3, 0, 3, 32) = fulldJ.block(3*bodies.size() + eLFoot * 3, 0, 3, 32);
	}
}

void HuboVPBody::getJacobian(Eigen::MatrixXd &J, int includeRoot)
{
	// 6dof root, 26 joints
	// 6dof root = 3 dof for position, 3 dof for orientation

	const int bodiessize = bodies.size();
	const int jointssize = joints.size();

	int rootOffset = 0;
	if(includeRoot == 1) rootOffset = 6;
	J.resize(6*bodiessize, jointssize+rootOffset);
	J.setZero();

	// root translation
	if(includeRoot == 1)
	{

		for(int i=0; i<3*bodiessize; i++)
		{
			J( i, i%3) = 1;
		}

		// root orientation

		{
			Vec3 x(1,0,0);
			Vec3 y(0,1,0);
			Vec3 z(0,0,1);
			Vec3 angx, angy, angz;

			Vec3 r, valongx, valongy, valongz;

			angx = x;
			angy = y;
			angz = z;

			for(int i=0; i < 3*bodiessize; i+=3)
			{
				r = bodies.at(i/3)->GetFrame().GetPosition()-vectorToVec3(getVpHipRotationalJointPosition());
				valongx = Cross(angx, r);
				valongy = Cross(angy, r);
				valongz = Cross(angz, r);
				J(i  , 3) = valongx[0];
				J(i+1, 3) = valongx[1];
				J(i+2, 3) = valongx[2];
				J(i  , 4) = valongy[0];
				J(i+1, 4) = valongy[1];
				J(i+2, 4) = valongy[2];
				J(i  , 5) = valongz[0];
				J(i+1, 5) = valongz[1];
				J(i+2, 5) = valongz[2];
			}

			for(int i=3*bodiessize; i < 6*bodiessize; i+=3)
			{
				J(i  , 3) = angx[0];
				J(i+1, 3) = angx[1];
				J(i+2, 3) = angx[2];
				J(i  , 4) = angy[0];
				J(i+1, 4) = angy[1];
				J(i+2, 4) = angy[2];
				J(i  , 5) = angz[0];
				J(i+1, 5) = angz[1];
				J(i+2, 5) = angz[2];
			}
		}
	}

	// joint part

	Vec3 rcom, rk, w, v; 
	// rcom : CoM of link
	// rk : position of joint which is considered
	// w : an axis of the joint 

	for(int j=0; j<jointssize; j++)
	{
		// linear velocity part
		for(int i=3; i < 3*bodiessize; i+=3)	
		{
			if( vpBodytohuboParentJointmap[bodies[i/3]]->isDescendant(vptohuboJointmap[joints[j]]) 
				|| vpBodytoJointmap[bodies[i/3]] == joints[j] )
			{
				rcom = bodies[i/3]->GetFrame().GetPosition();
				rk = vectorToVec3(getVpJointPosition(joints[j]));
				w = vectorToVec3(getVpJointAxis(joints[j]));
				v = Cross(w, rcom-rk);

				J(i  , j+rootOffset) = v[0];
				J(i+1, j+rootOffset) = v[1];
				J(i+2, j+rootOffset) = v[2];
				J(i  +3*bodiessize, j+rootOffset) = w[0];
				J(i+1+3*bodiessize, j+rootOffset) = w[1];
				J(i+2+3*bodiessize, j+rootOffset) = w[2];
			}
		}
	}
}

void HuboVPBody::getDifferentialJacobian(Eigen::MatrixXd &dJ, int includeRoot)
{

	const int bodiessize = bodies.size();
	const int jointssize = joints.size();

	dJ.resize(6*bodiessize, 32);
	dJ.setZero();

	// root translation
	{

	}

	// root orientation
	for(int i=0; i < 3*bodiessize; i+=3)	
	{
		vpRJoint *parentJoint = NULL, *joint  = NULL;
		Vec3 rcom, rk, wb, v, wp, w, sumWcrossRk, rKpforSum, rKcforSum, wKforSum;
		for (int j = 0; j < 3; j++)
		{
			wb = Vec3(0, 0, 0);
			wb[j] = 1;

			wp = Hip->GetAngVelocity();
			w = Cross(wp, wb);

			rcom = bodies[i / 3]->GetFrame().GetPosition();
			rk = vectorToVec3(getVpHipRotationalJointPosition());
			sumWcrossRk = Vec3(0, 0, 0);
			
			if (bodies[i / 3] != Hip)
			{
				joint = vpBodytoJointmap[bodies[i / 3]];

				sumWcrossRk += Cross(
					vpBodytoParentBody[bodies[i/3]]->GetAngVelocity(),
					rcom - vectorToVec3(getVpJointPosition(joint))
					);

				for (joint = vpBodytoJointmap[bodies[i / 3]], parentJoint = getParentJoint(joint);
					parentJoint != NULL;
					joint = parentJoint, parentJoint = getParentJoint(parentJoint)
					)
				{
					// <sum k=j to n> { w(k+1)<cross>P(k+1, k) }
					rKcforSum = vectorToVec3(getVpJointPosition(joint));
					rKpforSum = vectorToVec3(getVpJointPosition(parentJoint));
					sumWcrossRk += Cross(
						//parentJoint->GetVelocity() * vectorToVec3(getVpJointAxis(parentJoint)),
						vpJointtoBodymap[parentJoint]->GetAngVelocity(),
						rKcforSum - rKpforSum);
					rKpforSum = rKcforSum;
				}

				sumWcrossRk += Cross(
					Hip->GetAngVelocity()[j] * wb,
					vectorToVec3(getVpJointPosition(joint) - getVpHipRotationalJointPosition())
					);
				
			}
			else
			{
				sumWcrossRk += Cross(
					Hip->GetAngVelocity()[j] * wb,
					Hip->GetFrame().GetPosition() - vectorToVec3(getVpHipRotationalJointPosition())
					);
			}
			//	# Z(j)<cross>(<sum k=j to n>w(k+1)<cross>P(k+1, k)) + (w(j)<cross>Z(j))<cross>P(n+1, j)

			int internal=1;
			//TODO:
			//w(j) == 0 for root
			if(internal)
			v = Cross(wb, sumWcrossRk);
			else
			v = Cross(wb, sumWcrossRk)
				+ Cross(w, Hip->GetFrame().GetPosition() - vectorToVec3(getVpHipRotationalJointPosition()));

			dJ(i, 3+j) = v[0];
			dJ(i + 1, 3 + j) = v[1];
			dJ(i + 2, 3 + j) = v[2];

			// # dZ(j) = w(j)<cross>Z(j)
			// w(j) == 0 for root
			if(!internal){
			dJ(i    +3*bodiessize, j+6) = w[0];
			dJ(i + 1+3*bodiessize, j+6) = w[1];
			dJ(i + 2+3*bodiessize, j+6) = w[2];
			}
		}
	}

	// other joint part
	for(int j=0; j<jointssize; j++)
	{
		Vec3 rcom, rk, wb, v, wp, w, sumWcrossRk, rKpforSum, rKcforSum, wKforSum;
		vpRJoint *parentJoint = NULL, *joint  = NULL;
		// rcom : com of link
		// rk : position of joint of basis
		// rKpforSum, rKcforSum : variables that calculating sumWcrossRk, position of each joint
		// wKforSum : variables that calculating sumWcrossRk, angular velocity of each body
		// wb : normalized angular velocity of joint : Z(j) 
		// wp : angular velocity of parent link : w(j)
		//sumWcrossRk : <sum k=j to n> {w(k+1)<cross>P(k+1, k)}
		
		// linear acceleration part
		for(int i=3; i < 3*bodiessize; i+=3)	
		{
			if( vpBodytohuboParentJointmap[bodies[i/3]]->isDescendant(vptohuboJointmap[joints[j]]) 
				|| vpBodytoJointmap[bodies[i/3]] == joints[j] )
			{
				
//				# Z(j)<cross>(<sum k=j to n>w(k+1)<cross>P(k+1, k)) + (w(j)<cross>Z(j))<cross>P(n+1, j)
//				instanteneousAcceleration_colth_dof = np.cross(axis_colth_dof, get_dP_effector_from_joint2(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
//				          + np.cross(np.cross(parentLinkAngVel_jth_joint, axis_colth_dof), effectorPositions[e]-jointPosition_jth_joint)

//				def get_dP_effector_from_joint2(jointIndex, jointPositions, linkAngVels, effectorJointMask, effectorPosition, internalJointsOnly=False):
//				    jointIndexes = jointMask_2_jointIndexesDownward(effectorJointMask, jointIndex)
//				
//				    dP = mm.v3(0,0,0)
//				    for i in range(len(jointIndexes)):
//				        index = jointIndexes[i]
//				        jointPosition = jointPositions[index]
//				        childPosition = jointPositions[jointIndexes[i+1]] if i < len(jointIndexes)-1 else effectorPosition
//				        linkAngVel = linkAngVels[index] if internalJointsOnly==False else linkAngVels[index+1]
//				        dP += np.cross(linkAngVel, childPosition - jointPosition)
//				    return dP 

//				# dZ(j) = w(j)<cross>Z(j)
				wb = vectorToVec3(getVpJointAxis(joints[j]));
				//wb = vectorToVec3(vptohuboJointmap[joints[j]]->constraintAxis);
				wb.Normalize();
				wp = vpBodytoParentBody[vpJointtoBodymap[joints[j]]]->GetAngVelocity();
				w = Cross(wp, wb);

				rcom = bodies[i / 3]->GetFrame().GetPosition(); 
				rk = vectorToVec3(getVpJointPosition(joints[j]));
				sumWcrossRk = Vec3(0,0,0);

				joint = vpBodytoJointmap[bodies[i / 3]];

				sumWcrossRk += Cross(
					vpBodytoParentBody[bodies[i/3]]->GetAngVelocity(),
					//rcom - vectorToVec3(getVpJointPosition(joint))
					rcom - rk
					);

				for (joint = vpBodytoJointmap[bodies[i / 3]], parentJoint = getParentJoint(joint);
					joint != joints[j];
					joint = parentJoint, parentJoint = getParentJoint(parentJoint)
					)
				{
					// <sum k=j to n> { w(k+1)<cross>P(k+1, k) }
					rKcforSum = vectorToVec3(getVpJointPosition(joint));
					rKpforSum = vectorToVec3(getVpJointPosition(parentJoint));
					sumWcrossRk += Cross(
						vpJointtoBodymap[parentJoint]->GetAngVelocity(),
						rKcforSum - rKpforSum);
				}
				
//				# Z(j)<cross>(<sum k=j to n>w(k+1)<cross>P(k+1, k)) + (w(j)<cross>Z(j))<cross>P(n+1, j)
				v = Cross(wb, sumWcrossRk) + Cross(w, rcom-rk);

				dJ(i  , j+6) = v[0];
				dJ(i+1, j+6) = v[1];
				dJ(i+2, j+6) = v[2];

				dJ(i    +3*bodiessize, j+6) = w[0];
				dJ(i + 1+3*bodiessize, j+6) = w[1];
				dJ(i + 2+3*bodiessize, j+6) = w[2];
			}
		}
	}
}

void HuboVPBody::getJacobian2(Eigen::MatrixXd &J)
{
	const int bodysize = bodies.size();
	J.resize(6 * bodysize, joints.size() + 6);
	J.setZero();
	Eigen::MatrixXd Jbody;

	for (int i = 0; i < bodysize; i++)
	{
		getBodyJacobian(bodies.at(i), Jbody);
		J.middleRows(3 * i, 3) = Jbody.topRows(3);
		J.middleRows(3 * i+3*bodysize, 3) = Jbody.bottomRows(3);
	}
}

void HuboVPBody::getDifferentialJacobian2(Eigen::MatrixXd &dJ)
{
	const int bodysize = bodies.size();
	dJ.resize(6 * bodysize, joints.size() + 6);
	dJ.setZero();
	Eigen::MatrixXd dJbody;

	for (int i = 0; i < bodysize; i++)
	{
		getBodyDifferentialJacobian(bodies.at(i), dJbody);
		dJ.middleRows(3 * i, 3) = dJbody.topRows(3);
		dJ.middleRows(3 * i+3*bodysize, 3) = dJbody.bottomRows(3);
	}
}

void HuboVPBody::getLinkMatrix(Eigen::MatrixXd &M)
{
	// [Macchietto 2009]
	const int bodiessize = bodies.size();

	M.resize(6, 6*bodiessize); 
	M.setZero();

	double m;
	double mat[36];
	Eigen::MatrixXd inertia;
	inertia.resize(6, 6);
	Vec3 rComToLink;

	for(int i=0; i<3*bodiessize; i+=3)
	{
		inertia.setZero();
		bodies[i/3]->GetInertia().ToArray<double>(mat);
		for (int j = 0; j < 6; j++)
			for (int k = 0; k < 6; k++)
				inertia(j, k) = mat[6 * j + k];

		M.block(0, i, 3, 3) = inertia.block(3, 3, 3, 3);

		M.block(3, 3 * bodiessize + i, 3, 3) = inertia.block(0, 0, 3, 3);

		rComToLink = bodies[i / 3]->GetFrame().GetPosition() - getCOM();

		M.block(3, i, 3, 3) = inertia(3,3) * vectorToSkewMat(Vec3Tovector(rComToLink));
	}
}

void HuboVPBody::getDifferentialLinkMatrix(Eigen::MatrixXd &dM)
{
	// [Macchietto 2009]
	const int bodiessize = bodies.size();

	dM.resize(6, 6 * bodiessize);
	dM.setZero();

	double m;
	double mat[36];
	Eigen::MatrixXd inertia;
	inertia.resize(6, 6);
	Vec3 vComToLink;

	for (int i = 0; i < 3 * bodiessize; i += 3)
	{
		inertia.setZero();
		bodies[i / 3]->GetInertia().ToArray<double>(mat);
		for (int j = 0; j < 6; j++)
			for (int k = 0; k < 6; k++)
				inertia(j, k) = mat[6 * j + k];

		dM.block(3, 3 * bodiessize + i, 3, 3) =
			vectorToSkewMat(Vec3Tovector(bodies[i / 3]->GetAngVelocity())) * inertia.block(0, 0, 3, 3);

		vComToLink = bodies[i / 3]->GetLinVelocity(Vec3(0, 0, 0)) - getCOMLinvel();

		dM.block(3, i, 3, 3) = inertia(3,3)*vectorToSkewMat(Vec3Tovector(vComToLink));

	}
}

void HuboVPBody::getSingleFootRootJacobian(Eigen::MatrixXd &J, int isLEFT)
{
	// TODO:
	// assume that root Foot does not move
	// 26 joints
	// root = single foot
	// joint order of dof is same

	const int bodiessize = bodies.size();
	const int jointssize = joints.size();

	J.resize(6*bodiessize, jointssize);
	J.setZero();

	int junctionJoint = isLEFT ? eLHY : eRHY;

	Vec3 rcom, rk, w, v;
	// rcom : CoM of link
	// rk : position of joint which is considered
	// w : an axis of the joint

	for(int j=0; j<jointssize; j++)
	{
		// linear velocity part
		for(int i=0; i < 3*bodiessize; i+=3)
		{
			if(vptohuboJointmap[joints.at(j)]
			   ->isDescendant( vptohuboJointmap[joints.at(junctionJoint)])
				|| joints.at(junctionJoint) == joints.at(j)
			   )
			{
				// if joint is contained in that leg
				// order of hierarchy changes
				if( !(vpBodytohuboParentJointmap[bodies[i/3]]->isDescendant(vptohuboJointmap[joints[j]])
					|| vpBodytoJointmap[bodies[i/3]] == joints[j]) )
				{
					rcom = bodies[i/3]->GetFrame().GetPosition();
					rk = vectorToVec3(getVpJointPosition(joints[j]));
					w = vectorToVec3(getVpJointAxis(joints[j]));
					v = Cross(w, rcom-rk);

					J(i  , j) = v[0];
					J(i+1, j) = v[1];
					J(i+2, j) = v[2];
					J(i  +3*bodiessize, j) = w[0];
					J(i+1+3*bodiessize, j) = w[1];
					J(i+2+3*bodiessize, j) = w[2];
				}
			}
			else
			{
				// otherwise
				// order of hierarchy does not change
				if( vpBodytohuboParentJointmap[bodies[i/3]]->isDescendant(vptohuboJointmap[joints[j]])
					|| vpBodytoJointmap[bodies[i/3]] == joints[j] )
				{
					rcom = bodies[i/3]->GetFrame().GetPosition();
					rk = vectorToVec3(getVpJointPosition(joints[j]));
					w = vectorToVec3(getVpJointAxis(joints[j]));
					v = Cross(w, rcom-rk);

					J(i  , j) = v[0];
					J(i+1, j) = v[1];
					J(i+2, j) = v[2];
					J(i  +3*bodiessize, j) = w[0];
					J(i+1+3*bodiessize, j) = w[1];
					J(i+2+3*bodiessize, j) = w[2];
				}
			}
		}
	}
}

void HuboVPBody::getSingleFootRootToHipJacobian(Eigen::MatrixXd &J, int isLEFT)
{
	// TODO:
	// assume that root Foot does not move
	// 26 joints
	// root = single foot
	// joint order of dof is same

	const int bodiessize = bodies.size();
	const int jointssize = joints.size();

	J.resize(6, jointssize);
	J.setZero();

	int junctionJoint = isLEFT ? eLHY : eRHY;

	Vec3 rcom, rk, w, v;
	// rcom : CoM of hip link
	// rk : position of joint which is considered
	// w : an axis of the joint
	rcom = Hip->GetFrame().GetPosition();

	for(int j=0; j<jointssize; j++)
	{
		// linear velocity part
		for(int i=0; i < 3; i+=3)
		{
			if(vptohuboJointmap[joints.at(j)]
			   ->isDescendant( vptohuboJointmap[joints.at(junctionJoint)])
				|| joints.at(junctionJoint) == joints.at(j)
			   )
			{
				// if joint is contained in that leg
				// order of hierarchy changes
				if( !(vpBodytohuboParentJointmap[bodies[i/3]]->isDescendant(vptohuboJointmap[joints[j]])
					|| vpBodytoJointmap[bodies[i/3]] == joints[j]) )
				{
					rk = vectorToVec3(getVpJointPosition(joints[j]));
					w = vectorToVec3(getVpJointAxis(joints[j]));
					v = Cross(w, rcom-rk);

					J(i  , j) = v[0];
					J(i+1, j) = v[1];
					J(i+2, j) = v[2];
					J(i  +3, j) = w[0];
					J(i+1+3, j) = w[1];
					J(i+2+3, j) = w[2];
				}
			}
		}
	}
}

void HuboVPBody::getEquationsOfMotion(vpWorld *world, Eigen::MatrixXd &M, Eigen::VectorXd &b)
{
	//RMatrix ddq = get_ddq(), tau = get_tau(); // save current ddq and tau
	//int n = getNumCoordinates();
	//M.ReNew(n,n);
	//set_ddq(Zeros(n,1));
	//GSystem::calcInverseDynamics();
	//b = get_tau();
	//for (int i=0; i<n; i++) {
	//	RMatrix unit = Zeros(n,1);
	//	unit[i] = 1;
	//	set_ddq(unit);
	//	GSystem::calcInverseDynamics();
	//	get_tau(&M[i*n]);
	//	for (int j=0; j<n; j++) {
	//		M[i*n+j] -= b[j];
	//	}
	//}
	//set_ddq(ddq); set_tau(tau); // restore ddq and tau


	// M * ddq + b = tau
	// ddq^T = [rootLin^T rootAng^T joints^T]^T

	Eigen::VectorXd ddq, tau;
	//save current ddq and tau
	getAllAngularAcc(ddq);
	getAllJointTorque(tau);
	se3 hipAcc = Hip->GetGenAcceleration();
	dse3 hipTorque = Hip->GetForce();


	int n = 6+joints.size();
	M.resize(n, n);
	M.setZero();
	b.resize(n);
	Eigen::VectorXd ddq_tmp;
	ddq_tmp.resize(joints.size());
	ddq_tmp.setZero();

	se3 zero_se3(0.0);
	Hip->SetGenAcceleration(zero_se3);
	setAllAngularAcc(ddq_tmp);
	Hip->GetSystem()->InverseDynamics();

	Eigen::VectorXd b_tmp;
	dse3 hipTorque_tmp = Hip->GetForce();
	getAllJointTorque(b_tmp);
	b.tail(joints.size()) = b_tmp;
	{
		b(0) = hipTorque_tmp[3];
		b(1) = hipTorque_tmp[4];
		b(2) = hipTorque_tmp[5];
		b(3) = hipTorque_tmp[0];
		b(4) = hipTorque_tmp[1];
		b(5) = hipTorque_tmp[2];
	}

	Eigen::VectorXd unit;
	unit.resize(n-6);
	for(int i=0; i<n; i++)
	{
		unit.setZero();
		se3 se3_tmp(0.0);
		if (i < 3)
		{
			se3_tmp[i+3] = 1.0;
		}
		else if(i<6)
		{
			se3_tmp[i-3] = 1.0;
		}
		else
			unit(i - 6) = 1;

		Hip->SetGenAcceleration(se3_tmp);
		setAllAngularAcc(unit);

		Hip->GetSystem()->InverseDynamics();

		getAllJointTorque(b_tmp);
		hipTorque_tmp = Hip->GetForce();
		for (int j = 0; j < 3; j++)
			M(j, i) = hipTorque_tmp[j+3] - b(j);
		for (int j = 3; j < 6; j++)
			M(j, i) = hipTorque_tmp[j-3] - b(j);
		M.col(i).tail(n - 6) = b_tmp - b.tail(n - 6);
	}

	// restore ddq and tau
	Vec3 zero_Vec3(0.0);
	Hip->SetGenAcceleration(hipAcc);
	Hip->ResetForce();
	Hip->ApplyGlobalForce(hipTorque, zero_Vec3);
	setAllAngularAcc(ddq);
	setAllJointTorque(tau);
}
