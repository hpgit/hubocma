#include "BVHMotionData.h"
#include <fstream>

int BVHMotionData::import(char* _filename)
{
	std::ifstream in;
	in.open(_filename);

	//importing skeleton
	int id=0, numChannels=0;
	string str="";
	double d[3]={.0f,.0f,.0f};

	//importing motion
	double temp;
	std::vector<double> dof;
	Vector3d offset(0,0,0);
	Vector3d translate(0,0,0);
	Quaterniond rotation(1,0,0,0);


	std::vector<Joint *>jointStack;
	Joint *joint = NULL;
	
	//import skeleton information
	do {

		in >> str;

		if( str == "{" )
			continue;

		if( str == "}" )
			jointStack.pop_back();

		if( str == "HIERARCHY" )
		{
			// "ROOT"
			in >> str;

			// ROOT name
			in >> str;
			
			// make ROOT joint
			jointStack.push_back( rootJoint = (joint = new Joint(str)) );
			jointNames.push_back(str);
			jointMap[str] = jointStack.back();

		}

		if( str == "OFFSET" )
		{
			// OFFSET x y z
			in >> d[0] >> d[1] >> d[2];
			Vector3d tempoffset(d[0]/25, d[1]/25, d[2]/25);
			jointStack.back()->setOffset(tempoffset);
		}
		
		if( str == "CHANNELS" )
		{
			in >> numChannels;
			for(int i=0; i<numChannels; i++)
			{
				in >> str;
				if(str == "Xposition")	
					jointStack.back()->dof.push_back(Xt);
				else if(str == "Yposition")	
					jointStack.back()->dof.push_back(Yt);
				else if(str == "Zposition")	
					jointStack.back()->dof.push_back(Zt);
				else if(str == "Xrotation")	
					jointStack.back()->dof.push_back(Xr);
				else if(str == "Yrotation")	
					jointStack.back()->dof.push_back(Yr);
				else if(str == "Zrotation")	
					jointStack.back()->dof.push_back(Zr);
			}
		}

		if( str == "JOINT" )
		{
			// JOINT name
			in >> str;

			// make new joint
			joint = new Joint(str);
			jointStack.back()->setChild(joint);
			jointStack.push_back( joint );
			jointNames.push_back(str);
			jointMap[str] = jointStack.back();

		}

		if( str == "End" )
		{
			// "Site"
			in >> str;
			
			// make new end effector ( name is parent's name + _End )
			str = jointStack.back()->name + "_End";
			joint = new Joint(str);
			jointStack.back()->setChild(joint);
			jointStack.push_back( joint );
			jointNames.push_back(str);
			jointMap[str] = jointStack.back();
		}

	} while( jointStack.size() !=0 );


	//import motion information
	in >> str;
	in >> str;
	in >> str;
	setMotionSize(atoi(str.c_str())); 
	in >> str;
	in >> str;
	in >> str;
	setFrameTime(atof(str.c_str()));

	for(int i=0; i<frameTotal; i++)
	{
		for(unsigned j=0; j<jointNames.size(); j++)
		{
			translate = Vector3d(0,0,0);
			rotation = Quaterniond(1,0,0,0);
			for(unsigned k=0; k < jointMap[jointNames.at(j)]->dof.size(); k++)
			{
				in >> temp;

				switch(jointMap[jointNames[j]]->dof[k])
				{
				case Xt:
					translate.x()=temp/25; break;
				case Yt:
					translate.y()=temp/25; break;
				case Zt:
					translate.z()=temp/25; break;
				case Xr:
					rotation =  rotation * Quaterniond( cos(temp*M_PI/360), sin(temp*M_PI/360), 0, 0);
					break;
				case Yr:
					rotation =  rotation * Quaterniond( cos(temp*M_PI/360), 0, sin(temp*M_PI/360), 0);
					break;
				case Zr:
					rotation =  rotation * Quaterniond( cos(temp*M_PI/360), 0, 0, sin(temp*M_PI/360));
					break;

				}
			}
			jointMap[jointNames[j]]->motions.push_back(new Motion(translate, rotation));
		}
	}
	in.close();
	return 0;
}

int BVHMotionData::importSkeleton(char* _filename)
{
	return 0;
}
