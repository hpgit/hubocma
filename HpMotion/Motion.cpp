#include "Motion.h"

void Motion::setRotation(Quaterniond &_rotation)
{
	rotation = _rotation;
}

void Motion::setTranslation(Vector3d &_translation)
{
	translation = _translation;
}

/*
double Motion::getAngle()
{
	double sign = 1;

	if(rotation.vec().norm() < DBL_EPSILON)
		return 0;
	if(rotation.vec().normalized().dot(Vector3d(1,1,1)) < 0)
		sign = -1;
	return atan2( rotation.vec().dot(rotation.vec().normalized()), rotation.w())*2;
}
*/
Quaterniond Motion::getRotation()
{
	return rotation;
}

Vector3d Motion::getTranslation()
{
	return translation;
}

void Motion::reset()
{
	rotation = Quaterniond(1,0,0,0);
	translation = Vector3d(0,0,0);
}
/*
jhm::quater euler2quater(string eulerOrder, jhm::vector radInOrder)
{
	jhm::quater q[3];
	DOF dofOrder[3];
	double initQuater[4] = {1,0,0,0};

	for(unsigned i=0; i < 3 ; i++)
		dofOrder[i] = (DOF)(eulerOrder.at(i) - 'X' + Xr);

	for(unsigned i=0; i < 3 ; i++)
	{
		q[i].setValue(initQuater);
		q[i].set_w(cos(radInOrder[i]/2));
		switch(dofOrder[i])
		{
		case Xr:
			q[i].set_x(sin(radInOrder[i]/2));
			break;
		case Yr:
			q[i].set_y(sin(radInOrder[i]/2));
			break;
		case Zr:
			q[i].set_z(sin(radInOrder[i]/2));
			break;
		default:
			break;
		}
	}

	return (q[2]*q[1]*q[0]).normalize();
}

jhm::vector quater2euler(string eulerOrder, jhm::quater rotation)
{
	jhm::vector v(0,0,0);
	jhm::quater q = rotation;

	if(!eulerOrder.compare("YXZ"))
	{
		v.set_x( std::asin( 2*(q.w()*q.x() - q.y()*q.z()) ) );
		v.set_y( std::atan2( 2*(q.w()*q.y()+q.x()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()) ) ); 
		v.set_z( std::atan2( 2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.x()*q.x()+q.z()*q.z()) ) ); 
	}
	else if(!eulerOrder.compare("ZXY"))
	{
		v.set_x( std::asin( 2*(q.w()*q.x() + q.y()*q.z()) ) );
		v.set_y( std::atan2( 2*(q.w()*q.y()-q.x()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()) ) ); 
		v.set_z( std::atan2( 2*(q.w()*q.z()-q.x()*q.y()), 1-2*(q.x()*q.x()+q.z()*q.z()) ) ); 
	}
	else if(!eulerOrder.compare("XYZ"))
	{
		v.set_x( std::atan2( 2*(q.w()*q.x() - q.y()*q.z()) , 1-2*(q.x()*q.x()+q.y()*q.y()) ) );
		v.set_y( std::asin( 2*(q.w()*q.y()+q.x()*q.z()) ) ); 
		v.set_z( std::atan2( 2*(q.w()*q.z()-q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()) ) ); 
	}
	else if(!eulerOrder.compare("XZY"))
	{
		v.set_x( std::atan2( 2*(q.w()*q.x() + q.y()*q.z()) , 1-2*(q.x()*q.x()+q.z()*q.z())  ) );
		v.set_y( std::asin( 2*(q.w()*q.z()-q.x()*q.y()) ) ); 
		v.set_z( std::atan2( 2*(q.w()*q.y()+q.x()*q.z()), 1-2*(q.y()*q.y()+q.z()*q.z()) ) ); 
	}
	else if(!eulerOrder.compare("YZX"))
	{
		v.set_x( std::atan2( 2*(q.w()*q.y() - q.x()*q.z()) , 1-2*(q.y()*q.y()+q.z()*q.z())  ) );
		v.set_y( std::asin( 2*(q.w()*q.z() + q.x()*q.y()) ) ); 
		v.set_z( std::atan2( 2*(q.w()*q.x()- q.y()*q.z()), 1-2*(q.x()*q.x()+q.z()*q.z()) ) ); 
	}
	else if(!eulerOrder.compare("ZYX"))
	{
		v.set_x( std::atan2( 2*(q.w()*q.z() + q.y()*q.x()) , 1-2*(q.z()*q.z()+q.y()*q.y()) ) );
		v.set_y( std::asin( 2*(q.w()*q.y() - q.x()*q.z()) ) ); 
		v.set_z( std::atan2( 2*(q.w()*q.x() + q.z()*q.y()), 1-2*(q.y()*q.y()+q.x()*q.x()) ) ); 
	}
	else if(!eulerOrder.compare("XZX"))
	{
	}
	else if(!eulerOrder.compare("XYX"))
	{
	}
	else if(!eulerOrder.compare("YZY"))
	{
	}
	else if(!eulerOrder.compare("YXY"))
	{
	}
	else if(!eulerOrder.compare("ZXZ"))
	{
	}
	else if(!eulerOrder.compare("ZYZ"))
	{
	}

	return v;
}
*/