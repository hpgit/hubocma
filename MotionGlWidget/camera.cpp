#include "camera.h"
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

Camera::Camera()
{
    se3.setIdentity();
    se3.translate(Eigen::Vector3d(.0, 3.0, 4.0));
    se3.rotate(Eigen::AngleAxisd( -atan((double)0.75), Eigen::Vector3d::UnitX()));
    transforming_se3 = se3;
    distance = 5.0;
    transforming = 0;
}
void Camera::camera_lookat()
{
    Eigen::Affine3d &lookse3 = transforming ? transforming_se3 : se3;
    vecInvalidate(lookse3);
    gluLookAt(eye.x(), eye.y(), eye.z(), obj.x(), obj.y(), obj.z(), up.x(), up.y(), up.z());
}
void Camera::invalidate()
{
    se3 = transforming_se3;
    vecInvalidate(se3);
}
void Camera::setDistance(double d)
{
    double rate = d / distance;
    vecInvalidate(se3);
    se3.translation() = (1-rate) * obj + rate * eye;
    if(d > 0) distance = d;
}
void Camera::setPosOnSphereAfterScreenRotate(int mousedx, int mousedy, int screenw, int screenh)
{
    double theta = (M_PI/5) * sqrt( ((double)(4*mousedx * mousedx))/(screenw*screenw) + ((double)(4*mousedy * mousedy))/(screenh*screenh) );
    double rotatePhi = 0;
    if( mousedx == 0)
    {
        if( mousedy > 0)
            rotatePhi = M_PI/2;
        else
            rotatePhi = -M_PI/2;
    }
    else
        rotatePhi = atan2(mousedy, mousedx);

    Eigen::Vector3d rotVec = -se3.linear() * Eigen::AngleAxisd(rotatePhi, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitY();
    vecInvalidate(se3);
    transforming_se3 = se3;
    transforming_se3.pretranslate(-obj);
    transforming_se3.prerotate(Eigen::AngleAxisd(theta, rotVec));
    transforming_se3.pretranslate(obj);
}
void Camera::addPosAfterScreenShift(int mousedx, int mousedy, int screenh, double fovyRad)
{
    double rate = 2. * distance * tan(fovyRad/2.);
    Eigen::Vector3d displacement( (double)mousedx / screenh, (double)mousedy / screenh, 0);
    transforming_se3 = se3;
    //transforming_se3.pretranslate(-displacement);
    transforming_se3.translate(-displacement);
}

void Camera::setTransforming()
{
    transforming = 1;
}
void Camera::unsetTransforming()
{
    transforming = 0;
}
void Camera::vecInvalidate(Eigen::Affine3d &_se3)
{
    eye = _se3.translation();
    view = _se3.linear() * (-1) * Eigen::Vector3d::UnitZ();
    obj = eye + ( distance * view );
    up = _se3.linear() * Eigen::Vector3d::UnitY();
}
