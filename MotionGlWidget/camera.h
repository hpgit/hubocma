#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Camera
{
    // viewing -z direction, right-handed coordinate
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Affine3d se3;
    Eigen::Affine3d transforming_se3;
    double distance;
    int transforming;

    Camera();

    void camera_lookat();

    void setDistance(double d);
    void setPosOnSphereAfterScreenRotate(int mousedx, int mousedy, int screenw, int screenh);
    void addPosAfterScreenShift(int mousedx, int mousedy, int screenh, double fovyRad);

    void setTransforming();
    void unsetTransforming();

    void invalidate();

	void yview();
	void xview();
	void zview();
	

private:
    Eigen::Vector3d eye;
    Eigen::Vector3d view;
    Eigen::Vector3d obj;
    Eigen::Vector3d up;
    void vecInvalidate(Eigen::Affine3d &_se3);
};

#endif // CAMERA_H
