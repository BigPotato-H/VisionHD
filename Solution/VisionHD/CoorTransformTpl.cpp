#include "CoorTransformTpl.h"
using namespace std;

namespace CTTPL{

template <typename T>
 void calcRotation(const Eigen::Matrix<T, 3, 1>& euler, Eigen::Matrix<T, 3, 3>& rotation_matrix)
{
	Eigen::AngleAxis<T> rollAngle(euler[0], Eigen::Matrix<T, 3, 1>::UnitX());
	Eigen::AngleAxis<T> pitchAngle(euler[1], Eigen::Matrix<T, 3, 1>::UnitY());
	Eigen::AngleAxis<T> yawAngle(euler[2], Eigen::Matrix<T, 3, 1>::UnitZ());

	rotation_matrix = rollAngle * pitchAngle * yawAngle;
}

template <typename T>
 void  RotateAndTranslate(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r, const Eigen::Matrix<T, 3, 1>& p)
{
	Eigen::Matrix<T, 3, 3> rotation_matrix;
	calcRotation(r, rotation_matrix);
	Eigen::Matrix<T, 3, 1> pt(p);
	p = rotation_matrix * pt + t;
}


template <typename T>
 void  projectPoint(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2)
{
	T fx = T(CalibSpace::FX); //mm
	T fy = T(CalibSpace::FY); //mm

	if (CalibSpace::distCoeffs.at<double>(0, 0) != 0)
	{
#if 0
		//fisheye
		T D[4];
		for (int i = 0; i < 4; i++)
		{
			D[i] = T(CalibSpace::distCoeffs.at<T>(0, i));
		}
		T a = p[0] / p[2];
		T b = p[1] / p[2];
		T r = sqrt(a * a + b * b);
		T theta = atan(r);
		T theta_d = theta * (T(1) + D[0] * pow(theta, 2) + D[1] * pow(theta, 4) + D[2] * pow(theta, 6) + D[3] * pow(theta, 8));
		a = (theta_d / r) * a;
		b = (theta_d / r) * b;
		x = fx * a + T(CalibSpace::CX);
		y = fy * b + T(CalibSpace::CY);
#endif
		T k1 = T(CalibSpace::distCoeffs.at<double>(0, 0));
		T k2 = T(CalibSpace::distCoeffs.at<double>(0, 1));
		T p1 = T(CalibSpace::distCoeffs.at<double>(0, 2));
		T p2 = T(CalibSpace::distCoeffs.at<double>(0, 3));
		T k3 = T(CalibSpace::distCoeffs.at<double>(0, 4));
		T x = p3[0] / p3[2];
		T y = p3[1] / p3[2];
		T r2 = x * x + y * y;
		T x_radial = x * (T(1.0) + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
		T y_radial = y * (T(1.0) + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
		T x_tangential = T(2.0) * p1*x*y + p2*(r2 + T(2.0) * x*x);
		T y_tangential = T(2.0) * p2*x*y + p1*(r2 + T(2.0) * y*y);
		T xd = x_radial + x_tangential;
		T yd = y_radial + y_tangential;

		//p2[0] = fx * xd + T(CalibSpace::CX);
		//p2[1] = fy * yd + T(CalibSpace::CY);

	}
	else
	{
		//无畸变
		p2[0] = fx * p3[0] / p3[2] + T(CalibSpace::CX);
		p2[1] = fy * p3[1] / p3[2] + T(CalibSpace::CY);
	}

}

template <typename T>
 void  projectPano(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2)
{
	const T& _x = p3[0];
	const T& _y = p3[1];
	const T& _z = p3[2];

	T rho = sqrt(_x * _x + _y * _y + _z * _z);
	T theta = M_PI - acos(_y / rho);
	T phi = atan2(_x, _z) + M_PI;

	p2[0] = phi / M_PI* T(CalibSpace::IMG_HEIGHT);
	p2[1] = theta / M_PI * T(CalibSpace::IMG_HEIGHT);
}

template <typename T>
 void  projectPoly(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2)
{
	const T& _x = p3[0];
	const T& _y = p3[1];
	const T& _z = p3[2];

	T rho = sqrt(_x * _x + _y * _y + _z * _z);

	//相机坐标系到像平面，原点到左上角了
	T theta = M_PI - acos(_y / rho);
	//atan2的范围是-π到π
	T phi = atan2(_x, _z) + M_PI / 2;

	p2[0] = phi / M_PI* T(CalibSpace::IMG_WIDTH);
	p2[1] = theta / M_PI * T(CalibSpace::IMG_HEIGHT);

}

template <typename T>
 bool  project2Image(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2)
{
	if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		//深度>0
		if (p3[2] <= T(0))
		{
			return false;
		}
		projectPoint(p3, p2);
	}
	else if (CalibSpace::camera_type == CAMERA_MSS_PANO)
	{
		projectPano(p3, p2);
	}
	else if (CalibSpace::camera_type == CAMERA_MSS_POLYWIDE)
	{
		projectPoly(p3, p2);
	}
	else
	{
		return false;
	}

	p2[0] -= T(CalibSpace::image_rect.tl().x);
	p2[1] -= T(CalibSpace::image_rect.tl().y);

	if (p2[0] < T(0) || p2[0] > T(CalibSpace::image_rect.width - 1) ||
		p2[1] < T(0) || p2[1] > T(CalibSpace::image_rect.height - 1))
	{
		return false;
	}

	return true;
}

template <typename T>
 bool  convertPoint3dTo2d(const Eigen::Matrix<T, 3, 1>& t, 
	const Eigen::Matrix<T, 3, 1>& r,
	const Eigen::Matrix<T, 3, 1>& XYZ, 
	Eigen::Matrix<T, 2, 1>& xy)
{
	Eigen::Matrix<T, 3, 1> p(XYZ);
	RotateAndTranslate(t, r, p);

	return project2Image(p, xy);
}

template <typename T>
 void convertPoint2dTo3d(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r, 
	const Eigen::Matrix<T, 2, 1>& xy,
	Eigen::Matrix<T, 3, 1>& c_xyz)
{
	calPointCamera3d(xy, c_xyz);

	Eigen::Matrix<T, 3, 3> r_matrix;
	calcRotation(r, r_matrix);
	r_matrix = r_matrix.inverse();

	c_xyz -= t;
	c_xyz = r_matrix * c_xyz;
}

template <typename T>
 void calPointCamera3d(Eigen::Matrix<T, 2, 1> xy,
	Eigen::Matrix<T, 3, 1>& XYZ)
{
	xy[0] += T(CalibSpace::image_rect.tl().x);
	xy[1] += T(CalibSpace::image_rect.tl().y);

	if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		if (abs(xy[1] - T(CalibSpace::CY)) < T(1.0))
		{
			XYZ[2] = T(200);
		}
		else
		{
			XYZ[2] = XYZ[1] * T(CalibSpace::FY) / (xy[1] - T(CalibSpace::CY));
		}

		XYZ[0] = (xy[0] - T(CalibSpace::CX)) * XYZ[2] / T(CalibSpace::FX);
	}
	else if (CalibSpace::camera_type == CAMERA_MSS_PANO)
	{
		T theta = xy[1] / T(CalibSpace::IMG_HEIGHT) * T(M_PI);
		T phi = xy[0] / T(CalibSpace::IMG_HEIGHT) * T(M_PI);

		T r = XYZ[1] / cos(T(M_PI) - theta);
		XYZ[2] = sqrt((r * r - XYZ[1] * XYZ[1]) / (T(1.0) + tan(phi - T(M_PI)) * tan(phi - T(M_PI))));
		XYZ[0] = XYZ[2] * tan(phi - T(M_PI));
	}
}

template <typename T>
 void convertPointCameraToWorld(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r,
	const Eigen::Matrix<T, 3, 1>& xyz,
	Eigen::Matrix<T, 3, 1>& XYZ)
{
	Eigen::Matrix<T, 3, 3> r_matrix;
	calcRotation(r, r_matrix);
	r_matrix = r_matrix.inverse();

	XYZ = r_matrix * xyz + t;
}
}