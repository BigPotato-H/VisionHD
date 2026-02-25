#include "DataSet.h"
using namespace std;

namespace CTTPL{

template <typename T>
void calcRotation(const Eigen::Matrix<T, 3, 1>& euler, Eigen::Matrix<T, 3, 3>& rotation_matrix);

template <typename T>
void  RotateAndTranslate(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r, const Eigen::Matrix<T, 3, 1>& p);

template <typename T>
void  projectPoint(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2);

template <typename T>
void  projectPano(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2);

template <typename T>
void  projectPoly(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2);

template <typename T>
bool  project2Image(const Eigen::Matrix<T, 3, 1>& p3, Eigen::Matrix<T, 2, 1>& p2);

template <typename T>
bool  convertPoint3dTo2d(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r,
	const Eigen::Matrix<T, 3, 1>& XYZ,
	Eigen::Matrix<T, 2, 1>& xy);

template <typename T>
void convertPoint2dTo3d(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r,
	const Eigen::Matrix<T, 2, 1>& xy,
	Eigen::Matrix<T, 3, 1>& c_xyz);

template <typename T>
void calPointCamera3d(Eigen::Matrix<T, 2, 1> xy,
	Eigen::Matrix<T, 3, 1>& XYZ);

template <typename T>
void convertPointCameraToWorld(const Eigen::Matrix<T, 3, 1>& t,
	const Eigen::Matrix<T, 3, 1>& r,
	const Eigen::Matrix<T, 3, 1>& xyz,
	Eigen::Matrix<T, 3, 1>& XYZ);


}