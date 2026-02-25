#pragma once
#ifndef Recon3D_H
#define Recon3D_H


#ifdef IMAGEMATCH_EXPORTS
#define IMAGEMATCH_API   __declspec(dllexport)
#else
#define IMAGEMATCH_API __declspec(dllimport)
#endif

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include "DataSet.h"
#include "AlignImageHD.h"


using namespace std;
struct FrameStartPoint
{
	TrajectoryData ins;
	cv::Point3d pt;
	cv::Point3d ept;
	int no;
	float h_dist;
};

enum ReconMethod
{
	ReconMethod_fixed = 0,
	ReconMethod_Interpolate  //interpolate
};

class IMAGEMATCH_API LineRecon3D : public AlignImageHD
{
public:
	LineRecon3D();
	~LineRecon3D();

	void processRecon3D();
private:

	void postprocessLane();
	void recon3DObjects(const vector<TrajectoryData>& ins_vec);

	void recon3DObjects(const vector<TrajectoryData>& ins_vec, 
		const vector<vector<cv::Point>>& source_vec, 
		vector<vector<cv::Point3d>>& tag_vec);

	float searchElevationInCamera(cv::flann::Index& ins_kdtree,
		const cv::Point& pt,
		const vector<float>& z_vec);
	void rt(Eigen::Vector3f& r_euler, Eigen::Vector3f& t, const TrajectoryData& ins);

	void recon3DObjectsSingleFrame(const vector<TrajectoryData>& ins_vec,
		const int& idx, 
		const map<int, LINE_VEC>& lines_map, 
		Object3D_VEC& obj_vec);

	void recon3DObjectsSingleFrame(const vector<TrajectoryData>& ins_vec, 
		const int& idx, 
		const vector<cv::Point>& pt_vec, 
		vector<cv::Point3d>& pt3_vec);

	void calcInsertInsPoint(const TrajectoryData& s_ins,
		const TrajectoryData& e_ins, 
		vector<TrajectoryData>& insert_ins);
	void calcTracePointsOnRoad(const vector<TrajectoryData>& ins_vec, 
		int idx, 
		vector<cv::Point3d>& insert_front_ins_vec);
	void buildRoadSurfaceByTrajectory(const vector<TrajectoryData>& ins_vec,
		int idx, 
		vector<cv::Point2f>& density_road_points, 
		vector<float>& z_vec);

	void removeUnvalidObjects();

	void adjustLinesInFrame();
	void inversePerspectiveAdjustForRecon(const string& img_na, map<int, LINE_VEC>& lines_map);

	void collectLocalObjects(const vector<TrajectoryData>& ins_vec,
		Object3D_VEC& local_obj_vec);

	void mergeLocalObjects(Object3D_VEC& local_obj_vec);

	void calFrameStartEndPoints(const vector<TrajectoryData>& ins_vec,
		int idx, 
		vector<FrameStartPoint>& fsp_vec);

	void calFrameStartEndPoints(const TrajectoryData& ins, 
		const Object3D_VEC& local_obj_vec,
		vector<FrameStartPoint>& fsp_vec);

	void clusterLocalObjects(vector<vector<double>> pts, Object3D_VEC& local_local_obj_vec);
	cv::Point3d getStartPoint(const Object3D_VEC& local_local_obj_vec,
		const double& x, 
		const Eigen::Matrix3f& R_matrix, 
		const Eigen::Vector3d& Trans);

	//¾«¶È·ÖÎö
	void analyzePrecision(const map<string, Object3D_VEC>& obj_frame_vec);
	void analyzeMeshDiff(const map<string, Object3D_VEC>& obj_frame_vec);
	void clusterDiffLocalObjects(const Object3D_VEC& outers);
private:

	string m_data_path;
	string m_station_id;
	map<pair<int, int>, double> m_z_mesh_map;
	map<string, Object3D_VEC> m_obj_frame_vec;
	map<int, vector<vector<cv::Point3d>>> m_line_map;

	map<string, vector<FrameStartPoint>> m_frame_spt_map;

	ReconMethod m_recon_method;
};

#endif