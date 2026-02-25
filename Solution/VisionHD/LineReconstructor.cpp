#include "LineReconstructor.h"
#include <set>
#include "TestCalib.h"
#include "HNMath/TransRotation.h"
#include<fstream>
#include "DataManager/WKTCSV.h"
#include "DataIO.h"
#include <io.h>
#include <glog/logging.h>
#include "DataManager/XmlConfig.h"
#include "HNMath/GeometricAlgorithm2.h"
#include "Optimize.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <numeric>

#include "DBSCAN/include/ANN/impl/dbscan.hpp"
#include "HNMath/HNLineMath.h"

#include "ShpDifference.h"


using namespace HNMath;

float NEAR_DIST = 10.0;
float ANGLE = 10;




string getPostfix(ReconMethod rm)
{
	string post_fix = "";
	switch (rm)
	{
	case ReconMethod_fixed:
		post_fix = "-fixed";
		break;
	case ReconMethod_Interpolate:
		post_fix = "-inp";
		break;
	default:
		break;
	}

	return post_fix;
}


#define  RECON_FRAME "recon_frame"
#define  RECON_DIFF "recon_diff" 

#define  RECON_LANE "recon_lane.csv"
#define  RECON_LANE_STARTS "recon_lane_frame_starts.csv"

#define  MAP_MESH "mesh_map.csv"
#define  RECON_MESH "mesh_recon_dif.csv"
#define  RECON_DIF_CLUSTER "mesh_recon_dif_cluster.csv"

double dist(const cv::Point3d& p1, const cv::Point3d& p2)
{
	auto  p = p1 - p2;
	double dis = sqrt(p.dot(p));
	return dis;
}

double length(const vector<cv::Point3d>& shp)
{
	double d = 0;
	auto itr_p = shp.begin();
	for (; itr_p != shp.end() - 1; itr_p++)
	{
		d += dist(*itr_p, *(itr_p + 1));
	}
	
	return d;
}

cv::Point3f calcFrontPoint(cv::Point3f cur_point, double dist, double angle)
{
	cv::Point3f point;
	point.x = cur_point.x + dist * sin(angle);
	point.y = cur_point.y + dist * cos(angle);
	point.z = cur_point.z;
	return point;
}

void saveRecon3DObjects(const string& file_path, const string& station_name, const Object3D_VEC& obj_vec)
{
	ofstream os(file_path, ios::app);
	//ofstream os(file_path, ios::trunc);

	if (!os.is_open())
	{
		return;
	}
	os.setf(ios::fixed, ios::floatfield);
	os.precision(4);

	for (int i = 0; i < obj_vec.size(); i++)
	{
		const auto& obj = obj_vec[i];
		int shp_type = 1;
		if (obj.shape.size() < 2)
		{
			shp_type = 0;
			continue;
		}
		vector<string> attr_vec;
		attr_vec.push_back(station_name);
		attr_vec.push_back(obj.prop_id);
		attr_vec.push_back(to_string(obj.type));
		outAttrCSVFields(os, attr_vec);
		outShpCSVFields(os, obj.shape, shp_type);
	}
	os.close();
}

void saveLaneFrameStarts(const string& file_path,  const vector<FrameStartPoint>& obj_vec)
{
	ofstream os(file_path, ios::app);

	if (!os.is_open())
	{
		return;
	}
	os.setf(ios::fixed, ios::floatfield);
	os.precision(4);

	for (int i = 0; i < obj_vec.size(); i++)
	{
		const auto& obj = obj_vec[i];
		vector<string> attr_vec;
		attr_vec.push_back(obj.ins.name);
		attr_vec.push_back(to_string(obj.no));
		attr_vec.push_back(to_string(obj.h_dist));
		outAttrCSVFields(os, attr_vec);

		vector<cv::Point3d> pt_vec;
		pt_vec.push_back(obj.pt);
		pt_vec.push_back(obj.ept);
		outShpCSVFields(os, pt_vec, 1);
	}
	os.close();
}

void readRecon3DObjects(const string& file_na, string& station_name,
	const set<ObjectClassification>& obj_set,
	map<string, Object3D_VEC>& obj_frame_vec)
{
	ifstream os(file_na);
	if (!os.is_open())
	{
		return;
	}
	string line = "";
	while (getline(os, line))
	{
		if (line.size() == 0)
		{
			continue;
		}
		Object3D obj;
		vector<string> attr_vec;
		inAttrCSVFields(line, attr_vec);
		int seq = 0;
//		station_name = attr_vec[seq++];
		obj.prop_id = attr_vec[seq++];
		obj.type = stoi(attr_vec[seq++]);
		if (obj_set.find((ObjectClassification)obj.type) == obj_set.end())
		{
			continue;
		}
		
		inShpCSVFields(attr_vec.back(), obj.shape, 1);

		Object3D_VEC& obj_vec = obj_frame_vec[obj.prop_id];
		obj_vec.emplace_back(obj);

		/*if (obj_frame_vec.size() > 100)
		{
			break;
		}*/
	}
	
	os.close();
}

void saveDiff(const string& file_path, const ObjectDiff_VEC& od_vec)
{
	ofstream os(file_path, ios::trunc);

	if (!os.is_open())
	{
		return;
	}
	os.setf(ios::fixed, ios::floatfield);
	os.precision(4);

	for (int i = 0; i < od_vec.size(); i++)
	{
		const auto& obj = od_vec[i];
		int shp_type = 1;
		for (int j = 0; j < obj.shape.size(); j++)
		{
			vector<string> attr_vec;
			attr_vec.push_back(obj.obj_id);
			attr_vec.push_back(obj.prop_id); 
			attr_vec.push_back(to_string(obj.type));
			
			vector<HPoint3d> shp(2);
			shp[0] = obj.shape[j];
			shp[1] = obj.diff_shape[j];

			HPoint3d dif = shp[1] - shp[0];
			attr_vec.push_back(to_string(obj.flgs[j]));
			double d = sqrt(dif.x * dif.x + dif.y * dif.y);
			attr_vec.push_back(to_string(d));
			attr_vec.push_back(to_string(0));
			attr_vec.push_back(to_string(abs(dif.z)));

			outAttrCSVFields(os, attr_vec);
			outShpCSVFields(os, shp, shp_type);
		}
	}
	os.close();
}

void saveMesh(const string& file_path, Mesh_VEC& mesh_vec)
{
	ofstream os(file_path, ios::trunc);

	if (!os.is_open())
	{
		return;
	}
	os.setf(ios::fixed, ios::floatfield);
	os.precision(4);

	int shp_type = 2;
	double d = 0.5;
	for (int i = 0; i < mesh_vec.size(); i++)
	{
		const auto& mesh = mesh_vec[i];
		
		vector<HPoint3d> shp(4);
		cv::Point2d p(mesh.p.x / 10.0, mesh.p.y / 10.0);
		double z = mesh.z;
		shp[0] = HPoint3d(p.x, p.y, z);
		shp[1] = HPoint3d(p.x + d, p.y, z);
		shp[2] = HPoint3d(p.x + d, p.y + d, z);
		shp[3] = HPoint3d(p.x, p.y + d, z);

		vector<string> attr_vec;
		attr_vec.push_back(to_string(i));
		outAttrCSVFields(os, attr_vec);
		outShpCSVFields(os, shp, shp_type);
	}
	os.close();
}

void saveObject(const string& file_path, Object3D_VEC& outers, int shp_type)
{
	ofstream os(file_path, ios::trunc);

	if (!os.is_open())
	{
		return;
	}
	os.setf(ios::fixed, ios::floatfield);
	os.precision(4);

	for (int i = 0; i < outers.size(); i++)
	{
		vector<string> attr_vec;
		attr_vec.push_back(to_string(i));
		outAttrCSVFields(os, attr_vec);
		outShpCSVFields(os, outers[i].shape, shp_type);
	}
	os.close();
}

float getAngle(const HPolyline3d& next_line, const HPolyline3d& last_line)
{
	if (next_line.size() == 0 ||
		last_line.size() == 0)
	{
		return 90;
	}
	const HPoint3d& spt = next_line.front();
	HPoint3d s_to_pt;
	KPolylineAlg::getPtAwayfromPolylineSPt(next_line, 4.0, s_to_pt);
	//计算上一条线的尾部，与当前起点构成的夹角
	const HPoint3d& ept = last_line.back();
	HPoint3d to_e_pt;
	KPolylineAlg::getPtAwayfromPolylineEPt(last_line, 4.0, to_e_pt);
	float angle_a = HNLineMath::calcVectorAngle(to_e_pt, ept, ept, spt);
	float angle_b = HNLineMath::calcVectorAngle(spt, s_to_pt, ept, spt);
	angle_a = angle_a > 180 ? 360 - angle_a : angle_a;
	angle_b = angle_b > 180 ? 360 - angle_b : angle_b;
	float angle = max(max(angle_a, angle_b), abs(angle_a - angle_b));
	return angle;
}

bool isTheSameLane(const HPolyline3d& next_line, const HPolyline3d& last_line, float foot_dist_thresh)
{
	float STRECH_LEN = 10.0;
	if (last_line.size() == 0 ||
		next_line.size() == 0)
	{
		return false;
	}
	const HPoint3d& spt = next_line.front();
	//计算上一条线的尾点和当前线的首点之间的距离
	double near_dist = HNLineMath::length(spt, last_line.back());
	float angle = getAngle(next_line, last_line);
	if (near_dist < NEAR_DIST &&
		angle < ANGLE)
	{
		return true;
	}
	else
	{
		HPolyline3d strech_line;
		KPolylineAlg::stretchPolylineAtEpoint(last_line, strech_line, STRECH_LEN);
		HPoint3d near_pt;
		double foot_dist = KPolylineAlg::calcP2PolylineNearestPoint(strech_line, spt, near_pt);
		if (foot_dist < foot_dist_thresh)
		{
			return true;
		}
	}
	return false;
}

bool isMergeLane(const vector<cv::Point3d>& next_line,
	const vector<cv::Point3d>& last_line,
	float foot_dist_thresh)
{
	HPolyline3d next_kline(next_line.size());
	std::transform(next_line.begin(), next_line.end(), next_kline.begin(), [](const auto& pt)->HPoint3d {
		return HPoint3d(pt.x, pt.y, pt.z);
		});

	HPolyline3d last_kline(last_line.size());
	std::transform(last_line.begin(), last_line.end(), last_kline.begin(), [](const auto& pt)->HPoint3d {
		return HPoint3d(pt.x, pt.y, pt.z);
		});

	return isTheSameLane(next_kline, last_kline, foot_dist_thresh);
}

bool isChain(int type)
{
	return type == OC_lane;
}

bool isOffsetRoadSurface(int type)
{
	return type == OC_car;
}

bool isRoadMark(int type)
{
	return type == OC_road ||
		type == 0;
}

LineRecon3D::LineRecon3D()
{
	FLAGS_minloglevel = 0;

	fLS::FLAGS_log_dir = "Logs";
	FLAGS_alsologtostderr = 1;
	google::SetLogDestination(google::GLOG_INFO, (FLAGS_log_dir + "\\log_").c_str());
	google::SetLogDestination(google::GLOG_WARNING, (FLAGS_log_dir + "\\log_").c_str());

}

LineRecon3D::~LineRecon3D()
{
}


void LineRecon3D::processRecon3D()
{
	//////////////
	m_data_path = MY_CONFIG.mid_path;
	//m_recon_method = ReconMethod(stoi(MY_CONFIG.method));
	m_recon_method = ReconMethod(m_reg_method);
	string postfix = getPostfix(m_recon_method);

#if 1
	if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		//需要优先将图像矫正，再输入。这里不使用畸变参数
//		CalibSpace::distCoeffs = cv::Mat::zeros(1, 5, cv::DataType<double>::type);
	}
	string obj_path = m_data_path + RECON_FRAME + getPostfix(m_recon_method) + ".csv";;
	if (_access(obj_path.c_str(), 0) == 0)
	{
		ofstream os(obj_path, ios::trunc);
		os.close();
	}
	
	recon3DObjects(m_ins_vec);
//#else
	postprocessLane();
#endif
}


void LineRecon3D::postprocessLane()
{
	set<ObjectClassification> obj_set;
	obj_set.insert(OC_lane);

	string file_na = m_data_path + RECON_FRAME + getPostfix(m_recon_method) + ".csv";
	readRecon3DObjects(file_na, m_station_id, obj_set, m_obj_frame_vec);

#if 1
	analyzePrecision(m_obj_frame_vec);
	analyzeMeshDiff(m_obj_frame_vec);
	return;
#endif

}

void LineRecon3D::removeUnvalidObjects()
{
	auto itr = m_obj_frame_vec.begin();
	for (; itr != m_obj_frame_vec.end(); itr++)
	{
		auto& obj_vec = itr->second;
		if (itr->first == "1649309075726867")
		{
			int a = 0;
		}
		auto itr_obj = obj_vec.begin();
		for (; itr_obj != obj_vec.end();)
		{
			auto& shp = itr_obj->shape;
			if (itr_obj->type == OC_lane && (shp.size() < 2 || length(shp) < 0.5))
			{
				itr_obj = obj_vec.erase(itr_obj);
				continue;
			}
			else
			{
				auto itr_pt = shp.begin();
				for (; itr_pt != shp.end() - 1;)
				{
					if (dist(*itr_pt, *(itr_pt + 1)) < 0.5)
					{
						shp.erase(itr_pt + 1);
					}
					else
					{
						itr_pt++;
					}
				}
				if (shp.size() < 2)
				{
					itr_obj = obj_vec.erase(itr_obj);
					continue;
				}

				if (dist(*shp.rbegin(), *(shp.rbegin() + 1)) < 0.5)
				{
					shp.erase((shp.rbegin() + 1).base());
				}

				if (shp.size() < 2)
				{
					itr_obj = obj_vec.erase(itr_obj);
					continue;
				}

				itr_obj++;
			}

			
		}
	}
}

void LineRecon3D::recon3DObjects(const vector<TrajectoryData>& ins_vec)
{
	set<ObjectClassification> lane_set;
	lane_set.insert(OC_lane);

	set<ObjectClassification> obj_set;

	if (!READ_MASK)
	{
		obj_set.insert(OC_lane);
	}

	Calib mc;

	int total_sz = ins_vec.size();
	int sz = 0;
//	int sz = ins_vec.size() - 1;
	int save_sz = 0;
	auto itr_ins = ins_vec.begin() + sz;
//	for (; itr_ins != ins_vec.end(); itr_ins = itr_ins+5, sz+= 5)
	for (; itr_ins != ins_vec.end(); itr_ins = itr_ins + 1, sz += 1)
	{
		const TrajectoryData& ins = *itr_ins;
		string img_na = itr_ins->name;
		
		if (img_na < "315973414399927221")//argo
		{
		//	continue;
		}
		LOG(INFO) << ("%d:...%s", sz, img_na.c_str());

		map<int, LINE_VEC> lines_map;
		mc.extractImageDeepLearningMultiLines(m_data_path, img_na, lane_set, lines_map, 0);
		inversePerspectiveAdjustForRecon(img_na, lines_map);

		Object3D_VEC obj_vec;
		recon3DObjectsSingleFrame(ins_vec, sz, lines_map, obj_vec);

		string file_na = m_data_path + RECON_FRAME + getPostfix(m_recon_method) + ".csv";;
		saveRecon3DObjects(file_na, "", obj_vec);
	}
}

void LineRecon3D::recon3DObjects(const vector<TrajectoryData>& ins_vec,
	const vector<vector<cv::Point>>& source_vec,
	vector<vector<cv::Point3d>>& tag_vec)
{
	int total_sz = ins_vec.size();
	int sz = 0;
	int save_sz = 0;
	string file_na = m_data_path + RECON_FRAME + getPostfix(m_recon_method) + ".csv";;
	ofstream of(file_na);
	auto itr_ins = ins_vec.begin() + sz;
	for (; itr_ins != ins_vec.end(); itr_ins++, sz++)
	{
		const TrajectoryData& ins = *itr_ins;
		string img_na = itr_ins->name;
		LOG(INFO) <<  ("%d:...%s", sz, img_na.c_str());
		recon3DObjectsSingleFrame(ins_vec, sz, source_vec[sz], tag_vec[sz]);

		for (const auto& pt : tag_vec[sz])
		{
			string line = to_string(pt.x) + "," + to_string(pt.y) + "," + to_string(pt.z);
			of << line << endl;
		}
	}
	of.close();
}



float LineRecon3D::searchElevationInCamera(cv::flann::Index& ins_kdtree,
	const cv::Point& pt,
	const vector<float>& z_vec)
{
	if (z_vec.size() <= 4)
	{
		float z = accumulate(z_vec.begin(), z_vec.end(), 0) / int(z_vec.size());
		return z;
	}

	cv::flann::SearchParams params(-1);
	int query_num = 4;

	vector<float> query(2);
	query[0] = pt.x;
	query[1] = pt.y;
	vector<int> indices(query_num);
	vector<float> dists(query_num);
	ins_kdtree.knnSearch(query, indices, dists, query_num, params);

	float z = 0;
	for_each(indices.begin(), indices.end(), [&](const auto& idx) {
		z += z_vec[idx];
	});
	z /= query_num;

	return z;
}

void LineRecon3D::rt(Eigen::Vector3f& r_euler, Eigen::Vector3f& t, const TrajectoryData& ins)
{
	//t << 0.25, 0, -1.3;
	t << MY_CONFIG.data_para.trans.x, MY_CONFIG.data_para.trans.y, MY_CONFIG.data_para.trans.z;

	r_euler << MY_CONFIG.data_para.rotate[0], MY_CONFIG.data_para.rotate[1], MY_CONFIG.data_para.rotate[2];
	r_euler = r_euler / 180.0 * M_PI;
}


void sampleIndex(int sz,vector<int>& indices)
{
	if (sz == 0)
	{
		return;
	}

	indices.push_back(0);
	if (sz < 2)
	{
		return;
	}

	if(sz < 10)
	{
		indices.push_back(sz - 1);
		return;
	}
	for(int i = 1; i < sz; i += 3)
	{
		indices.push_back(i);
	}

	indices.push_back(sz - 1);
	return;
}

void forceModify(int type, vector<Eigen::Vector3f>& p3f_vec, float MAX_DEPTH = 40.0)
{
	if (p3f_vec.size() == 0)
	{
		return;
	}

	float MIN_DEPTH = -2.0;

	auto reserve_itr = min_element(p3f_vec.begin(), p3f_vec.end(), [](const auto& p3f1, const auto& p3f2)->bool {
		return p3f1[2] < p3f2[2];
	});
	auto z = (*reserve_itr)[2];

	if (z > MAX_DEPTH || z < MIN_DEPTH)
	{
		p3f_vec.clear();
		return;
	}

	if (p3f_vec.size() == 0)
	{
		return;
	}

	if (isChain(type))
	{
		auto remover_itr = remove_if(p3f_vec.begin(), p3f_vec.end(), [&](const auto& p3f)->bool {
			return  (p3f[2] > MAX_DEPTH || p3f[2] <= 0);
		});
		p3f_vec.erase(remover_itr, p3f_vec.end());
	}
	else if (isRoadMark(type))
	{
		auto remover_itr = p3f_vec.begin();
		for (; remover_itr != p3f_vec.end(); )
		{
			const auto& p3f = *remover_itr;
			if (p3f[2] > MAX_DEPTH || p3f[2] <= 0)
			{
				remover_itr = p3f_vec.erase(remover_itr);
			}
			else
			{
				remover_itr++;
			}
		}
	}
	
}

void forceModifyManual(vector<Eigen::Vector3f>& p3f_vec)
{
	float MAX_DEPTH = 35.0;
	float MIN_DEPTH = 0;

	for_each(p3f_vec.begin(), p3f_vec.end(), [&](auto& p3f) {
		if (p3f[2] < MIN_DEPTH)
		{
			p3f[2] = MIN_DEPTH;
		}
		if (p3f[2] > MAX_DEPTH)
		{
			p3f[2] = MAX_DEPTH;
		}
	});
}

void removeTooShort(vector<vector<Eigen::Vector3f>>& p3f_vec_vec)
{
	vector<vector<Eigen::Vector3f>> tmp;
	for (const auto& line : p3f_vec_vec)
	{
		double len = 0;
		for (int i =0; i < line.size() - 1; i++)
		{
			len += (line[i] - line[i + 1]).norm();
		}
		if (len > 2.0)
		{
			tmp.push_back(line);
		}
	}
	p3f_vec_vec.swap(tmp);
}

void removeTooSkew(vector<vector<Eigen::Vector3f>>& p3f_vec_vec)
{
	vector<vector<Eigen::Vector3f>> tmp;
	for (const auto& line : p3f_vec_vec)
	{
		Eigen::Vector3f direction = line.back() - line.front();
		Eigen::Vector2f v1(direction[0], direction[2]);
		Eigen::Vector2f v2(0, 1);
		double angle = v1.dot(v2) / (v1.norm() * v2.norm());
		angle = acos(angle) / M_PI * 180;		
		if (angle < 30)
		{
			tmp.push_back(line);
		}
	}
	p3f_vec_vec.swap(tmp);
}

//根据航向的变化率简单定义一下
double getMaxDepth(float delta_yaw)
{
	double max_depth = 40.0;

	if (MY_CONFIG.data_name == "ARG")
	{
		max_depth = 20;
		return max_depth;
	}

	delta_yaw = delta_yaw / M_PI * 180;
	if (abs(delta_yaw) > 2 &&
		abs(delta_yaw) < 10)
	{
		max_depth = 10;
	}

	return max_depth;
}

void LineRecon3D::recon3DObjectsSingleFrame(const vector<TrajectoryData>& ins_vec, const int& idx, const map<int, LINE_VEC>& lines_map,
	Object3D_VEC& obj_vec)
{
	const TrajectoryData& ins = ins_vec[idx];
	if (ins.name != "315973414399927221")
	{
	//return;
	}
	float delta_yaw = 0;
	if (idx > 0)
	{
		delta_yaw = ins_vec[idx].heading - ins_vec[idx - 1].heading;
	}
	double max_depth = getMaxDepth(delta_yaw);

	vector<cv::Point2f> density_road_points;
	vector<float> z_vec;
	buildRoadSurfaceByTrajectory(ins_vec, idx, density_road_points, z_vec);

	if (density_road_points.size() == 0)
	{
		return;
	}
	cv::flann::KDTreeIndexParams indexParams(1);
	cv::Mat source = cv::Mat(density_road_points).reshape(1);
	source.convertTo(source, CV_32F);
	cv::flann::Index ins_kdtree(source, indexParams);

	/////////////
	Eigen::Vector3f r_euler(0, 0, 0);
	Eigen::Vector3f t(0, 0, 0);
	rt(r_euler, t, ins);

	Eigen::Matrix3f r_matrix;
	TransRotation::eigenEuler2RotationMatrix(r_euler, r_matrix);
	r_matrix = r_matrix.inverse();

	///////////////
	Eigen::Vector3f R_euler;
	R_euler << M_PI / 2 + ins.roll, ins.pitch, ins.heading;

	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);
	
	Eigen::Matrix3f R_matrix_inv = R_matrix.inverse();
	Eigen::Vector3f temp_r = R_matrix_inv.eulerAngles(2, 1, 0) / M_PI * 180.0;
	//Eigen::Matrix3f p = R_matrix_inv * R_matrix;
	//Eigen::Matrix3d R_matrix_invd = R_matrix_inv.cast<double>();
	Eigen::Vector3d Trans;
	Trans << ins.point.x, ins.point.y, ins.point.z;

 	//ofstream of("c.txt");
 	//of << "x,y" << endl;
	//of << "px,py,cx,cy,cz" << endl;
	auto itr_line = lines_map.begin();
	for (; itr_line != lines_map.end(); itr_line++)
	{
		int type = itr_line->first;
		const auto& line_vec = itr_line->second;
		for_each(line_vec.begin(), line_vec.end(), [&](const auto& line) {

			vector<int> indices;
			sampleIndex(line.size(), indices);

			//多段线
			vector<vector<Eigen::Vector3f>> p3f_vec_vec;
			vector<Eigen::Vector3f> p3f_vec;

			for (int i : indices)
			{
				auto pt = line[i];
				//search for elevation
				Eigen::Vector3f p3f(0, 0, 0);
				//default fixed hight
				switch (m_recon_method)
				{
				case ReconMethod_fixed:
					p3f[1] = CalibSpace::ego_height + MY_CONFIG.data_para.trans.y;
					break;
				case ReconMethod_Interpolate:
					p3f[1] = searchElevationInCamera(ins_kdtree, pt, z_vec);

					break;
				default:
					break;
				}
				double a = CalibSpace::ego_height + MY_CONFIG.data_para.trans.y;
				double b = p3f[1] - a;

				if (OptimizeCeres::convertPixel2Camera3d((float)pt.x, (float)pt.y, p3f))
				{
					p3f -= t;
					if (p3f_vec.size() == 0)
					{
						p3f_vec.push_back(p3f);
					}
					else
					{
						double to_last = (p3f - p3f_vec.back()).norm();
						if (to_last < 0.5)
						{
							continue;
						}
						if (to_last > 3.0)
						{
							p3f_vec_vec.push_back(p3f_vec);
							p3f_vec.clear();
						}
						p3f_vec.push_back(p3f);
					}
				}
			}
			if (p3f_vec.size() > 1)
			{
				p3f_vec_vec.push_back(p3f_vec);
			}
			
			removeTooShort(p3f_vec_vec);
			removeTooSkew(p3f_vec_vec);

			for (auto& p3f_vec: p3f_vec_vec)
			{
				forceModify(type, p3f_vec, max_depth);
				if (p3f_vec.size() > 0)
				{
					Object3D obj;
					obj.type = type;
					obj.prop_id = ins.name;

					for (auto p3f : p3f_vec)
					{
						p3f = r_matrix * p3f;
						p3f = R_matrix_inv * p3f;

						cv::Point3d p3d;
						p3d.x = p3f[0] + Trans[0];
						p3d.y = p3f[1] + Trans[1];
						p3d.z = p3f[2] + Trans[2];

						obj.shape.push_back(p3d);
					}

					if ((isChain(type) || isRoadMark(type))
						&& obj.shape.size() >= 1)
					{
						obj_vec.push_back(obj);
					}
					else if (isOffsetRoadSurface(type) && obj.shape.size() > 1)
					{
						obj.shape.push_back(ins.point);
						obj_vec.push_back(obj);
					}
				}
			}
			
			
		});
	}

 	//of.close();

	ins_kdtree.release();
}

void LineRecon3D::recon3DObjectsSingleFrame(const vector<TrajectoryData>& ins_vec,
	const int& idx, const vector<cv::Point>& pt_vec,
	vector<cv::Point3d>& pt3_vec)
{
	const TrajectoryData& ins = ins_vec[idx];

	vector<cv::Point2f> density_road_points;
	vector<float> z_vec;
	buildRoadSurfaceByTrajectory(ins_vec, idx, density_road_points, z_vec);

	if (density_road_points.size() == 0)
	{
		return;
	}
	cv::flann::KDTreeIndexParams indexParams(1);
	cv::Mat source = cv::Mat(density_road_points).reshape(1);
	source.convertTo(source, CV_32F);
	cv::flann::Index ins_kdtree(source, indexParams);

	/////////////
	Eigen::Vector3f r_euler(0, 0, 0);
	Eigen::Vector3f t(0, 0, 0);
	rt(r_euler, t, ins);

	Eigen::Matrix3f r_matrix;
	TransRotation::eigenEuler2RotationMatrix(r_euler, r_matrix);
	r_matrix = r_matrix.inverse();

	///////////////
	Eigen::Vector3f R_euler;
	R_euler << M_PI / 2 + ins.roll, ins.pitch, ins.heading;

	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);

	Eigen::Matrix3f R_matrix_inv = R_matrix.inverse();
	Eigen::Vector3f temp_r = R_matrix_inv.eulerAngles(2, 1, 0) / M_PI * 180.0;
	//Eigen::Matrix3f p = R_matrix_inv * R_matrix;
	//Eigen::Matrix3d R_matrix_invd = R_matrix_inv.cast<double>();
	Eigen::Vector3d Trans;
	Trans << ins.point.x, ins.point.y, ins.point.z;

	// 	ofstream of("c.txt");
	// 	of << "x,y" << endl;

	vector<Eigen::Vector3f> p3f_vec;
	for (auto pt : pt_vec)
	{
		//search for elevation
		Eigen::Vector3f p3f;
		p3f[1] = searchElevationInCamera(ins_kdtree, pt, z_vec);
	//	OptimizeCeres::convertPixel2Camera3d((float)pt.x, (float)pt.y, p3f);

		string str = to_string(p3f[0]) + "," + to_string(p3f[2]);
		//		string str = to_string(pt.x) + "," + to_string(pt.y) + to_string(p3f[0]) + "," + to_string(p3f[1]) + "," + to_string(p3f[2]);
		//		of << str << endl;

		p3f -= t;
		p3f_vec.push_back(p3f);
	}

	forceModifyManual(p3f_vec);

	for (auto p3f : p3f_vec)
	{
		p3f = r_matrix * p3f;
		p3f = R_matrix_inv * p3f;

		cv::Point3d p3d;
		p3d.x = p3f[0] + Trans[0];
		p3d.y = p3f[1] + Trans[1];
		p3d.z = p3f[2] + Trans[2];

		pt3_vec.push_back(p3d);
	}
	ins_kdtree.release();
}

void LineRecon3D::calcInsertInsPoint(const TrajectoryData& s_ins, const TrajectoryData& e_ins, vector<TrajectoryData>& insert_ins)
{
	auto  p = e_ins.point - s_ins.point;
	double dis = sqrt(p.dot(p));
	double mid_dis = 1.0;
	int num = dis / mid_dis;

	insert_ins.push_back(s_ins);
	for (int i = 1; i < num; i ++)
	{
		TrajectoryData ins = s_ins;
		ins.point = s_ins.point + p / num * i;
		ins.roll = s_ins.roll + (e_ins.roll - s_ins.roll) / num * i;
		insert_ins.push_back(ins);
	}
	insert_ins.push_back(e_ins);

}

void LineRecon3D::calcTracePointsOnRoad(const vector<TrajectoryData>& ins_vec, int idx,
	vector<cv::Point3d>& insert_lr_ins_vec)
{
	double len = 60;
	//	double len = 150;
	vector<TrajectoryData> local_ins_vec;
	collectLocalIns(ins_vec, idx, len, local_ins_vec);

	vector<TrajectoryData> insert_front_ins_vec;
	//front

#if 0
	ofstream of("z_org.txt");
	of << "x,y,z" << endl;
	for_each(insert_lr_ins_vec.begin(), insert_lr_ins_vec.end(), [&of](const auto& ins) {
		string line = to_string(ins.x) + "," + to_string(ins.y) + "," + to_string(ins.z);
		of << line << endl;
	});
	of.close();
#endif

	auto itr_ins = local_ins_vec.begin();
	for (; itr_ins != local_ins_vec.end() - 1; itr_ins++)
	{
		const auto& ins = *itr_ins;
		const auto& ins_next = *(itr_ins + 1);
		vector<TrajectoryData> insert_ins;
		calcInsertInsPoint(ins, ins_next, insert_ins);

		auto org_sz = insert_front_ins_vec.size();
		insert_front_ins_vec.resize(org_sz + insert_ins.size() - 1);
		transform(insert_ins.begin(), insert_ins.end() - 1, insert_front_ins_vec.begin() + org_sz, [](const auto& s) {
			return s;
		});
	}

	//left+right
	float side_dis = 10.0;
	itr_ins = insert_front_ins_vec.begin();
	for (; itr_ins != insert_front_ins_vec.end(); itr_ins++)
	{
		const auto& ins = *itr_ins;
		TrajectoryData left_ins;
		left_ins.point = calPointByAltitude(ins.point, true, ins.heading, side_dis);

		TrajectoryData right_ins;
		right_ins.point = calPointByAltitude(ins.point, false, ins.heading, side_dis);
		
		//update z 
		left_ins.point.z = ins.point.z + side_dis * tan(ins.roll);
		right_ins.point.z = ins.point.z + side_dis * tan(-ins.roll);

		//left_ins.point.z = ins.point.z + side_dis * tan(-ins.roll);
		//right_ins.point.z = ins.point.z + side_dis * tan(ins.roll);

		vector<TrajectoryData> insert_ins;
		calcInsertInsPoint(left_ins, right_ins, insert_ins);
// 		insert_ins.push_back(ins);
// 		insert_ins.push_back(ins);

		auto org_sz = insert_lr_ins_vec.size();
		insert_lr_ins_vec.resize(org_sz + insert_ins.size() - 1);
		transform(insert_ins.begin(), insert_ins.end() - 1, insert_lr_ins_vec.begin() + org_sz, [](const auto& s) {
			return s.point;
		});
	}

	//for debug insert trajectory points
// 	ofstream of(ins_vec[idx].name + "z.txt");
// 	of << "x,y,z" << endl;
// 	for_each(insert_lr_ins_vec.begin(), insert_lr_ins_vec.end(), [&of](const auto& ins) {
// 		string line = to_string(ins.x) + "," + to_string(ins.y) + "," + to_string(ins.z);
// 		of << line << endl;
// 		});
// 	of.close();

}

#if 1
void LineRecon3D::buildRoadSurfaceByTrajectory(const vector<TrajectoryData>& ins_vec, int idx, vector<cv::Point2f>& density_road_points, vector<float>& z_vec)
{
	vector<cv::Point3d> insert_lr_ins_vec;
	calcTracePointsOnRoad(ins_vec, idx, insert_lr_ins_vec);
		
	const TrajectoryData& cur_ins = ins_vec[idx];

	Eigen::Vector3f r_euler(0, 0, 0);
	Eigen::Vector3f t(0, 0, 0);
	rt(r_euler, t, cur_ins);
	Eigen::Matrix3f r_matrix;
	TransRotation::eigenEuler2RotationMatrix(r_euler, r_matrix);

	Eigen::Vector3f R_euler;
	R_euler << M_PI / 2 + cur_ins.roll, cur_ins.pitch, cur_ins.heading;
	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);
	Eigen::Vector3d Trans;
	Trans << cur_ins.point.x, cur_ins.point.y, cur_ins.point.z;

//	cv::Mat t_img(2048, 2048, CV_8U);
//	cv::Mat t_img = cv::imread("D:\\0huinian\\data\\update\\49GT1_20180718_2\\004_G25changshengaosu_04\\Dlink\\00000000-01-20180718152431128.png");
//	cv::Mat t_img = cv::imread("D:\\data\\5VL86_20210417_2\\008_G4221hwgs_08\\Image\\2\\00000000-01-20210417165451790.jpg");

	//for debug
//	cv::Mat t_img = cv::imread("F:\\0hn\\4data\\whu_pc\\HD099_20181029_1\\location\\image\\1\\00000000-01-20181029120135253.jpg");
//	cv::Mat t_img = cv::imread("F:\\0hn\\4data\\whu_pc\\HD099_20181029_1\\location\\image\\1\\00000000-01-20181029120211610.jpg");
	
//	ofstream of("z_ego.txt");
//	of << "x,y,z" << endl;
	
	for (int i = 0; i < insert_lr_ins_vec.size(); i++)
	{
		auto& p = insert_lr_ins_vec[i];
		Eigen::Vector3d pp;
		pp << p.x, p.y, p.z - CalibSpace::ego_height;
		//pp << p.x, p.y, p.z;
		Eigen::Vector3f lp;
		CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);
		lp = r_matrix * lp + t;

		string line = to_string(lp[0]) + "," + to_string(lp[1]) + "," + to_string(lp[2]);
	//	of << line << endl;

		double xyz[3] = {lp[0], lp[1], lp[2]};
		double ij[2] = { -1, -1 };
		if (!OptimizeCeres::project2Image(xyz, ij, false))
		{
			continue;
		}
// 		ij[0] += CalibSpace::image_rect.tl().x;
// 		ij[1] += CalibSpace::image_rect.tl().y;
		density_road_points.emplace_back(cv::Point2f(ij[0], ij[1]));
		z_vec.emplace_back(xyz[1]);
//		cv::drawMarker(t_img, cv::Point(ij[0], ij[1]), (0,0,0));
	}
//	of.close();

#if 0
	////////////////////debug///////////////////for 画图
	vector<RAW_INS> local_ins_vec;
	collectLocalIns(ins_vec, idx, 60, local_ins_vec);


	of.open("z_ego_org.txt");
 	of << "x,y,z" << endl;

	for (int i = 0; i < local_ins_vec.size(); i++)
	{
		auto& p = local_ins_vec[i].point;
		Eigen::Vector3d pp;
		//pp << p.x, p.y, p.z - CalibSpace::ego_height;
		pp << p.x, p.y, p.z;
		Eigen::Vector3f lp;
		CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);
		lp = r_matrix * lp + t;

 		string line = to_string(lp[0]) + "," + to_string(lp[1]) + "," + to_string(lp[2]);
 		of << line << endl;
	}
	of.close();
#endif

//	cv::imwrite("0.jpg", t_img);

	//ofstream of("insert_z.txt");
	//of << "x,y,z" << endl;
	//for_each(density_road_points.begin(), density_road_points.end(), [&of](const auto& p) {
	//	string line = to_string(p.x) + "," + to_string(p.y) + "," + to_string(p.z);
	//	of << line << endl;
	//	});
	//of.close();
}
#else
void LineRecon3D::buildRoadSurfaceByTrajectory(const vector<TrajectoryData>& ins_vec, int idx, vector<cv::Point2f>& density_road_points, vector<float>& z_vec)
{
	double len = 60;
	//	double len = 150;
	vector<TrajectoryData> local_ins_vec;
	//世界坐标系
	collectLocalIns(ins_vec, idx, len, local_ins_vec);
	//变换到局部坐标系
	convInsFromWorld2Lidar(ins_vec, ins_vec[idx], local_ins_vec);


	const TrajectoryData& cur_ins = ins_vec[idx];

	Eigen::Vector3f r_euler(0, 0, 0);
	Eigen::Vector3f t(0, 0, 0);
	rt(r_euler, t, cur_ins);
	Eigen::Matrix3f r_matrix;
	TransRotation::eigenEuler2RotationMatrix(r_euler, r_matrix);


	//	cv::Mat t_img(2048, 2048, CV_8U);
	//	cv::Mat t_img = cv::imread("D:\\0huinian\\data\\update\\49GT1_20180718_2\\004_G25changshengaosu_04\\Dlink\\00000000-01-20180718152431128.png");
	//	cv::Mat t_img = cv::imread("D:\\data\\5VL86_20210417_2\\008_G4221hwgs_08\\Image\\2\\00000000-01-20210417165451790.jpg");

		//for debug
	//	cv::Mat t_img = cv::imread("F:\\0hn\\4data\\whu_pc\\HD099_20181029_1\\location\\image\\1\\00000000-01-20181029120135253.jpg");
	//	cv::Mat t_img = cv::imread("F:\\0hn\\4data\\whu_pc\\HD099_20181029_1\\location\\image\\1\\00000000-01-20181029120211610.jpg");

	ofstream of("z_ego.txt");
	of << "x,y,z" << endl;
	vector<Eigen::Vector3f> org_local_ins_vec;
	for (int i = 0; i < local_ins_vec.size() - 1; i++)
	{
		auto& p = local_ins_vec[i].point;
		Eigen::Vector3f lp;
		lp << p.x, p.y + CalibSpace::ego_height, p.z ;
		lp = r_matrix * lp + t;
		org_local_ins_vec.push_back(lp);
	}
	for (int i = 0; i < org_local_ins_vec.size() - 1; i++)
	{
		const Eigen::Vector3f& p = org_local_ins_vec[i];
		const Eigen::Vector3f& p_next = org_local_ins_vec[i+1];

		//front
		vector<Eigen::Vector3f> front_vec;
		Eigen::Vector3f  dp = p_next - p;
		double dis = sqrt(p.dot(p));
		double mid_dis = 1.0;
		int num = dis / mid_dis;

		front_vec.push_back(p);
		for (int i = 1; i <= num; i++)
		{
			auto tp = p + dp / num * i;
			front_vec.push_back(tp);
		}
		front_vec.push_back(p_next);

		//left and right
		vector<Eigen::Vector3f> lr_vec;
		for (int i = 0; i < front_vec.size(); i++)
		{
			const auto& p = front_vec[i];
			for (int j = -20; j <= 20; j++)
			{
				if ( j == 0)
				{
					continue;
				}
				Eigen::Vector3f lp;
				lp[0] = p[0] + j;
				lp[1] = p[1];
				lp[2] = p[2];
				lr_vec.push_back(lp);

				//project
				string line = to_string(lp[0]) + "," + to_string(lp[1]) + "," + to_string(lp[2]);
				of << line << endl;

				double xyz[3] = { lp[0], lp[1], lp[2] };
				double ij[2] = { -1, -1 };
				if (!OptimizeCeres::project2Image(xyz, ij, false))
				{
					continue;
				}
				density_road_points.emplace_back(cv::Point2f(ij[0], ij[1]));
				z_vec.emplace_back(xyz[1]);
				//		cv::drawMarker(t_img, cv::Point(ij[0], ij[1]), (0,0,0));
			}
		}
	}
	of.close();

	//	cv::imwrite("0.jpg", t_img);

		//ofstream of("insert_z.txt");
		//of << "x,y,z" << endl;
		//for_each(density_road_points.begin(), density_road_points.end(), [&of](const auto& p) {
		//	string line = to_string(p.x) + "," + to_string(p.y) + "," + to_string(p.z);
		//	of << line << endl;
		//	});
		//of.close();
}
#endif

void LineRecon3D::adjustLinesInFrame()
{
	if (m_ins_vec.size() == 0)
	{
		return;
	}

//	KalmanPred KP;
// 	ofstream of("kf.txt");
// 	of << "x,y,px,py" << endl;
	int sz = 0;
	//	int sz = ins_vec.size() - 1;
	int save_sz = 0;
	auto itr_ins = m_ins_vec.begin() + sz;

	int is_start = false;
	TrajectoryData last_ins;
	for (; itr_ins != m_ins_vec.end() - 1; itr_ins++, sz++)
	{
		const TrajectoryData& ins = *itr_ins;
		string img_na = itr_ins->name;
		LOG(INFO) << ("%d:...%s", sz, img_na.c_str());
		if(img_na < "00000000-01-20210417165527190")
		{
//			continue;
		}

		auto get_obj = m_obj_frame_vec.find(img_na);
		if (get_obj == m_obj_frame_vec.end())
		{
			continue;
		}

		if (m_frame_spt_map.size() > 0 &&
			dist(itr_ins->point, last_ins.point) < 10.0)
		{
			continue;
		}

		vector<FrameStartPoint> fsp_vec;
		calFrameStartEndPoints(m_ins_vec, sz, fsp_vec);
		if (fsp_vec.size() == 0)
		{
			continue;
		}
		string file_na = m_data_path + RECON_LANE_STARTS;
		saveLaneFrameStarts(file_na, fsp_vec);

		m_frame_spt_map[img_na] = fsp_vec;
		last_ins = *itr_ins;

	}
// 	of.close();
}



void LineRecon3D::mergeLocalObjects(Object3D_VEC& local_obj_vec)
{
	auto itr_obj = local_obj_vec.begin();
	for (; itr_obj != local_obj_vec.end();)
	{
		if (itr_obj->shape.size() < 2 ||
			length(itr_obj->shape) < 0.5)
		{
			itr_obj = local_obj_vec.erase(itr_obj);
		}
		else
		{
			itr_obj++;
		}
	}

	itr_obj = local_obj_vec.begin();
	for (; itr_obj != local_obj_vec.end();)
	{
		auto& cur_obj = *itr_obj;
		equalizPolyline(cur_obj.shape, 1.0);

		auto itr_other = local_obj_vec.begin();
		for (; itr_other != local_obj_vec.end(); itr_other++)
		{
			if (itr_other == itr_obj)
			{
				continue;
			}
			const auto& other_obj = *itr_other;
			if (isMergeLane(other_obj.shape, cur_obj.shape, 0.5))
			{
				break;
			}
		}
		
		if (itr_other == local_obj_vec.end())
		{
			itr_obj++;
			continue;
		}
		const auto& merge_obj = *itr_other;
		const auto& ept = cur_obj.shape.back();
		vector<cv::Point3d> merge_line;
		merge_line.push_back(ept);
		copy_if(merge_obj.shape.begin(), merge_obj.shape.end(), back_inserter(merge_line),
			[&](const auto& pt)->bool {
			return pt.y > ept.y;
		});	
		equalizPolyline(merge_line, 1.0);
		copy(merge_line.begin() + 1, merge_line.end(), back_inserter(cur_obj.shape));
		local_obj_vec.erase(itr_other);
	}

	itr_obj = local_obj_vec.begin();
	for (; itr_obj != local_obj_vec.end();)
	{
		if (itr_obj->shape.size() < 2 ||
			length(itr_obj->shape) < 3.0)
		{
			itr_obj = local_obj_vec.erase(itr_obj);
		}
		else
		{
			itr_obj++;
		}
	}
}


void LineRecon3D::collectLocalObjects(const vector<TrajectoryData>& local_ins_vec, Object3D_VEC& local_obj_vec)
{
	for_each(local_ins_vec.begin(), local_ins_vec.end(), [&, this](const auto& ls) {
		auto get_obj = m_obj_frame_vec.find(ls.name);
		if (get_obj != m_obj_frame_vec.end())
		{
			copy(get_obj->second.begin(), get_obj->second.end(), back_inserter(local_obj_vec));
		}
	});

}

float calLineGroupHeading(const Object3D_VEC& local_obj_vec, float ref_heading)
{
	vector<float> heading_vec;
	for_each(local_obj_vec.begin(), local_obj_vec.end(), [&](const auto& obj) {
		for (int i = 0; i < obj.shape.size() - 1; i++)
		{
			double heading = calcPolylineYaw2(obj.shape, i);
			heading_vec.push_back(heading);	
		}
	});

	float avg_heading = 0;
	int sz = 0;
	for (auto heading : heading_vec)
	{
		if ((heading - ref_heading) / M_PI * 180 > 350)
		{
			heading -= M_PI * 2.0;
		}
		else if ((heading - ref_heading) / M_PI * 180 < -350)
		{
			heading += M_PI * 2.0;
		}

		if (abs(heading - ref_heading) / M_PI * 180 > 30)
		{
			continue;
		}
		avg_heading += heading;
		sz++;
	}
	if (sz > 0)
	{
		avg_heading /= sz;
	}
	else
	{
		avg_heading = ref_heading;
	}
	return avg_heading;
}

void LineRecon3D::calFrameStartEndPoints(const vector<TrajectoryData>& ins_vec, int idx, 
	vector<FrameStartPoint>& fsp_vec)
{
	double len = 70;

	vector<TrajectoryData> local_ins_vec;
	collectLocalIns(ins_vec, idx, len / 2.0, local_ins_vec, false, false);
	collectLocalIns(ins_vec, idx, len / 2.0, local_ins_vec, false, true);

	Object3D_VEC local_obj_vec;
	collectLocalObjects(local_ins_vec, local_obj_vec);

	const TrajectoryData& ins = ins_vec[idx];
	calFrameStartEndPoints(ins, local_obj_vec, fsp_vec);
}

void sortLocalObjects(vector<FrameStartPoint>& fsp_vec)
{
	set<float> x_clusters;
	for (auto fsp : fsp_vec)
	{
		x_clusters.insert(fsp.h_dist);
	}

	auto get_right = find_if(x_clusters.begin(), x_clusters.end(), [](const auto& x)->bool {
		return x > 0;
	});
	int right_no = distance(x_clusters.begin(), get_right);
	for (auto& fsp : fsp_vec)
	{
		auto pos = x_clusters.find(fsp.h_dist);
		fsp.no = distance(x_clusters.begin(), pos) - right_no;
	}
}

void LineRecon3D::clusterLocalObjects(vector<vector<double>> pts, Object3D_VEC& local_local_obj_vec)
{
	if (pts.size() == 0)
	{
		return;
	}
	dbscan::Dbscan<double> dbs(2, 0.3, 2);
	dbs.Run(pts);

	vector<vector<vector<double>>> res;
	dbs.GetCluster(res);

	Object3D_VEC cluster_objs;
	for (auto cluster : res)
	{
		float cluster_center = 0;
		vector<cv::Point2d> line;
		for (auto pt : cluster)
		{
			cv::Point2d cp(pt[0], pt[1]);
			line.push_back(cp);
		}
		fitByOpenCV(line, 2);

		Object3D o3d;
		o3d.type = 13;
		for_each(line.begin(), line.end(), [&](const auto& p) {
			cv::Point3d p3(p.x, p.y, 0);
			o3d.shape.emplace_back(p3);
		});
		cluster_objs.push_back(o3d);
	}
	local_local_obj_vec.swap(cluster_objs);
}

void LineRecon3D::calFrameStartEndPoints(const TrajectoryData& ins, const Object3D_VEC& local_obj_vec,
	vector<FrameStartPoint>& fsp_vec)
{
	float heading = calLineGroupHeading(local_obj_vec, ins.heading);

// 	ofstream of("h.txt");
// 	of << "x,y" << endl;

//	Eigen::Vector3f R_euler(M_PI / 2,0, ins.heading);

//	Eigen::Vector3f R_euler(M_PI / 2, 0, heading);
	Eigen::Vector3f R_euler(0, 0, heading);
//	LOG_INFO("%s-%.2f-%.2f", ins.name.c_str(), ins.heading, heading);

	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);
	Eigen::Vector3d Trans;
	Trans << ins.point.x, ins.point.y, ins.point.z;

	Object3D_VEC local_local_obj_vec = local_obj_vec;

	vector<vector<double>> db_pts;

	for (int i = 0; i < local_local_obj_vec.size(); i++)
	{
		auto& obj = local_local_obj_vec[i];
		for (int j = 0; j < obj.shape.size(); j++)
		{
			auto& p = obj.shape[j];
			Eigen::Vector3d pp;
			pp << p.x, p.y, p.z;
			Eigen::Vector3f lp;
			CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);
			p.x = lp[0];
			p.y = lp[1];
			p.z = lp[2];	

			vector<double> pt(2);
			pt[0] = p.x;
			pt[1] = p.y;
			db_pts.push_back(pt);
		}
	}

	clusterLocalObjects(db_pts, local_local_obj_vec);

	mergeLocalObjects(local_local_obj_vec);


	int no = 0;
	for (const auto& obj : local_local_obj_vec)
	{
		auto get_y = min_element(obj.shape.begin(), obj.shape.end(), [](const auto& ele1, const auto& ele2) ->bool {
			return abs(ele1.y) < abs(ele2.y);
		});
		cv::Point3f local_left_start_point = *get_y;
		if (abs(local_left_start_point.y) > 1.0)
		{
//			continue;
		}
		
		Eigen::Vector3f pf(local_left_start_point.x, local_left_start_point.y, local_left_start_point.z);
		Eigen::Vector3d pd = (R_matrix.inverse() * pf).cast<double>() + Trans;
		cv::Point3d sp = cv::Point3d(pd[0], pd[1], pd[2]);
		FrameStartPoint fsp;
		fsp.ins = ins;
		fsp.pt = sp;
		fsp.no = 0;
		fsp.h_dist = local_left_start_point.x;

		get_y = min_element(obj.shape.begin(), obj.shape.end(), [](const auto& ele1, const auto& ele2) ->bool {
			return abs(ele1.y - 10) < abs(ele2.y - 10);
		});
		cv::Point3f local_left_e_point = *get_y;

		if (local_left_e_point.y - local_left_start_point.y < 1.0)
		{
			continue;
		}
		Eigen::Vector3f epf(local_left_e_point.x, local_left_e_point.y, local_left_e_point.z);
		Eigen::Vector3d epd = (R_matrix.inverse() * epf).cast<double>() + Trans;
		fsp.ept = cv::Point3d(epd[0], epd[1], epd[2]);

		fsp_vec.emplace_back(fsp);
	}
	sortLocalObjects(fsp_vec);

	return;

#if 0
	vector<double> x_vec;
	for_each(local_local_obj_vec.begin(), local_local_obj_vec.end(), [&](const auto& obj) {
		for_each(obj.shape.begin(), obj.shape.end(), [&](const auto& pt){
			x_vec.push_back(pt.x);
		});
	});
//	sort(x_vec.begin(), x_vec.end());

	vector<vector<double>> pts;
	transform(x_vec.begin(), x_vec.end(), back_inserter(pts), [](const auto& x)->vector<double> {
		return{ x };
	});
//	of.close();

	//
	if (pts.size() == 0)
	{
		return;
	}
	dbscan::Dbscan<double> dbs(1, 0.5, 10);
	dbs.Run(pts);

	vector<vector<vector<double>>> res;
	dbs.GetCluster(res);

	set<float> x_clusters;
	for ( auto cluster : res)
	{
		float cluster_center = 0;
		for (auto pt : cluster)
		{
			cluster_center  += pt[0];
		}
		cluster_center /= cluster.size();
		x_clusters.insert(cluster_center);
	}

	auto get_right = find_if(x_clusters.begin(), x_clusters.end(), [](const auto& x)->bool {
		return x > 0;
	});
	int right_no = distance(x_clusters.begin(), get_right);

	auto itr_x = x_clusters.begin();
	int no = 0;
	for (; itr_x != x_clusters.end(); itr_x++, no++)
	{
		int line_no = no - right_no;
		double x = *itr_x;
		cv::Point3d sp = getStartPoint(local_local_obj_vec, x, R_matrix, Trans);
		FrameStartPoint fsp;
		fsp.ins = ins;
		fsp.pt = sp;
		fsp.no = line_no;
		fsp.h_dist = x;
		fsp_vec.emplace_back(fsp);
		/*auto& line = m_line_map[line_no];
		if (line.size() == 0)
		{
			line.push_back({ sp });
		}
		else
		{
			const auto& back = line.back().back();
			if (dist(sp, back) > 50.0 ||
				abs(calcPointYaw(back, sp) - ins.heading) > M_PI / 2)
			{
				line.push_back({ sp });
			}
			else
			{
				line.back().push_back(sp);
			}
		}*/
	}
#endif
}


cv::Point3d LineRecon3D::getStartPoint(const Object3D_VEC& local_local_obj_vec, 
	const double& x,
	const Eigen::Matrix3f& R_matrix,
	const Eigen::Vector3d& Trans	)
{
	vector<cv::Point3f> this_line;
	for (int i = 0; i < local_local_obj_vec.size(); i++)
	{
		auto& obj = local_local_obj_vec[i];
		for (int j = 0; j < obj.shape.size(); j++)
		{
			if (abs(obj.shape[j].x - x) < 0.5)
			{
				this_line.push_back(cv::Point3f(obj.shape[j]));
			}
		}
	}
	auto get_y = min_element(this_line.begin(), this_line.end(), [](const auto& ele1, const auto& ele2) ->bool {
	//	return ele1.z < ele2.z;
		return abs(ele1.y) < abs(ele2.y);
	});

	cv::Point3f local_left_start_point = *get_y;
	Eigen::Vector3f pf(local_left_start_point.x, local_left_start_point.y, local_left_start_point.z);
	Eigen::Vector3d pd = (R_matrix.inverse() * pf).cast<double>() + Trans;
	cv::Point3d lsp = cv::Point3d(pd[0], pd[1], pd[2]);

	return lsp;
}



bool findNearestExistLineStart(const HPolyline3d& last_line, const vector<HPolyline3d>& cur_lines,
	int& lane_idx, float&angle, bool left)
{
	int idx = 0;
	map<float, int> angle_idx_map;
	for (const auto& exist_line : cur_lines)
	{
		//已存在车道线的起点
		const auto& exist_l_line = cur_lines[idx];
		if (exist_l_line.size() == 0)
		{
			idx++;
			continue;
		}
		//2.5米以内有车道线，就算！没有车道小于0.5m
		if (!isTheSameLane(exist_l_line, last_line, 2.5))
		{
			idx++;
			continue;
		}

		angle = getAngle(exist_l_line, last_line);
		angle_idx_map.insert(make_pair(angle, idx));
		idx++;
	}

	if (angle_idx_map.size() == 0)
	{
		return false;
	}
	lane_idx = angle_idx_map.begin()->second;
	angle = angle_idx_map.begin()->first;
	return true;
}

bool findMatchedLinesStart(const vector<HPolyline3d>& cur_lines,
	const vector<HPolyline3d>& next_lines,
	map<int, int>& cur_last_map)
{
	int idx = 0;
	map<int, float> match_idx_angle_map;

	auto itr = cur_lines.begin();
	for (; itr != cur_lines.end(); itr++, idx++)
	{
		const auto& cur_line = *itr;
		if (cur_line.size() == 0)
		{
			continue;
		}
		//根据起点位置，查找上一帧里最接近的车道线，串起来
		const auto& spt = cur_line.front();
		int match_idx = idx;
		float match_angle = 90;
		bool find_flg = findNearestExistLineStart(cur_line, next_lines, match_idx, match_angle, true);
		if (find_flg)
		{
			//这里默认左边连接！
			//高速场景，右边分歧较多，将右侧的车道线断开，左侧保持连续
			auto find_already_matched = find_if(cur_last_map.begin(), cur_last_map.end(), [&](const auto& _pair)->bool {
				return _pair.second == match_idx;
			});
			if (find_already_matched != cur_last_map.end())
			{
				const auto& already_matched_angle = match_idx_angle_map[match_idx];
				if (already_matched_angle < match_angle)
				{
					continue;
				}
				else
				{
					cur_last_map.erase(find_already_matched);
				}
			}
			cur_last_map.insert(make_pair(idx, match_idx));
			match_idx_angle_map[match_idx] = match_angle;
		}
	}

	return cur_last_map.size() > 0;
}

void getScaleZOnStrechPolyline(const HPolyline3d& _polyline, HPoint3d& stretch_pt, bool be_start)
{
	if (_polyline.size() == 0)
	{
		return;
	}

	double two_dist = 0;
	double strech_dist = 0;


	HPoint3d pt1;
	HPoint3d pt2;
	if (be_start)
	{
		//起始处往外延伸
		//z值按照前两个点的z值变化按比例赋值

		pt1 = _polyline[0];
		pt2 = _polyline[1];
	}
	else
	{
		//尾部向外延伸
		//z值按照最后两个点的z值变化按比例赋值
		int pt_sz = _polyline.size();
		pt1 = _polyline[pt_sz - 1];
		pt2 = _polyline[pt_sz - 2];
	}
	two_dist = PlaneMath<double>::getTwoPointsDistance(pt1, pt2);
	strech_dist = PlaneMath<double>::getTwoPointsDistance(pt1, stretch_pt);
	if (two_dist < 0.1)
	{
		stretch_pt.m_z = pt1.m_z;
		return;
	}
	stretch_pt.m_z = pt1.m_z + (pt1.m_z - pt2.m_z) / two_dist * strech_dist;

}

void stretchPolylineAtSpoint(const HPolyline3d& _polyline, HPolyline3d& _stretch_polyline, double _dist)
{
	if (_polyline.size() < 2)
	{
		return;
	}
	double yaw = calcPointYaw(_polyline[1], _polyline[0]);

	HPoint3d stretch_pt;
	stretch_pt.m_x = _polyline[0].m_x + _dist * sin(yaw);
	stretch_pt.m_y = _polyline[0].m_y + _dist * cos(yaw);
	//z值按照前两个点的z值变化按比例赋值
	getScaleZOnStrechPolyline(_polyline, stretch_pt, true);
	// 	double two_dist = PlaneMath<double>::getTwoPointsDistance(_polyline[1], _polyline[0]);
	// 	stretch_pt.m_z = _polyline[0].m_z + (_polyline[0].m_z - _polyline[1].m_z) / two_dist * _dist;

	_stretch_polyline.resize(_polyline.size() + 1);
	_stretch_polyline[0] = stretch_pt;
	copy(_polyline.begin(), _polyline.end(), _stretch_polyline.begin() + 1);

}

void stretchPolylineAtEpoint(const HPolyline3d& _polyline, HPolyline3d& _stretch_polyline, double _dist)
{
	if (_polyline.size() < 2)
	{
		return;
	}
	double yaw = calcPointYaw(_polyline[_polyline.size() - 2], _polyline[_polyline.size() - 1]);

	HPoint3d stretch_pt;
	stretch_pt.m_x = _polyline[_polyline.size() - 1].m_x + _dist * sin(yaw);
	stretch_pt.m_y = _polyline[_polyline.size() - 1].m_y + _dist * cos(yaw);
	//z值按照最后两个点的z值变化按比例赋值
	double two_dist = PlaneMath<double>::getTwoPointsDistance(_polyline[_polyline.size() - 2], _polyline[_polyline.size() - 1]);
	stretch_pt.m_z = _polyline[_polyline.size() - 1].m_z + (_polyline[_polyline.size() - 1].m_z - _polyline[_polyline.size() - 2].m_z) / two_dist * _dist;
	getScaleZOnStrechPolyline(_polyline, stretch_pt, false);

	_stretch_polyline.resize(_polyline.size() + 1);
	_stretch_polyline[_stretch_polyline.size() - 1] = stretch_pt;
	copy(_polyline.begin(), _polyline.end(), _stretch_polyline.begin());
}


double getLaneWidth(const HPolyline3d& a_ply, const HPolyline3d& b_ply, bool be_start)
{
	double width = 0;

	if (a_ply.size() == 0 ||
		b_ply.size() == 0)
	{
		return width;
	}

	HPoint3d a_s = a_ply.front();
	HPoint3d a_e = a_ply.back();
	HPoint3d b_s = b_ply.front();
	HPoint3d b_e = b_ply.back();

	HPoint3d near_s_b;
	HPoint3d near_s_a;

	HPolyline3d strech_a_ply;
	HPolyline3d strech_temp;
	HPolyline3d strech_b_ply;

	double s_b_a = 0;
	double s_a_b = 0;
	if (be_start)
	{
		s_b_a = KPolylineAlg::calcP2PolylineNearestPoint(a_ply, b_s, near_s_b);
		//b起点的最近点与a的尾点重合，将a的尾巴延伸
		if (abs(near_s_b.m_x - a_e.m_x) < 0.01 && abs(near_s_b.m_y - a_e.m_y) < 0.01)
		{
			stretchPolylineAtEpoint(a_ply, strech_a_ply, 100.0);
			s_b_a = KPolylineAlg::calcP2PolylineNearestPoint(strech_a_ply, b_s, near_s_b);
		}

		s_a_b = KPolylineAlg::calcP2PolylineNearestPoint(b_ply, a_s, near_s_a);
		//a起点的最近点与b的尾点重合，将b的尾巴延伸
		if (abs(near_s_a.m_x - b_e.m_x) < 0.01 && abs(near_s_a.m_y - b_e.m_y) < 0.01)
		{
			stretchPolylineAtEpoint(b_ply, strech_b_ply, 100.0);
			s_a_b = KPolylineAlg::calcP2PolylineNearestPoint(strech_b_ply, a_s, near_s_a);
		}
	}
	else
	{
		s_b_a = KPolylineAlg::calcP2PolylineNearestPoint(a_ply, b_e, near_s_b);
		//b尾点的最近点与a的起点重合，将a的起始延伸
		if (abs(near_s_b.m_x - a_s.m_x) < 0.01 && abs(near_s_b.m_y - a_s.m_y) < 0.01)
		{
			stretchPolylineAtSpoint(a_ply, strech_a_ply, 100.0);
			s_b_a = KPolylineAlg::calcP2PolylineNearestPoint(strech_a_ply, b_s, near_s_b);
		}

		s_a_b = KPolylineAlg::calcP2PolylineNearestPoint(b_ply, a_e, near_s_a);
		//a尾点的最近点与b的起点重合，将b的起始延伸
		if (abs(near_s_a.m_x - b_s.m_x) < 0.01 && abs(near_s_a.m_y - b_s.m_y) < 0.01)
		{
			stretchPolylineAtSpoint(b_ply, strech_b_ply, 100.0);
			s_a_b = KPolylineAlg::calcP2PolylineNearestPoint(strech_b_ply, a_s, near_s_a);
		}
	}

	width = min(s_b_a, s_a_b);
	return width;
}

//计算上游的车道宽
bool calcLastLaneWidth(const HPolyline3d& ref_line, const vector<HPolyline3d>& last_lines, const int& last_idx,
	double& l_last_e_width, double& r_last_e_width, bool& to_left)
{
	if (ref_line.size() == 0)
	{
		return false;
	}

	const auto& s_pt = ref_line.front();
	const auto& e_pt = ref_line.back();

	//查找参考线在上一帧里对应的车道线
	auto find_ref_last = find_if(last_lines.begin(), last_lines.end(), [&](const auto& lastline)->bool {
		return isTheSameLane(ref_line, lastline,0.5);
	});
	if (find_ref_last == last_lines.end())
	{
		return false;
	}

	const auto& ref_last_line = (*find_ref_last);
	const auto& hole_last_line = last_lines[last_idx];

	l_last_e_width = getLaneWidth(ref_last_line, hole_last_line, false);
	to_left = PointAtLineLR(ref_last_line.front(), ref_last_line.back(), hole_last_line.front()) == 1;

	return true;
}






void getShapeVec(const Object3D_VEC& obj_vec, vector<HPolyline3d>& plys)
{
	plys.resize(obj_vec.size());
	transform(obj_vec.begin(), obj_vec.end(), plys.begin(), [](const auto& obj)->HPolyline3d {
		HPolyline3d kply(obj.shape.size());
		transform(obj.shape.begin(), obj.shape.end(), kply.begin(), [](const auto& pt)->HPoint3d {
			return HPoint3d(pt.x, pt.y, pt.z);
		});
		return kply;
	});
}

void getFspShapeVec(const vector<FrameStartPoint>& obj_vec, vector<HPolyline3d>& plys)
{
	plys.resize(obj_vec.size());
	transform(obj_vec.begin(), obj_vec.end(), plys.begin(), [](const auto& obj)->HPolyline3d {
		HPolyline3d kply(2);
		kply[0] = HPoint3d(obj.pt.x, obj.pt.y, obj.pt.z);
		kply[1] = HPoint3d(obj.ept.x, obj.ept.y, obj.ept.z);
		return kply;
	});
}
map<string, vector<HPolyline3d>> m_obj_kplys_vec;

void organizeLastEBlockData(set<string> old_e_string_set, vector<string>& total_se_ids, vector<HPolyline3d>& total_last_plys)
{
	//eblock结束的车道线们
	//统一装起来
	auto e_itr = old_e_string_set.begin();
	for (; e_itr != old_e_string_set.end(); e_itr++)
	{
		auto e_line_id = *e_itr;
		auto find_e_plys = m_obj_kplys_vec.find(e_line_id);
		if (find_e_plys == m_obj_kplys_vec.end())
		{
			continue;
		}
		const auto& last_plys = find_e_plys->second;
		std::copy(last_plys.begin(), last_plys.end(), back_inserter(total_last_plys));
		for (int i = 0; i < last_plys.size(); i++)
		{
			total_se_ids.push_back(e_line_id);
		}
	}

}

bool findNearestExistLineEnd(const HPolyline3d& sline, const vector<HPolyline3d>& last_lines,
	int& lane_idx, float& angle, bool left)
{
	int idx = 0;
	map<float, int> angle_idx_map;
	for (const auto& exist_line : last_lines)
	{
		//已存在车道线的尾点
		const auto& exist_l_line = last_lines[idx];
		if (exist_l_line.size() == 0)
		{
			idx++;
			continue;
		}
		if (!isTheSameLane(sline, exist_l_line,0.5))
		{
			idx++;
			continue;
		}

		angle = getAngle(sline, exist_l_line);
		angle_idx_map.insert(make_pair(angle, idx));
		idx++;
	}

	if (angle_idx_map.size() == 0)
	{
		return false;
	}
	lane_idx = angle_idx_map.begin()->second;
	angle = angle_idx_map.begin()->first;
	return true;
}

bool findMatchedLinesEnd(const string& cur_block_id, const vector<HPolyline3d>& cur_lines, const vector<HPolyline3d>& last_lines, map<int, int>& cur_last_map)
{

	//车道合流开始
	//从上一帧待匹配的集合中剔除车道合流的引导线
	vector<HPolyline3d> last_lines_bak = last_lines;
	int remove_idx = -1;

	int idx = 0;
	map<int, float> match_idx_angle_map;

	auto itr = cur_lines.begin();
	for (; itr != cur_lines.end(); itr++, idx++)
	{
		//左线
		const auto& cur_line = *itr;
		if (cur_line.size() == 0)
		{
			continue;
		}
		//根据起点位置，查找上一帧里最接近的车道线，串起来
		const HPoint3d& spt = cur_line.front();
		int match_idx = idx;
		float  match_angle = 90;
		//left
		bool find_flg = findNearestExistLineEnd(cur_line, last_lines_bak, match_idx, match_angle, true);
		if (find_flg)
		{
#if 1
			//还原剔除引导线以前的索引顺序
			if (remove_idx != -1 && match_idx >= remove_idx)
			{
				match_idx++;
			}

			//这里默认左边连接！
			//高速场景，右边分歧较多，将右侧的车道线断开，左侧保持连续
			//2019/6/17但是分歧路口一般用宽虚线引导，不用和细线连起来
			//需要连接的，一般是右侧车道增加，保持右侧线连续
			auto find_already_matched = find_if(cur_last_map.begin(), cur_last_map.end(), [&](const auto& _pair)->bool {
				return _pair.second == match_idx;
			});
			if (find_already_matched != cur_last_map.end())
			{
				const auto& already_matched_angle = match_idx_angle_map[match_idx];
				if (already_matched_angle < match_angle)
				{
					continue;
				}
				else
				{
					cur_last_map.erase(find_already_matched);
				}
			}
#endif

			cur_last_map.insert(make_pair(idx, match_idx));
			match_idx_angle_map[match_idx] = match_angle;
		}
	}

	return cur_last_map.size() > 0;
}


void LineRecon3D::analyzePrecision(const map<string, Object3D_VEC>& obj_frame_vec)
{
	//transform point format
	ObjectDiff_VEC od_vec;
	std::for_each(obj_frame_vec.begin(), obj_frame_vec.end(),  [&](const auto& frame_item) {
		const auto& obj_vec = frame_item.second;
		for_each(obj_vec.begin(), obj_vec.end(), [&, this](const auto& obj) {

			ObjectDiff od;
			od.obj_id = to_string(od_vec.size());
			od.prop_id = obj.prop_id;
			od.shape.resize(obj.shape.size());
			
			transform(obj.shape.begin(), obj.shape.end(), od.shape.begin(), [&](const auto& pt)->HPoint3d {
				return HPoint3d(pt.x, pt.y, pt.z);
				});

			KPolylineAlg::calcBoundBox(od.shape, od.rect);
			od_vec.push_back(od);
			});

			
		});


	const HDObject_VEC& hd_line_vec = m_hd_map.ld_vec;
	ObjectDiff_VEC hd_vec;
	for_each(hd_line_vec.begin(), hd_line_vec.end(), [&](const auto& obj) {
		ObjectDiff od;
		od.obj_id = obj.obj_id;
		od.prop_id = obj.prop_id;
		od.shape.resize(obj.shape.size());
		transform(obj.shape.begin(), obj.shape.end(), od.shape.begin(), [&](const auto& pt)->HPoint3d {
			return HPoint3d(pt.x, pt.y, pt.z);
			});
	
		KPolylineAlg::calcBoundBox(od.shape, od.rect);
		hd_vec.push_back(od);
	});		

	ShpDifference shp_dif;
	shp_dif.calcDiff(od_vec, hd_vec);

	string file_na = m_data_path + RECON_DIFF + getPostfix(m_recon_method) + ".csv";
	saveDiff(file_na, od_vec);
}

void getIndexPoint(double x, double dd, int& xx)
{
	xx = int(x / dd) * dd * 10;
}

void LineRecon3D::analyzeMeshDiff(const map<string, Object3D_VEC>& obj_frame_vec)
{
	const HDObject_VEC& junction_vec = m_hd_map.junction_vec;
	
	const HDObject_VEC& hd_line_vec = m_hd_map.ld_vec;
	Mesh_VEC hd_vec;
	Mesh_MAP hd_map;
	double d = 0.25;
	double dd = 2 * d;

	for_each(hd_line_vec.begin(), hd_line_vec.end(), [&](const auto& obj) {
		auto shp = obj.shape;
		equalizPolyline(shp, 1.0);
		for_each(shp.begin(), shp.end(), [&](const auto& pt) {
			vector<cv::Point2d> box(4);
			
			box[0] = cv::Point2d(pt.x - d, pt.y - d);
			box[1] = cv::Point2d(pt.x + d, pt.y - d);
			box[2] = cv::Point2d(pt.x - d, pt.y + d);
			box[3] = cv::Point2d(pt.x + d, pt.y + d);

			for (int i = 0; i < 4 ; i++)
			{
				Mesh mesh;
				getIndexPoint(box[i].x, dd, mesh.p.x);
				getIndexPoint(box[i].y, dd, mesh.p.y);
				mesh.z = pt.z;

				pair<int, int> xy(mesh.p.x, mesh.p.y);
				hd_map.insert(make_pair(xy, mesh));
			}
			});
	});

	for_each(hd_map.begin(), hd_map.end(), [&](const auto& _pair) {
		hd_vec.push_back(_pair.second);
		});

	string file_na = m_data_path + MAP_MESH;
	saveMesh(file_na, hd_vec);

	//transform point format
	Object3D_VEC outers;
	std::for_each(obj_frame_vec.begin(), obj_frame_vec.end(), [&](const auto& frame_item) {
		const auto& obj_vec = frame_item.second;
		for_each(obj_vec.begin(), obj_vec.end(), [&, this](const auto& obj) {
			auto dif_obj = obj;
			dif_obj.shape.clear();
			for_each(obj.shape.begin(), obj.shape.end(), [&](const auto& pt) {
				pair<int, int> xy;
				
				getIndexPoint(pt.x, dd, xy.first);
				getIndexPoint(pt.y, dd, xy.second);

				auto check_itr = hd_map.find(xy);
				if (check_itr == hd_map.end())
				{
					auto check_junction_itr = find_if(junction_vec.begin(), junction_vec.end(), [&](const auto& _junc)->bool {
						cv::Point2d p2(pt.x, pt.y);
						return _junc.rect_3d.contains(p2);
						});
					if (check_junction_itr == junction_vec.end())
					{
						dif_obj.shape.push_back(pt);
					}
					
				}
			});
			if (dif_obj.shape.size() > 1)
			{
				outers.push_back(dif_obj);
			}
		});


	});
	file_na = m_data_path + RECON_MESH;
	int shp_type = 1;
	saveObject(file_na, outers, shp_type);

	clusterDiffLocalObjects(outers);

}

void LineRecon3D::clusterDiffLocalObjects(const Object3D_VEC& outers)
{
	vector<vector<double>> pts;
	for_each(outers.begin(), outers.end(), [&](const auto& outer) {
		for_each(outer.shape.begin(), outer.shape.end(), [&](const auto& pt) {
			pts.push_back({pt.x, pt.y});
			});
	});
	if (pts.size() == 0)
	{
		return;
	}
	dbscan::Dbscan<double> dbs(2, 1.0, 10);
	dbs.Run(pts);

	vector<vector<vector<double>>> res;
	dbs.GetCluster(res);

	Object3D_VEC cluster_objs;

	for (auto cluster : res)
	{
		float cluster_center = 0;
		vector<cv::Point> cpts;
		for (auto pt : cluster)
		{
			cv::Point cp(pt[0], pt[1]);
			cpts.push_back(cp);
		}
		Point_VEC hull;
		cv::convexHull(cpts, hull);

		Object3D o3d;
		o3d.type = 13;
		for_each(hull.begin(), hull.end(), [&](const auto& p) {
			cv::Point3d p3(p.x, p.y, 0);
			o3d.shape.emplace_back(p3);
			});

		cluster_objs.push_back(o3d);
	}
	
	string file_na = m_data_path + RECON_DIF_CLUSTER;
	int shp_type = 2;
	saveObject(file_na, cluster_objs, shp_type);

}

void LineRecon3D::inversePerspectiveAdjustForRecon(const string& img_na, map<int, LINE_VEC>& lines_map)
{
	auto find_lines = lines_map.find(OC_lane);
	if (find_lines == lines_map.end())
	{
		return;
	}
	auto& lines = find_lines->second;
	if (lines.size() == 0)
	{
		return;
	}

	splitLaneContours(lines);

	return;
}