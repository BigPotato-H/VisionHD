#include "AlignImageHD.h"
#include "DataIO.h"
#include "DataIOArgoverse.h"

#include <opencv2/flann.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>


#include <opencv2/highgui.hpp>

#include<Eigen/Core>
#include<Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "CCalib.h"
#include "TestCalib.h"
#include "Optimize.h"

//#include "HNMath/CoordinateTransform.h"
#include "HNMath/TransRotation.h"
#include "HNMath/GeometricAlgorithm2.h"

#include "HNString/HNString.h"
#include "HNString/EncodeStr.h"
#include "glog/logging.h"
#include "DataManager/XmlConfig.h"
#include <io.h>
#include <direct.h>
#include <fstream>

#include"HNFile/File.h"

#include <memory>
#include <numeric>
#include "LineReconstructor.h"

cv::flann::Index mss_kdtree;

const string& DATA_ARG("ARG");


string getPostfix(RegMethod rm)
{
	string post_fix = "";
	switch (rm)
	{
	case RegMethod_NONE:
		post_fix = "-0init";
		break;
	case RegMethod_BMM:
		post_fix = "-1icp";
		break;
	case RegMethod_BMM_EKF:
		post_fix = "-2icp-ekf";
		break;
	case RegMethod_NONE_GT:
		post_fix = "-5gt";
		break;
	case RegMethod_CE_EKF:
		post_fix = "-6ce-ekf";
		break;
	default:
		break;
	}

	return post_fix;
}

float get_rand()
{
	//rand() / double(RAND_MAX) 0~1的浮点数
	return 2.0 * rand() / double(RAND_MAX) - 1.0;
}

void randomJitter(vector<double>& camPose)
{
	int sz = 6;
	vector<double> ratio_vec = { 0.5 ,0.5 ,0.5, 0.04 ,0.04 ,0.04 };
	for (int i = 0; i < sz; i++)
	{
		camPose[i] += get_rand() * ratio_vec[i];
	}
}


//像素坐标系p到相机坐标系x
cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K)
{
	return cv::Point2d
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}

AlignImageHD::AlignImageHD()
{
	m_reg_oc_set.clear();
	m_reg_oc_set.insert(OC_lane);
	m_io = nullptr;

	google::InitGoogleLogging("a");
	FLAGS_minloglevel = 0;

	FLAGS_log_dir = "Logs";
	FLAGS_alsologtostderr = 1;
	google::SetLogDestination(google::GLOG_INFO, (FLAGS_log_dir + "\\log_").c_str());
	google::SetLogDestination(google::GLOG_WARNING, (FLAGS_log_dir + "\\log_").c_str());
}

AlignImageHD::~AlignImageHD()
{
}

void AlignImageHD::creatFolder(const string& folder_path)
{
	string ply_folder = folder_path + SUB_FOLDER_PLY;
	if (_access(ply_folder.c_str(), 0) != 0)
	{
		_mkdir(ply_folder.c_str());
	}
}


void AlignImageHD::preprocess(int step, int method)
{
	CalibSpace::band = 114;

	LOG(INFO) <<("initialize camera...");
	HN_GENERAL::read_xml_config();
	initCalibSpace();
	
	if (!readTrajecotry(m_ins_vec))
	{
		LOG(ERROR) << "无法读取轨迹...";
		return;
	}

	

	string para_tbl = CONFIG_PARA_TBL;
	bool keep_inside_intersection = false;//是否读取路口内的要素，重建精度分析需要
	if (step == 3)
	{
		para_tbl = CONFIG_PARA_RECON_TBL;		
		keep_inside_intersection = true;
	}
	
	readHDMap(keep_inside_intersection);
	//add 20230408 把方法配置放到这里来，不读配置了
	m_reg_method = RegMethod(method);
	SUB_FOLDER_PLY = "ply" + getPostfix(m_reg_method) + "\\";


	LOG(INFO) <<("regist method:") << m_reg_method;

	if(!preprocess(m_ins_vec,m_hd_map,MY_CONFIG.data_path,MY_CONFIG.mid_path,CalibSpace::band, para_tbl))
	{
		return;
	}
}

void AlignImageHD::process(int step)
{
	
}

bool readDataImageSize(const std::string & _dataPath, const std::string & _midPath, const string& img_name)
{
	//
	string img_path = "";

	if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		img_path = _midPath + "/Image/2/";
	}
	else
	{
		img_path = _dataPath + "/Image/1/";
	}
	
	img_path = img_path + img_name + ".jpg";

	auto t = cv::imread(img_path);
	if (t.cols == 0 ||
		t.rows == 0) 
	{
		LOG(INFO) << ("cannot read original image size: check images path...");
		return false;
	}

	CalibSpace::IMG_WIDTH = t.cols;
	CalibSpace::IMG_HEIGHT = t.rows;

	if (CalibSpace::camera_type == CAMERA_MSS_PANO)
	{
		CalibSpace::image_rect.width = 2048;
		CalibSpace::image_rect.height = 2048;

		int32_t wOff = (CalibSpace::IMG_WIDTH - CalibSpace::image_rect.width) / 2;
		wOff = wOff < 0 ? 0 : wOff;

		int32_t hOff = (CalibSpace::IMG_HEIGHT - CalibSpace::image_rect.height) / 2;
		hOff = hOff < 0 ? 0 : hOff;

		CalibSpace::image_rect.x = wOff;
		CalibSpace::image_rect.y = hOff;
	}
	else
	{
		CalibSpace::image_rect = cv::Rect(0, 0, CalibSpace::IMG_WIDTH, CalibSpace::IMG_HEIGHT);
	}

	LOG(INFO) <<("image size:[") << CalibSpace::IMG_WIDTH << "," << CalibSpace::IMG_HEIGHT <<"]";
	LOG(INFO) <<("image rect:");
	LOG(INFO) << CalibSpace::image_rect.tl().x;
	LOG(INFO) << CalibSpace::image_rect.tl().y;
	LOG(INFO) << CalibSpace::image_rect.width;
	LOG(INFO) << CalibSpace::image_rect.height;

	return true;
}


void relateInsTimeAndImageTime(const vector<TrajectoryData>& ins_vec)
{
	map<string, TrajectoryData> t_ins_map;
	for_each(ins_vec.begin(), ins_vec.end(), [&](const auto& ins) {
		t_ins_map.insert(make_pair(ins.name, ins));
	});
	vector<string> files;
	HN_GENERAL::getAllFilesName(MY_CONFIG.img_path, ".JPG", files);

	for (const auto& fna : files)
	{
		string fna_str = fna;
		HNString::ReplaceA(fna_str, ".jpg", "");
		auto high_itr = t_ins_map.find(fna_str);

		if (high_itr == t_ins_map.end())
		{
			string file_path = MY_CONFIG.img_path+ fna;
			remove(file_path.c_str());
		}
	}

}


void removeInvalidTracePoints(const string& _path, vector<TrajectoryData>& ins_vec)
{
	ofstream of(_path + "ins.txt", ios::trunc);
	of << "x,y,z,name" << endl;

	auto itr_ins = ins_vec.begin();
	for (; itr_ins != ins_vec.end(); )
	{
		TrajectoryData& ins = *itr_ins;
		string img_path = MY_CONFIG.img_path + ins.name + ".jpg";
		if (_access(img_path.c_str(), 0) != 0)
		{
			itr_ins = ins_vec.erase(itr_ins);
		}
		else
		{
			itr_ins++;
			string str = to_string(ins.lonlat.x) + "," + to_string(ins.lonlat.y) + "," + to_string(ins.lonlat.z) + "," + ins.name;
			of << str << endl;

		}
	}

	of.close();

}

bool AlignImageHD::preprocess(vector<TrajectoryData>& ins_vec, HDMapData& map_data,
	const std::string & _dataPath,
	const std::string & _midPath,
	const int& band,
	const string& para_tbl_na,
	string car_id)
{
	MY_CONFIG.data_path = _dataPath + "/";
	MY_CONFIG.mid_path = _midPath + "/";
	creatFolder(MY_CONFIG.mid_path);

	if (ins_vec.size() == 0 ||
		map_data.ld_vec.size() == 0)
	{
//		return;
	}


	string img_name = ins_vec[0].name;
	string sta_name = "";
	getImageName(sta_name, img_name);
	//initLog(sta_name);

	//ins_vec.resize(10);

	//set
	m_hd_map.ld_vec.swap(map_data.ld_vec);
	m_hd_map.hd_obj_vec.swap(map_data.hd_obj_vec);

	CalibSpace::band = band;

	//	transformCoord(ins_vec);
	transformCoord(m_hd_map.ld_vec);
	transformCoord(m_hd_map.hd_obj_vec);

	//m_io->saveHDMap(MY_CONFIG.mid_path,m_hd_map.ld_vec);
	//m_io->saveHDMap(MY_CONFIG.mid_path,m_hd_map.hd_obj_vec);

	//用于识别对应的配置项

	LOG(INFO) <<("central band:") << CalibSpace::band;
	LOG(INFO) <<("data path:");
	LOG(INFO) <<_dataPath;
	LOG(INFO) <<("mid path:");
	LOG(INFO) <<_midPath;
	LOG(INFO) <<("creat mid folder for registration...");
	LOG(INFO) <<("initialize config...");
	
	initConfig(para_tbl_na, car_id);

	removeInvalidTracePoints(_midPath, ins_vec);
//	relateInsTimeAndImageTime(ins_vec);
	if (ins_vec.size() == 0)
	{
		LOG(INFO) <<("no valid ins points: check images path...");
		return false;
	}
	return readDataImageSize(_dataPath, _midPath, ins_vec[0].name);
}

bool AlignImageHD::isInIntersection(CamPose& cp)
{
	const auto& junction_vec = m_hd_map.junction_vec;
	const auto& pt = cp.ins.point;
	cv::Point2f p2(pt.x, pt.y);
	auto check_junction_itr = find_if(junction_vec.begin(), junction_vec.end(), [&](const auto& _junc)->bool {
		if (_junc.rect_3d.contains(p2))
		{			
			vector<cv::Point2f> shape(_junc.shape.size());
			transform(_junc.shape.begin(), _junc.shape.end(), shape.begin(), [](const auto& pt)->cv::Point2f {
				return cv::Point2f(pt.x, pt.y);
				});
			return cv::pointPolygonTest(shape, p2, false) > 0;
		}
		else
		{
			return false;
		}
		});
	return check_junction_itr != junction_vec.end();
}

void  AlignImageHD::kalmanFilterInitialize(const vector<double>& pose)
{
	m_kf = new FusionEKF(pose);
}

bool  AlignImageHD::kalmanFilterEstimate(CamPose& cp)
{
	auto find_cp = m_pos_map.find(cp.img_na);
	if (find_cp == m_pos_map.end() ||
		find_cp == m_pos_map.begin())
	{
		return false;
	}
	find_cp--;
	auto& last_cp = find_cp->second;

	bool is_in_intersection = isInIntersection(cp);

	vector<double> config_pos(6);
	updateCamPose(config_pos);

	Eigen::VectorXd est = m_kf->ProcessMeasurement(config_pos, last_cp, is_in_intersection);
	vector<double> kf_cp(6);
	for (int i = 0; i < 6; i++)
	{
		kf_cp[i] = est[i];
	}
	
	cp.camPose.swap(kf_cp);

	return true;
}

bool AlignImageHD::matchImageWithLog(const string& img_na, double t0, double t_end, float time_res, int& idx)
{
	double img_ts = stoll(img_na)*1.0 / 1e6;

	if (img_ts < t0 - time_res ||
		img_ts > t_end + time_res)
	{
		return false;
	}
	float img_ts_shift = img_ts - t0;

	float d = 100;
	//预设knnSearch所需参数及容器
	int queryNum = 1;//用于设置返回邻近点的个数
	vector<float> vecQuery(1);//存放查询点的容器
	vector<int> vecIndex(queryNum);//存放返回的点索引
	vector<float> vecDist(queryNum);//存放距离
	cv::flann::SearchParams params(32);//设置knnSearch搜索参数S
	vecQuery[0] = img_ts_shift;
	mss_kdtree.knnSearch(vecQuery, vecIndex, vecDist, queryNum, params);

	d = sqrt(vecDist[0]);
	idx = vecIndex[0];
	// 1second
	if (d > time_res)
	{
		return false;
	}

	return true;
}

void AlignImageHD::buildTimeIndex(cv::Mat& source, const vector<TrajectoryData>& ins_vec)
{
	vector<double> time_vec;
	double t0 = ins_vec.front().time;
	for_each(ins_vec.begin(), ins_vec.end(), [&](const auto& ins) {
		time_vec.push_back(ins.time - t0);
	});

	source = cv::Mat(time_vec).reshape(1);
	source.convertTo(source, CV_32F);
	cv::flann::KDTreeIndexParams indexParams(1);
	mss_kdtree.build(source, indexParams);
}

int AlignImageHD::getImageIndex(const string& img_na)
{
	int idx = -1;
	string t = img_na.substr(3, img_na.size() - 3);
	auto epos = t.find('_');
	if (epos == string::npos)
	{
		return idx;
	}
	string idx_str = t.substr(0, epos);
	idx = atoi(idx_str.c_str());
	return idx;
}

double getDiffAngle(const double& a)
{
	double b = a;
	if (a > M_PI / 6 * 5)
	{
		b = a - M_PI;
	}
	else if (a < -M_PI / 6 * 5)
	{
		b = a + M_PI;
	}
	return b;
}

void calDiffPose(const vector<double>& campos_ini, const vector<double>& campos,
	vector<double>& campos_dif)
{
	Eigen::Vector3d euler1(campos_ini[3], campos_ini[4], campos_ini[5]);
	Eigen::Vector3d euler2(campos[3], campos[4], campos[5]);

	Eigen::Vector3d euler_dif;
	TransRotation::eigenEulerDiff(euler1, euler2, euler_dif);

	campos_dif.resize(6);
	campos_dif[0] = campos[0] - campos_ini[0];
	campos_dif[1] = campos[1] - campos_ini[1];
	campos_dif[2] = campos[2] - campos_ini[2];

	campos_dif[3] = getDiffAngle(euler_dif[0]);
	campos_dif[4] = getDiffAngle(euler_dif[1]);
	campos_dif[5] = getDiffAngle(euler_dif[2]);
}

void AlignImageHD::jitterCamPose(map<string, CamPose>& pose_map)
{
	//如果存在就直接读
	string file_path = MY_CONFIG.mid_path + "jitter_pose.csv";
	if (_access(file_path.c_str(), 0) !=  0)
	{
		LOG(ERROR) << "无法读取jitter_pose.csv...";
		return;
	}
	m_io->readJitterCamPose(file_path, pose_map);
}

void AlignImageHD::groudTruthCamPose(map<string, CamPose>& pose_map)
{
	//如果存在就直接读
	string file_path = MY_CONFIG.mid_path + "gt_pose.csv";
	if (_access(file_path.c_str(), 0) != 0)
	{
// 		ofstream os(MY_CONFIG.data_path + "gt_pose.csv", ios::trunc);
// 		os.close();

		const vector<TrajectoryData>& ins_vec = m_ins_vec;
		vector<CamPose> pose_vec;

		for (const auto& ins : ins_vec)
		{
			CamPose cp;
			cp.img_na = ins.name;
			auto& campos = cp.camPose;
			updateCamPose(campos);

			pose_vec.push_back(cp);
		}
		m_io->saveGTCamPose(file_path, pose_vec);

	}

	m_io->readGTCamPose(file_path, pose_map);
	
#if 0
	vector<CamPose> jitter_pose_vec;
	auto itr_cp = pose_map.begin();
	for (; itr_cp != pose_map.end(); itr_cp++)
	{
		auto jitter_cp = itr_cp->second;
		randomJitter(jitter_cp.camPose);
		jitter_pose_vec.push_back(jitter_cp);
	}
	string file_path_j = MY_CONFIG.mid_path + "jitter_pose.csv";
	m_io->saveJitterCamPose(file_path_j, jitter_pose_vec);
#endif
}

void AlignImageHD::processRegistHDAndMSSImages()
{
	const vector<TrajectoryData>& ins_vec = m_ins_vec;
	if (ins_vec.size() == 0)
	{
		return;
	}
	string para_na = "reg" + getPostfix(m_reg_method) + ".csv";
//	string para_na_kf = "para" + getPostfix(RegM) + "_kf.csv";

	const string& folder_path = MY_CONFIG.data_path;
	const string& mid_folder_path = MY_CONFIG.mid_path;

	const auto& para_file = mid_folder_path + para_na;
	if (_access(para_file.c_str(), 0) == 0)
	{
		ofstream os(para_file,ios::trunc);
		os.close();
	//	remove(para_file.c_str());
	}

	const auto& map_file = mid_folder_path + "map.csv";
	if (_access(map_file.c_str(), 0) == 0)
	{
		remove(map_file.c_str());
	}

	map<string, CamPose> gt_pose_map;
	groudTruthCamPose(gt_pose_map);

	map<string, CamPose> init_pose_map;
	if (m_reg_method == RegMethod_NONE_GT)//ground truth
	{
		groudTruthCamPose(init_pose_map);
	}
	else 
	{
		jitterCamPose(init_pose_map);//抖动的初值
	}

	int total_sz = ins_vec.size();
	int sz = 0;
	int save_sz = 0;
	auto itr_ins = ins_vec.begin();
	for (; itr_ins != ins_vec.end(); itr_ins++, sz++)
//	for (; itr_ins != ins_vec.end(); itr_ins+= 10, sz += 10)
	{
		const TrajectoryData& ins = *itr_ins;
		string img_na = itr_ins->name;
		string sta_na = "";
		getImageName(sta_na, img_na);
		string img_path = MY_CONFIG.img_path + img_na + ".jpg";
		if (_access(img_path.c_str(), 0) != 0)
		{
			continue;
		}

		time_t stime = GetCurrentTime();
		LOG(INFO) <<("*****************************");
		string line = "";
		HNString::FormatA(line, "%d:...%s...%d/%d", sz, itr_ins->name.c_str(), (sz + 1), total_sz);
		LOG(INFO) << line;
		CamPose& cp = m_pos_map[img_na];
		cp.img_na = img_na;
		cp.ins = ins;
		cp.idx = save_sz;
		vector<double> campos(6);
		
		campos = init_pose_map[img_na].camPose;
		auto gt_campos = gt_pose_map[img_na].camPose;
		cp.camPose = campos;

		if (m_pos_map.size() == 1)
		{
			vector<double> init_pos(6);
			updateCamPose(init_pos);
			kalmanFilterInitialize(init_pos);
		}

	//	updateCamPose(cp.camPose);

		if (m_reg_method == RegMethod_BMM_EKF ||
			m_reg_method == RegMethod_CE_EKF)
		{
			kalmanFilterEstimate(cp);
		}

		if (!registHDImageLaneDividers(ins_vec, sz, cp))
		{
			//continue;
		}
		
		auto save_delta_cp = cp;
#if 1	
		calDiffPose(gt_campos, cp.camPose, save_delta_cp.camPose);
#endif
		m_io->saveParas(mid_folder_path + para_na, save_delta_cp);

		save_sz++;

		time_t etime = GetCurrentTime();
		LOG(INFO) <<("time:") << (etime - stime) / 1000.0;
	}
	LOG(INFO) <<("complited.");
}

bool AlignImageHD::isValid(const vector<double>&  cam, const vector<double>&  base, const vector<double>&  diff_threshold)
{
	for (int i = 0; i < 3; i++)
	{
		if (abs(cam[i] - base[i]) > diff_threshold[i])
		{
			return false;
		}
	}

	return true;
}

bool AlignImageHD::readTrajecotry(vector<TrajectoryData>& ins_vec)
{
	const auto& data_na = MY_CONFIG.data_name;
	LOG(INFO) <<("read data:") << data_na;
	 if (data_na == DATA_ARG)
	{
		m_io = new DataIOArgoverse;
		m_io->initDataPath(MY_CONFIG.data_path);
	}
	
	if (m_io == nullptr)
	{
		return false;
	}

	m_io->getTracePoints(ins_vec);
	m_io->getTraceXYBox(ins_vec, m_trace_box, 10.0);
	if (m_trace_box.size() == 0)
	{
		//		return;
	}

	return ins_vec.size() > 0;
}

void AlignImageHD::readHDMap(bool keep_inside_intersection)
{
	if (m_io == nullptr)
	{
		return;
	}

	const auto& data_na = MY_CONFIG.data_name;
	LOG(INFO) <<( "read data:") << data_na.c_str();
	if (data_na == DATA_ARG)
	{
		m_io->getHDLaneDividerInBox(m_trace_box, m_hd_map.ld_vec, keep_inside_intersection);
	}
	
}

//read mask and thin
bool AlignImageHD::getLocalHDMap(const vector<TrajectoryData>& ins_vec, const int& idx,  HDObject_VEC& local_local_map_vec)
{
	transformHDMapFromWorld2Ego(m_hd_map.ld_vec, ins_vec[idx], local_local_map_vec);
	vector<TrajectoryData> local_ins_vec;
	transformInsFromWorld2Ego(ins_vec, idx, local_ins_vec);
	removeByTraceRange(local_local_map_vec, local_ins_vec);

	if (local_local_map_vec.size() == 0)
	{
		LOG(INFO) <<("local map lane divider data empty...");
		return false;
	}
}

bool AlignImageHD::registHDImageLaneDividers(const vector<TrajectoryData>& ins_vec, const int& idx, CamPose& cp)
{
	//记录当前帧的初始值
	auto init_frame_pos = cp.camPose;
	bool ipm_flg = (m_reg_method == RegMethod_BMM) || (m_reg_method == RegMethod_BMM_EKF);
	//bool ipm_flg = false;
	if (m_hd_map.ld_vec.size() == 0)
	{
		//LOG(ERROR) << "map lane divider data empty...");
		LOG(INFO) <<("map lane divider data empty...");
		return false;
	}
	const auto& ins = ins_vec[idx];
	HDObject_VEC local_local_map_vec;
	getLocalHDMap(ins_vec, idx, local_local_map_vec);

	m_reg_oc_set.clear();
	m_reg_oc_set.insert(OC_lane);
	const string& folder_path =  MY_CONFIG.data_path;
	const string& mid_folder_path = MY_CONFIG.mid_path;

	//*********************************2d********************************//
	vector<vector<cv::Point>> line_vec;
	cv::Mat camera_img = cv::imread(MY_CONFIG.img_path + cp.img_na + ".jpg");

	map<int, LINE_VEC> lines_map;
	m_calib.setCameraImage(camera_img);
	m_calib.extractImageDeepLearningMultiLines(mid_folder_path, cp.img_na, m_reg_oc_set, lines_map, 0);

	string ply_path = mid_folder_path + SUB_FOLDER_PLY + cp.img_na + ".jpg";
	cv::Mat t = camera_img.clone();
	saveImage(SUB_FOLDER_PLY, cp.img_na, t, cv::Scalar(0, 0, 255), lines_map);

	//备份
	map<int, LINE_VEC> camera_lines_map = lines_map;
	if (ipm_flg)
	{
		inversePerspectiveAdjustImageLines(cp.img_na, lines_map);
#if 0
		cv::Mat ipm_img;
		m_calib.inversePerspectiveMapping(camera_img, CalibSpace::warpmat_src2ipm, ipm_img);
		string ipm_path = mid_folder_path + "ipm/" + cp.img_na + ".jpg";
		cv::imwrite(ipm_path, ipm_img);
#endif
	}

	if (lines_map[OC_lane].size() == 0)
	{
		LOG(INFO) <<("deeplearning lane divider data empty...");
		return false;
	}
	
	map<int, Point3f_VEC> xyz_vec_map;
	getEgoMapPoints(local_local_map_vec, cp.camPose, xyz_vec_map);

	if (xyz_vec_map[OC_lane].size() == 0)
	{
		LOG(INFO) <<("local map lane divider data empty...");
		return false;
	}

	optimizePose(xyz_vec_map, lines_map, camera_lines_map, ins, cp.camPose);

	
	map<int, LINE_VEC> contours_map;
	getEgoMapContours(local_local_map_vec, cp.camPose, contours_map,false);

	float eval = evaluteSimilarity(camera_lines_map, contours_map);
	cp.regist_probability = 1 - eval;

	//配准的太差了，
	if (cp.regist_probability < 0.1)
	{
		map<int, LINE_VEC> init_contours_map;
		getEgoMapContours(local_local_map_vec, init_frame_pos, init_contours_map, false);
		float init_prob = 1 - evaluteSimilarity(camera_lines_map, init_contours_map);
		if (init_prob > cp.regist_probability)
		{
			cp.camPose = init_frame_pos;
			cp.regist_probability = init_prob;
			contours_map.swap(init_contours_map);
		}
		
	}
	string eval_str = to_string(cp.regist_probability);
	eval_str = eval_str.substr(0, eval_str.find(".") + 3);

	// ply image
	t = cv::imread(ply_path);

	//逆透视
	//t = ipm_img;
	//cv::putText(t, to_string(cp.idx), cv::Point(0, 40), 2, 1.0, cv::Scalar(0, 0, 0), 2);
	cv::putText(t, eval_str, cv::Point(1000, 40), 2, 2.0, cv::Scalar(255, 255, 255), 2);
	saveImage(SUB_FOLDER_PLY, cp.img_na, t, cv::Scalar(255, 0, 0), contours_map);

	return true;
}


bool AlignImageHD::optimizePose(map<int, Point3f_VEC>& xyz_vec_map, 
	const map<int, LINE_VEC>& lines_map, 
	const map<int, LINE_VEC>& camera_lines_map,
	const TrajectoryData& ins,
	vector<double>& campos)
{
	vector<double> diff_threshold = { 0.5, 0.5, 2, 0.05, 0.05, 0.05 };
	vector<double> base_cam = campos;
	bool no_image_ll = false;
	for_each(lines_map.begin(), lines_map.end(), [&](const auto& l) {
		no_image_ll |= l.second.size() > 0;
	});
	if (!no_image_ll)
	{
		LOG(INFO) <<( "no image lines...");
		return 0;
	}


	if (xyz_vec_map.size() == 0)
	{
		return 0;
	}

#if 1
	bool is_valid = false;
	vector<double> bmm_campose = campos;
	double resdual = 0;
	switch (m_reg_method)
	{
	case RegMethod_NONE:
	case RegMethod_NONE_GT:
		break;
	case RegMethod_BMM:
	case RegMethod_BMM_EKF:
		//bmm
		resdual = m_calib.iterateClosestPoint2d3d(xyz_vec_map, lines_map,
			camera_lines_map,
			bmm_campose, diff_threshold);
		break;
	//crossentropy
	case RegMethod_CE_EKF:
		resdual = 	m_calib.iterateCrossEntropy(xyz_vec_map, lines_map,ins.pitch, bmm_campose);
		break;
	default:
		break;
	}
	
	is_valid = (!isnan(bmm_campose[0])) && isValid(bmm_campose, base_cam, diff_threshold);
	campos = bmm_campose;
#endif

	return is_valid;
}


float AlignImageHD::evaluteSimilarity(const map<int, LINE_VEC>& lines_map_rgb,
	const map<int, LINE_VEC>& lines_map_hdmap)
{
	cv::Mat blur_mat_rgb = cv::Mat(CalibSpace::image_rect.height, CalibSpace::image_rect.width, CV_8UC1, cv::Scalar(0));
	auto itr = lines_map_rgb.begin();
	for (; itr != lines_map_rgb.end(); itr++)
	{
		const auto& type = itr->first;
		const auto& line_vec = itr->second;
		cv::polylines(blur_mat_rgb, line_vec, false, cv::Scalar(255), 20);
	}

	cv::Mat blur_mat_hdmap = cv::Mat(CalibSpace::image_rect.height, CalibSpace::image_rect.width, CV_8UC1, cv::Scalar(0));
	itr = lines_map_hdmap.begin();
	for (; itr != lines_map_hdmap.end(); itr++)
	{
		const auto& type = itr->first;
		const auto& line_vec = itr->second;
		cv::polylines(blur_mat_hdmap, line_vec, false, cv::Scalar(255), 20);
	}

	cv::Mat dif;
	cv::bitwise_and(blur_mat_rgb, blur_mat_hdmap, dif);
	int cnt_and = cv::countNonZero(dif);

	cv::bitwise_or(blur_mat_rgb, blur_mat_hdmap, dif);
	int cnt_or = cv::countNonZero(dif);

	cv::absdiff(blur_mat_rgb, blur_mat_hdmap, dif);
	int cnt_dif = cv::countNonZero(dif);

	float ratio_dif = cnt_dif * 1.0 / cnt_or;
	float ratio_and = cnt_and * 1.0 / cnt_or;
	return ratio_dif;
}


bool AlignImageHD::initConfig(const string& para_tbl_na, const string& car_id)
{
	vector<vector<string>> value_vec;
	{
		HN_GENERAL::read_xml_config();
	}

	return initCalibSpace();
}

bool AlignImageHD::initCalibSpace()
{
	if ( MY_CONFIG.data_para.intrinsic_para.size() == 0)
	{
		//LOG(ERROR) << "can't initialize camera...");
		LOG(INFO) <<("can't initialize camera...");
		return false;
	}
	
	CalibSpace::CX =  MY_CONFIG.data_para.intrinsic_para[0];
	CalibSpace::CY =  MY_CONFIG.data_para.intrinsic_para[1];
	CalibSpace::FX =  MY_CONFIG.data_para.intrinsic_para[2];
	CalibSpace::FY =  MY_CONFIG.data_para.intrinsic_para[3];

	if (MY_CONFIG.data_para.camera_type == "pano")
	{
		CalibSpace::camera_type = CAMERA_MSS_PANO;
	}
	else if (MY_CONFIG.data_para.camera_type == "wide")
	{
		CalibSpace::camera_type = CAMERA_MSS_WIDE;
	}


	if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		MY_CONFIG.img_path = MY_CONFIG.mid_path + "Image/2/";
	}
	else
	{
		MY_CONFIG.img_path = MY_CONFIG.mid_path + "Image/1/";
	}

	CCalib mc;
	mc.initCamera(CalibSpace::intrisicMat, CalibSpace::distCoeffs);

	if (MY_CONFIG.data_para.corners.size() > 0)
	{
		CalibSpace::initInversePerspectiveMappingMat(MY_CONFIG.data_para.corners,
			CalibSpace::warpmat_src2ipm,
			CalibSpace::warpmat_ipm2src);
	}

	CalibSpace::ego_height = MY_CONFIG.data_para.ego_height;
	//20230417激活函数
	CalibSpace::activate_flg = false;
	LOG(INFO) <<("initialize camera pose...");
	LOG(INFO) <<MY_CONFIG.data_para.trans.x;
	LOG(INFO) <<MY_CONFIG.data_para.trans.y;
	LOG(INFO) <<MY_CONFIG.data_para.trans.z;
	LOG(INFO) <<MY_CONFIG.data_para.rotate[0];
	LOG(INFO) <<MY_CONFIG.data_para.rotate[1];
	LOG(INFO) <<MY_CONFIG.data_para.rotate[2];

	return true;
}


void AlignImageHD::updateCamPose(vector<double>& campos)
{
	campos.resize(6);

	campos[0] =  MY_CONFIG.data_para.trans.x;
	campos[1] =  MY_CONFIG.data_para.trans.y;
	campos[2] =  MY_CONFIG.data_para.trans.z;

	campos[3] = MY_CONFIG.data_para.rotate[0] / 180.0 * M_PI;
	campos[4] = MY_CONFIG.data_para.rotate[1] / 180.0 * M_PI;
	campos[5] = MY_CONFIG.data_para.rotate[2] / 180.0 * M_PI;

}

template<typename T>
void AlignImageHD::getEgoMapPoints(const vector<T>& map_vec,	
	const vector<double>& camPose, map<int, Point3f_VEC>& xyz_vec_map)
{
	for (int i = 0; i < map_vec.size(); i++)
	{
		const int& type = map_vec[i].type;
		auto obj = map_vec[i].shape;

		vector<cv::Point> contour;
		int oc = convHDMapType2ObjectClassification(type);

		Point3f_VEC& xyz_vec = xyz_vec_map[oc];
		for (int j = 0; j < obj.size(); j++)
		{
			const auto& lp = obj[j];
			double xyz[3] = { lp.x, lp.y, lp.z };
			cv::Point pp;
			
			double ij[2] = { -1,-1 };
			if (!OptimizeCeres::convertPoint3dTo2d(camPose, xyz, ij, false))
			{
				continue;
			}
			xyz_vec.push_back(lp);
		}
	}
}

template<typename T>
void AlignImageHD::getEgoMapPoints(const vector<T>& map_vec, const vector<double>& camPose,
	map<int, vector<Point3f_VEC>>& xyz_vec_map)
{
	for (int i = 0; i < map_vec.size(); i++)
	{
		const int& type = map_vec[i].type;
		auto obj = map_vec[i].shape;

		vector<cv::Point> contour;
		int oc = convHDMapType2ObjectClassification(type);

		auto& xyz_vec = xyz_vec_map[oc];

		Point3f_VEC line;
		for (int j = 0; j < obj.size(); j++)
		{
			const auto& lp = obj[j];
			double xyz[3] = { lp.x, lp.y, lp.z };
			cv::Point pp;

			double ij[2] = { -1,-1 };
			if (!OptimizeCeres::convertPoint3dTo2d(camPose, xyz, ij, false))
			{
				continue;
			}
			line.push_back(lp);
		}
		if (line.size() > 0)
		{
			xyz_vec.push_back(line);
		}
	}
}

template<typename T>
void AlignImageHD::getEgoMapContours(const vector<T>& map_vec, const vector<double>& camPose,
map<int, LINE_VEC>& contours_map, bool ipm)
{
	for (int i = 0; i < map_vec.size(); i++)
	{
		const int& type = map_vec[i].type;
		auto obj = map_vec[i].shape;

		vector<cv::Point> contour;
		int oc = convHDMapType2ObjectClassification(type);

		for (int j = 0; j < obj.size(); j++)
		{
			const auto& lp = obj[j];
			cv::Point pp;
			double xyz[3] = {lp.x, lp.y, lp.z };
			double ij[2] = {-1,-1};
			if (!OptimizeCeres::convertPoint3dTo2d(camPose, xyz, ij, ipm))
			{
				continue;
			}
			
			pp.x = ij[0];
			pp.y = ij[1];

			contour.push_back(pp);
		}

		if (contour.size() > 0)
		{
			LINE_VEC& contours = contours_map[oc];
			contours.push_back(contour);
		}
	}
}


ObjectClassification AlignImageHD::convHDMapType2ObjectClassification(int type)
{
	ObjectClassification oc = OC_lane;
	return oc;
}

template<typename T>
void AlignImageHD::transformHDMapFromWorld2Ego(const vector<T>& map_vec, const TrajectoryData& ins, vector<T>& local_local_map_vec)
{
	Eigen::Vector3f R_euler;
	R_euler << M_PI / 2 + ins.roll, ins.pitch, ins.heading;

	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);
	
	Eigen::Vector3d Trans;
	Trans << ins.point.x, ins.point.y, ins.point.z;
	for (int i = 0; i < map_vec.size(); i++)
	{
		const auto& obj = map_vec[i];
		
		int64 id = stoull(obj.obj_id);
		vector<cv::Point3d> reserv_shp;
		for (int j = 0; j < obj.shape.size(); j++)
		{
			const auto& p = obj.shape[j];
	//		cv::Point3d lp;
			Eigen::Vector3d pp;
			pp << p.x, p.y, p.z;
			Eigen::Vector3f lp;
			CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);
			
			if (beyongImageRange(obj.type, lp))
			{
				continue;
			}
			reserv_shp.push_back(cv::Point3d(lp[0], lp[1], lp[2]));
		}

		if (reserv_shp.size() > 0)
		{
			T lobj = obj;
			lobj.shape.swap(reserv_shp);
			local_local_map_vec.push_back(lobj);
		}
	}

}
void AlignImageHD::convInsFromWorld2Ego(const vector<TrajectoryData>& ins_vec, const TrajectoryData& ins,  vector<TrajectoryData>& local_ins_vec)
{
	Eigen::Vector3f R_euler;
	R_euler << M_PI / 2 + ins.roll, ins.pitch, ins.heading;

	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);

	Eigen::Vector3d Trans;
	Trans << ins.point.x, ins.point.y, ins.point.z;
	for (int i = 0; i < local_ins_vec.size(); i++)
	{
		auto& p = local_ins_vec[i].point;
		Eigen::Vector3d pp;
		pp << p.x, p.y, p.z;
		Eigen::Vector3f lp;
		CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);

		p = cv::Point3d(lp[0], lp[1], lp[2]);
	}
}

void AlignImageHD::transformInsFromWorld2Ego(const vector<TrajectoryData>& ins_vec, const int& idx, vector<TrajectoryData>& local_ins_vec)
{
	double len = 100;
	collectLocalIns(ins_vec, idx, len, local_ins_vec);
	convInsFromWorld2Ego(ins_vec, ins_vec[idx], local_ins_vec);
}

double getVerticalValue(const Eigen::Vector3f& lp)
{
	return lp[2];
}

double getHorizonalValue(const Eigen::Vector3f& lp)
{
	return lp[0];
}

double getElevateValue(const Eigen::Vector3f& lp)
{
	double y = lp[1];
	if (y >= 0)
	{
		y = y - CalibSpace::ego_height;
	}
	else
	{
		y = -y + CalibSpace::ego_height;
	}
	return y;
}

bool AlignImageHD::beyongImageRange(int type, const Eigen::Vector3f& lp)
{
	double v = getVerticalValue(lp);
	double h = getHorizonalValue(lp);
	double e = getElevateValue(lp);
	if (v < 0)
	{
		return true;
	}

	if (type < 80 || type == 255)
	{
		if(abs(h) > 15||//横向
			       v > 60 ||//纵向
			abs(e) > 5)//高
		{
			return true;
		}
	}
	else
	{
		if (abs(h) > 40||
					v > 100||
					(e < -2/*-1.5*/ || e > 30))//高
		{
			return true;
		}
	}
	
	return false;
}


bool getNearstPoint(const vector<TrajectoryData>& ins_vec, const cv::Point3d& point, TrajectoryData& ins)
{
	if (ins_vec.size() == 0)
	{
		return false;
	}

	map<double, TrajectoryData> dist_map;
	for_each(ins_vec.begin(), ins_vec.end(), [&](const auto& ins) {
		auto  p = point - ins.point;
		double dis = p.ddot(p);
		dist_map[dis] = ins;
	});
	if (dist_map.size() == 0/* ||
		dist_map.begin()->first >*/ )
	{
		return false;
	}
	ins = dist_map.begin()->second;
	return true;
}


bool beyongTraceDiffZlimit(int type, double diff_z)
{
	//地面印刷
	if (type < 80 || type == 255)
	{
		return abs(diff_z)  > 2;
	}
	else //高起
	{
		return diff_z > 20 || diff_z < -1.5;
	}

}

template<typename T>
void AlignImageHD::removeByTraceRange(vector<T>& local_local_map_vec, const vector<TrajectoryData>& local_ins_vec)
{
	if (local_ins_vec.size() == 0)
	{
		return;
	}
		
	auto remove_itr = local_local_map_vec.begin();
	for (; remove_itr != local_local_map_vec.end();)
	{
		auto& obj = *remove_itr;
		const auto& spt = obj.shape.front();
		const auto& ept = obj.shape.back();

		TrajectoryData s_ins;
		getNearstPoint(local_ins_vec, spt, s_ins);

		TrajectoryData e_ins;
		getNearstPoint(local_ins_vec, ept, e_ins);

		Eigen::Vector3f dif_s;
		dif_s[0] = spt.x - s_ins.point.x;
		dif_s[1] = spt.y - s_ins.point.y;
		dif_s[2] = spt.z - s_ins.point.z;

		Eigen::Vector3f dif_e;
		dif_e[0] = ept.x - e_ins.point.x;
		dif_e[1] = ept.y - e_ins.point.y;
		dif_e[2] = ept.z - e_ins.point.z;

		double diff_s = getElevateValue(dif_s);
		double diff_e = getElevateValue(dif_e);
		if (beyongTraceDiffZlimit(obj.type, diff_s) ||
			beyongTraceDiffZlimit(obj.type, diff_e))
		{
			remove_itr = local_local_map_vec.erase(remove_itr);
		}
		else
		{
			remove_itr++;
		}
	}

}

template<typename T>
void AlignImageHD::transformEgo2Camera(const vector<T>& map_vec, 
	const vector<double>& camPose,
	map<int, Point3f_VEC>& xyz_vec_map)
{
	for (int i = 0; i < map_vec.size(); i++)
	{
		const int& type = map_vec[i].type;
		const auto& obj = map_vec[i].shape;
		vector<cv::Point> contour;
		int oc = convHDMapType2ObjectClassification(type);

		Point3f_VEC& xyz_vec = xyz_vec_map[oc];
		for (int j = 0; j < obj.size(); j++)
		{
			const auto& lp = obj[j];
			double xyz[3];
			OptimizeCeres::convertPointByEigen(camPose, lp.x, lp.y, lp.z, xyz);
			xyz_vec.push_back(cv::Point3f(xyz[0], xyz[1], xyz[2]));
		}
	}
}

template<typename T>
void AlignImageHD::transformHDMapFromWorld2Camera(const vector<T>& map_vec, 
	const TrajectoryData& ins, 
	const vector<double>& camPose, 
	map<HDMapPointID, cv::Point3f>& cam_pts_map)
{
	Eigen::Vector3f R_euler(M_PI / 2 + ins.roll, ins.pitch, ins.heading);
	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);
	Eigen::Vector3d Trans;
	Trans << ins.point.x, ins.point.y, ins.point.z;
	for (int i = 0; i < map_vec.size(); i++)
	{
		const auto& obj = map_vec[i];
		for (int j = 0; j < obj.shape.size(); j++)
		{
			const auto& p = obj.shape[j];
			Eigen::Vector3d pp;
			pp << p.x, p.y, p.z;
			Eigen::Vector3f lp;
			CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);
			if (beyongImageRange(obj.type, lp))
			{
				continue;
			}
			double cp[3];
			OptimizeCeres::convertPointByEigen(camPose, (double)lp[0], (double)lp[1], (double)lp[2], cp);

			HDMapPointID pid;
			pid.obj_id = obj.obj_id;
			pid.v_id = j;
			cv::Point3f tp(cp[0], cp[1], cp[2]);
			cam_pts_map.insert(make_pair(pid, tp));
		}
	}

}

template<typename T>
void AlignImageHD::transformHDMapFromWorld2Image2(const vector<T>& map_vec, const CamPose& cp,
	const Eigen::Matrix3f& RR, const Eigen::Vector3f& TT,
	map<int, LINE_VEC>& contours_map	)
{
	Eigen::Vector3f R_euler(M_PI / 2 + cp.ins.roll, cp.ins.pitch, cp.ins.heading);
	Eigen::Matrix3f R_matrix;
	TransRotation::eigenEuler2RotationMatrix(R_euler, R_matrix);
	Eigen::Vector3d Trans;
	Trans << cp.ins.point.x, cp.ins.point.y, cp.ins.point.z;

	for (int i = 0; i < map_vec.size(); i++)
	{
		const auto& obj = map_vec[i];
		int oc = convHDMapType2ObjectClassification(obj.type);
		vector<vector<cv::Point>>& line_vec = contours_map[oc];
		vector<cv::Point> line;
		for (int j = 0; j < obj.shape.size(); j++)
		{
			const auto& p = obj.shape[j];
			Eigen::Vector3d pp;
			pp << p.x, p.y, p.z;
			Eigen::Vector3f lp;
			CalibSpace::EigenTranslateAndRot(pp, lp, Trans, R_matrix);
			if (beyongImageRange(obj.type, lp))
			{
				continue;
			}

			double cp1[3];
			OptimizeCeres::convertPointByEigen(cp.camPose, (double)lp[0], (double)lp[1], (double)lp[2], cp1);
			Eigen::Vector3f cpv(cp1[0], cp1[1], cp1[2]);
			Eigen::Vector3f cp2 = RR * cpv + TT;
			double xyz[3] = { cp2[0], cp2[1], cp2[2] };
			double ij[2] = { -1, -1 };
			if (!OptimizeCeres::project2Image(xyz, ij, false))
			{
				continue;
			}

			line.push_back(cv::Point(ij[0], ij[1]));
		}

		if (line.size() > 0)
		{
			line_vec.push_back(line);
		}
	}
}


void AlignImageHD::getImageCenter(cv::Point2f& c)
{
	if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		c.x = CalibSpace::CX;
		c.y = CalibSpace::CY;
	}
	else
	{
		c.x = CalibSpace::IMG_WIDTH / 2;
		c.y = CalibSpace::IMG_HEIGHT / 2;
	}
}

#include "HNMath/Histogram.h"
void AlignImageHD::splitLaneContours(LINE_VEC& contoursf)
{
	LINE_VEC new_contoursf;

	for (auto& line : contoursf)
	{
		if (line.size() == 1)
		{
			new_contoursf.push_back(line);
			continue;
		}
	/*	vector<cv::Point> new_line;
		auto itr_s = line.begin();
		for (; itr_s != line.end() - 1; itr_s++)
		{
			auto itr_e = itr_s + 1;
			if (itr_s->y <= itr_e->y)
			{
				new_line.push_back(*itr_s);
			}
			else
			{
				new_line.push_back(*itr_s);
				new_contoursf.push_back(new_line);
				new_line.clear();
			}
		}
		if (new_line.size() > 0)
		{
		}*/
		while (line.size() > 1)
		{
			vector<cv::Point> new_line;
			int y = line[0].y;
			auto itr_l = find_if(line.begin() + 1, line.end(), [&](const auto& p)->bool {
				if (p.y < y)
				{
					y = p.y;
					return true;
				}
				y = p.y;
				return false;
			});
			copy(line.begin(), itr_l, back_inserter(new_line));
			reverse(new_line.begin(), new_line.end());
			new_contoursf.push_back(new_line);

			line.erase(line.begin(), itr_l);
			if (line.size() <= 1)
			{
				break;
			}
			y = line[0].y;
			itr_l = find_if(line.begin() + 1, line.end(), [&](const auto& p)->bool {
				if (p.y > y)
				{
					y = p.y;
					return true;
				}
				y = p.y;
				return false;
			});
			line.erase(line.begin(), itr_l);
		}
		
	}
	contoursf.swap(new_contoursf);

	int a = 0;
}

void AlignImageHD::inversePerspectiveAdjustImageLines(const string& img_na, map<int, LINE_VEC>& lines_map)
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

//	return;

	if (CalibSpace::warpmat_src2ipm.cols == 0)
	{
		return;
	}
	vector<vector<cv::Point2f>> contoursf;
	for_each(lines.begin(), lines.end(), [&](const auto& line) {
		//auto max_y = max_element(line.rbegin(), line.rend(), [](const auto& x1, const auto& x2) ->bool {
		//	return x1.y < x2.y;
		//});
		//auto max_base = max_y.base();
		vector<cv::Point2f> contour;
		for_each(line.begin(), line.end(), [&](const auto& pt) {
			contour.push_back(cv::Point2f(pt));
		});
		cv::perspectiveTransform(contour, contour, CalibSpace::warpmat_src2ipm);

		//reverse(contour.begin(), contour.end());
		auto spt = contour.front();
		auto ept = contour.back();
		spt.y += 20;
		ept.y -= 10;
		contour.insert(contour.begin(), spt);
		contour.push_back(ept);
		contoursf.push_back(contour);
	});


	//return;
#if 1
	//区分横向分布
	//auto itr_line = contoursf.begin();
	//for (; itr_line != contoursf.end(); itr_line++)
	//{
	//	auto& line = *itr_line;
	//	auto max_y = max_element(line.begin(), line.end(), [](const auto& x1, const auto& x2) ->bool {
	//		/*return int(x1.y) < int(x2.y);*/
	//		return x1.y < x2.y;
	//	});
	//	line.erase(max_y + 1, line.end());
	//	reverse(line.begin(), line.end());
	//}


	mergeImageLines(contoursf);

	map<int, LINE_VEC> tmp_map;
	map<int, vector<vector<cv::Point2f>>> h_lines_map;
	for_each(contoursf.begin(), contoursf.end(), [&](const auto& cf) {
		vector<cv::Point> contour(cf.size());
		transform(cf.begin(), cf.end(), contour.begin(), [&](const auto& pf)->cv::Point {
			return cv::Point(pf);
			});

		tmp_map[OC_lane].push_back(contour);
		});
	lines_map.swap(tmp_map);

	return;
#endif
}

void AlignImageHD::mergeImageLines(vector<vector<cv::Point2f>>& line_vec)
{
	auto itr_l = line_vec.begin();
	for (; itr_l != line_vec.end();)
	{
		auto& line = *itr_l;
		const auto& ept = line.back();

		map<double, int> dis_idx_map;
		int idx = 0;
		for_each(line_vec.begin(), line_vec.end(), [&](const auto& line_o){
			const auto& spt = line_o.front();
			auto  p = ept - spt;
			double dis = p.ddot(p);
			//return (abs(p.x) <= 5 &&
			//	        p.y >= 0 && p.y < 150);		
			if (abs(p.x) <= 20 &&
				p.y >= 0 && p.y < 100)
			{
				dis_idx_map.insert(make_pair(dis, idx));
			}
			idx++;
		});

		if (dis_idx_map.size() > 0)
		{
			int idx = dis_idx_map.begin()->second;

			const auto& line_o = line_vec[idx];
			vector<cv::Point2f> insert_line(2);
			insert_line[0] = line.back();
			insert_line[1] = line_o.front();
			insertPoint(insert_line, 1.0);

			//		copy(insert_line.begin() + 1, insert_line.end() - 1, back_inserter(line));

			copy(line_o.begin(), line_o.end(), back_inserter(line));
			auto itr_o = line_vec.begin() + idx;
			line_vec.erase(itr_o);
		}
		else
		{
			itr_l++;
		}
	}
}

template<typename T>
void AlignImageHD::horizonalAssignHDMap(vector<T>& local_local_map_vec, const vector<TrajectoryData>& local_ins_vec)
{
	if (local_local_map_vec.size() == 0 ||
		local_ins_vec.size() == 0)
	{
		return;
	}

	vector<double> x_vec;
	auto itr = local_local_map_vec.begin();
	for (; itr != local_local_map_vec.end(); itr++)
	{
		auto& obj = *itr;
		const auto& spt = obj.shape.front();
		const auto& ept = obj.shape.back();
		x_vec.push_back(spt.x);
	}

	if (x_vec.size() == 0)
	{
		return;
	}

	auto itr_max = max_element(x_vec.begin(), x_vec.end(), [](const auto& x1, const auto& x2) ->bool {
		return x1 < x2;
	});

	auto itr_min = min_element(x_vec.begin(), x_vec.end(), [](const auto& x1, const auto& x2) ->bool {
		return x1 < x2;
	});

	map<double, double> value_p_map;
	double unitInterval = 1.0;
	HN_GENERAL::calcHistogram(x_vec, *itr_min, *itr_max, value_p_map, unitInterval);

	vector<double> interval_vec;
	double merge_width = 3.0;
	findPeakPoints(value_p_map, interval_vec, merge_width);
	for_each(interval_vec.begin(), interval_vec.end(), [&](auto& v) {
		v = v *unitInterval + *itr_min;
	});

	vector<int> hx_vec;
	horizonalAssignLocalHDMapX(interval_vec, local_ins_vec[0], hx_vec);

//	map<int, Point3d_VEC> temp_lines_map;
	itr = local_local_map_vec.begin();
	for (; itr != local_local_map_vec.end(); itr++)
	{
		auto& obj = *itr;
		const auto& spt = obj.shape.front();
		auto find_v = min_element(interval_vec.begin(), interval_vec.end(),
			[&](const auto& v1, const auto& v2)->bool {
			return abs(spt.x - v1) < abs(spt.x - v2);
		});
	
		double dista = distance(interval_vec.begin(), find_v);
		int sq = hx_vec[dista];
		if (sq > 2) { sq = 2; }
		if (sq < -2) { sq = -2; }
		obj.horizonal_idx = sq;
	}
}

void AlignImageHD::saveImage(const string& sub_folder, 
	const string& img_na,
	const cv::Mat& img, 
	const cv::Scalar& sca,
	map<int, LINE_VEC>& lines_map)
{
	//if (!cp.regist_flg)
	//{
	//	cv::putText(t, "RESET", cv::Point(0, 40), 2, 1.0, cv::Scalar(0, 0, 0), 2);
	//}
	//cv::putText(t, to_string(cp.regist_probability), cv::Point(0, 80), 2, 1.0, cv::Scalar(255, 0, 0), 2);
	//cv::putText(t, to_string(cp.res), cv::Point(0, 160), 2, 1.0, cv::Scalar(255, 0, 0), 2);


	cv::Mat mat_ply = img.clone();
	for_each(lines_map.begin(), lines_map.end(), [&](const auto&l) {
		cv::polylines(mat_ply, l.second, false, sca, 5);
	});

	string ply_path = MY_CONFIG.mid_path + sub_folder + img_na + ".jpg";
	cv::imwrite(ply_path, mat_ply);
}

void AlignImageHD::undistortImage(const cv::Mat& src_img,
	cv::Mat& dest_img)
{
	if (CalibSpace::distCoeffs.at<double>(0, 0) != 0)
	{
		cv::undistort(src_img, dest_img, CalibSpace::intrisicMat, CalibSpace::distCoeffs);
	}
}

void AlignImageHD::collectLocalIns(const vector<TrajectoryData>& ins_vec, int idx, double len, vector<TrajectoryData>& local_ins_vec,
	bool add, bool farward)
{
	if (idx >= ins_vec.size())
	{
		return;
	}
	const TrajectoryData& cur_ins = ins_vec[idx];
	if (farward)
	{
		local_ins_vec.push_back(cur_ins);
	}

	double dis = 0;
	auto last_ins = cur_ins;

	if (farward)
	{
		for (int i = idx + 1; i < ins_vec.size(); i++)
		{
			auto  p = last_ins.point - ins_vec[i].point;
			dis += sqrt(p.dot(p));
			last_ins = ins_vec[i];
			if (dis > len)
			{
				break;
			}
			local_ins_vec.push_back(last_ins);
		}
	}
	else
	{
		for (int i = idx - 1; i >= 0; i--)
		{
			auto  p = last_ins.point - ins_vec[i].point;
			dis += sqrt(p.dot(p));
			last_ins = ins_vec[i];
			if (dis > len)
			{
				break;
			}
			local_ins_vec.insert(local_ins_vec.begin(), last_ins);
		}
	}
	if (!add)
	{
		return;
	}
}


void AlignImageHD::transformCoord(HDObject_VEC& _data)
{
	if (_data.size() < 1 ||
		_data.front().shape.size() == 0)
	{
		return;
	}
	auto spt = _data.front().shape.front();
	for_each(_data.begin(), _data.end(), [&](auto& d) {
		equalizPolyline(d.shape, 0.5);
		d.shape_org = d.shape;

		});
	return;
}