#include "SingleImageMatch_Interface.h"
#include "CalibMSS.h"
#include "CameraParasInfo.h"

IMAGEMATCH_API bool ALGProcess_AI::initSensors(const RegistDataSet::CameraPara& cp)
{
	RegistDataSet::CameraPara cpp = cp;
	cpp.cx = 617.9050552012687;
	cpp.cy = 360.0552131723484;
	cpp.fx = 878.5576550221689;
	cpp.fy = 879.1440846856915;
	cpp.distortion.resize(5);
	cpp.distortion = { 0.1659187942726637, -0.2099688386400616,
		-0.0005478097533839729, -0.001973176737947737,
		0.06450080389474298 };
	cpp.dx = 0;
	cpp.dy = 1.43;
	cpp.dz = 0;
	cpp.dheading = 0;
	cpp.dpitch = 0;
	cpp.droll = 0;

	CalibSpace::CX = cpp.cx;
	CalibSpace::CY = cpp.cy;
	CalibSpace::FX = cpp.fx;
	CalibSpace::FY = cpp.fy;

	CalibSpace::distCoeffs = cv::Mat_<double>(1, 5, cv::DataType<double>::type);   // Distortion vector
	for (int i = 0; i < 5; i++)
	{
		CalibSpace::distCoeffs.at<double>(0, i) = cpp.distortion[i];
	}

	CalibSpace::extrinsic_para[0] = cpp.dx;
	CalibSpace::extrinsic_para[1] = cpp.dy;
	CalibSpace::extrinsic_para[2] = cpp.dz;
	CalibSpace::extrinsic_para[3] = cpp.dheading;
	CalibSpace::extrinsic_para[4] = cpp.dpitch;
	CalibSpace::extrinsic_para[5] = cpp.droll;

	setCameraDS();

	return true;
}

IMAGEMATCH_API bool ALGProcess_AI::ImageMatch_Process(const RegistDataSet::LocationInfo& loc,
	RegistDataSet::LocalMapObject_VEC & local_map)
{
	CalibMSS mss;
	mss.process(0);

	if (local_map.size() == 0)
	{
		return false;
	}

	RAW_INS ins;
	ins.name = to_string(loc.timestamp);
	ins.point = loc.point;
	ins.heading = loc.heading;
	ins.pitch = loc.pitch;
	ins.roll = loc.roll;

	HDMapData gbd;
	for_each(local_map.begin(), local_map.end(), [&gbd](const auto& grb) {
		HDObject go;
		go.obj_id = to_string(grb.obj_id);
		go.type = grb.type;
		go.shape.resize(grb.shp.size());
		transform(grb.shp.begin(), grb.shp.end(), go.shape.begin(), [](const auto& pt)->cv::Point3d {
			cv::Point3d cp(pt.x, pt.y, pt.z);
			return cp;
		});
		go.shape_org = go.shape;
		gbd.ld_vec.push_back(go);

	});

	
	//mss.processRegistLocalMapAndSingleImage(ins, gbd, loc.percep_contours);

	return true;
}