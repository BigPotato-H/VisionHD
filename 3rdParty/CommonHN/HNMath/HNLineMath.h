#pragma once
#ifndef _POLYLINEMath_H
#define _POLYLINEMath_H
#include "PlaneMath.h"
#include <windows.h>
#include "CommonHN.h"

using namespace std;

namespace HNMath{

#define  MATH_BASE PlaneMath< double, TPoint3d>

typedef TPoint3d<double> HPoint3d;
typedef pair<HPoint3d, HPoint3d> Point3dPair;
typedef vector< HPoint3d> HPolyline3d;
typedef	 size_t						SIZET;
//typedef	 int							BOOL;

class  COMMONHN_API HNLineMath
{
public:
	static double s_iden_position_threshold;
	// purpose: 起点尾部点是否为同一位置上的点
	// 注：判断同一位置，使用在KGeometry 中的阈值
	// purpose:求线段的空间长度
	static double length( const HPoint3d& _s_pt, const HPoint3d& _e_pt, bool ignore_z = true);


	static bool isIdenticalLocation(const HPoint3d& _s_pt, const HPoint3d& _e_pt);
	static double adjustZ(const HPoint3d& _s_pt, const HPoint3d& _e_pt, HPoint3d& _pt_online, double _threshold = 0.1);
	//purpose:对直线段切割div_n等分后，返回第div_idx等分上的点
	//@<-_div_n 等分数
	//@<-_div_seq 指定所需返回的第几个等分点
	static HPoint3d calcDivPoint( const HPoint3d& _s_pt, const HPoint3d& _e_pt, UINT32 _div_n, UINT32 _div_seq );	

	//purpose: 计算点距离线段上最近的一个点，返回的点必须是线段上的点
	//@<-_pt ：线外的点
	//@->_nearest_pt: 距离_pt最近的一个点
	//return: 0->垂足， 1->起点， 2->尾点
	static int calcP2LNearestPoint( const HPoint3d& _s_pt, const HPoint3d& _e_pt, const HPoint3d& _pt,  HPoint3d& _nearest_pt );

	//purpose：计算点到线段上的最短距离
	static double calcP2LDistance( const HPoint3d& _s_pt, const HPoint3d& _e_pt, const HPoint3d& _pt);

	//purpose::计算点到射线上的最短距离（最近点可以在线上也可以在延迟线上）
	static double calcP2RayDistance( const HPoint3d& _s_pt, const HPoint3d& _e_pt, const HPoint3d& _pt);

	// purpose:判定点是否在线段上
	// return : 0->不在线上，1->线上， 2->起点重合, 3->尾点重合
	static int  isOnLineSegment( const HPoint3d& _s_pt, const HPoint3d& _e_pt, const HPoint3d& _pt);

	// purpose:: 基于线段上一点求垂线
	// @->_pt_r_toward 基于线段的s->e 方向，垂直射线基于线段的右侧朝向点
	// @->_pt_l_toward 基于线段的s->e 方向，垂直射线基于线段的左侧朝向点
	static int getVerticalRayLine( const HPoint3d& _s_pt, const HPoint3d& _e_pt, const HPoint3d& _ray_s_pt, HPoint3d& _ray_r_toward_ept, HPoint3d& _ray_l_toward_ept );

	// purpose:: 基于一条线段计算器射线方向指定距离的一个点
	// @->_s_pt 线段起点
	// @->_e_pt 线段起点
	// @-> _toward_dist: 基于e_pt 处的距离
	// @<- _ray_toward_pt 在指定距离处的一个点
	static BOOL calcRayLinePointByDistance(const HPoint3d& _s_pt, const HPoint3d& _e_pt, double _toward_dist, HPoint3d& _ray_pt);

	// purpose: 计算两条线段的交点
	//@->_s1_pt,
	//@->_e1_pt,
	//@->_s2_pt,
	//@->_e2_pt,
	//@<- _cross_pt
	static BOOL calcCrossingPoint(const HPoint3d& _s1_pt, const HPoint3d& _e1_pt, const HPoint3d& _s2_pt, const HPoint3d& _e2_pt, HPoint3d& _crossing_point );

	// purpose:计算一条射线与一条线段的交点
	// @->_ray_s_pt ， 射线起点
	// @->_ray_toward_pt, 射线朝向点
	// @<- _crossing_point, 交叉点
	// return : 0 没有交点； 1： 交点在线段上（_s_pt->_e_pt) ; 2: 交点在线段（_s_pt->_e_pt)起点； 3 交点在线段（_s_pt->_e_pt)尾点;
	static BOOL calcRayLineCrossingPoint(const HPoint3d& _s_pt, const HPoint3d& _e_pt, const HPoint3d& _ray_s_pt, const HPoint3d& _ray_toward_pt, HPoint3d& _crossing_point );

	// purpose: 计算线段1->线段2段的夹角，已起点到尾点方向
	// 角度>0, 则线段1位于线段2的顺时针方向
	// 角度<0, 则线段1位于线段2的逆时针方向
	// return : 返回角度（-180 <=0<=+180I )
	static double calcLineSegmentVectorAngle( const HPoint3d& _s1_pt, const HPoint3d _e1_pt, const HPoint3d& _s2_pt, const HPoint3d _e2_pt );	

	// purpose: 计算线段1->线段2段的夹角，起点到尾点方向	
	// return : 返回角度（0~360 )
	static double calcVectorAngle(const HPoint3d& _s1_pt, const HPoint3d _e1_pt, const HPoint3d& _s2_pt, const HPoint3d _e2_pt );

	// purpose:: 使用矢量（方向线段）的方位关系
	//@<-_s_pt1, 线段1起点
	//@<-_e_pt1, 线段1尾点
	//@<-_s_pt2, 线段2起点
	//@<-_e_pt2, 线段2尾点
	// return <0 线段2位于线段1的顺时针方向；否则为逆时针方向
	static double calcLineSegmentVectorDirection(const HPoint3d& _s1_pt, const HPoint3d _e1_pt, const HPoint3d& _s2_pt, const HPoint3d _e2_pt  );

	// purpose::旋转一个矢量到指定的角度
	//@<-_s_pt1, 矢量起点
	//@<-_e_pt1, 矢量尾点
	//@<- _angle, 旋转角度	(0~360)
	//@<-_e_pt2, 旋转后的矢量尾点（起点保持不变）
	// return 
//	static double rotateVector( const KLonLatZ& _s1_pt, const KLonLatZ& _e1_pt, double angle, KLonLatZ& _e_pt2 );

	// purpose : 基于向量间夹角，构成向量对子	
	// @<- linevec_set 待排序的向量集合
	//@->_linevector_pair_group 对子集合，first 位于 second 的逆时针方向
	// return :如果有落单的情况（输入向量为奇数）返回 FALSE
	static BOOL groupVectorPairByAngle( const vector<Point3dPair>& _linevec_set, vector< pair< Point3dPair, Point3dPair> >& _linevector_pair_group  );




	//purpose: 计算线段（s->e)与正北方向夹角
	//@<-toward_flg = 1 正北方向朝向
	//@<-toward_flg = 2 正东方向朝向
	static double calcHeading( const HPoint3d& _s_pt, const HPoint3d& _e_pt, int _toward_flg = 1 );	

	//purpose:: 计算坡度
	//@->_slope : 计算所得坡度，如果计算失败，设置值为DBL_INF
	//return 标识计算是否成功
	static bool calcSlope( const HPoint3d& _s_pt, const HPoint3d& _e_pt, double& _slope );

	static void rotateAtSpt(const HPoint3d& _s_pt, HPoint3d& _e_pt, double _angle);
	static void rotateAtEpt(HPoint3d& _s_pt, const HPoint3d& _e_pt, double _angle);
	static void rotate(HPoint3d& _s_pt, HPoint3d& _e_pt, const HPoint3d& _base_pt, double _angle);
	//purpose: 计算两线段交叉点
	//return 0: 不想交
	// return 1: 空间相交；2 平面相交
	static UINT8 calcLineSegmentCrossPoint( const HPoint3d& _s_pt1, const HPoint3d& _e_pt1,const HPoint3d& _s_pt2, const HPoint3d& _e_pt2, HPoint3d& _cross_pt );

	// 比较两个向量大小
	static bool VectorLess( Point3dPair _left, Point3dPair _right );

};

typedef HNLineMath KLineVectorAlg;

class  KLineVectorLess
{
public:
	bool operator()( const Point3dPair& _left, const Point3dPair& _right ) const
	{
		return KLineVectorAlg::VectorLess( _left, _right );
	}
};
class  COMMONHN_API KPolylineAlg
{
public:
	//purpose: calcuate polyline boundbox
	static void calcBoundBox( const HPolyline3d& _polyline , KRect<double>& _box );

	//purpose:计算曲线长度
	static double calcPolylineLength( const HPolyline3d& _polyline );

	//purpose: 根据起点和终点，曲线的中间所用形点的Z值
	//@<- 起始位置高度的点
	//@<- 结束位置的高度点
	//@<-> 需要调整Z值得Polyline
	static void adjustZ( HPoint3d _s_pt, HPoint3d _e_pt, HPolyline3d& _polyline );

	//purpose: 计算从曲线起点开始，到指定点的距离
	// _to_pt : 指点点，该点必须在曲线上，否则，返回-1.000000
	static double calcProtionLengthFromS( const HPolyline3d& _polyline, const HPoint3d& _to_pt );
	static double calcProtionLengthFromE( const HPolyline3d& _polyline, const HPoint3d& _to_pt );
	
	// purpose:计算一点距离折现最近的一个顶点
	static HPoint3d calcNearestVertex( const HPolyline3d& _polyline, const HPoint3d& _point );

	// purpose:计算一个点到曲线上最短距离的一个点
	//@<-_polyline 曲线
	//@<-_point 计算点
	//@->_nearest_pt 距离_point 最近的一个曲线上的点
	//return 最近距离
	static double calcP2PolylineNearestPoint( const HPolyline3d& _polyline, const HPoint3d& _point, HPoint3d& _nearest_pt );

	// purpose: 判定一个点是否在折线上
	// @return:	
	//  < -1, -1> : not no polyline
	// < n, -1> : the split point is overlapped  on start point[n]
	// < n, n+1 >: the split point is between  point[n]  and point[n+1]
	static pair< int, int > isOnPolyline( const HPolyline3d& _polyline, const HPoint3d& _pt );

	// purpose: 通过一个点切割一条折线
	//@->_front_polyline :切割点前的区段
	//@->_post_polyline :切割点后的区段
	static bool cutPolyline( const HPolyline3d& _polyline, const HPoint3d& _pt, HPolyline3d& _front_polyline, HPolyline3d& _post_polyline );

	//purpose: 通过一个点和长度切割一条曲线
	//@<-lenght 基于切割垫_pt 向前或向后（_length <0 )推进的长度
	//@->post_polyling 基于_pt点在_polyline 上获取_length 长度的局部区段。
	static bool cutPolyline( const HPolyline3d& _polyline, const HPoint3d& _pt, double _length, HPolyline3d& _post_polyline );

	// purpose::通过一系列点切割一条折线
	static SIZET cutPolyline( const HPolyline3d& _polyline, const vector<HPoint3d>& _split_pt, vector<HPolyline3d>& _splitted_polyline );
	// 依靠形点位置切割polyline
	static bool cutPolyline( const HPolyline3d& _polyline, int _split_idx, HPolyline3d& _front_polyline, HPolyline3d& _post_polyline);


	/*
	purpose:将曲线按照指定长度切割成一组曲线
	@<-_polyline : 待切割的曲线
	*/
	static bool cutPolylineByLen(const HPolyline3d& _polyline, double _length, vector<HPolyline3d>& _splitted_polylines);

	/*
	purpose: 指定曲线上的一个点，给出从该点出发，沿曲线数字化方向和逆数字化方向一定长度，裁剪出一条曲线
	@<-_polyline : 待切割的曲线
	@<-_base_pt: 裁剪的基准点
	@<- _toward_dist: 顺长度
	@<-_backward_dist:逆长度
	@->_cliped_polyline:裁剪后的曲线
	@return : 返回切割垫在cliped 中的下标位置
	*/
	static SIZET clipPolylineByBasePoint( const HPolyline3d& _polyline, HPoint3d _base_pt, double _toward_dist, double _backward_dist, HPolyline3d& _cliped_polyline  );

	//purpose:对一条曲线进行形点均匀化处理，保持形点间距一致
	//@<-_src_polyline 曲线
	//@<-_equalize_density 均匀化密度（单位：米）
	//@->equlized_polyline 均匀化后曲线
	static SIZET equalizPolyline( const HPolyline3d& _src_polyline, double _equalize_density, HPolyline3d& _equlized_polyline  );

	//purpos:计算距离曲线起点一定距离的一个点
	//@<- _polyline :曲线
	//@<- _dist: 给定距离起点的距离
	//@->_point:计算出的点
	//return ：true 获取成功，false 给定距离大于曲线距离
	static BOOL getPtAwayfromPolylineSPt( const HPolyline3d& _polyline, double _dist, HPoint3d& _point );

	// 下面这个版本适用于对同一条曲线如果多次多次调用该函数时，避免重复计算长度使用
	static BOOL getPtAwayfromPolylineSPt( const HPolyline3d& _polyline, double _length, double _dist, HPoint3d& _point );

	//purpos:计算距离曲线尾点一定距离的一个点
	//@<- _polyline :曲线
	//@<- _dist: 给定距离起点的距离
	//@->_pointt:计算出的点
	//return ：true 获取成功，false 给定距离大于曲线距离
	static BOOL getPtAwayfromPolylineEPt( const HPolyline3d& _polyline, double _dist, HPoint3d& _point );

	//purpose: 计算从曲线某形点开始为起始点，距离一定距离后的一个点
	//@<- _polyline :曲线
	//@<-_splitpt_idx: 曲线上某点
	//@<- _dist: 给定距离起点的距离
	//@->_point:计算出的点
	//@<-_to_from_flg:  ture: _split point->startpoint; false： _split_point->end point
	//return ：1： 正确；0：给定的距离大于曲线长度；-1：_split_ptidx 大于总形点数
	static BOOL getPtAwayfromPolylinePt( const HPolyline3d& _polyline, UINT32 _split_ptidx, double _dist, HPoint3d& _point , bool _to_from_flg); 

	//purpose: 计算从曲线某形点作为起始点，距离一定距离后的取一个点
	//@<- _polyline :曲线
	//@<-_s_pt: 曲线上某点,必须在线
	//@<- _dist: 给定距离起点的距离
	//@->_point:计算出的点
	//@<-_to_from_flg:  ture: _split point->end point; false： _split_point->start point
	//return ：1： 正确；0：给定的距离大于曲线长度；-1：_dist 
	static BOOL getPtAwayfromPolylinePt( const HPolyline3d& _polyline, const HPoint3d& _s_pt, double _dist, HPoint3d& _point , bool _to_from_flg); 


	//purpos:基于给定曲线的起点，进行整体旋转，
	// 使得给曲线的尾点，落在指定的点上
	// @<- _polyline
	// @<- _to_pt 曲线尾点需要到达的点位置
	// @<- _roate_len 扭转长度
	static bool rotatePolylineAtSpoint( const HPolyline3d& _polyline, HPoint3d _to_pt, double _rotate_len, HPolyline3d& _tidy_polyline );

	static bool rotatePolylineAtEpoint(const HPolyline3d& _polyline, HPoint3d _to_pt, double _rotate_len, HPolyline3d& _tidy_polyline);
	static void stretchPolylineAtSpoint(const HPolyline3d& _polyline, HPolyline3d& _stretch_polyline, double _dist);
	static void stretchPolylineAtEpoint(const HPolyline3d& _polyline, HPolyline3d& _stretch_polyline, double _dist);
	//purpos:基于给定曲线的尾点，进行整体旋转，
	// 使得给曲线的起点，落在指定的点上
	// @<- _polyline
	// @<- _to_pt 曲线尾点需要到达的点位置
	// @<- _roate_len 扭转长度
// 	static bool rotatePolylineAtEpoint( const HPolyline3d& _polyline, KLonLatZ _to_pt, double _rotate_len, HPolyline3d& _tidy_polyline );

	// purpose - 在曲线的起点处向外或内侧拉升或收敛给定的距离
	//@<-_dist：给定的距离 >0.0 向外拉升；否则向内收敛
// 	static void stretchPolylineAtSpoint( const HPolyline3d& _polyline, HPolyline3d& _stretch_polyline, double _dist );
// 	static void stretchPolylineAtEpoint( const HPolyline3d& _polyline, HPolyline3d& _stretch_polyline, double _dist );


	//purpose: 判定曲线是否是相似
	//@<-_threshold ：判定阈值
	static bool chkSimilarlityPolyline(  const HPolyline3d& _polyline1, const HPolyline3d& _polyline2, double _threshold = 0.85);

	//purpose: 计算两条曲线的相似度系数
	//该方法应用里程-HEADING坐标系
	//@<-_density 垂线扫描密度，注意平面投影坐标系下位0.1米，经纬度下需要换算所对应的秒
	static double calcSimilarlityFactor( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2 , double _density=0.1, int _toward_flg = 1 );

	//purpose: 将曲线转换为里程-Heading坐标系（简称SH)
	static void conv2SHPolyline(const HPolyline3d& _xy_polyline, HPolyline3d& _sh_polyline, int _toward_flg = 1 );

	// 计算曲线上所有点的朝向
	static double calcHeading( const HPolyline3d& _polyline, vector< double >& _heading, int _toward_flg = 1  );

	// purpose: 计算曲线上每一个形点基于后一个形点所构成线段的坡度
	//@<-_slopes 坡度值集合
	static SIZET calcSlope( const HPolyline3d& _polyline,map<int/*pt idx*/, double /*slope*/> & _slopes );

	//purpose: 计算射线与曲线的所有交点，交点的按照距离射线起点的距离从近到远排序
	//@<- _polyline :求交的曲线
	//@<-_ray_s_pt :射线起点
	//@<-_ray_toward_pt:射线朝向点
	//@->_cross_pts :所有交点
	//return: 交点数
	static SIZET calcRayLineCrossingPoint( const HPolyline3d& _polyline, const HPoint3d _ray_s_pt, const HPoint3d _ray_toward_pt, vector< HPoint3d >& _cross_pts  );

	//purpose: 计算线段与曲线的所有交点
	//@<- _polyline :求交的曲线
	//@<-_s_pt :线段起点
	//@<-_e_pt:线段尾点
	//@->_cross_pts :所有交点
	//return: 交点数
	static SIZET calcLineSegmentCrossingPoint( const HPolyline3d& _polyline, const HPoint3d _s_pt, const HPoint3d _e_pt, vector< HPoint3d >& _cross_pts, bool _ignored_z = true );

	//purpose: 计算线段与正北朝向BOX边界的所有交点
	//@<- _polyline :求交的曲线
	//@<-_s_pt :线段起点
	//@<-_e_pt:线段尾点
	//@->_cross_pts :所有交点
	//return: 交点数
	static SIZET calcBoxCrossingPoint( const HPolyline3d& _polyline, const KRect<double>& _box, vector< HPoint3d >& _cross_pts, bool _ignored_z = true );



	//purpose: 计算两条曲线的所有交点集合
	//@<- _polyline1: 求交点曲线1
	//@<- _polyline2: 求交点曲线2
	//@->_cross_pts :所有交点
	//return: 交点数
	static SIZET calcPolylineCrossingPoint( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2, vector< HPoint3d >& _cross_pts, bool _ignored_z = true  );

	/*
	purpose: 指定距离步长切割polyline，返回除起点和尾点之间的所有切割点
	注意：最后一个切割点到尾点距离是小于_div_len的
	@<- _polyline 待切割的曲线
	@<-_div_len: 等分切割长度
	@-> _div_pts ：切割点集合
	@return:切割点数量*/
	static SIZET divPolylineByLength( const HPolyline3d& _polyline, double _div_len,  vector<HPoint3d>& _div_pts );

	
	

	//purpose: 计算两条折线之间的宽度集合
	// 每隔指定距离取一个宽度
	//@<-_polyline1 轴线一侧曲线
	//@<-_polyline2 轴线一侧曲线	
	//@<-_density 采集密度
	//@<-_dist_width_map : 距离-〉宽度集合
//	static SIZET calcWidthofTwoPolyline( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2, double _density, vector< KDWCoordinate >& _dist_width_vec );

	//purpose: 计算两条并行曲线的中轴线
	//@<-_polyline1 轴线一侧曲线
	//@<-_polyline2 轴线一侧曲线
	//@->_axis_polyline 轴线
	//@<-_density 采集密度
	//return 轴线形点数
	static SIZET calcAxisofTwoPolyline( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2, HPolyline3d& _axis_polyline, double _density = 1.0 );

	// purpose - 将曲线1投射到曲线2上的区间
	// @return 
	//<-1,-1> 无法获取投影
	// first=1 ：polyline1 起点投影到polyline2上  -1：没有投射到	
	// second=1：polyline1 尾点投影到polyline2上  -1：没有投射到		
	static pair<int,int> calcProjProtionPolyline( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2, HPolyline3d& _proj_polyline);
	
	//purpose: 评定曲线是否包含
	// @<- _polyline1 曲线
	// @<-_polyline2  包含曲线
	// return true: false
	static bool include( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2 );

	
	//purpose: 评定两条曲线是否在阈值控制范围内保持平行和形状相近
	//@<- _polyline1，_polyline2  需进行评判的两条曲线
	//@<-_threshold ，给定两条曲线之间的最大间隔阈值
	//@<-_simpling_density，曲线比较的采样点密度（默认2米）
	// return : FALSE 不平行; 1 平行；2 完全重叠
//	static int isParallel( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2,  double _threshold , double _simpling_density = 2.0 );

	//purpose:按照曲线的数字化方向，评定曲线2是否在曲线1的左侧或右侧
	//return
	//1: _polyline2 位于 _polyline1 的右侧
	//2:  _polyline2 位于 _polyline1的左侧
	//3: 交错了
	static UINT8 calcPolylineDigitalSide( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2 );

	//purpose:根据控制点建立贝塞尔曲线
	// @<-_pts: 控制点，控制点的数量决定了贝塞尔曲线的阶
	// @->_bezier_polyline: 贝塞尔曲线
	// return :形点数量
	static SIZET calcBezierPolyline( const vector< HPoint3d> _cpts, HPolyline3d& _bezier_polyline, double _simpling_density );

	// purpose: 计算一条折现线上产生最大弯曲的位置
	//@<- _polyline 折线
	//@<- _density  计算密度（米）
	//@->_buget_pos 沿着折现的数字化方向，弯曲后端位于右侧（凸）
	//@->_conave_pos: 沿着折现的数字化方向，弯曲后端位于左侧（凹）
	//return : 0 计算失败（折现形点少于3个），1（有凸有凹），2（有凸无凹），3（无凸有凹）
	static BOOL calcPolylineBendPos( const HPolyline3d& _polyline, double _density, HPoint3d& _buget_pos, HPoint3d& _conave_pos );


private:
	/*
	Purpose:通过垂直扫描线长度法计算两条曲线的相似度,不能直接利用(XY)直角坐标系
	*/
	class SimilityAlg
	{
	public:
		//@<-_step_len: The vertical scanningline step len
		static double calcSimilarlityFactor( const HPolyline3d& _polyline1, const HPolyline3d& _polyline2, double _step_len );
		
		//Purpose:计算指定垂直扫描线与曲线相交点到BOX底边的长度
		//@<-_polyline 
		//@<- _scanning_x  The vertical scanningline's x 
		//@<- _box 比较范围的BOX
		//return the length from cross point on polyline to box bottom
		static double calcVertScanningLineCutLength( const HPolyline3d& _polyline, double _scanning_x, const KRect<double>& _box );
	};
	
};


}
#endif