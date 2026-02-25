#pragma once
#include <string>
#include "HNMath/HNLineMath.h"
#include <opencv2/core/core.hpp>
using namespace HNMath;
using namespace std;
//差异类型
enum DIFF_TYPE
{
    UNDEF = 0,  //未定义
    SAME = 1,   //一致
    DIFF,       //不一致
    NOKHC,      //缺少khc
    NOKHB       //缺少khb
};


class ObjectDiff
{
public:
    ObjectDiff() {};
    ~ObjectDiff() {};

	string obj_id;
	string prop_id;
	int type;

    vector<HPoint3d> shape;
    KRect<double> rect;
    vector<HPoint3d> diff_shape;
    vector<int> flgs;
private:
};
typedef vector<ObjectDiff> ObjectDiff_VEC;


class ShpDifference 
{
public:
    ShpDifference() {};
	~ShpDifference() {};
    void calcDiff(ObjectDiff_VEC& obj_vec, ObjectDiff_VEC hd_vec);

};

class Mesh
{
public:
    Mesh() {};
    ~Mesh() {};

    cv::Point p;
    int z;
};
typedef vector<Mesh> Mesh_VEC;
typedef map<pair<int, int>, Mesh> Mesh_MAP;

