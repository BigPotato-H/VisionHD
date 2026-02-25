#include "ShpDifference.h"
#include <algorithm>


void removeOverloadData(const ObjectDiff_VEC& obj_vec, ObjectDiff_VEC& hd_vec)
{
	if (obj_vec.size() == 0)
	{
		return;
	}

	KRect<double> big_box = obj_vec.front().rect;
	std::for_each(obj_vec.begin() + 1, obj_vec.end(), [&](const auto& obj) {
			big_box = UnionRect(big_box, obj.rect);
		}
	);
	size_t org_sz = hd_vec.size();
	auto remove_itr = remove_if(hd_vec.begin(), hd_vec.end(), [&](const auto& obj)->bool {
		return !(big_box.isCross(obj.rect) || obj.rect.isCross(big_box));
		});
	hd_vec.erase(remove_itr, hd_vec.end());
	size_t remove_sz = org_sz - hd_vec.size();
	return;
}



void ShpDifference::calcDiff(ObjectDiff_VEC& obj_vec, ObjectDiff_VEC hd_vec)
{
//	removeOverloadData(obj_vec, hd_vec);

	float ex_d = 1.5;
	auto itr = obj_vec.begin();
	for (; itr != obj_vec.end(); itr++)
	{
		const string& obj_id = itr->obj_id;
		if (obj_id == "6092")
		{
			int a = 0;
		}
		const auto& shp = itr->shape;
		vector<HPoint3d>& foot_shp = itr->diff_shape;
		auto& flgs = itr->flgs;

		auto itr_pt = shp.begin();
		for (; itr_pt != shp.end(); itr_pt++)
		{
			const auto& pt = *itr_pt;	
			//KRect(const T & left, const T & bottom, const T & right, const T & top) :
			KRect<double> pt_rect(pt.x - ex_d, pt.y - ex_d, pt.x + ex_d, pt.y + ex_d);
			map<double, HPoint3d> dist_foot_map;
			HPoint3d near_foot;
			for_each(hd_vec.begin(), hd_vec.end(), [&](const auto& hd) {
				if (hd.rect.isCross(pt_rect) ||
					pt_rect.isCross(hd.rect))
				{
					HPoint3d foot;
					double d = KPolylineAlg::calcP2PolylineNearestPoint(hd.shape, pt, foot);
					dist_foot_map.insert(make_pair(d, foot));
				}
				});
			if (dist_foot_map.size() > 0 && 
				//dist_foot_map.begin()->first < 1.0)
				dist_foot_map.begin()->first < 1.5)
			{
				near_foot = dist_foot_map.begin()->second;
				flgs.push_back(1);
			}
// 			else if (dist_foot_map.size() > 1)
// 			{
// 				near_foot = dist_foot_map.begin()->second;
// 				flgs.push_back(0);
// 			}
			else
			{
				near_foot = pt + HPoint3d(1.0, 1.0, 1.0);
				flgs.push_back(0);
			}

			foot_shp.push_back(near_foot);
		}
	}
}

