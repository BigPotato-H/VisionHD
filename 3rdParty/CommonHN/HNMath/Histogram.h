#ifndef CHITOGRAM_H
#define CHITOGRAM_H

#include<vector>
#include<map>
#include "CommonHN.h"

using namespace std;
namespace HN_GENERAL
{

	COMMONHN_API bool calcHistogram(const vector<double>& z_vec, const double& minz, const double& maxz, 
		map<double, double>& value_p_map, double unitInterval = 1.0/*¼ä¸ô*/);
	
	COMMONHN_API bool calcMergeHistogram(const vector<double>& z_vec, const double& minz, const double& maxz,
		map<double, double>& value_p_map, double unitInterval = 1.0, double merge_with = 5);


	COMMONHN_API int findPeakSection(const map<double, double>& value_p_map);
	COMMONHN_API int findMaxPropertySection(const map<double, double>& value_p_map);

};
#endif