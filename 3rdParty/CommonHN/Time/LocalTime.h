#ifndef LOCALTIME_H_
#define LOCALTIME_H_

#include <string>
#include <time.h>
#include "CommonHN.h"

namespace HN_GENERAL
{
	// 获取时间字符串
	std::string GetCurrentTimeStr();

	// 获取时间字符串标识文件
	std::string GetTimeFlag();

	// 获取时分秒
	void GetTimeHMS(time_t t, uint32_t& hour, uint32_t& min, uint32_t& second);

	// 秒转时间字符串
	std::string FormatTime(time_t time1);

	// 获取时间秒
	time_t GetTimeS();

	std::string getFileModifyTime(const std::wstring& file_na);

	//标准时间转换成时间戳
	COMMONHN_API long long standard_to_stamp(char *str_time);

	//时间戳转换成标准时间
	COMMONHN_API tm stamp_to_standard(long long stampTime);

	//时间戳秒级运算
	long long calcTimeStampAfterSeconds(long long stampTime, int sec);

	long long calcTimeStampOffset(long long Timestamp1, long long Timestamp2);
}
#endif  // LOCALTIME_H_


