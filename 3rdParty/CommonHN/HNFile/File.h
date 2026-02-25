#pragma once
#ifndef COM_FILE_H
#define COM_FILE_H
#include <string>
#include <vector>
#include "../CommonHN.h"
//#include "Tinyxml2.h"

using namespace std;

namespace HN_GENERAL{
	//查找所有文件夹
	void COMMONHN_API getFolders(const string& path, vector<string>& folders);

	//查找文件路径
	bool COMMONHN_API getAllFiles(const string& path, const string& file, vector<string>& files);
	//查找文件名称
	void COMMONHN_API getAllFilesName(const string& path, const string& file, vector<string>& files);
	//查找指定类型文件路径
	void getAllFilesPath(const string& path, vector<string>& files);

	//从文件中读取sql语句
	bool getSQLs(const string &sql_file_path, vector<string>& sql_vec);

	//删除文件夹
	bool removeDir(const string &szFileDir);
}

#endif // !COM_FILE_H


