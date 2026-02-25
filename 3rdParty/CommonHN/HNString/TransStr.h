#ifndef  TRANSSTR_H_
#define TRANSSTR_H_

#include <atlconv.h>
#include <string>
//#include <boost/process.hpp>

inline std::string W2Astr(const std::wstring& wstr)
{
	USES_CONVERSION;
	return std::string(W2A(wstr.c_str()));
}

inline std::wstring A2Wstr(const std::string& wstr)
{
	USES_CONVERSION;
	return std::wstring(A2W(wstr.c_str()));
}

#endif //TRANSSTR_H_
