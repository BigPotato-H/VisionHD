#ifndef  NEW_KSTRING_H_
#define NEW_KSTRING_H_
#include "CommonHN.h"
#include "TransStr.h"

#include <vector>
#include <tchar.h>

#define AEOS	('\0')

static const INT32 KFORMAT_MAX = 1024 * 100;
static const INT32 KFORMAT_SIZE = KFORMAT_MAX + 1;

typedef std::wstring StringBase;
//const StringBase EMPTY_STR(_T(""));

/////////////////////////////////kstringœ‡πÿ∫Ø ˝“∆÷≤ 20200305///////////////////////////////////////////////////////////
class COMMONHN_API HNString {
public:

	static void MB_STR(LPCSTR lpcsz,
		std::wstring& outString,
		UINT32 chrset,
		UINT32 dstset = CP_ACP);
	
	static void STR_MB(LPCTSTR lpcsz,
		std::string& outString,
		UINT32 chrset,
		UINT32 srcset = CP_ACP);
	
	static void SplitA(const std::string& str,
		std::vector<std::string>& zstrFileds,
		LPCSTR fieldT,
		bool nvl = false);
	
	static void ReplaceA(std::string& str,
		LPCSTR oldStr,
		LPCSTR newStr);

	static void ReplaceW(std::wstring& str,
		LPCWSTR oldStr,
		LPCWSTR newStr);

	static void FormatA(std::string& tString, LPCSTR format, ...);

	static void FormatW(std::wstring& tString, LPCWSTR format, ...);

	static bool IsRightA(const std::string& tString, LPCSTR lpsz);

	static bool IsRightW(const std::wstring& tString, LPCWSTR lpsz);

	static bool IsLeftA(const std::string& tString, LPCSTR lpsz);

	static bool IsLeftW(const std::wstring& tString, LPCWSTR lpsz);
};
#endif  // ! NEW_KSTRING_H_

