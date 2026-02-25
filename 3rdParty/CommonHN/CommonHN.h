#pragma once

#ifdef COMMONHN_EXPORTS
#define COMMONHN_API __declspec(dllexport)
#else
#define COMMONHN_API __declspec(dllimport)
#endif