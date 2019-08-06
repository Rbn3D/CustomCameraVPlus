#pragma once

#include <Windows.h>

namespace MT 
{
	void* CheckAddr(HMODULE lib, const char* funcName);
	extern void InitializeMtApiIntegration();

	extern bool FunctionsPresent();

	extern bool MT_LookingLeft();
	extern bool MT_LookingRight();
	extern bool MT_LookingBack();
}
