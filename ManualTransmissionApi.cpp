#pragma once

#include "ManualTransmissionApi.h"

namespace MT
{
	bool FunctionsPresent_Internal;

	HMODULE Module;

	void* CheckAddr(HMODULE lib, const char* funcName)
	{
		FARPROC mtFunc = GetProcAddress(lib, funcName);
		if (mtFunc == nullptr)
		{
			return nullptr;
		}
		return mtFunc;
	}

	extern void InitializeMtApiIntegration()
	{
		//Gears.asi integration
		Module = GetModuleHandle("Gears.asi");
		if (Module)
		{
			FunctionsPresent_Internal = true;
		}
	}

	extern bool FunctionsPresent()
	{
		return FunctionsPresent_Internal;
	}

	extern bool MT_LookingLeft()
	{
		return FunctionsPresent_Internal && (bool(*)(void))CheckAddr(Module, "MT_LookingLeft");
	}

	extern bool MT_LookingRight()
	{
		return FunctionsPresent_Internal && (bool(*)(void))CheckAddr(Module, "MT_LookingRight");
	}

	extern bool MT_LookingBack()
	{
		return FunctionsPresent_Internal && (bool(*)(void))CheckAddr(Module, "MT_LookingBack");
	}
}