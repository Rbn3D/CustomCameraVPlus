#include "ManualTransmissionApi.h"
//#include "Util/Logger.hpp"
#include <Windows.h>
#include <string>

namespace MT {
	bool Present = false;
	const char* (*GetVersion)() = nullptr;
	bool(*GetActive)() = nullptr;
	void(*SetActive)(bool active) = nullptr;
	bool(*NeutralGear)() = nullptr;
	int(*GetShiftMode)() = nullptr;
	void(*SetShiftMode)(int mode) = nullptr;
	int(*GetShiftIndicator)() = nullptr;
	void(*AddIgnoreVehicle)(int vehicle) = nullptr;
	void(*DelIgnoreVehicle)(int vehicle) = nullptr;
	void(*ClearIgnoredVehicles)() = nullptr;
	unsigned(*NumIgnoredVehicles)() = nullptr;
	const int*  (*GetIgnoredVehicles)() = nullptr;
	int(*GetManagedVehicle)() = nullptr;
	bool(*LookingLeft)() = nullptr;
	bool(*LookingRight)() = nullptr;
	bool(*LookingBack)() = nullptr;

	HMODULE GearsModule = nullptr;

}

template <typename T>
T CheckAddr(HMODULE lib, std::string funcName)
{
	FARPROC func = GetProcAddress(lib, funcName.c_str());
	if (!func)
	{
		//logger.Writef("Couldn't get function [%s]", funcName.c_str());
		return nullptr;
	}
	//logger.Writef("Found function [%s]", funcName.c_str());
	return reinterpret_cast<T>(func);
}

void setupCompatibility() {
	//logger.Writef("Setting up Manual Transmission compatibility");
	MT::GearsModule = GetModuleHandle("Gears.asi");
	if (!MT::GearsModule) {
		//logger.Writef("Gears.asi not found");
		return;
	}

	MT::Present = true;

	MT::GetVersion = CheckAddr<const char* (*)()>(MT::GearsModule, "MT_GetVersion");
	MT::GetActive = CheckAddr<bool(*)()>(MT::GearsModule, "MT_IsActive");
	MT::SetActive = CheckAddr<void(*)(bool)>(MT::GearsModule, "MT_SetActive");
	MT::NeutralGear = CheckAddr<bool(*)()>(MT::GearsModule, "MT_NeutralGear");
	MT::GetShiftMode = CheckAddr<int(*)()>(MT::GearsModule, "MT_GetShiftMode");
	MT::SetShiftMode = CheckAddr<void(*)(int)>(MT::GearsModule, "MT_SetShiftMode");
	MT::GetShiftIndicator = CheckAddr<int(*)()>(MT::GearsModule, "MT_GetShiftIndicator");
	MT::AddIgnoreVehicle = CheckAddr<void(*)(int)>(MT::GearsModule, "MT_AddIgnoreVehicle");
	MT::DelIgnoreVehicle = CheckAddr<void(*)(int)>(MT::GearsModule, "MT_DelIgnoreVehicle");
	MT::ClearIgnoredVehicles = CheckAddr<void(*)()>(MT::GearsModule, "MT_ClearIgnoredVehicles");
	MT::NumIgnoredVehicles = CheckAddr<unsigned(*)()>(MT::GearsModule, "MT_NumIgnoredVehicles");
	MT::GetIgnoredVehicles = CheckAddr<const int* (*)()>(MT::GearsModule, "MT_GetIgnoredVehicles");
	MT::GetManagedVehicle = CheckAddr<int(*)()>(MT::GearsModule, "MT_GetManagedVehicle");
	MT::LookingLeft = CheckAddr<bool(*)()>(MT::GearsModule, "MT_LookingLeft");
	MT::LookingRight = CheckAddr<bool(*)()>(MT::GearsModule, "MT_LookingRight");
	MT::LookingBack = CheckAddr<bool(*)()>(MT::GearsModule, "MT_LookingBack");
}

void releaseCompatibility() {
	if (MT::GearsModule) {
		FreeLibrary(MT::GearsModule);
		MT::Present = false;
	}
}