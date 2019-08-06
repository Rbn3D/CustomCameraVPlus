#pragma once
namespace MT {
	extern bool Present;
	extern const char* (*GetVersion)();
	extern bool(*GetActive)();
	extern void(*SetActive)(bool active);
	extern bool(*NeutralGear)();
	extern int(*GetShiftMode)();
	extern void(*SetShiftMode)(int mode);
	extern int(*GetShiftIndicator)();

	// AI Management
	extern void(*AddIgnoreVehicle)(int vehicle);
	extern void(*DelIgnoreVehicle)(int vehicle);
	extern void(*ClearIgnoredVehicles)();
	extern unsigned(*NumIgnoredVehicles)();
	extern const int* (*GetIgnoredVehicles)();
	extern int(*GetManagedVehicle)();

	// Camera
	extern bool(*LookingLeft)();
	extern bool(*LookingRight)();
	extern bool(*LookingBack)();

}

void setupCompatibility();

void releaseCompatibility();