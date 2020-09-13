// Native memory code originally taken from https://github.com/E66666666/GTAVManualTransmission

#pragma once
#include <cstdint>
#include <vector>

#include "inc/shv/types.h"

namespace mem {
void init();
uintptr_t FindPattern(const char* pattern, const char* mask); 
uintptr_t FindPattern(const char* pattStr);
std::vector<uintptr_t> FindPatterns(const char* pattern, const char* mask);
extern uint64_t(*VehicleBrakeAnimCheck_orig)(Ped* ped, Vehicle* vehicle, float a3, int a4);
}
