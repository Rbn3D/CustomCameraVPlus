#pragma once
#include <cstdint>
#include <vector>

#include "NativeMemory.hpp"
#include "inc/PolyHook/polyhook2/Detour/x64Detour.hpp"
#include "inc/PolyHook/polyhook2/ZydisDisassembler.hpp"

namespace hook {
    void init();
    bool SomeVehicleBrakeAnimCheck_hook(Ped* ped, Vehicle* vehicle, float a3, int a4);

    extern Ped* PlayerPed;
    extern Vehicle* PlayerVehicle;
    extern bool CustomCameraEnabled;
    extern int CurrentCamera;
}
