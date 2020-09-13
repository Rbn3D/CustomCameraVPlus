#include "FunctionHooks.hpp"

#include <Windows.h>
#include <Psapi.h>
#include <sstream>

namespace hook {

    uint64_t oVehicleBrakeAnimCheck_orig = 0;
    PLH::ZydisDisassembler dis = PLH::ZydisDisassembler(PLH::Mode::x64);

    Ped* PlayerPed;
    Vehicle* PlayerVehicle;
    bool CustomCameraEnabled;
    int CurrentCamera;

    void init() 
    {
        mem::init();

        PLH::x64Detour detour((uint64_t)&mem::VehicleBrakeAnimCheck_orig, (uint64_t)&SomeVehicleBrakeAnimCheck_hook, (uint64_t*)&oVehicleBrakeAnimCheck_orig, dis);
        detour.hook();
    }

    bool SomeVehicleBrakeAnimCheck_hook(Ped* ped, Vehicle* vehicle, float a3, int a4)
    {
        if (ped == PlayerPed
            && vehicle == PlayerVehicle
            && CustomCameraEnabled
            && CurrentCamera == 0 // first person
            )

            return false;

        return PLH::FnCast(mem::VehicleBrakeAnimCheck_orig, mem::VehicleBrakeAnimCheck_orig)(ped, vehicle, a3, a4);

        return mem::VehicleBrakeAnimCheck_orig(ped, vehicle, a3, a4);
    }
}
