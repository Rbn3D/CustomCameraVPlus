/*
	THIS FILE IS A PART OF GTA V SCRIPT HOOK SDK
				http://dev-c.com
			(C) Alexander Blade 2015
*/

#pragma once

#include "..\..\inc\natives.h"
#include "..\..\inc\types.h"
#include "..\..\inc\enums.h"

#include "..\..\inc\main.h"

/*
 Eigen Math lib
*/
#include <iostream>
#include "..\..\inc\Eigen\Dense"

using namespace Eigen;

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif 


void DisableCustomCamera();

float getVehicleLongitude(Vehicle vehicle);

float getVehicleHeight(Vehicle vehicle);

void ScriptMain();

void PrintDebugVar();

void lookBehind1p();

Vector3f GetBonePos(Entity entity, char * boneName);
