/*
	THIS FILE IS A PART OF GTA V SCRIPT HOOK SDK
				http://dev-c.com
			(C) Alexander Blade 2015
*/

#pragma once

#include "..\..\inc\natives.h"
#include "..\..\inc\types.h"
#include "..\..\inc\enums.h"

#include "utils.h"
#include <map>

#include <Windows.h>
#include <Psapi.h>

/*
 Eigen Math lib
*/
#include <iostream>
#include "..\..\inc\Eigen\Dense"

// SimpleINI
#include "..\..\inc\simpleini\SimpleIni.h"

#include <string.h>

// Manual Transmission input integration
#include "ManualTransmissionApi.h"

using namespace Eigen;

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif 

void DisableCustomCamera();

float getVehicleLongitude(Vehicle vehicle);

float getVehicleLongitudeFromCenterBack(Vehicle vehicle);

float getVehicleHeight(Vehicle vehicle);

float getVehicleHeightFromCenterUp(Vehicle vehicle);

void ScriptMain();

void lookBehind1p();

void ProccessLookLeftRightOrBackInput();

Vector3f GetBonePos(Entity entity, char * boneName);

bool LastInputMethodWasMouseAndKeyboard();

void updateVehicleProperties();

void setupCurrentCamera();

void setGameplayCamRelativeRotation(float heading);

void ShowNotification(char * msg);

void ReadSettings(bool notify);

void setGameplayCameraDirection(Vector3f dir);

Vector3f getCameraForwardVector(Camera cam);

enum eDecorType {
	DECOR_TYPE_FLOAT = 1,
	DECOR_TYPE_BOOL,
	DECOR_TYPE_INT,
	DECOR_TYPE_UNK,
	DECOR_TYPE_TIME
};

const char* const vehicleBones[] =
{
	"chassis",
	"chassis_lowlod",
	//"chassis_dummy",
	//"door_dside_f",
	//"door_dside_r",
	//"door_pside_f",
	//"door_pside_r",
	//"handle_dside_f",
	//"handle_dside_r",
	//"handle_pside_f",
	//"handle_pside_r",
	//"wheel_lf",
	//"wheel_rf",
	//"wheel_lm1",
	//"wheel_rm1",
	//"wheel_lm2",
	//"wheel_rm2",
	//"wheel_lm3",
	//"wheel_rm3",
	//"wheel_lr",
	//"wheel_rr",
	"suspension_lf",
	"suspension_rf",
	"suspension_lm",
	"suspension_rm",
	"suspension_lr",
	"suspension_rr",
	"spring_rf",
	"spring_lf",
	"spring_rr",
	"spring_lr",
	"transmission_f",
	"transmission_m",
	"transmission_r",
	"hub_lf",
	"hub_rf",
	"hub_lm1",
	"hub_rm1",
	"hub_lm2",
	"hub_rm2",
	"hub_lm3",
	"hub_rm3",
	"hub_lr",
	"hub_rr",
	"windscreen",
	"windscreen_r",
	//"window_lf",
	//"window_rf",
	//"window_lr",
	//"window_rr",
	//"window_lm",
	//"window_rm",
	"bodyshell",
	"bumper_f",
	"bumper_r",
	"wing_rf",
	"wing_lf",
	//"bonnet",
	//"boot",
	"exhaust",
	"exhaust_2",
	"exhaust_3",
	"exhaust_4",
	"exhaust_5",
	"exhaust_6",
	"exhaust_7",
	"exhaust_8",
	"exhaust_9",
	"exhaust_10",
	"exhaust_11",
	"exhaust_12",
	"exhaust_13",
	"exhaust_14",
	"exhaust_15",
	"exhaust_16",
	"engine",
	"overheat",
	"overheat_2",
	"petrolcap",
	"petroltank",
	"petroltank_l",
	"petroltank_r",
	"steering",
	"hbgrip_l",
	"hbgrip_r",
	//"headlight_l",
	//"headlight_r",
	//"taillight_l",
	//"taillight_r",
	//"indicator_lf",
	//"indicator_rf",
	//"indicator_lr",
	//"indicator_rr",
	//"brakelight_l",
	//"brakelight_r",
	//"brakelight_m",
	//"reversinglight_l",
	//"reversinglight_r",
	//"extralight_1",
	//"extralight_2",
	//"extralight_3",
	//"extralight_4",
	//"numberplate",
	//"interiorlight",
	"siren1",
	"siren2",
	"siren3",
	"siren4",
	"siren5",
	"siren6",
	"siren7",
	"siren8",
	"siren9",
	"siren10",
	"siren11",
	"siren12",
	"siren13",
	"siren14",
	"siren15",
	"siren16",
	"siren17",
	"siren18",
	"siren19",
	"siren20",
	"siren_glass1",
	"siren_glass2",
	"siren_glass3",
	"siren_glass4",
	"siren_glass5",
	"siren_glass6",
	"siren_glass7",
	"siren_glass8",
	"siren_glass9",
	"siren_glass10",
	"siren_glass11",
	"siren_glass12",
	"siren_glass13",
	"siren_glass14",
	"siren_glass15",
	"siren_glass16",
	"siren_glass17",
	"siren_glass18",
	"siren_glass19",
	"siren_glass20",
	"spoiler",
	"struts",
	"misc_a",
	"misc_b",
	"misc_c",
	"misc_d",
	"misc_e",
	"misc_f",
	"misc_g",
	"misc_h",
	"misc_i",
	"misc_j",
	"misc_k",
	"misc_l",
	"misc_m",
	"misc_n",
	"misc_o",
	"misc_p",
	"misc_q",
	"misc_r",
	"misc_s",
	"misc_t",
	"misc_u",
	"misc_v",
	"misc_w",
	"misc_x",
	"misc_y",
	"misc_z",
	"misc_1",
	"misc_2",
	"weapon_1a",
	"weapon_1b",
	"weapon_1c",
	"weapon_1d",
	"weapon_1a_rot",
	"weapon_1b_rot",
	"weapon_1c_rot",
	"weapon_1d_rot",
	"weapon_2a",
	"weapon_2b",
	"weapon_2c",
	"weapon_2d",
	"weapon_2a_rot",
	"weapon_2b_rot",
	"weapon_2c_rot",
	"weapon_2d_rot",
	"weapon_3a",
	"weapon_3b",
	"weapon_3c",
	"weapon_3d",
	"weapon_3a_rot",
	"weapon_3b_rot",
	"weapon_3c_rot",
	"weapon_3d_rot",
	"weapon_4a",
	"weapon_4b",
	"weapon_4c",
	"weapon_4d",
	"weapon_4a_rot",
	"weapon_4b_rot",
	"weapon_4c_rot",
	"weapon_4d_rot",
	"turret_1base",
	"turret_1barrel",
	"turret_2base",
	"turret_2barrel",
	"turret_3base",
	"turret_3barrel",
	"ammobelt",
	"searchlight_base",
	"searchlight_light",
	"attach_female",
	"roof",
	"roof2",
	"soft_1",
	"soft_2",
	"soft_3",
	"soft_4",
	"soft_5",
	"soft_6",
	"soft_7",
	"soft_8",
	"soft_9",
	"soft_10",
	"soft_11",
	"soft_12",
	"soft_13",
	"forks",
	"mast",
	"carriage",
	"fork_l",
	"fork_r",
	"forks_attach",
	"frame_1",
	"frame_2",
	"frame_3",
	"frame_pickup_1",
	"frame_pickup_2",
	"frame_pickup_3",
	"frame_pickup_4",
	"freight_cont",
	"freight_bogey",
	"freightgrain_slidedoor",
	"door_hatch_r",
	"door_hatch_l",
	"tow_arm",
	"tow_mount_a",
	"tow_mount_b",
	"tipper",
	"combine_reel",
	"combine_auger",
	"slipstream_l",
	"slipstream_r",
	"arm_1",
	"arm_2",
	"arm_3",
	"arm_4",
	"scoop",
	"boom",
	"stick",
	"bucket",
	"shovel_2",
	"shovel_3",
	//"Lookat_UpprPiston_head",
	//"Lookat_LowrPiston_boom",
	"Boom_Driver",
	"cutter_driver",
	"vehicle_blocker",
	"extra_1",
	"extra_2",
	"extra_3",
	"extra_4",
	"extra_5",
	"extra_6",
	"extra_7",
	"extra_8",
	"extra_9",
	"extra_ten",
	"extra_11",
	"extra_12",
	"break_extra_1",
	"break_extra_2",
	"break_extra_3",
	"break_extra_4",
	"break_extra_5",
	"break_extra_6",
	"break_extra_7",
	"break_extra_8",
	"break_extra_9",
	"break_extra_10",
	"mod_col_1",
	"mod_col_2",
	"mod_col_3",
	"mod_col_4",
	"mod_col_5",
	"handlebars",
	"forks_u",
	"forks_l",
	//"wheel_f",
	"swingarm",
	//"wheel_r",
	"crank",
	"pedal_r",
	"pedal_l",
	"static_prop",
	"moving_prop",
	"static_prop2",
	"moving_prop2",
	"rudder",
	"rudder2",
	//"wheel_rf1_dummy",
	//"wheel_rf2_dummy",
	//"wheel_rf3_dummy",
	//"wheel_rb1_dummy",
	//"wheel_rb2_dummy",
	//"wheel_rb3_dummy",
	//"wheel_lf1_dummy",
	//"wheel_lf2_dummy",
	//"wheel_lf3_dummy",
	//"wheel_lb1_dummy",
	//"wheel_lb2_dummy",
	//"wheel_lb3_dummy",
	"bogie_front",
	"bogie_rear",
	"rotor_main",
	"rotor_rear",
	"rotor_main_2",
	"rotor_rear_2",
	"elevators",
	"tail",
	"outriggers_l",
	"outriggers_r",
	"rope_attach_a",
	"rope_attach_b",
	"prop_1",
	"prop_2",
	"elevator_l",
	"elevator_r",
	"rudder_l",
	"rudder_r",
	"prop_3",
	"prop_4",
	"prop_5",
	"prop_6",
	"prop_7",
	"prop_8",
	"rudder_2",
	"aileron_l",
	"aileron_r",
	"airbrake_l",
	"airbrake_r",
	"wing_l",
	"wing_r",
	"wing_lr",
	"wing_rr",
	"engine_l",
	"engine_r",
	"nozzles_f",
	"nozzles_r",
	"afterburner",
	"wingtip_1",
	"wingtip_2",
	"gear_door_fl",
	"gear_door_fr",
	"gear_door_rl1",
	"gear_door_rr1",
	"gear_door_rl2",
	"gear_door_rr2",
	"gear_door_rml",
	"gear_door_rmr",
	"gear_f",
	"gear_rl",
	"gear_lm1",
	"gear_rr",
	"gear_rm1",
	"gear_rm",
	"prop_left",
	"prop_right",
	"legs",
	"attach_male",
	"draft_animal_attach_lr",
	"draft_animal_attach_rr",
	"draft_animal_attach_lm",
	"draft_animal_attach_rm",
	"draft_animal_attach_lf",
	"draft_animal_attach_rf",
	//"wheelcover_l",
	//"wheelcover_r",
	"pontoon_l",
	"pontoon_r",
	"no_ped_col_step_l",
	"no_ped_col_strut_1_l",
	"no_ped_col_strut_2_l",
	"no_ped_col_step_r",
	"no_ped_col_strut_1_r",
	"no_ped_col_strut_2_r",
	"light_cover",
	"emissives",
	"neon_l",
	"neon_r",
	"neon_f",
	"neon_b"
};

Quaternionf QuatEuler(Eigen::Vector3f &camVec);
