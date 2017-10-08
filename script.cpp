/*
*  Custom Camera V Plus
*/
#include "script.h"
#include "utils.h"
#include <string>

#include <Windows.h>
#include <Psapi.h>

#include "..\..\inc\GTAVMenuBase\menu.h"
#include "..\..\inc\GTAVMenuBase\menu.cpp"
#include "..\..\inc\GTAVMenuBase\menucontrols.h"
#include "..\..\inc\GTAVMenuBase\menucontrols.cpp"
#include "..\..\inc\GTAVMenuBase\menukeyboard.h"
#include "..\..\inc\GTAVMenuBase\menukeyboard.cpp"

using namespace Eigen;
using namespace NativeMenu;

float smoothIsMouseLooking = 0.f;
float viewLock = 0.f;
float smoothViewLock = 0.f;

BOOL modEnabled = true;
BOOL camInitialized = false;
Vehicle veh;
Ped playerPed;

Vector3f vehPos;
Vector3f vehRot;
Vector3f vehVelocity;
float vehSpeed;
uint16_t vehGear;
float lastVelocityMagnitude;

Vector3f vehForwardVector;
Vector3f vehRightVector;
Vector3f vehUpVector;

float vehAcceleration = 0.f;
float smoothAccelerationFactor = 0.f;

eVehicleClass vehClass;
bool vehHasTowBone = false;
bool vehHasTrailerBone = false;

Camera customCam = NULL;
float fov3P = 75.f;
float fov1P = 90.f;
const float PI = 3.1415926535897932f;
int lastVehHash = -1;
bool isBike = false;

float longitudeOffset3P = 0.f;
float heightOffset3P = 0.f;

float rotationSpeed3P = 4.75f;

//bool useVariableRotSpeed3P = true;
//float minRotSpeed3P = 4.55f;
//float maxRotSpeed3P = 7.6f;
//float maxRotSpeedAngle3P = 90.0f;
//
//float variableRotLerpSpeed = 1.9f;

bool useVariableRotSpeed3P = true;
float minRotSpeed3P = 3.55f;
float maxRotSpeed3P = 9.6f;
float maxRotSpeedAngle3P = 90.0f;

float variableRotLerpSpeed = 0.75f;

float currentRotSpeed3P = 2.0f;

float useRoadSurfaceNormal = true;
Vector3f cachedSurfaceNormal;
Vector3f smoothSurfaceNormal;

Vector3f smoothVelocity = Vector3f();
Quaternionf velocityQuat3P = Quaternionf();
Quaternionf smoothQuat3P = Quaternionf();
Vector3f smoothRotSeat = Vector3f();
float smoothIsInAir = 0.f;
float maxHighSpeed = 130.f;
float maxHighSpeedDistanceIncrement = 1.8f;
float accelerationCamDistanceMultiplier = 4.38f;

Vector3f up(0.0f, 0.0f, 1.0f);
Vector3f down(0.0f, 0.0f, -1.0f);
Vector3f back(0.0f, -1.0f, 0.0f);
Vector3f front(0.0f, 1.0f, 0.0f);

Vector2i lastMouseCoords;
float mouseMoveCountdown = 0.f;

bool isAiming = false;
float smoothIsAiming = 0.f;

bool customCamEnabled = true;

enum eCamType {
	Smooth3P = 0,
	DriverSeat1P = 1
};

int camsLength = 2;
int currentCam = eCamType::Smooth3P;

NativeMenu::Menu menu;
NativeMenu::MenuControls menuControls;

bool showDebug = false;
UINT_PTR gamePlayCameraAddr;

bool isSuitableForCam;

float towLongitudeIncrement = .0f;
float currentTowLongitudeIncrement = .0f;

float towHeightIncrement = .0f;
float currentTowHeightIncrement = .0f;

float lowUpdateTimerInterval = 1.0f;
float lowUpdateTimerCurrentTime = lowUpdateTimerInterval;

Vehicle lastTowVehicle;
float lastTowVehicleLongitude = 0.f;
Vehicle lastTrailer;
float lastTrailerLongitude = 0.f;

bool AreSameFloat(float a, float b)
{
	return fabs(a - b) < FLT_EPSILON;
}

float getDeltaTime() {
	return SYSTEM::TIMESTEP();
}

std::string SubstringOfCString(const char *cstr,
	size_t start, size_t length)
{
	return std::string(cstr + start, length);
}

float isMouseLooking() {
	return mouseMoveCountdown > 0.001f;
}

void ResetMouseLook() {
	mouseMoveCountdown = 0.f;
	smoothIsMouseLooking = 0.f;
}

float mathRepeat(float t, float length)
{
	return t - floor(t / length) * length;
}

float clamp01(float t) {
	if ((double)t < 0.0)
		return 0.0f;
	if ((double)t > 1.0)
		return 1.f;
	return t;
}

float clamp(float n, float lower, float upper) {
	return max(lower, min(n, upper));
}

float DeltaAngle(float current, float target)
{
	float num = mathRepeat(target - current, 360.f);
	if (num > 180.f)
	{
		num -= 360.f;
	}
	return num;
}

float InertialDamp(float previousValue, float targetValue, float smoothTime)
{
	float x = previousValue - targetValue;
	float newValue = x + getDeltaTime() * (-1.f / smoothTime * x);
	return targetValue + newValue;
}

float InertialDampAngle(float previousValue, float targetValue, float smoothTime)
{
	float x = DeltaAngle(previousValue, targetValue);
	float newValue = x + getDeltaTime() * (-1.f / smoothTime * x);
	return targetValue + newValue;
}

float distanceOnAxis(Vector3f A, Vector3f B, Vector3f axis) {
	//The key is to normalize the arrow vector, so it doesn't scale your distances.
	axis.normalize();
	float ADistanceAlongAxis = axis.dot(A);
	float BDistanceAlongAxis = axis.dot(B);

	return abs(BDistanceAlongAxis - ADistanceAlongAxis);
}

float distanceOnAxisNoAbs(Vector3f A, Vector3f B, Vector3f axis) {
	//The key is to normalize the arrow vector, so it doesn't scale your distances.
	axis.normalize();
	float ADistanceAlongAxis = axis.dot(A);
	float BDistanceAlongAxis = axis.dot(B);

	return BDistanceAlongAxis - ADistanceAlongAxis;
}

uintptr_t FindPattern(const char* pattern, const char* mask) {
	MODULEINFO modInfo = { nullptr };
	GetModuleInformation(GetCurrentProcess(), GetModuleHandle(nullptr), &modInfo, sizeof(MODULEINFO));

	const char* start_offset = reinterpret_cast<const char *>(modInfo.lpBaseOfDll);
	const uintptr_t size = static_cast<uintptr_t>(modInfo.SizeOfImage);

	intptr_t pos = 0;
	const uintptr_t searchLen = static_cast<uintptr_t>(strlen(mask) - 1);

	for (const char* retAddress = start_offset; retAddress < start_offset + size; retAddress++) {
		if (*retAddress == pattern[pos] || mask[pos] == '?') {
			if (mask[pos + 1] == '\0') {
				return (reinterpret_cast<uintptr_t>(retAddress) - searchLen);
			}

			pos++;
		}
		else {
			pos = 0;
		}
	}

	return 0;
}

float easeInCubic(float t) {
	return pow(t, 3.f);
}

float easeOutCubic(float t) {
	return 1.f - easeInCubic(1.f - t);
}

inline void vadd_sse(const float *a, const float *b, float *r)
{
	_mm_storeu_ps(r, _mm_add_ps(_mm_loadu_ps(a), _mm_loadu_ps(b)));
}

Vector3 Subtract(Vector3 left, Vector3 right)
{
	vadd_sse((float*)&left, (float*)&right, (float*)&left);
	return left;
}

Quaternionf negateQuat(Quaternionf q) {
	return Quaternionf(-q.w(), -q.x(), -q.y(), -q.z());
}

float Vector3Angle(Vector3f from, Vector3f to)
{
	double dot = from.normalized().dot(to.normalized());
	return (float)((acos(dot)) * (180.0 / PI));
}

Vector3 getRightVector(Vector3f rotation)
{
	float num = cos(rotation.y() * (PI / 180.0f));
	Vector3 vec;

	vec.x = cos(-rotation.z() * (PI / 180.0f)) * num;
	vec.y = sin(rotation.z() * (PI / 180.0f)) * num;
	vec.z = sin(-rotation.y() * (PI / 180.0f));

	return vec;
}

Vector2i getMouseCoords() {
	int mX = CONTROLS::GET_CONTROL_VALUE(0, 239) - 127 / 127.f * 1280;
	int mY = CONTROLS::GET_CONTROL_VALUE(0, 240) - 127 / 127.f * 720;

	return Vector2i(mX, mY);
}

void updateMouseState() {
	int lastX = lastMouseCoords.x();
	int lastY = lastMouseCoords.y();

	Vector2i currXY = getMouseCoords();
	int currX = currXY.x();
	int currY = currXY.y();

	int movX = abs(lastX - currX);
	int movY = abs(lastY - currY);

	if (isAiming ||movX > 2.f || movY > 2.f) {
		mouseMoveCountdown = 1.5f;
	}

	if((veh != NULL && (vehSpeed > 0.1)) || veh == NULL)
		mouseMoveCountdown = max(0.f, mouseMoveCountdown - (SYSTEM::TIMESTEP() * clamp01(vehSpeed * 0.08f)));

	lastMouseCoords = currXY;
}

// Color struct taken from https://github.com/E66666666/GTAVManualTransmission/
struct Color {
	int R;
	int G;
	int B;
	int A;
};

const Color solidWhite = { 255,	255, 255, 255 };
const Color solidBlack = { 0, 0, 0, 255 };

const Color solidRed = { 255, 0, 0,	255 };
const Color solidGreen = { 0, 255, 0, 255 };
const Color solidBlue = { 0, 0, 255, 255 };

const Color solidPink = { 255, 0, 255, 255 };
const Color solidYellow = { 255, 255, 0, 255 };
const Color solidCyan = { 0, 255, 255, 255 };

const Color solidOrange = { 255, 127, 0, 255 };
const Color solidLime = { 127, 255, 0, 255 };
const Color solidPurple = { 127, 0, 255, 255 };

const Color transparentGray = { 75, 75, 75, 75 };

// showText() taken from https://github.com/E66666666/GTAVManualTransmission/
void showText(float x, float y, float scale, const char* text, int font, const Color &rgba, bool outline) {
	UI::SET_TEXT_FONT(font);
	UI::SET_TEXT_SCALE(scale, scale);
	UI::SET_TEXT_COLOUR(rgba.R, rgba.G, rgba.B, rgba.A);
	UI::SET_TEXT_WRAP(0.0, 1.0);
	UI::SET_TEXT_CENTRE(0);
	if (outline) UI::SET_TEXT_OUTLINE();
	UI::BEGIN_TEXT_COMMAND_DISPLAY_TEXT("STRING");
	UI::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME(CharAdapter(text));
	UI::END_TEXT_COMMAND_DISPLAY_TEXT(x, y);
}


Vector3f toV3f(Vector3 vec) {
	return Vector3f(vec.x, vec.y, vec.z);
}

Vector3 toV3(Vector3f vec) {
	Vector3 ret;
	ret.x = vec.x();
	ret.y = vec.y();
	ret.z = vec.z();

	return ret;
}

Vector3f getDimensions(Hash modelHash) {
	Vector3 min;
	Vector3 max;

	GAMEPLAY::GET_MODEL_DIMENSIONS(modelHash, &min, &max);

	Vector3 ret;

	ret = Subtract(max, min);

	return toV3f(ret);
}

Vector3f getDimensions() {
	Hash modelHash = VEHICLE::GET_VEHICLE_LAYOUT_HASH(veh);
	return getDimensions(modelHash);
}

float getVehicleAcceleration() {
	float mag = vehVelocity.norm();
	float ret = (mag - lastVelocityMagnitude) * SYSTEM::TIMESTEP();

	lastVelocityMagnitude = mag;

	return ret;
}

void nextCam() {
	currentCam++;
	currentCam = currentCam % camsLength;
}

void firstInit()
{
	UINT_PTR address = FindPattern("\x48\x8B\xC7\xF3\x0F\x10\x0D", "xxxxxxx") - 0x1D;
	address = address + *reinterpret_cast<int*>(address) + 4;
	gamePlayCameraAddr = *reinterpret_cast<UINT_PTR*>(*reinterpret_cast<int*>(address + 3) + address + 7);
}

Vector3f getGameplayCameraDirection() {
	const auto data = reinterpret_cast<const float *>(gamePlayCameraAddr + 0x200);
	return Vector3f(data[0], data[1], data[2]);
}

void setGameplayCameraDirection(Vector3f dir) {
	const auto address = (Vector3*)gamePlayCameraAddr + 0x200;

	Vector3 nativeDir = toV3(dir);

	*address = nativeDir;
}

Vector3f getGameplayCameraPos() {
	Vector3 camPos = CAM::GET_GAMEPLAY_CAM_COORD();

	return Vector3f(camPos.x, camPos.y, camPos.z);
}


float dot(Vector3f a, Vector3f b)  //calculates dot product of a and b
{
	return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

float mag(Vector3f a)  //calculates magnitude of a
{
	return std::sqrt(a.x() * a.x() + a.y() * a.y() + a.z() * a.z());
}

float AreSameVector(Vector3f v1, Vector3f v2)
{
	return AreSameFloat(v1.x(), v2.x()) &&
		AreSameFloat(v1.y(), v2.y()) &&
		AreSameFloat(v1.z(), v2.z());
}

float AngleBetweenVectors(Vector3f v1, Vector3f v2)
{
	if (AreSameVector(v1, v2))
		return 0.f;

	return std::acos(dot(v1, v2) / (mag(v1)*mag(v2)));
}

float unlerp(float min, float max, float value) {
	return (value - min) / (max - min);
}

float lerp(float a, float b, float f)
{
	return a + f * (b - a);
}

Vector3f lerp(Vector3f& v1, Vector3f& v2, float t)
{
	return Vector3f(lerp(v1.x(), v2.x(), t),
		lerp(v1.y(), v2.y(), t),
		lerp(v1.z(), v2.z(), t));
}

float lerpAngle(float a, float b, float t)
{
	float num = mathRepeat(b - a, 360.f);
	if (num > 180.f)
	{
		num -= 360.f;
	}
	return a + num * clamp01(t);
}

Vector3f Vector3fLerpAngle(Vector3f& v1, Vector3f& v2, float t)
{
	return Vector3f(lerpAngle(v1.x(), v2.x(), t),
		lerpAngle(v1.y(), v2.y(), t),
		lerpAngle(v1.z(), v2.z(), t));
}

Vector3f Vector3fInertialDamp(Vector3f& v1, Vector3f& v2, float t)
{
	return Vector3f(InertialDamp(v1.x(), v2.x(), t),
		InertialDamp(v1.y(), v2.y(), t),
		InertialDamp(v1.z(), v2.z(), t));
}

Vector3f Vector3fInertialDampAngle(Vector3f& v1, Vector3f& v2, float t)
{
	return Vector3f(InertialDampAngle(v1.x(), v2.x(), t),
		InertialDampAngle(v1.y(), v2.y(), t),
		InertialDampAngle(v1.z(), v2.z(), t));
}

//Quaternionf QuatScalarMultiply(Quaternionf& input, float scalar)
//{
//	return Quaternionf(input.x()/* * scalar*/, input.y()/* * scalar*/, input.z()/* * scalar*/, input.w() * scalar);
//}
//
//Quaternionf QuatAdd(Quaternionf& p, Quaternionf& q)
//{
//	return Quaternionf(p.x() + q.x(), p.y() + q.y(), p.z() + q.z(), p.w() + q.w());
//}
//
//float reciprocal(float f) {
//	return 1.0 / f;
//}
//
//Quaternionf lerp(Quaternionf q1, Quaternionf q2, float time)
//{
//	const float scale = 1.0f - time;
//	return QuatAdd(QuatScalarMultiply(q1, scale), QuatScalarMultiply(q2, time));
//}

Quaternionf slerp(Quaternionf& q1, Quaternionf& q2, float time)
{
	//float angle = q1.dot(q2);

	//// make sure we use the short rotation
	////if (angle < 0.0f)
	////{
	////	q1 = QuatScalarMultiply(q1, -1.0f);
	////	angle *= -1.0f;
	////}

	//if (angle <= (1 - threshold)) // spherical interpolation
	//{
	//	const float theta = acosf(angle);
	//	const float invsintheta = reciprocal(sinf(theta));
	//	const float scale = sinf(theta * (1.0f - time)) * invsintheta;
	//	const float invscale = sinf(theta * time) * invsintheta;
	//	return QuatAdd(QuatScalarMultiply(q1, scale), QuatScalarMultiply(q2, invscale));
	//}
	//else // linear interpolation
	//	return lerp(q1, q2, time);

	return q1.slerp(time, q2);
}

//Quaternionf lerp(float t, const Quaternionf& a, const Quaternionf& b)
//{
//	return a.slerp(t, b);
//}

float smoothStep(float from, float to, float t)
{
	t = clamp01(t);
	t = (float)(-2.0 * (double)t * (double)t * (double)t + 3.0 * (double)t * (double)t);
	return (float)((double)to * (double)t + (double)from * (1.0 - (double)t));
}

Matrix3f matrixLookAt(Vector3f& dir, Vector3f& up)
{
	dir.normalize();
	Vector3f s(dir.cross(up));
	s.normalize();
	Vector3f u(s.cross(dir));
	u.normalize();

	Matrix3f mat;

	mat <<
		s[0], u[0], -dir[0],
		s[1], u[1], -dir[1],
		s[2], u[2], -dir[2];

	return mat;
	//preMultTranslate(-eye);
}

Quaternionf lookRotation(Vector3f dir) 
{
	// Fix for gta coord system
	//Roll pitch and yaw in Radians (-90.0, 0.0, 0.0)
	float roll = -1.5707963267948966f, pitch = 0.f, yaw = 0.f;
	Quaternionf compensation;
	compensation = AngleAxisf(roll, Vector3f::UnitX())
		* AngleAxisf(pitch,  Vector3f::UnitY())
		* AngleAxisf(yaw, Vector3f::UnitZ());

	Matrix3f lookM = matrixLookAt(dir, up);
	Quaternionf ret = Quaternionf(lookM);

	return ret * compensation;
}

Quaternionf lookRotation(Vector3f dir, Vector3f upVector)
{
	// Fix for gta coord system
	//Roll pitch and yaw in Radians (-90.0, 0.0, 0.0)
	float roll = -1.5707963267948966f, pitch = 0.f, yaw = 0.f;
	Quaternionf compensation;
	compensation = AngleAxisf(roll, Vector3f::UnitX())
		* AngleAxisf(pitch, Vector3f::UnitY())
		* AngleAxisf(yaw, Vector3f::UnitZ());

	Matrix3f lookM = matrixLookAt(dir, upVector);
	Quaternionf ret = Quaternionf(lookM);

	return ret * compensation;
}

Vector3f lookRotEuler(Vector3f dir, Vector3f upVector) {
	Matrix3f lookM = matrixLookAt(dir, upVector);

	return lookM.eulerAngles(0, 1, 2);
}

Vector3f QuatEuler(Quaternionf quat) {
	return quat.toRotationMatrix().eulerAngles(0, 1, 2);
}

Quaternionf getEntityQuaternion(Entity entity) {

	float x, y, z, w;
	ENTITY::GET_ENTITY_QUATERNION(entity, &x, &y, &z, &w);

	return Quaternionf(w, x, y, z);
}

void setCamPos(Camera cam, Vector3f pos) {
	CAM::SET_CAM_COORD(cam, pos.x(), pos.y(), pos.z());
}

void camPointAt(Camera cam, Vector3f pos) {
	CAM::POINT_CAM_AT_COORD(cam, pos.x(), pos.y(), pos.z());
}

float getVehLongitude3P(Vehicle veh) {

	Vector3f backPos;
	bool backFound = false;

	int bumperRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "bumper_r");
	if (bumperRBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, bumperRBone));

		backFound = true;
		//goto findFront;
	}

	if (!isBike) {
		int brakeLBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "brakelight_l");
		int brakeRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "brakelight_r");

		if (brakeLBone != -1 && brakeRBone != -1) {
			Vector3f brakeLPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, brakeLBone));
			Vector3f brakeRPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, brakeRBone));

			backPos = (brakeLPos + brakeRPos) * 0.5f;

			backFound = true;
			//goto findFront;
		}
	}

	int spoilerBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "spoiler");
	if (spoilerBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, spoilerBone));

		backFound = true;
		//goto findFront;
	}

	int neonBBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "neon_b");
	if (neonBBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, neonBBone));

		backFound = true;
		//goto findFront;
	}


	int BootBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "boot");
	if (BootBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, BootBone));

		backFound = true;
		//goto findFront;
	}

	int windscreenRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "windscreen_r");
	if (windscreenRBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, windscreenRBone)) + (vehForwardVector * -.9f);

		backFound = true;
		//goto findFront;
	}

	if (!backFound)
		return 5.f;

	Vector3f frontPos;

	int bumperFBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "bumper_f");
	if (bumperFBone != -1)
	{
		frontPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, bumperFBone));

		return distanceOnAxis(backPos, frontPos, vehForwardVector);
	}

	if (!isBike) {
		int headlightLBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "headlight_l");
		int headlightRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "headlight_r");

		if (headlightLBone != -1 && headlightRBone != -1) {
			Vector3f headlightLPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, headlightLBone));
			Vector3f headlightRPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, headlightRBone));

			frontPos = (headlightLPos + headlightRPos) * 0.5f;

			return distanceOnAxis(backPos, frontPos, vehForwardVector);
		}
	}

	int neonFBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "neon_f");
	if (neonFBone != -1)
	{
		frontPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, neonFBone));

		return distanceOnAxis(backPos, frontPos, vehForwardVector);
	}

	return 5.f;
}

float getCamDistance3P(Vehicle veh) {

	Vector3f backPos;

	if (isBike) {
		int wheelRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "wheel_r");

		if (wheelRBone != -1) {
			backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, wheelRBone));

			return distanceOnAxis(backPos, vehPos, vehForwardVector);
		}
	}

	if (!isBike) {
		int brakeLBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "brakelight_l");
		int brakeRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "brakelight_r");

		if (brakeLBone != -1 && brakeRBone != -1) {
			Vector3f brakeLPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, brakeLBone));
			Vector3f brakeRPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, brakeRBone));

			backPos = (brakeLPos + brakeRPos) * 0.5f;

			return distanceOnAxis(backPos, vehPos, vehForwardVector);
		}
	}

	int bumperRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "bumper_r");
	if (bumperRBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, bumperRBone));

		return distanceOnAxis(backPos, vehPos, vehForwardVector);
	}

	int spoilerBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "spoiler");
	if (spoilerBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, spoilerBone));

		return distanceOnAxis(backPos, vehPos, vehForwardVector);
	}

	int neonBBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "neon_b");
	if (neonBBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, neonBBone));

		return distanceOnAxis(backPos, vehPos, vehForwardVector);
	}

	int windscreenRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "windscreen_r");
	if (windscreenRBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, windscreenRBone)) + (vehForwardVector * -.9f);

		return distanceOnAxis(backPos, vehPos, vehForwardVector);
	}

	int BootBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "boot");
	if (BootBone != -1)
	{
		backPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, BootBone));

		return distanceOnAxis(backPos, vehPos, vehForwardVector);
	}

	return 2.0f;
}

float getVehHeight3P(Vehicle veh)
{
	Vector3f roofPos;
	bool roofFound = false;
	int intLightBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "interiorlight");
	if (intLightBone != -1)
	{
		roofPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, intLightBone));

		roofFound = true;
		//return (vehPos - toV3f(lbp)).size() - 1.5f;
	}

	int roofBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "roof");
	if (roofBone != -1)
	{
		roofPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, roofBone));

		roofFound = true;
		//return (vehPos - toV3f(roofPos)).size() - 1.5f;
	}

	int roofBone2 = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "roof2");
	if (roofBone2 != -1)
	{
		roofPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, roofBone2));

		roofFound = true;
		//return (vehPos - toV3f(roofPos2)).size() - 1.5f;
	}

	if (roofFound) {
		int exhaustBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "exhaust");
		if (exhaustBone != -1)
		{
			Vector3f exhaustPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, exhaustBone));

			return distanceOnAxis(exhaustPos, roofPos, vehUpVector);
		}
	}

	return 1.5f;
}

void updateVehicleProperties() 
{
	int vehClass = VEHICLE::GET_VEHICLE_CLASS(veh);
	isBike = vehClass == eVehicleClass::VehicleClassCycles || vehClass == eVehicleClass::VehicleClassMotorcycles;
	isSuitableForCam = vehClass != eVehicleClass::VehicleClassTrains && vehClass != eVehicleClass::VehicleClassPlanes && vehClass != eVehicleClass::VehicleClassHelicopters && vehClass != eVehicleClass::VehicleClassBoats;

	longitudeOffset3P = getVehicleLongitude(veh) + 0.25f;
	heightOffset3P = getVehicleHeight(veh) + 0.12f;

	if (heightOffset3P >= 2.45f) {
		heightOffset3P += 0.7f;
		longitudeOffset3P += 1.8f;
	}

	if (isBike) {
		longitudeOffset3P += .85f;
		heightOffset3P += .25f;
	}
}

bool vehHasBone(char *boneName) {
	return (ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, boneName) != -1);
}

void updateVehicleVars() 
{
	vehPos = toV3f(ENTITY::GET_ENTITY_COORDS(veh, true));
	vehRot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, false));
	vehVelocity = toV3f(ENTITY::GET_ENTITY_VELOCITY(veh));
	vehSpeed = ENTITY::GET_ENTITY_SPEED(veh);
	smoothVelocity = lerp(smoothVelocity, vehVelocity, 10.f * getDeltaTime());
	smoothIsInAir = lerp(smoothIsInAir, (ENTITY::IS_ENTITY_IN_AIR(veh) || ENTITY::IS_ENTITY_UPSIDEDOWN(veh)) ? 1.f : 0.f, 2.f * getDeltaTime());
	vehForwardVector = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(veh));
	vehRightVector = toV3f(getRightVector(vehRot));
	vehUpVector = vehRightVector.cross(vehForwardVector);
	vehClass = (eVehicleClass)VEHICLE::GET_VEHICLE_CLASS(veh);
	vehAcceleration = getVehicleAcceleration();
	vehHasTowBone = vehHasBone("tow_arm");
	vehHasTrailerBone = vehHasBone("attach_female");
}

Vector3f getLatVector() {
	Vector3f f = vehForwardVector;
	float upRightVal = ENTITY::GET_ENTITY_UPRIGHT_VALUE(veh);


	//90 degrees CW about x - axis: (x, y, z) -> (x, -z, y)
	//90 degrees CCW about x - axis : (x, y, z) -> (x, z, -y)

	//90 degrees CW about y - axis : (x, y, z) -> (-z, y, x)
	//90 degrees CCW about y - axis : (x, y, z) -> (z, y, -x)

	//90 degrees CW about z - axis : (x, y, z) -> (y, -x, z)
	//90 degrees CCW about z - axis : (x, y, z) -> (-y, x, z)


	Vector3f latByForward = -Vector3f(-f.y(), f.x(), f.z());

	Vector3f rightVector = toV3f(getRightVector(vehRot));

	float factor = clamp01(unlerp(0.5f, -0.5f, upRightVal));

	Vector3f finalVector = lerp(rightVector, latByForward, factor);

	return finalVector;
}

Quaternionf getSurfaceNormalRotation() {

	Vector3f rightVector = toV3f(getRightVector(vehRot));

	Vector3f source = vehPos;
	Vector3f direction = down;
	float maxDistance = 4.f;
	Vector3f target = source + direction * maxDistance;

	int ray = WORLDPROBE::_START_SHAPE_TEST_RAY(source.x(), source.y(), source.z(), target.x(), target.y(), target.z(), 1, veh, 7);

	Vector3 endCoords, surfaceNormal;
	BOOL hit;
	Entity entityHit = 0;

	WORLDPROBE::GET_SHAPE_TEST_RESULT(ray, &hit, &endCoords, &surfaceNormal, &entityHit);

	if (hit)
		cachedSurfaceNormal = toV3f(surfaceNormal);
	else
		cachedSurfaceNormal = vehUpVector;

	//Vector3f latVector = rightVector;
	Vector3f latVector = getLatVector();
	//Vector3f latVector = useupRightVal >= .0f ? rightVector : leftVector;

	smoothSurfaceNormal = Vector3fLerpAngle(smoothSurfaceNormal, cachedSurfaceNormal, 0.9f * getDeltaTime());

	//Vector3f cachedSurfaceNormal = useRaycast ? smoothSurfaceNormal.cross(latVector).normalized() : cachedSurfaceNormal.normalized();
	Vector3f flinalSF = cachedSurfaceNormal.cross(latVector).normalized();

	return lookRotation(flinalSF);
}

void setupCurrentCamera() {
	if (currentCam == eCamType::DriverSeat1P) {
		ENTITY::SET_ENTITY_ALPHA(playerPed, 0, false);
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.05f);
		CAM::SET_CAM_FAR_CLIP(customCam, 740.0f);
		CAM::SET_CAM_FOV(customCam, 90.f);
		smoothRotSeat = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 0));
	}
	else if (currentCam == eCamType::Smooth3P) {
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.15f);
		CAM::SET_CAM_FAR_CLIP(customCam, 800.0f);
		CAM::SET_CAM_FOV(customCam, 75.f);
		viewLock = 0.f;
		mouseMoveCountdown = 0.5f;
	}

}

void setupCustomCamera() {
	customCam = CAM::CREATE_CAM_WITH_PARAMS("DEFAULT_SCRIPTED_CAMERA", vehPos.x(), vehPos.y(), vehPos.z(), vehRot.x(), vehRot.y(), vehRot.z(), fov3P, true, 2);
	CAM::SET_CAM_ACTIVE(customCam, true);
	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, true, false);

	camInitialized = true;

	setupCurrentCamera();
}

void updateCameraDriverSeat() {
	char *boneName = (isBike ? "seat_f" : "steeringwheel");

	Vector3f seatPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, boneName)));

	Vector3f camPos;
	if (isBike)
		camPos = seatPos + (vehUpVector * 0.4f) + (vehForwardVector * 0.32f); // bike
	else
		//camPos = seatPos + (vehUpVector * 0.350f) + (vehForwardVector  * -0.172f); // car
		//camPos = seatPos + (vehUpVector * 0.280f) + (vehForwardVector  * -0.152f); // car
		camPos = seatPos + (vehUpVector * (vehClass == eVehicleClass::VehicleClassSuper ? 0.320f : 0.350f)) + (vehForwardVector  * -0.190f); // car

	Vector3f pointAt;

	if (isBike)
		pointAt = camPos + vehForwardVector; // bike
	else
		//pointAt = camPos + vehForwardVector + (vehUpVector * 0.290f); // car
		pointAt = camPos + vehForwardVector + (vehUpVector * 0.328f); // car

	if (smoothIsMouseLooking > 0.001f) {
		pointAt = camPos + getGameplayCameraDirection();
	}

	if (smoothIsAiming > 0.00001f) {
		float currentFov = lerp(fov1P, 75.f, smoothIsAiming);
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	CAM::SET_CAM_COORD(customCam, camPos.x(), camPos.y(), camPos.z());

	if (isBike) {
		if (smoothIsMouseLooking > 0.001f) {
			CAM::POINT_CAM_AT_COORD(customCam, pointAt.x(), pointAt.y(), pointAt.z());
		}
		else
		{
			//Vector3f surfRot = getSurfaceNormalRotationBike();
			//Vector3f bikeRot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 0));

			//Vector3f rot = lerp(surfRot, bikeRot, smoothIsInAir);

			//CAM::STOP_CAM_POINTING(customCam);
			//CAM::SET_CAM_ROT(customCam, rot.x(), rot.y(), rot.z(), 0);
			////CAM::SET_CAM_ROT(customCam, bikeRot.x(), bikeRot.y(), bikeRot.z(), 0);
			CAM::STOP_CAM_POINTING(customCam);

			Vector3 rot = ENTITY::GET_ENTITY_ROTATION(veh, 0);
			CAM::SET_CAM_ROT(customCam, rot.x, rot.y * 0.5f, rot.z, 0);
		}
	}
	else
	{
		if (smoothIsMouseLooking > 0.001f) {
			CAM::POINT_CAM_AT_COORD(customCam, pointAt.x(), pointAt.y(), pointAt.z());
		}
		else
		{
			CAM::STOP_CAM_POINTING(customCam);

			Vector3f rot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 0));
			//smoothRotSeat = Vector3fLerpAngle(smoothRotSeat, rot, clamp01(30.f * SYSTEM::TIMESTEP()));
			//smoothRotSeat = Vector3fInertialDampAngle(smoothRotSeat, rot, 0.06f);
			smoothRotSeat = Vector3fLerpAngle(smoothRotSeat, rot, clamp01(30.f * getDeltaTime()));
			CAM::SET_CAM_ROT(customCam, smoothRotSeat.x(), smoothRotSeat.y(), smoothRotSeat.z(), 0);
		}
	}

	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, 1, 0);
}

void updateCameraSmooth3P() {
	Vector3f extraCamHeight = up * 0.11f;
	Vector3f posCenter = vehPos + (up * heightOffset3P);
	Vector3f vehForwardVector = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(veh));

	float rotSpeed = rotationSpeed3P;
	if (useVariableRotSpeed3P)
	{
		Vector3f forward = vehForwardVector.normalized();
		Vector3f velo = vehVelocity.normalized();

		float dot = vehForwardVector.dot(velo);

		if (dot < 0.f)
			velo = -velo;

		//Vector3f correctedVelocity = vehSpeed > 0.1f ? vehVelocity : vehForwardVector;
		float desiredRootSpeed = smoothStep(minRotSpeed3P, maxRotSpeed3P, clamp01(AngleBetweenVectors(forward, velo)));
		currentRotSpeed3P = smoothStep(currentRotSpeed3P, desiredRootSpeed, variableRotLerpSpeed * getDeltaTime());
	}

	Quaternionf vehQuat;

	if (useRoadSurfaceNormal) {
		vehQuat = getSurfaceNormalRotation();
	}
	else 
	{
		vehQuat = getEntityQuaternion(veh);
	}

	float speedFactor = clamp01((vehSpeed - 5.f) * 0.35f);

	if(speedFactor >= 0.00001f)
		velocityQuat3P = lookRotation(smoothVelocity);

	velocityQuat3P = slerp(vehQuat, velocityQuat3P, speedFactor);

	if (isBike && vehSpeed >= 3.f) 
	{
		//if (smoothQuat3P.dot(velocityQuat3P) < 0.f)
		//	velocityQuat3P = negateQuat(velocityQuat3P);

		smoothQuat3P = slerp(smoothQuat3P, velocityQuat3P, currentRotSpeed3P * getDeltaTime());
	}
	else
	{
		//if (smoothQuat3P.dot(vehQuat) < 0.f)
		//	smoothQuat3P = negateQuat(smoothQuat3P);
		smoothQuat3P = slerp(smoothQuat3P, vehQuat, currentRotSpeed3P * getDeltaTime());
	}

	//if (smoothQuat3P.dot(velocityQuat3P) < 0.f)
	//	velocityQuat3P = negateQuat(velocityQuat3P);

	Quaternionf finalQuat = slerp(smoothQuat3P, velocityQuat3P, smoothIsInAir);

	/*viewLock = clamp01(viewLock - (getDeltaTime() *10.f));
	viewLock = clamp01(viewLock + (vehSpeed *.05f));

	smoothViewLock = lerp(smoothViewLock, viewLock, clamp01(1.6f * getDeltaTime()));

	float vehAcceleration = getVehicleAcceleration();

	if (smoothViewLock >= 0.001f && smoothViewLock <= 0.99f && !isMouseLooking()) {
		Vector3f dirCustomCam = finalQuat * front;
		setGameplayCameraDirection(vehSpeed > 10.f ? dirCustomCam : vehForwardVector);
	}*/

	Quaternionf mouseLookRot = lookRotation(getGameplayCameraDirection());

	//finalQuat = slerp(finalQuat, mouseLookRot, max(smoothIsMouseLooking, 1.f - smoothViewLock));
	finalQuat = slerp(finalQuat, mouseLookRot, smoothIsMouseLooking);

	if (smoothIsAiming > 0.00001f) {
		float currentFov = lerp(fov3P, 60.f, smoothIsAiming);
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	float finalDistMult = 1.f;
	if (vehSpeed > 0.15f)
	{
		// TODO Optimize?
		finalDistMult = Vector3Angle(finalQuat * front, vehVelocity) /* / 180f */ * 0.0055555555555556f;
		finalDistMult = lerp(1.f, -0.45f, finalDistMult);
	}

	float factor = clamp01(vehSpeed / maxHighSpeed);
	float currentDistanceIncrement = lerp(0.f, maxHighSpeedDistanceIncrement, easeOutCubic(factor));

	factor = clamp01(vehAcceleration / (maxHighSpeed));

	smoothAccelerationFactor = lerp(smoothAccelerationFactor, factor, clamp01(35.f * getDeltaTime()));
	currentDistanceIncrement += lerp(0.f, accelerationCamDistanceMultiplier, easeOutCubic(smoothAccelerationFactor));

	if (isBike)
		currentDistanceIncrement *= 1.1f;

	Vector3f relativeLookDir = back;

	if (CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind)) {
		relativeLookDir = front;
		finalDistMult *= -1.f;
	}

	currentTowHeightIncrement = lerp(currentTowHeightIncrement, towHeightIncrement, 1.45f * getDeltaTime());
	currentTowLongitudeIncrement = lerp(currentTowLongitudeIncrement, towLongitudeIncrement, 1.75f * getDeltaTime());

	Vector3f V3CurrentTowHeightIncrement = up * currentTowHeightIncrement;

	Vector3f camPos = posCenter + extraCamHeight + V3CurrentTowHeightIncrement + (finalQuat * relativeLookDir * (longitudeOffset3P + (currentDistanceIncrement * finalDistMult) + currentTowLongitudeIncrement));
	setCamPos(customCam, camPos);

	//if (smoothIsMouseLooking > 0.001f) {
	//	Vector3f camPos = lerp(camPos, getGameplayCameraPos(), smoothIsMouseLooking);
	//	setCamPos(customCam, camPos);
	//}

	Vector3f finalPosCenter = posCenter + V3CurrentTowHeightIncrement;

	int ray = WORLDPROBE::_START_SHAPE_TEST_RAY(finalPosCenter.x(), finalPosCenter.y(), finalPosCenter.z(), camPos.x(), camPos.y(), camPos.z(), 1, veh, 7);

	Vector3 endCoords, surfaceNormal; 
	BOOL hit; 
	Entity entityHit = 0; 

	WORLDPROBE::GET_SHAPE_TEST_RESULT(ray, &hit, &endCoords, &surfaceNormal, &entityHit);

	if (hit) {
		setCamPos(customCam, toV3f(endCoords) + (finalQuat * front * 0.01f));
	}

	camPointAt(customCam, finalPosCenter);
}

void updateCustomCamera() 
{
	if (currentCam == eCamType::Smooth3P) {
		updateCameraSmooth3P();
	}
	else if(currentCam == eCamType::DriverSeat1P) {
		updateCameraDriverSeat();
	}
}

void haltCurrentCamera() {
	if (currentCam == eCamType::DriverSeat1P) {
		ENTITY::SET_ENTITY_ALPHA(playerPed, 255, false);
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.15f);
		CAM::SET_CAM_FAR_CLIP(customCam, 800.0f);
	}
}

void DisableCustomCamera()
{
	if (!camInitialized)
		return;

	CAM::SET_CAM_ACTIVE(customCam, false);
	CAM::DESTROY_CAM(customCam, true);
	CAM::RENDER_SCRIPT_CAMS(false, false, 3000, true, false);
	camInitialized = false;
	
	haltCurrentCamera();
}

float getVehicleLongitude(Vehicle vehicle) {

	Vector3f vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));
	Vector3f forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));

	float maxBackDistance = 0.f;
	float maxFrontDistance = 0.f;

	//Vector3f backBonePos;
	//Vector3f frontBonePos;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3f bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			float currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, forward);
			float currFrontDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, -forward);

			if (currBackDistance > maxBackDistance) {
				maxBackDistance = currBackDistance;
			}

			if (currFrontDistance > maxFrontDistance) {
				maxFrontDistance = currFrontDistance;
			}
		}
	}

	return maxBackDistance + maxFrontDistance;
}

float getVehicleHeight(Vehicle vehicle) {

	Vector3f rotation = toV3f(ENTITY::GET_ENTITY_ROTATION(vehicle, false));

	Vector3f rightVector = toV3f(getRightVector(rotation));
	Vector3f forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));
	Vector3f upVector = vehRightVector.cross(forward);

	Vector3f vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));

	float maxBackDistance = 0.f;
	float maxFrontDistance = 0.f;

	//Vector3f backBonePos;
	//Vector3f frontBonePos;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3f bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			float currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, upVector);
			float currFrontDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, -upVector);

			if (currBackDistance > maxBackDistance) {
				maxBackDistance = currBackDistance;
			}

			if (currFrontDistance > maxFrontDistance) {
				maxFrontDistance = currFrontDistance;
			}
		}
	}

	return maxBackDistance + maxFrontDistance;
}

float getTowedVehicleOrTrailerLongitude() {
	if (vehHasTowBone)
	{
		Vehicle towed = VEHICLE::GET_ENTITY_ATTACHED_TO_TOW_TRUCK(veh);

		if (towed != NULL) {
			if (towed == lastTowVehicle) {
				return lastTowVehicleLongitude;
			}
			else
			{
				towHeightIncrement = .75f;
				float longitude = getVehicleLongitude(towed) + 2.0f;
				lastTowVehicle = towed;
				lastTowVehicleLongitude = longitude;

				return longitude;
			}
		}
		else
		{
			towHeightIncrement = .0f;
			float longitude = 0.f;
			lastTowVehicle = NULL;
			lastTowVehicleLongitude = 0.f;

			return 0.f;
		}
	}
	else if (vehHasTrailerBone && VEHICLE::IS_VEHICLE_ATTACHED_TO_TRAILER(veh))
	{
		Vehicle trailer = NULL;
		VEHICLE::GET_VEHICLE_TRAILER_VEHICLE(veh, &trailer);

		if (trailer != NULL) {
			if (trailer == lastTrailer)
				return lastTrailerLongitude;
			else
			{
				towHeightIncrement = 1.45f;
				float longitude = getVehicleLongitude(trailer) + 1.85f;
				lastTrailer = trailer;
				lastTrailerLongitude = longitude;

				return longitude;
			}
		}
		else
		{
			towHeightIncrement = .0f;
			float longitude = 0.f;
			lastTrailer = NULL;
			lastTrailerLongitude = longitude;

			return longitude;
		}
	}
	else
	{
		towHeightIncrement = .0f;
		float longitude = 0.f;
		lastTrailer = NULL;
		lastTrailerLongitude = longitude;

		return longitude;
	}
}

void onLowTimeUpdate() {
	towLongitudeIncrement = getTowedVehicleOrTrailerLongitude();
}

void updateTimers() {
	if (lowUpdateTimerCurrentTime > 0.f)
		lowUpdateTimerCurrentTime -= getDeltaTime();
	else
	{
		lowUpdateTimerCurrentTime = lowUpdateTimerInterval;
		onLowTimeUpdate();
	}
}

void update()
{
	if (IsKeyJustUp(str2key("F9"))) {
		showDebug = !showDebug;
	}

	if (IsKeyJustUp(str2key("1"))) {
		customCamEnabled = !customCamEnabled;
	}

	if (showDebug) {
		showText(0.01f, 0.200f, 0.4, ("MouseLookCountDown: " + std::to_string(mouseMoveCountdown)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.250f, 0.4, ("height: " + std::to_string(heightOffset3P)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.300f, 0.4, ("long: " + std::to_string(longitudeOffset3P)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.350f, 0.4, ("currentRotSpeed3P: " + std::to_string(currentRotSpeed3P)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.400f, 0.4, ("vehGear: " + std::to_string(vehGear)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.450f, 0.4, ("vehUpright: " + std::to_string(ENTITY::GET_ENTITY_UPRIGHT_VALUE(veh))).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.500f, 0.4, ("vehRotX: " + std::to_string(vehRot.x())).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.550f, 0.4, ("vehRotY: " + std::to_string(vehRot.y())).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.600f, 0.4, ("vehRotZ: " + std::to_string(vehRot.z())).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.650f, 0.4, ("viewLock: " + std::to_string(viewLock)).c_str(), 4, solidWhite, true);
	}

	Player player = PLAYER::PLAYER_ID();
	playerPed = PLAYER::PLAYER_PED_ID();

	// check if player ped exists and control is on (e.g. not in a cutscene)
	if (!ENTITY::DOES_ENTITY_EXIST(playerPed) || !PLAYER::IS_PLAYER_CONTROL_ON(player))
	{
		DisableCustomCamera();
		return;
	}

	// check for player ped death and player arrest
	if (ENTITY::IS_ENTITY_DEAD(playerPed) || PLAYER::IS_PLAYER_BEING_ARRESTED(player, TRUE))
	{
		DisableCustomCamera();
		return;
	}

	updateMouseState();
	smoothIsMouseLooking = lerp(smoothIsMouseLooking, isMouseLooking() ? 1.f : 0.f, 8.f * SYSTEM::TIMESTEP());

	isAiming = PLAYER::IS_PLAYER_FREE_AIMING(player);
	smoothIsAiming = lerp(smoothIsAiming, isAiming ? 1.f : 0.f, 8.f * SYSTEM::TIMESTEP());

	// check if player is in a vehicle
	if (PED::IS_PED_IN_ANY_VEHICLE(playerPed, FALSE))
	{
		Vehicle newVeh = PED::GET_VEHICLE_PED_IS_USING(playerPed);

		if (newVeh != veh) {
			veh = newVeh;
			updateVehicleVars();
			updateVehicleProperties();
		}
		if (isSuitableForCam && customCamEnabled) {

			CONTROLS::DISABLE_CONTROL_ACTION(0, eControl::ControlNextCamera, true);

			updateVehicleVars();

			if (!camInitialized) {
				setupCustomCamera();
				CAM::SET_FOLLOW_VEHICLE_CAM_VIEW_MODE(2);
			}

			if (CONTROLS::IS_DISABLED_CONTROL_JUST_PRESSED(2, eControl::ControlNextCamera)) {
				haltCurrentCamera();
				nextCam();
				setupCurrentCamera();
			}

			updateTimers();
			updateCustomCamera();
		}
		else
		{
			ResetMouseLook();
			DisableCustomCamera();
			return;
		}
	}
	else
	{
		veh = -1;
		ResetMouseLook();
		DisableCustomCamera();
		return;
	}
}

void main()
{
	firstInit();

	while (true)
	{
		update();
		WAIT(0);
	}
}

void ScriptMain()
{
	main();
}
