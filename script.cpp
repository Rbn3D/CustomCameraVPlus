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

using namespace NativeMenu;

HWND hWnd;

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
Vector3f vehRelativeSpeedVector;
Vector3f vehAngularVelocity;

float vehAcceleration = 0.f;
float smoothAccelerationFactor = 0.f;
float prevVehAcceleration = 0.f;

eVehicleClass vehClass;
bool vehHasTowBone = false;
bool vehHasTrailerBone = false;

Camera customCam = NULL;
//float fov3P = 85.f;
//float fov1P = 90.f;
float fov3P = 77.5f;
float fov1P = 77.5F;
float fov1PAiming = 60.f;
float fov3PAiming = 60.f;
float distanceOffset = 0.f;
const float PI = 3.1415926535897932f;
int lastVehHash = -1;
bool isBike = false;

bool isInVehicle = false;

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
float RotSpeedFast3P = 4.7f;
float RotSpeedSlow3P = 3.8f;
float maxRotSpeedAngle3P = 90.0f;

//float variableRotLerpSpeed = 7.75f;

float currentRotSpeed3P = 2.0f;

float smoothSpeedFactor = 0.f;

float useRoadSurfaceNormal = false;
Vector3f cachedSurfaceNormal;
Vector3f smoothSurfaceNormal;

Vector3f smoothVelocity = Vector3f();
Vector3f ultraSmoothVelocity = Vector3f();
Quaternionf velocityQuat3P = Quaternionf();
Quaternionf smoothQuat3P = Quaternionf();
Quaternionf ultraSmoothVelocity3P = Quaternionf();
Vector3f smoothRotSeat = Vector3f();
Quaternionf smoothQuatSeat = Quaternionf();
float smoothIsInAir = 0.f;
float smoothIsInAirNfs = 0.f;
float maxHighSpeed = 130.f;
float maxHighSpeedDistanceIncrement = 1.45f;
float accelerationCamDistanceMultiplier = 1.45f;
Vector3f playerVehOffset;

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

float smoothRadarAngle = 0.f;
float extraAngleCamHeight = 0.f;
float relAngle3p = 0.f;

const float DEG_TO_RAD = 0.0174532925f;
const float RADIAN_TO_DEG = 57.29577951f;

float LookLeftAngle1p = 75.0f;
float LookRightAngle1p = 80.0f;

float LookLeftAngle3p = 90.0f;
float LookRightAngle3p = 90.0f;

float semiDelayedVehSpeed = 0.f;
float delayedVehSpeed = 0.f;

Vector3f prevCamPos;

bool isLookingBack = false;

float timerResetLook = 0.f;
Quaternionf lookQuat;

float mouseDeltaX = 0.f;
float mouseDeltaY = 0.f;


// look left = -1.f; lookRight = 1.f; Center = 0.f (For smooth transitions)
float RelativeLookFactor = 0.f;

bool readInputFromMt = true;
float deccelerationSmooth = 0.f;

char * reloadKey = "F10";
char * toggleModKey = "1";
char * lookLeftKey = "B";
char * lookRightKey = "N";

bool hasInputThisFrame = false;
bool hasMouseInputThisFrame = false;

bool AreSameFloat(float a, float b)
{
	return fabs(a - b) < FLT_EPSILON;
}

float getDeltaTime() {
	//return SYSTEM::TIMESTEP();
	return GAMEPLAY::GET_FRAME_TIME();
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

float easeInCirq(float t) {
	return -1 * (sqrt(1 - t * t) - 1);
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

Vector3f QuatToEuler(Quaternionf q) 
{
	float r11 = -2 * (q.x() * q.y() - q.w() * q.z());
	float r12 = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
	float r21 = 2 * (q.y() * q.z() + q.w() * q.x());
	float r31 = -2 * (q.x() * q.z() - q.w() * q.y());
	float r32 = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();

	float ax = asinf(r21);
	float ay = atan2f(r31, r32);
	float az = atan2f(r11, r12);

	const float f = 360.0f / 2.0f / 3.1415926535897f;
	ax *= f;
	ay *= f;
	az *= f;

	Vector3f ret = Vector3f(ax, ay, az);

	return ret;
}

void SET_CAM_QUATERNION(Camera c, Quaternionf q)
{
	//Matrix3f rot = q.toRotationMatrix();

	//float ax = asinf(rot(1,0));
	//float ay = atan2f(rot(2,0), rot(2,1));
	//float az = atan2f(rot(0,0), rot(0,1));

	Vector3f ret = QuatToEuler(q);

	CAM::SET_CAM_ROT(c, ret.x(), ret.y(), ret.z(), 2);
}

Vector3f toV3f(const Vector3 &vec) {
	return Vector3f(vec.x, vec.y, vec.z);
}

Vector3 toV3(Vector3f vec) {
	Vector3 ret;
	ret.x = vec.x();
	ret.y = vec.y();
	ret.z = vec.z();

	return ret;
}


float RadToDeg(const float &rad)
{
	return float(rad / M_PI * 180.0);
}

float DegToRad(const float &deg)
{
	return float(deg * M_PI / 180.0);
}

Quaternionf QuatEuler(Vector3f &euler)
{
	Vector3f rotVec = DegToRad(1.0f) * euler;

	Matrix3f xRot = AngleAxisf(rotVec.x(), Vector3f(1.0f, 0.0f, 0.0f)).matrix();
	Matrix3f yRot = AngleAxisf(rotVec.y(), Vector3f(0.0f, 1.0f, 0.0f)).matrix();
	Matrix3f zRot = AngleAxisf(rotVec.z(), Vector3f(0.0f, 0.0f, 1.0f)).matrix();

	Matrix3f rot = zRot * yRot * xRot;

	return Quaternionf(rot);
}

Quaternionf GET_CAM_QUATERNION(Camera c)
{
	Vector3f camVec = toV3f(CAM::GET_CAM_ROT(c, 0));
	return QuatEuler(camVec);
}

Vector2i getMouseCoords() {
	int mX = CONTROLS::GET_CONTROL_VALUE(0, 239) - 127 / 127.f * 1280;
	int mY = CONTROLS::GET_CONTROL_VALUE(0, 240) - 127 / 127.f * 720;

	return Vector2i(mX, mY);
}

void updateMouseState() 
{
	float x = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight);
	float y = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown);

	if (
		(fabs(x) > 0.005f)
		||
		(fabs(y) > 0.005f)
	   )
	{
		hasInputThisFrame = true;
	}
	else
		hasInputThisFrame = false;
}

void setGameplayCamRelativeRotation(float heading) {
	CAM::SET_GAMEPLAY_CAM_RELATIVE_HEADING(relAngle3p);
	CAM::SET_GAMEPLAY_CAM_RELATIVE_PITCH(0.f, 1.f);

	//WAIT(0);
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

void ShowNotification(const char* msg) 
{
	UI::_SET_NOTIFICATION_TEXT_ENTRY("CELL_EMAIL_BCON");

	std::string strMsg(msg);
	const int maxStringLength = 99;

	for (int i = 0; i < strlen(msg); i += maxStringLength)
	{
		UI::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME((char*)strMsg.substr(i, min(maxStringLength, strlen(msg) - i)).c_str());
	}

	UI::_DRAW_NOTIFICATION(false, true);
}

void ReadSettings(bool notify) 
{
	CSimpleIniA ini;
	//ini.SetUnicode();
	SI_Error res = ini.LoadFile("CustomCameraVPlus.ini");

	if (res == SI_Error::SI_OK) {
		const char* dist3rdPerson = ini.GetValue("3rdPersonView", "distanceOffset", "0.0");

		const char* fov3rdPerson = ini.GetValue("3rdPersonView", "fov", "77.5");
		const char* fov1stPerson = ini.GetValue("1stPersonView", "fov", "77.5");

		const char* c3rdPersonLookLeft = ini.GetValue("3rdPersonView", "lookLeftAngle", "90.0");
		const char* c3rdPersonLookRight = ini.GetValue("3rdPersonView", "lookRightAngle", "90.0");

		const char* c1stPersonLookLeft = ini.GetValue("1stPersonView", "lookLeftAngle", "75.0");
		const char* c1stPersonLookRight = ini.GetValue("1stPersonView", "lookRightAngle", "80.0");

		const char* cReadFromMt = ini.GetValue("general", "GetInputFromGearsAsi", "1");

		reloadKey =    strdup(ini.GetValue("keyMappings", "reloadSettingsKey", "F10"));
		toggleModKey = strdup(ini.GetValue("keyMappings", "toggleModKey", "1"));
		lookLeftKey =  strdup(ini.GetValue("keyMappings", "lookLeftKey", "B"));
		lookRightKey = strdup(ini.GetValue("keyMappings", "lookRightKey", "N"));

		distanceOffset = std::stof(dist3rdPerson);

		fov3P = std::stof(fov3rdPerson);
		fov1P = std::stof(fov1stPerson);

		LookLeftAngle3p = std::stof(c3rdPersonLookLeft);
		LookRightAngle3p = std::stof(c3rdPersonLookRight);

		LookLeftAngle1p = std::stof(c1stPersonLookLeft);
		LookRightAngle1p = std::stof(c1stPersonLookRight);

		readInputFromMt = std::stoi(cReadFromMt) > 0;

		if (notify) {
			ShowNotification("CCVPlus: Settings reloaded");
			updateVehicleProperties();
			setupCurrentCamera();
		}
			
	}
	else
		ShowNotification("CCVPlus: Cannot load settings! Missing ini file?");
}


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

void showText(int index, const char* text) {
	showText(0.1f, 0.03f * (float)index, 0.3f, text, 0, solidWhite, true);
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

	hWnd = FindWindow(NULL, "Grand Theft Auto V");

	enum Button
	{
		ButtonConfirm
	};

	ReadSettings(false);
}

Vector3f getGameplayCameraDirection() {
	const auto data = reinterpret_cast<const float *>(gamePlayCameraAddr + 0x200);
	return Vector3f(data[0], data[1], data[2]);
}

Vector3f getGameplayCameraRightVector() {
	const auto data = reinterpret_cast<const float *>(gamePlayCameraAddr + 0x1F0);
	return Vector3f(data[0], data[1], data[2]);
}

void setGameplayCameraDirection(Vector3f dir) {
	const auto address = (Vector3*)gamePlayCameraAddr + 0x200;

	Vector3 nativeDir = toV3(dir);

	*address = nativeDir;
}

Vector3f getGameplayCameraPos() {
	//Vector3 camPos = CAM::GET_GAMEPLAY_CAM_COORD();

	//return Vector3f(camPos.x, camPos.y, camPos.z);
	const auto data = reinterpret_cast<const float *>(gamePlayCameraAddr + 0x220);
	return Vector3f(data[0], data[1], data[2]);
}

Vector3f getCameraForwardVector(Camera cam) {
	Vector3f rotation = toV3f(CAM::GET_CAM_ROT(cam, 2));

	double num1 = (double)rotation.x() / (180.0f / PI);
	double num2 = (double)rotation.z() / (180.0f / PI);
	double num3 = abs(cos(num1));
	return Vector3f((float)-(sin(num2) * num3), (float)(cos(num2) * num3), (float)sin(num1));
}

Vector3f getCameraRightVector(Camera cam) {
	Vector3f rotation = toV3f(CAM::GET_CAM_ROT(cam, 2));

	return toV3f(getRightVector(rotation));
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

float MAX_ANGLE = 360.f;

float mod(float a, float b)
{
	return fmod((fmod(a,b) + b), b);
}

float NormalizeAngle(float angle) {
	//while (angle < 0)
	//	angle += MAX_ANGLE;
	//while (angle >= MAX_ANGLE)
	//	angle -= MAX_ANGLE;
	//return angle;

	return mod(angle, MAX_ANGLE);
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

float lerpAngle360(float start, float end, float amount)
{
	float difference = abs(end - start);
	if (difference > 180.f)
	{
		// We need to add on to one of the values.
		if (end > start)
		{
			// We'll add it on to start...
			start += 360.f;
		}
		else
		{
			// Add it on to end.
			end += 360.f;
		}
	}

	// Interpolate it.
	float value = (start + ((end - start) * amount));

	// Wrap it..
	float rangeZero = 360.f;

	if (value >= 0 && value <= 360)
		return value;

	return fmod(value, rangeZero);
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

Quaternionf slerp(Quaternionf& q1, Quaternionf& q2, float time)
{
	return q1.slerp(time, q2);
}

Quaternionf nlerp(Quaternionf& q1, Quaternionf& q2, float time)
{
	return QuatEuler(lerp(QuatToEuler(q1), QuatToEuler(q2), time));
}

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

	return lookM.eulerAngles(2, 1, 0);
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

bool vehHasBone(char *boneName) {
	return (ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, boneName) != -1);
}

void updateVehicleProperties() 
{
	vehClass = (eVehicleClass)VEHICLE::GET_VEHICLE_CLASS(veh);

	Vector3f skelPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "seat_dside_f")));
	Vector3f wheelPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "steeringwheel")));
	Vector3f windscreenPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "windscreen")));

	float playerHeadAltitude;
	float playerHeadDistance;

	if (vehClass == eVehicleClass::VehicleClassCoupes || vehClass == eVehicleClass::VehicleClassSports || vehClass == eVehicleClass::VehicleClassSportsClassics || vehClass == eVehicleClass::VehicleClassMuscle)
	{
		// sports - coupes - muscles
		playerHeadAltitude = 0.65f;
		playerHeadDistance = -0.53f;
	}
	else if (vehClass == eVehicleClass::VehicleClassSuper)
	{
		// super sport
		playerHeadAltitude = 0.615f;
		playerHeadDistance = -0.58f;

	}
	else
	{
		// anything else
		playerHeadAltitude = 0.67f;
		playerHeadDistance = -0.53f;
	}

	skelPos += vehUpVector * playerHeadAltitude;
	skelPos += vehRightVector * distanceOnAxisNoAbs(skelPos, wheelPos, vehRightVector);
	skelPos += vehForwardVector * (distanceOnAxisNoAbs(skelPos, windscreenPos, vehForwardVector) + playerHeadDistance);

	float distSteeringWheel = distanceOnAxisNoAbs(skelPos, wheelPos, vehForwardVector);
	float minDistSteeringWheel = 0.285f;

	if (distSteeringWheel < minDistSteeringWheel)
		skelPos += vehForwardVector * (distSteeringWheel - minDistSteeringWheel);


	playerVehOffset = toV3f(ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(veh, skelPos.x(), skelPos.y(), skelPos.z()));

	isBike = vehClass == eVehicleClass::VehicleClassCycles || vehClass == eVehicleClass::VehicleClassMotorcycles;
	isSuitableForCam = vehClass != eVehicleClass::VehicleClassTrains && vehClass != eVehicleClass::VehicleClassPlanes && vehClass != eVehicleClass::VehicleClassHelicopters && vehClass != eVehicleClass::VehicleClassBoats;

	longitudeOffset3P = getVehicleLongitudeFromCenterBack(veh) + 0.7f;

	heightOffset3P = clamp(getVehicleHeightFromCenterUp(veh) + 0.5f, 0.f, 2.0f);
	//ShowNotification(std::to_string(heightOffset3P).c_str());

	if (heightOffset3P > 1.75f)
	{
		extraAngleCamHeight = clamp(lerp(0.1f, 2.0f, unlerp(1.75f, 2.00f, heightOffset3P)), 0.f, 3.0f);
		heightOffset3P += (extraAngleCamHeight * 0.5f);
		longitudeOffset3P += extraAngleCamHeight;

		extraAngleCamHeight = clamp(extraAngleCamHeight, 0.f, 2.25f);
	}
	else
		extraAngleCamHeight = 0.0f;

	if (isBike) {
		longitudeOffset3P += 1.f;
		heightOffset3P = 1.38f;
	}

	longitudeOffset3P += 1.45f + distanceOffset;

	//ShowNotification(std::to_string(longitudeOffset3P).c_str());

	vehHasTowBone = vehHasBone("tow_arm");
	vehHasTrailerBone = vehHasBone("attach_female");
}

void updateVehicleVars() 
{
	vehPos = toV3f(ENTITY::GET_ENTITY_COORDS(veh, true));
	vehRot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
	vehVelocity = toV3f(ENTITY::GET_ENTITY_VELOCITY(veh));
	vehForwardVector = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(veh));
	vehSpeed = ENTITY::GET_ENTITY_SPEED(veh);
	smoothVelocity = lerp(smoothVelocity, vehSpeed > 2.f ? vehVelocity : vehForwardVector, 10.f * getDeltaTime());
	ultraSmoothVelocity = lerp(ultraSmoothVelocity, vehSpeed > 2.f ? vehVelocity : vehForwardVector, 3.f * getDeltaTime());

	if ((ENTITY::IS_ENTITY_IN_AIR(veh) || (ENTITY::GET_ENTITY_UPRIGHT_VALUE(veh) < 0.6f)) && smoothIsInAir < 0.001f)
	{
		smoothVelocity = getCameraForwardVector(customCam);
		velocityQuat3P = lookRotation(smoothVelocity);
	}

	smoothIsInAir = lerp(smoothIsInAir, (ENTITY::IS_ENTITY_IN_AIR(veh) || ENTITY::IS_ENTITY_UPSIDEDOWN(veh)) ? 1.f : 0.f, 2.f * getDeltaTime());
	smoothIsInAirNfs = lerp(smoothIsInAirNfs, (ENTITY::IS_ENTITY_IN_AIR(veh)) ? 1.f : 0.f, 0.75f * getDeltaTime());
	vehRightVector = toV3f(getRightVector(vehRot));
	vehUpVector = vehRightVector.cross(vehForwardVector);
	vehAcceleration = getVehicleAcceleration();
	vehRelativeSpeedVector = toV3f(ENTITY::GET_ENTITY_SPEED_VECTOR(veh, true));
	vehAngularVelocity = toV3f(ENTITY::GET_ENTITY_ROTATION_VELOCITY(veh));
}

void setupCurrentCamera() {
	if (currentCam == eCamType::DriverSeat1P) {
		ENTITY::SET_ENTITY_ALPHA(playerPed, 0, false);
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.05f);
		CAM::SET_CAM_FAR_CLIP(customCam, 740.0f);
		CAM::SET_CAM_FOV(customCam, fov1P);
		smoothRotSeat = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		smoothQuatSeat = getEntityQuaternion(veh);
		relAngle3p = 0.f;
	}
	else if (currentCam == eCamType::Smooth3P) {
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.15f);
		CAM::SET_CAM_FAR_CLIP(customCam, 800.0f);
		CAM::SET_CAM_FOV(customCam, fov3P);
		viewLock = 0.f;
		if (vehSpeed < 1.f) {
			mouseMoveCountdown = 0.5f;
			smoothIsMouseLooking = 1.0f;
		}
		else
		{
			mouseMoveCountdown = 0.f;
			smoothIsMouseLooking = 0.f;
		}
		smoothQuat3P = getEntityQuaternion(veh);
	}

	CAM::SET_FOLLOW_VEHICLE_CAM_VIEW_MODE(1);
}

void setupCustomCamera() {
	customCam = CAM::CREATE_CAM_WITH_PARAMS("DEFAULT_SCRIPTED_CAMERA", vehPos.x(), vehPos.y(), vehPos.z(), vehRot.x(), vehRot.y(), vehRot.z(), fov3P, true, 2);
	CAM::SET_CAM_ACTIVE(customCam, true);
	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, true, false);

	smoothRadarAngle = mathRepeat(CAM::GET_GAMEPLAY_CAM_ROT(2).z, 360.f);

	camInitialized = true;
	isInVehicle = true;

	CAM::SET_CINEMATIC_MODE_ACTIVE(false);

	setupCurrentCamera();
}

void updateCameraDriverSeat() {

	if (CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind) || isLookingBack)
	{
		lookBehind1p();

		CAM::RENDER_SCRIPT_CAMS(true, false, 3000, 1, 0);
		smoothRotSeat = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		smoothQuatSeat = slerp(smoothQuatSeat, getEntityQuaternion(veh), clamp01(30.f * getDeltaTime()));

		return;
	}

	Vector3f seatPos;
	if (!isBike) 
	{
		seatPos = toV3f(ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(veh, playerVehOffset.x(), playerVehOffset.y(), playerVehOffset.z()));
		//seatPos = getGameplayCameraPos();
	}
	else
		seatPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "seat_f")));
		

	Vector3f camPos;
	if (isBike)
		camPos = seatPos + (vehUpVector * 0.4f) + (vehForwardVector * 0.32f); // bike
	else
		camPos = seatPos; // car; // car

	if (smoothIsAiming > 0.00001f) {
		float currentFov = lerp(fov1P, fov1PAiming, smoothIsAiming);
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	CAM::SET_CAM_COORD(customCam, camPos.x(), camPos.y(), camPos.z());

	float btnLookingFactor = abs(RelativeLookFactor);

	if (isBike) {
		CAM::STOP_CAM_POINTING(customCam);
		Vector3f rot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		rot[1] = rot.y() * 0.5f; // rot.y = rot.y * .5f

		//smoothRotSeat = Vector3fLerpAngle(smoothRotSeat, rot, clamp01(30.f * getDeltaTime()));
		smoothQuatSeat = getEntityQuaternion(veh);

		float leftRightAngle = RelativeLookFactor < 0 ?
			lerp(0.f, -LookLeftAngle1p, -RelativeLookFactor)
			:
			lerp(0.f, LookRightAngle1p, RelativeLookFactor)
			;

		float leftRightRad = leftRightAngle * DEG_TO_RAD;

		float roll = 0.f, pitch = 0.f, yaw = leftRightRad;
		Quaternionf qLookLeftRight;
		qLookLeftRight = AngleAxisf(roll, Vector3f::UnitX())
			* AngleAxisf(pitch, Vector3f::UnitY())
			* AngleAxisf(yaw, Vector3f::UnitZ());

		Quaternionf finalQ = smoothQuatSeat * qLookLeftRight;

		if (isAiming || hasInputThisFrame)
		{
			if (timerResetLook < 0.00001f)
			{
				lookQuat = finalQ;
			}
			timerResetLook = 2.f;

			float mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -5.f;
			float my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * (LastInputMethodWasMouseAndKeyboard ? -5.f : 5.f);

			if (!LastInputMethodWasMouseAndKeyboard())
			{
				mx *= 0.6f;
				my *= 0.6f;
			}

			Vector3f vecLook = Vector3f(my, 0.f, mx);

			Quaternionf result = lookQuat * QuatEuler(vecLook);
			Vector3f resultEuler = QuatToEuler(result);

			float rx = clamp(resultEuler[0], -62.f, 40.f);

			lookQuat = QuatEuler(Vector3f(rx, 0.f, resultEuler[2]));
		}

		timerResetLook = clamp(timerResetLook - getDeltaTime(), 0.f, 2.f);

		finalQ = slerp(finalQ, lookQuat, clamp01(timerResetLook));

		SET_CAM_QUATERNION(customCam, finalQ);
	}
	else
	{
		CAM::STOP_CAM_POINTING(customCam);

		Vector3f rot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		smoothQuatSeat = getEntityQuaternion(veh);


			float leftRightAngle = RelativeLookFactor < 0 ?
					lerp(0.f, -LookLeftAngle1p, -RelativeLookFactor)
				:
					lerp(0.f, LookRightAngle1p, RelativeLookFactor)
				;

			float leftRightRad = leftRightAngle * DEG_TO_RAD;

			float roll = 0.f, pitch = 0.f, yaw = leftRightRad;
			Quaternionf qLookLeftRight;
			qLookLeftRight = AngleAxisf(roll, Vector3f::UnitX())
				* AngleAxisf(pitch, Vector3f::UnitY())
				* AngleAxisf(yaw, Vector3f::UnitZ());

			Quaternionf finalQ = smoothQuatSeat * qLookLeftRight;

			if (isAiming || hasInputThisFrame)
			{
				if (timerResetLook < 0.00001f)
				{
					lookQuat = finalQ;
				}
				timerResetLook = 2.f;

				float mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -5.f;
				float my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * (LastInputMethodWasMouseAndKeyboard ? -5.f : 5.f);

				if (!LastInputMethodWasMouseAndKeyboard())
				{
					mx *= 0.6f;
					my *= 0.6f;
				}

				Vector3f vecLook = Vector3f(my, 0.f, mx);

				Quaternionf result = lookQuat * QuatEuler(vecLook);
				Vector3f resultEuler = QuatToEuler(result);

				float rx = clamp(resultEuler[0], -62.f, 40.f);

				lookQuat = QuatEuler(Vector3f(rx, 0.f, resultEuler[2]));
			}

			timerResetLook = clamp(timerResetLook - getDeltaTime(), 0.f, 2.f);

			finalQ = slerp(finalQ, lookQuat, clamp01(timerResetLook));

			SET_CAM_QUATERNION(customCam, finalQ);
	}

	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, 1, 0);
}

void ProccessLookLeftRightOrBackInput()
{
	const float rotSpeed = 9.f;

	bool evalLeft = IsKeyDown(str2key(lookLeftKey)) || (readInputFromMt && GetDecoratorBool("mt_looking_left"));
	bool evalRight = IsKeyDown(str2key(lookRightKey)) || (readInputFromMt && GetDecoratorBool("mt_looking_right"));

	isLookingBack = CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlVehicleLookBehind) || (readInputFromMt && GetDecoratorBool("mt_looking_back")) || (evalLeft && evalRight);

	if (evalLeft && !evalRight) {
		RelativeLookFactor += rotSpeed * getDeltaTime();
	}
	else if (evalRight) {
		RelativeLookFactor -= rotSpeed * getDeltaTime();
	}
	else
	{
		if (RelativeLookFactor > 0.f)
			RelativeLookFactor = clamp01(RelativeLookFactor - rotSpeed * getDeltaTime());
		else
			RelativeLookFactor = clamp(RelativeLookFactor + rotSpeed * getDeltaTime(), -1.f, 0.f);
	}

	RelativeLookFactor = clamp(RelativeLookFactor, -1.f, 1.f);
}

BOOL GetDecoratorBool(char * decoratorKey)
{
	return DECORATOR::DECOR_IS_REGISTERED_AS_TYPE(decoratorKey, eDecorType::DECOR_TYPE_BOOL) && DECORATOR::DECOR_EXIST_ON(veh, decoratorKey) && DECORATOR::DECOR_GET_BOOL(veh, (char *)decoratorKey);
}

void lookBehind1p()
{
	Vector3f camPos;

	if (isBike)
		camPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "seat_f"))) + (vehUpVector * 0.3f); // bike
	else {
		char *boneName;
		Vector3f offset;

		bool hasBone = false;
		if (vehHasBone("windscreen_r"))
		{
			boneName = "windscreen_r";
			offset = (vehUpVector * 0.08f) + (-vehForwardVector * 0.08f);
			hasBone = true;

			goto setBonePos;
		}

		if (vehHasBone("bumper_r"))
		{
			boneName = "bumper_r";
			offset = (vehUpVector * 0.18f) + (-vehForwardVector * 0.08f);
			hasBone = true;

			goto setBonePos;
		}

		if (vehHasBone("brakelight_m"))
		{
			boneName = "brakelight_m";
			offset = (vehUpVector * 0.08f) + (-vehForwardVector * 0.08f);
			hasBone = true;

			goto setBonePos;
		}

		if (vehHasBone("boot"))
		{
			boneName = "boot";
			offset = vehUpVector * 0.28f;
			hasBone = true;

			goto setBonePos;
		}

		 setBonePos:

		if (hasBone) {

			if (VEHICLE::GET_VEHICLE_MOD(veh, 0) >= 0) // Has spoiler?
			{
				offset += vehUpVector * 0.30f;
			}

			camPos = GetBonePos(veh, boneName) + offset;
		}
			
		else
		{
			char* boneName1 = "brakelight_l";
			char* boneName2 = "brakelight_l";

			Vector3f pos1 = GetBonePos(veh, boneName1);
			Vector3f pos2 = GetBonePos(veh, boneName2);

			Vector3f posCenter = lerp(pos1, pos2, 0.5f);
			offset = (vehUpVector * 0.08f) + (-vehForwardVector * 0.08f);

			camPos = posCenter + offset; // car
		}
	}

	Quaternionf quat = getEntityQuaternion(veh);

	float roll = 0.f, pitch = 0.f, yaw = DegToRad(180.f);
	Quaternionf invert180;
	invert180 = AngleAxisf(roll, Vector3f::UnitX())
		* AngleAxisf(pitch, Vector3f::UnitY())
		* AngleAxisf(yaw, Vector3f::UnitZ());

	setCamPos(customCam, camPos);
	CAM::STOP_CAM_POINTING(customCam);
	SET_CAM_QUATERNION(customCam, quat * invert180);
}

Vector3f GetBonePos(Entity entity, char * boneName)
{
	return toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(entity, boneName)));
}

void updateCam3pSmoothAlgorithm()
{
	Vector3f extraCamHeight = up * (0.14f + extraAngleCamHeight);
	Vector3f posCenter = vehPos + (up * heightOffset3P);

	float rotSpeed = rotationSpeed3P;
	if (useVariableRotSpeed3P)
	{
		float lerpFactor = min(1.f, easeOutCubic(vehSpeed / (maxHighSpeed * 0.5f)));
		float rotFactor = abs(vehRelativeSpeedVector.x()) * 2.75f;

		lerpFactor -= rotFactor;

		currentRotSpeed3P = lerp(max(0.25f, RotSpeedSlow3P /*- (rotFactor * .0032f)*/), RotSpeedFast3P, easeInCubic(clamp01(lerpFactor)));
	}

	Quaternionf vehQuat = getEntityQuaternion(veh);

	float speedFactor = clamp01((vehSpeed - 5.f) * 0.35f);

	if (speedFactor >= 0.00001f)
		velocityQuat3P = /*lookRotation(smoothVelocity);*/ slerp(velocityQuat3P, lookRotation(smoothVelocity), 4.f * getDeltaTime());
	else if (vehSpeed >= 0.00001f) // fix instant cam rotation when rollovers/sudden rotation changes and speeds goes below 5.0f (if speedFactor < 0.00001f)
		velocityQuat3P = slerp(velocityQuat3P, lookRotation(smoothVelocity), 0.2f * getDeltaTime());

	smoothSpeedFactor = lerp(smoothSpeedFactor, speedFactor, clamp(6.f * getDeltaTime(), 0.f, 0.1f));

	velocityQuat3P = slerp(vehQuat, velocityQuat3P, smoothSpeedFactor);

	if (isBike && vehSpeed >= 3.f)
		smoothQuat3P = slerp(smoothQuat3P, velocityQuat3P, currentRotSpeed3P * getDeltaTime());
	else
		smoothQuat3P = slerp(smoothQuat3P, vehQuat, currentRotSpeed3P * getDeltaTime());

	Quaternionf finalQuat = slerp(smoothQuat3P, velocityQuat3P, smoothIsInAir);

	Quaternionf mouseLookRot = lookRotation(getGameplayCameraDirection());

	bool lookBehind = false;
	if (CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind) || isLookingBack)
		lookBehind = true;

	float leftRightAngle = 0.f;
	if (!lookBehind) {
		leftRightAngle = RelativeLookFactor < 0 ?
			lerp(0.f, -LookLeftAngle3p, -RelativeLookFactor)
			:
			lerp(0.f, LookRightAngle3p, RelativeLookFactor)
			;
	}

	float leftRightRad = leftRightAngle * DEG_TO_RAD;

	float roll = 0.f, pitch = 0.f, yaw = leftRightRad;
	Quaternionf qLookLeftRight;
	qLookLeftRight = AngleAxisf(roll, Vector3f::UnitX())
		* AngleAxisf(pitch, Vector3f::UnitY())
		* AngleAxisf(yaw, Vector3f::UnitZ());

	float btnLookingFactor = (abs(RelativeLookFactor));

	if (lookBehind)
		btnLookingFactor = 1.f;

	//finalQuat = slerp(finalQuat, mouseLookRot, max(smoothIsMouseLooking, 1.f - smoothViewLock));
	finalQuat = slerp(finalQuat * qLookLeftRight, mouseLookRot, clamp01(smoothIsMouseLooking - btnLookingFactor));

	if (smoothIsAiming > 0.00001f) {
		float currentFov = lerp(fov3P, fov3PAiming, smoothIsAiming);
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

	float factor = clamp01(vehSpeed / (maxHighSpeed));
	float currentDistanceIncrement = lerp(0.f, maxHighSpeedDistanceIncrement, easeOutCubic(factor));

	factor = clamp01(vehAcceleration);

	smoothAccelerationFactor = lerp(smoothAccelerationFactor, factor, clamp01(27.5f * getDeltaTime()));
	currentDistanceIncrement += lerp(0.f, accelerationCamDistanceMultiplier, easeOutCubic(smoothAccelerationFactor));

	if (isBike)
		currentDistanceIncrement *= 1.1f;

	Vector3f relativeLookDir = back;

	if (lookBehind) {
		relativeLookDir = front;
		finalDistMult *= -1.f;
	}

	currentTowHeightIncrement = lerp(currentTowHeightIncrement, towHeightIncrement, 1.45f * getDeltaTime());
	currentTowLongitudeIncrement = lerp(currentTowLongitudeIncrement, towLongitudeIncrement, 1.75f * getDeltaTime());

	Vector3f V3CurrentTowHeightIncrement = up * currentTowHeightIncrement;

	//Vector3f camPos = posCenter + extraCamHeight + V3CurrentTowHeightIncrement + (finalQuat * relativeLookDir * (longitudeOffset3P + (currentDistanceIncrement * finalDistMult) + currentTowLongitudeIncrement));
	Vector3f camPos = posCenter + extraCamHeight + V3CurrentTowHeightIncrement + (finalQuat * relativeLookDir * (longitudeOffset3P + (currentDistanceIncrement * finalDistMult) + currentTowLongitudeIncrement));

	Vector3f offsetLatPoint = Vector3f(0.f, 0.f, 0.f);
	Vector3f offsetLatPointFront = Vector3f(0.f, 0.f, 0.f);


	Vector3f camForward = getCameraForwardVector(customCam).normalized();
	Vector3f camRight = getCameraRightVector(customCam).normalized();
	//Vector3f latVector = lerp(vehRightVector, camRight, 0.5f);
	Vector3f latVector = camRight;

	float angle = GAMEPLAY::GET_ANGLE_BETWEEN_2D_VECTORS(camForward.x(), camForward.y(), vehForwardVector.x(), vehForwardVector.y());

	relAngle3p = angle;

	float maxAngle = 35.f;
	angle = clamp(angle, 0.f, maxAngle);

	float angleFront = angle;

	angle = lerp(0.f, maxAngle + 2.f, easeInCubic(unlerp(0.f, maxAngle, angle))); // apply easing
	angleFront = lerp(0.f, maxAngle + 2.f, unlerp(0.f, maxAngle, angleFront)); // apply easing

	if (distanceOnAxisNoAbs(camPos, vehPos, latVector) < 0.f)
	{
		angle = -angle;
		relAngle3p = -relAngle3p;
		angleFront = -angleFront;
	}

	if (!lookBehind /* && !isBike */)
	{
		angle = angle * 0.003f;
		angleFront = angleFront * 0.003f;
		//showText(0.01f, 0.200f, 0.4, ("angle: " + std::to_string(angle)).c_str(), 4, solidWhite, true);
		offsetLatPoint = latVector * (angle * (1.f - smoothIsMouseLooking) * (1.f - smoothIsInAir) * (1.f - smoothIsAiming) * clamp01((vehSpeed - 0.002f) * 0.1f) * clamp01(1.f - ((vehSpeed * 0.01f))));
		offsetLatPointFront = latVector * (angleFront * (1.f - smoothIsMouseLooking) * (1.f - smoothIsInAir) * (1.f - smoothIsAiming) * clamp01((vehSpeed - 0.002f) * 0.1f) * clamp01(1.f - ((vehSpeed * 0.01f))));
	}

	//if (isBike)
	//	setCamPos(customCam, camPos);
	//else
	setCamPos(customCam, camPos + (offsetLatPointFront * 1.455f));

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

	//if(isBike)
	//	camPointAt(customCam, finalPosCenter + (-up * .170f));
	//else
	Vector3f rotEuler = QuatToEuler(finalQuat);

	rotEuler[1] = 0.f;

	CAM::SET_CAM_ROT(customCam, rotEuler.x(), rotEuler.y(), rotEuler.z(), 2);
}

bool LastInputMethodWasMouseAndKeyboard()
{
	return CONTROLS::_IS_INPUT_DISABLED(2);
}

Vector3f V3Reflect(Vector3f vector, Vector3f normal) 
{
	float dot = vector.dot(normal);
	Vector3f temp = normal * dot * 2.f;
	return vector - temp;
}

void updateCam3pNfsAlgorithm()
{
	Vector3f extraCamHeight = up * (0.14f + extraAngleCamHeight);
	Vector3f posCenter = vehPos + (up * heightOffset3P) + (vehForwardVector * 0.365f);

	semiDelayedVehSpeed = lerp(semiDelayedVehSpeed, vehSpeed, clamp01(max(2.5f, vehSpeed) * getDeltaTime()));
	delayedVehSpeed = lerp(delayedVehSpeed, semiDelayedVehSpeed, clamp01(max(0.5f, semiDelayedVehSpeed * 0.75f) * getDeltaTime()));

	Quaternionf vehQuat = getEntityQuaternion(veh);
	smoothQuat3P = slerp(smoothQuat3P, vehQuat, 3.f * getDeltaTime());

	velocityQuat3P = lookRotation(smoothVelocity);
	ultraSmoothVelocity3P = lookRotation(ultraSmoothVelocity);

	//float vehPitch = QuatToEuler(velocityQuat3P).x();

	float forwardness = vehForwardVector.dot(vehVelocity);

	if (/*isBike && */forwardness >= 0.f && vehSpeed > 1.f)
	{
		Vector3f VquatVel = QuatToEuler(ultraSmoothVelocity3P);
		Vector3f sm3p;

		//if (forwardness >= 0.f)
			sm3p = QuatToEuler(smoothQuat3P);
		//else
		//	sm3p = QuatToEuler(inverseSmoothQuat3P);

		VquatVel[2] = sm3p[2];

		smoothQuat3P = QuatEuler(VquatVel);
	}

	currentTowHeightIncrement = lerp(currentTowHeightIncrement, towHeightIncrement, 1.45f * getDeltaTime());
	currentTowLongitudeIncrement = lerp(currentTowLongitudeIncrement, towLongitudeIncrement, 1.75f * getDeltaTime());

	Vector3f V3CurrentTowHeightIncrement = up * currentTowHeightIncrement;

	deccelerationSmooth = lerp(deccelerationSmooth, clamp(vehAcceleration * 65.f, -9999.f, 0.f), 1.25f * getDeltaTime());

	float distInc = clamp(semiDelayedVehSpeed * 0.035f * (vehVelocity.normalized().dot(vehForwardVector) * clamp01(vehSpeed + 0.05f)), -5.f, 5.f);
	float discIncAir = clamp(semiDelayedVehSpeed * 0.035f * clamp01(vehSpeed + 0.05f), -5.f, 5.f);

	float distIncFinal = lerp(distInc, discIncAir, smoothIsInAir);

	distIncFinal += deccelerationSmooth * (1.f - smoothIsInAir);

	float airDistance = lerp(0.f, 2.5f, smoothIsInAirNfs * (lerp(0.6f, 1.2f, smoothIsInAirNfs)));

	bool lookBehind = false;
	if (CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind) || isLookingBack)
		lookBehind = true;

	float lookHorizontalAngle = 0.f;
	if (!lookBehind) {
		lookHorizontalAngle = RelativeLookFactor < 0 ?
			lerp(0.f, -LookLeftAngle3p, -RelativeLookFactor)
			:
			lerp(0.f, LookRightAngle3p, RelativeLookFactor)
			;
	}
	else
	{
		lookHorizontalAngle = 180.f;
	}

	//if (!isAiming && hasInputThisFrame && AreSameFloat(lookHorizontalAngle, 0.f)) 
	//{
	//	//float leftNormal = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftOnly);
	//	float rightNormal = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeft);
	//	float upNormal = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookDown);
	//	//float downNormal = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookDownOnly);

	//	float x = rightNormal;// - leftNormal;
	//	float y = upNormal;// - downNormal;

	//	float r = sqrt(x * x + y * y);
	//	float maxComp = max(abs(x), abs(y));
	//	if (maxComp > 0) {
	//		x *= r / maxComp;
	//		y *= r / maxComp;
	//	}

	//	float degress = atan2(y, x) * 180.f / PI;

	//	lookHorizontalAngle = degress - 90.f;
	//}

	float leftRightRad = lookHorizontalAngle * DEG_TO_RAD;

	float roll = 0.f, pitch = 0.f, yaw = leftRightRad;
	Quaternionf qLookLeftRight;
	qLookLeftRight = AngleAxisf(roll, Vector3f::UnitX())
		* AngleAxisf(pitch, Vector3f::UnitY())
		* AngleAxisf(yaw, Vector3f::UnitZ());

	float btnLookingFactor = (abs(RelativeLookFactor));

	if (lookBehind)
		btnLookingFactor = 1.f;

	Quaternionf finalQuat3P = slerp(smoothQuat3P * qLookLeftRight, velocityQuat3P, smoothIsInAir);

	if (isAiming || hasInputThisFrame)
	{
		if (timerResetLook < 0.00001f)
		{
			lookQuat = finalQuat3P;
		}
		timerResetLook = 2.f;

		float mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -5.f;
		float my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * (LastInputMethodWasMouseAndKeyboard ? -5.f : 5.f);

		if (!LastInputMethodWasMouseAndKeyboard())  // if gamepad
		{
			mx *= 0.2f;
			my *= 0.2f;

			//showText(1, std::to_string(mx).c_str());
			//showText(2, std::to_string(my).c_str());

			float deadzone = 0.275f;
			Vector2f stickInput = Vector2f(mx, my);
			if (stickInput.norm() < deadzone)
				stickInput = Vector2f(0.f, 0.f);
			else
				stickInput = stickInput.normalized() * ((stickInput.norm() - deadzone) / (1.f - deadzone));

			mx = stickInput.x() * 2.f;
			my = stickInput.y() * 2.f;
		}

		Vector3f vecLook = Vector3f(my, 0.f, mx);

		Quaternionf result = lookQuat * QuatEuler(vecLook);
		Vector3f resultEuler = QuatToEuler(result);

		float rx = clamp(resultEuler[0], -62.f, 40.f);

		lookQuat = QuatEuler(Vector3f(rx, 0.f, resultEuler[2]));
	}

	timerResetLook = clamp(timerResetLook - getDeltaTime(), 0.f, 2.f);

	finalQuat3P = slerp(finalQuat3P, lookQuat, clamp01(timerResetLook));
	//finalQuat3P = lookQuat;

	if (smoothIsAiming > 0.00001f) {
		float currentFov = lerp(fov3P, fov3PAiming, smoothIsAiming);
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	float aimHeightIncrement = lerp(0.f, 0.22f, smoothIsAiming);

	Vector3f camPosSmooth = posCenter + extraCamHeight + V3CurrentTowHeightIncrement + (finalQuat3P * back * ((longitudeOffset3P) + 0.185f + airDistance)) + (up * aimHeightIncrement);
	camPosSmooth += slerp(smoothQuat3P, velocityQuat3P, smoothIsInAir) * back * distIncFinal;

	Vector3f camPosFinal = camPosSmooth;

	setCamPos(customCam, camPosFinal);

	// Raycast //
	int ray = WORLDPROBE::_START_SHAPE_TEST_RAY(posCenter.x(), posCenter.y(), posCenter.z(), camPosFinal.x(), camPosFinal.y(), camPosFinal.z(), 1, veh, 7);

	Vector3 endCoords, surfaceNormal;
	BOOL hit;
	Entity entityHit = 0;

	WORLDPROBE::GET_SHAPE_TEST_RESULT(ray, &hit, &endCoords, &surfaceNormal, &entityHit);

	if (hit) {
		setCamPos(customCam, toV3f(endCoords) + (finalQuat3P * front * 0.1f));
	}
	// End raycast //

	Vector3f rotEuler = QuatToEuler(finalQuat3P);

	rotEuler[1] = 0.f;

	CAM::SET_CAM_ROT(customCam, rotEuler.x(), rotEuler.y(), rotEuler.z(), 2);
}

void updateCameraSmooth3P() {
	//updateCam3pSmoothAlgorithm();
	updateCam3pNfsAlgorithm();
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
	UI::UNLOCK_MINIMAP_ANGLE();
	if (!camInitialized)
		return;

	CAM::SET_CAM_ACTIVE(customCam, false);
	CAM::DESTROY_CAM(customCam, true);
	CAM::RENDER_SCRIPT_CAMS(false, false, 3000, true, false);
	camInitialized = false;
	isInVehicle = false;
	//veh = NULL;
	
	haltCurrentCamera();
}

float getVehicleLongitude(Vehicle vehicle) {

	Vector3f vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));
	Vector3f forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));

	float maxBackDistance = 0.f;
	float maxFrontDistance = 0.f;

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
				//boneNameBack = boneName;
			}

			if (currFrontDistance > maxFrontDistance) {
				maxFrontDistance = currFrontDistance;
				//boneNameFront = boneName;
			}
		}
	}

	//ShowNotification(boneNameBack);
	//ShowNotification(boneNameFront);

	return maxBackDistance + maxFrontDistance;
}

float getVehicleLongitudeFromCenterBack(Vehicle vehicle) {

	Vector3f vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));
	Vector3f forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));

	float maxBackDistance = 0.f;

	//const char * maxFrontName;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3f bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			float currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, forward);

			if (currBackDistance > maxBackDistance) {
				maxBackDistance = currBackDistance;

				//maxFrontName = boneName;
			}
		}
	}

	//ShowNotification(maxFrontName);

	return maxBackDistance;
}

float getVehicleHeight(Vehicle vehicle) {

	Vector3f rotation = toV3f(ENTITY::GET_ENTITY_ROTATION(vehicle, false));

	Vector3f rightVector = toV3f(getRightVector(rotation));
	Vector3f forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));
	Vector3f upVector = rightVector.cross(forward);

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

float getVehicleHeightFromCenterUp(Vehicle vehicle) {

	Vector3f rotation = toV3f(ENTITY::GET_ENTITY_ROTATION(vehicle, false));

	Vector3f rightVector = toV3f(getRightVector(rotation));
	Vector3f forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));
	Vector3f upVector = rightVector.cross(forward);

	Vector3f vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));

	//float maxBackDistance = 0.f;
	float maxFrontDistance = 0.f;

	//Vector3f backBonePos;
	//Vector3f frontBonePos;
	const char * maxFrontName;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3f bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			//float currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, upVector);
			float currFrontDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, -upVector);

			//if (currBackDistance > maxBackDistance) {
			//	maxBackDistance = currBackDistance;
			//}

			if (currFrontDistance > maxFrontDistance) {
				maxFrontDistance = currFrontDistance;
				maxFrontName = boneName;
			}
		}
	}

	//ShowNotification(maxFrontName);

	return /*maxBackDistance + */ maxFrontDistance;
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
	if (IsKeyJustUp(str2key(toggleModKey))) {
		customCamEnabled = !customCamEnabled;
	}

	/*
	if (showDebug) {
		showText(0.01f, 0.200f, 0.4, ("MouseLookCountDown: " + std::to_string(mouseMoveCountdown)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.250f, 0.4, ("height: " + std::to_string(heightOffset3P)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.300f, 0.4, ("long: " + std::to_string(longitudeOffset3P)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.350f, 0.4, ("currentRotSpeed3P: " + std::to_string(currentRotSpeed3P)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.400f, 0.4, ("vehAccel_X100: " + std::to_string(vehAcceleration * 100.f)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.450f, 0.4, ("vehUpright: " + std::to_string(ENTITY::GET_ENTITY_UPRIGHT_VALUE(veh))).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.500f, 0.4, ("vehRotX: " + std::to_string(vehRot.x())).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.550f, 0.4, ("vehRotY: " + std::to_string(vehRot.y())).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.600f, 0.4, ("vehRotZ: " + std::to_string(vehRot.z())).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.650f, 0.4, ("viewLock: " + std::to_string(viewLock)).c_str(), 4, solidWhite, true);
		showText(0.01f, 0.700f, 0.4, ("vehClass: " + std::to_string(vehClass)).c_str(), 4, solidWhite, true);
	}
	*/

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
			WAIT(0); // Wait one frame so we are sure we can fetch vehicle properties
			updateVehicleVars();
			updateVehicleProperties();
		}
		if (isSuitableForCam && customCamEnabled) {

			CONTROLS::DISABLE_CONTROL_ACTION(0, eControl::ControlNextCamera, true);
			CONTROLS::DISABLE_CONTROL_ACTION(0, eControl::ControlVehicleCinCam, true);

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

			if (IsKeyJustUp(str2key(reloadKey))) {
				ReadSettings(true);
				updateVehicleVars();
				updateVehicleProperties();
				setupCustomCamera();
			}

			updateTimers();
			ProccessLookLeftRightOrBackInput();
			updateCustomCamera();

			Vector3 rotCam = CAM::GET_CAM_ROT(customCam, 2);
			bool lBehind = false;

			if (CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind)) 
			{
				lBehind = true;
				smoothRadarAngle = lerpAngle(smoothRadarAngle, NormalizeAngle(rotCam.z - 180.f), 1.f);
			}
			else
				smoothRadarAngle = lerpAngle(smoothRadarAngle, NormalizeAngle(rotCam.z), lerp(6.f * getDeltaTime(), 1.f, smoothIsMouseLooking));

			smoothRadarAngle = NormalizeAngle(smoothRadarAngle);

			UI::LOCK_MINIMAP_ANGLE(NormalizeAngle(smoothRadarAngle - (lBehind ? 180.f : 0.f)));
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
