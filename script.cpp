/*
*  Custom Camera V Plus
*/
#include "script.h"


float smoothIsMouseLooking = 0.f;
float viewLock = 0.f;
float smoothViewLock = 0.f;

BOOL modEnabled = true;
BOOL camInitialized = false;
Vehicle veh;
Ped playerPed;

Vector3f prevVehPos;
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
Vector3f smoothVehAngularVelocity;

//Vector3f smoothVehRightVector;

float vehAcceleration = 0.f;
float smoothAccelerationFactor = 0.f;
float prevVehAcceleration = 0.f;

eVehicleClass vehClass;
bool vehHasTowBone = false;
bool vehHasTrailerBone = false;

//float pivotFrontOffset = 0.365f;
//float pivotFrontOffsetHighSpeed = 0.0f;
float pivotFrontOffsetStraight = 0.f;
float pivotFrontOffsetTurn = 0.f;
float finalPivotFrontOffset = 0.f;

float smoothAngularDiff = 0.f;
float smoothAuxLerpFactor = 0.f;

Camera customCam = NULL;
//float fov3P = 85.f;
//float fov1P = 90.f;
float fov3P = 77.5f;
float fov1P = 75.F;
float fov1PAiming = 60.f;
float fov3PAiming = 60.f;
float distanceOffset3p = 0.f;
float heightOffset3p = 0.f;
float cameraAngle3p = 3.5f;
float InertiaForce3p = 1.f;
const float PI = 3.1415926535897932f;
int lastVehHash = -1;
bool isBike = false;

bool isInVehicle = false;

float calcLongitudeOffset3P = 0.f;
float calcHeightOffset3P = 0.f;

float heightIcrementCalc = 0.f;

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

float smoothFactor = 0.f;

//float variableRotLerpSpeed = 7.75f;

float currentRotSpeed3P = 2.0f;

float smoothSpeedFactor = 0.f;

float useRoadSurfaceNormal = false;
Vector3f cachedSurfaceNormal;
Vector3f smoothSurfaceNormal;

float smoothCurveEval = 0.f;
float smoothCurveEvalLat = 0.f;

Vector3f smoothVehForwardVector = Vector3f();

Vector3f smoothVelocity = Vector3f();
Vector3f ultraSmoothVelocity = Vector3f();
Quaternionf dirQuat3P = Quaternionf();
Quaternionf veloQuat3P = Quaternionf();
Quaternionf veloCompQuat3P = Quaternionf();
Quaternionf veloRotQuat3P = Quaternionf();
Quaternionf smoothQuat3P = Quaternionf();
//Quaternionf smootherQuat3P = Quaternionf();
Quaternionf ultraSmoothVelocity3P = Quaternionf();
Vector3f smoothRotSeat = Vector3f();
Quaternionf smoothQuatSeat = Quaternionf();
Quaternionf PrevCamQuat = Quaternionf();
float smoothIsInAir = 0.f;
float smootherIsInAir = 0.f;
float smootherIsInAirStep = 0.f;
float smoothIsInAirNfs = 0.f;
float maxHighSpeed = 130.f;
float maxHighSpeedDistanceIncrement = 1.45f;
float accelerationCamDistanceMultiplier = 1.45f;
Vector3f playerVehOffset;

Vector3f velocityDir = Vector3f();
Vector3f prevVelocityDir = Vector3f();

float VeloRotDiff = 0.f;

Vector3f smoothVelocityDir;

Vector3f up(0.0f, 0.0f, 1.0f);
Vector3f down(0.0f, 0.0f, -1.0f);
Vector3f back(0.0f, -1.0f, 0.0f);
Vector3f front(0.0f, 1.0f, 0.0f);
Vector3f right(1.0f, 0.0f, 0.0f);

Vector2i lastMouseCoords;
float mouseMoveCountdown = 0.f;

bool isAiming = false;
float smoothIsAiming = 0.f;

bool customCamEnabled = true;

enum eCamType {
	DriverSeat1P = 0,

	//Smooth3P = 1, // Legacy
	//Racing3P = 2, // New

	Racing3P = 1, // New
};

int camsLength = 2;
int currentCam = eCamType::Racing3P; // Use new by default

// Input stuff taken from https://github.com/E66666666/GTAVMenuBase

const int KEYS_SIZE = 255;

static const std::unordered_map<std::string, int> KeyMap = {
	// CTRL/SHIFT already have their left and right variants mapped
	//keymap["SHIFT"] = VK_SHIFT;
	//keymap["CTRL"] = VK_CONTROL;
	{ "XB1" , VK_XBUTTON1 },
	{ "XB2" , VK_XBUTTON2 },
	{ "LMB" , VK_LBUTTON },
	{ "RMB" , VK_RBUTTON },
	{ "CANCEL" , VK_CANCEL },
	{ "MMB" , VK_MBUTTON },
	{ "BACKSPACE" , VK_BACK },
	{ "TAB" , VK_TAB },
	{ "CLEAR" , VK_CLEAR },
	{ "RETURN" , VK_RETURN },
	{ "ALT" , VK_MENU },
	{ "PAUSE" , VK_PAUSE },
	{ "CAPSLOCK" , VK_CAPITAL },
	{ "ESCAPE" , VK_ESCAPE },
	{ "SPACE" , VK_SPACE },
	{ "PAGEUP" , VK_PRIOR },
	{ "PAGEDOWN" , VK_NEXT },
	{ "END" , VK_END },
	{ "HOME" , VK_HOME },
	{ "LEFT" , VK_LEFT },
	{ "UP" , VK_UP },
	{ "RIGHT" , VK_RIGHT },
	{ "DOWN" , VK_DOWN },
	{ "SELECT" , VK_SELECT },
	{ "PRINT" , VK_PRINT },
	{ "EXECUTE" , VK_EXECUTE },
	{ "PRINTSCREEN" , VK_SNAPSHOT },
	{ "INSERT" , VK_INSERT },
	{ "DELETE" , VK_DELETE },
	{ "HELP" , VK_HELP },
	{ "LWIN" , VK_LWIN },
	{ "RWIN" , VK_RWIN },
	{ "APPS" , VK_APPS },
	{ "SLEEP" , VK_SLEEP },
	{ "NUM0" , VK_NUMPAD0 },
	{ "NUM1" , VK_NUMPAD1 },
	{ "NUM2" , VK_NUMPAD2 },
	{ "NUM3" , VK_NUMPAD3 },
	{ "NUM4" , VK_NUMPAD4 },
	{ "NUM5" , VK_NUMPAD5 },
	{ "NUM6" , VK_NUMPAD6 },
	{ "NUM7" , VK_NUMPAD7 },
	{ "NUM8" , VK_NUMPAD8 },
	{ "NUM9" , VK_NUMPAD9 },
	{ "*" , VK_MULTIPLY },
	{ "PLUS" , VK_ADD },
	{ "," , VK_SEPARATOR },
	{ "MINUS" , VK_SUBTRACT },
	{ "." , VK_DECIMAL },
	{ "/" , VK_DIVIDE },
	{ "F1" , VK_F1 },
	{ "F2" , VK_F2 },
	{ "F3" , VK_F3 },
	{ "F4" , VK_F4 },
	{ "F5" , VK_F5 },
	{ "F6" , VK_F6 },
	{ "F7" , VK_F7 },
	{ "F8" , VK_F8 },
	{ "F9" , VK_F9 },
	{ "F10" , VK_F10 },
	{ "F11" , VK_F11 },
	{ "F12" , VK_F12 },
	{ "F13" , VK_F13 },
	{ "F14" , VK_F14 },
	{ "F15" , VK_F15 },
	{ "F16" , VK_F16 },
	{ "F17" , VK_F17 },
	{ "F18" , VK_F18 },
	{ "F19" , VK_F19 },
	{ "F20" , VK_F20 },
	{ "F21" , VK_F21 },
	{ "F22" , VK_F22 },
	{ "F23" , VK_F23 },
	{ "F24" , VK_F24 },
	{ "VK_F1" , VK_F1 },
	{ "VK_F2" , VK_F2 },
	{ "VK_F3" , VK_F3 },
	{ "VK_F4" , VK_F4 },
	{ "VK_F5" , VK_F5 },
	{ "VK_F6" , VK_F6 },
	{ "VK_F7" , VK_F7 },
	{ "VK_F8" , VK_F8 },
	{ "VK_F9" , VK_F9 },
	{ "VK_F10" , VK_F10 },
	{ "VK_F11" , VK_F11 },
	{ "VK_F12" , VK_F12 },
	{ "VK_F13" , VK_F13 },
	{ "VK_F14" , VK_F14 },
	{ "VK_F15" , VK_F15 },
	{ "VK_F16" , VK_F16 },
	{ "VK_F17" , VK_F17 },
	{ "VK_F18" , VK_F18 },
	{ "VK_F19" , VK_F19 },
	{ "VK_F20" , VK_F20 },
	{ "VK_F21" , VK_F21 },
	{ "VK_F22" , VK_F22 },
	{ "VK_F23" , VK_F23 },
	{ "VK_F24" , VK_F24 },
	{ "NUMLOCK" , VK_NUMLOCK },
	{ "SCROLL" , VK_SCROLL },
	{ "LSHIFT" , VK_LSHIFT },
	{ "RSHIFT" , VK_RSHIFT },
	{ "LCTRL" , VK_LCONTROL },
	{ "RCTRL" , VK_RCONTROL },
	{ "LMENU" , VK_LMENU },
	{ "RMENU" , VK_RMENU },
	{ "BROWSER_BACK" , VK_BROWSER_BACK },
	{ "BROWSER_FORWARD" , VK_BROWSER_FORWARD },
	{ "BROWSER_REFRESH" , VK_BROWSER_REFRESH },
	{ "BROWSER_STOP" , VK_BROWSER_STOP },
	{ "BROWSER_SEARCH" , VK_BROWSER_SEARCH },
	{ "BROWSER_FAVORITES" , VK_BROWSER_FAVORITES },
	{ "BROWSER_HOME" , VK_BROWSER_HOME },
	{ "VOLUME_MUTE" , VK_VOLUME_MUTE },
	{ "VOLUME_DOWN" , VK_VOLUME_DOWN },
	{ "VOLUME_UP" , VK_VOLUME_UP },
	{ "MEDIA_NEXT_TRACK" , VK_MEDIA_NEXT_TRACK },
	{ "MEDIA_PREV_TRACK" , VK_MEDIA_PREV_TRACK },
	{ "MEDIA_STOP" , VK_MEDIA_STOP },
	{ "MEDIA_PLAY_PAUSE" , VK_MEDIA_PLAY_PAUSE },
	{ "LAUNCH_MAIL" , VK_LAUNCH_MAIL },
	{ "LAUNCH_MEDIA_SELECT" , VK_LAUNCH_MEDIA_SELECT },
	{ "LAUNCH_APP1" , VK_LAUNCH_APP1 },
	{ "LAUNCH_APP2" , VK_LAUNCH_APP2 },
	{ "PLAY" , VK_PLAY },
	{ "ZOOM" , VK_ZOOM },
	{ "VK_OEM_1" , VK_OEM_1 },			// ';:' for US
	{ "VK_OEM_PLUS" , VK_OEM_PLUS },		// '+' any country
	{ "VK_OEM_COMMA" , VK_OEM_COMMA },	// ',' any country
	{ "VK_OEM_MINUS" , VK_OEM_MINUS },	// '-' any country
	{ "VK_OEM_PERIOD" , VK_OEM_PERIOD }, // '.' any country
	{ "VK_OEM_2" , VK_OEM_2 },			// '/?' for US
	{ "VK_OEM_3" , VK_OEM_3 },			// '`~' for US
	{ "VK_OEM_4" , VK_OEM_4 },			// '{' for US
	{ "VK_OEM_5" , VK_OEM_5 },			// '\|' for US
	{ "VK_OEM_6" , VK_OEM_6 },			// '}' for US
	{ "VK_OEM_7" , VK_OEM_7 },			// ''"' for US
	{ "VK_OEM_8" , VK_OEM_8 },			// � !
	{ "VK_OEM_102" , VK_OEM_102 },		// > <	
};

struct {
	BOOL curr;
	BOOL prev;
} _keyStates[KEYS_SIZE];

bool IsWindowFocused() {
	auto foregroundHwnd = GetForegroundWindow();
	DWORD foregroundProcId;
	GetWindowThreadProcessId(foregroundHwnd, &foregroundProcId);
	auto currentProcId = GetCurrentProcessId();
	if (foregroundProcId == currentProcId) {
		return true;
	}
	return false;
}

bool IsKeyDown(DWORD key) {
	if (!IsWindowFocused()) return false;
	if (GetAsyncKeyState(key) & 0x8000) return true;
	return false;
}

bool IsKeyJustUp(DWORD key, bool exclusive) {
	_keyStates[key].curr = IsKeyDown(key);
	if (!_keyStates[key].curr && _keyStates[key].prev) {
		_keyStates[key].prev = _keyStates[key].curr;
		return true;
	}
	_keyStates[key].prev = _keyStates[key].curr;
	return false;
}

// https://stackoverflow.com/questions/2333728/stdmap-default-value
template <template<class, class, class...> class C, typename K, typename V, typename... Args>
V GetWithDef(const C<K, V, Args...>& m, K const& key, const V& defval) {
	typename C<K, V, Args...>::const_iterator it = m.find(key);
	if (it == m.end())
		return defval;
	return it->second;
}

DWORD str2key(std::string humanReadableKey) {
	if (humanReadableKey.length() == 1) {
		char letter = humanReadableKey.c_str()[0];
		if ((letter >= 0x30 && letter <= 0x39) || (letter >= 0x41 && letter <= 0x5A)) {
			return static_cast<int>(letter);
		}
	}
	return GetWithDef(KeyMap, humanReadableKey, -1);
}

std::string key2str(DWORD key) {
	if (key == -1) return "UNKNOWN";
	if ((key >= 0x30 && key <= 0x39) || (key >= 0x41 && key <= 0x5A)) {
		std::string letter;
		letter = (char)key;
		return std::string(letter);
	}
	for (auto k : KeyMap) {
		if (k.second == key) return k.first;
	}
	return "UNKNOWN";
}

bool showDebug = false;

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

bool InertiaAffectsPitch3p = false;
bool SmartHeadingEnabled = true;
float SmartHeadingIntensity = 1.f;
float smoothSmartHeading = 0.f;
float smoothSmartHeading2 = 0.f;

bool InertiaEffects1p = true;

float semiDelayedVehSpeed = 0.f;
float delayedVehSpeed = 0.f;

Vector3f prevCamPos = Vector3f();
Vector3f camPosSmooth = Vector3f();

float smoothLatFactor = 0.f;

float smoothLatDist = 0.f;
float smoothLongDist = 0.f;
float smoothUpDist = 0.f;

//float smoothTurnForce3P = 0.f;
float smoothAngular1 = 0.f;
float smoothAngular2 = 0.f;
float smoothAngular3 = 0.f;

bool isLookingBack = false;

float timerResetLook = 0.f;
Quaternionf lookQuat;

float mouseDeltaX = 0.f;
float mouseDeltaY = 0.f;

float deadzone = 0.f;
bool gamepadAimEasing = true;

float gamepadSensibility = 1.f;
float mouseSensibility = 1.f;

// look left = -1.f; lookRight = 1.f; Center = 0.f (For smooth transitions)
float RelativeLookFactor = 0.f;

float prevLookHorizontalAngle = 0.f;

bool readInputFromMt = true;
float vehDelayedAccel1 = 0.f;
float vehDelayedAccel2 = 0.f;
float vehDelayedAccel3 = 0.f;
float vehDelayedAccel4 = 0.f;
float smoothIsGoingForwardInc = 0.f;

float smoothAccelSlower = 0.f;
float accelDiffSlow = 0.f;
float smoothAngVelFactor = 0.f;

Vector3f smoothVeloDir;
Vector3f smoothVeloEdit;

char * reloadKey = "F10";
char * toggleModKey = "1";
char * lookLeftKey = "B";
char * lookRightKey = "N";

bool hasInputThisFrame = false;
bool hasMouseInputThisFrame = false;

bool AreFloatsSimilar(float a, float b)
{
	return fabs(a - b) < 0.001f;
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

float easeInCubic(float t) {
	return pow(t, 3.f);
}

float easeInCirq(float t) {
	return -1 * (sqrt(1 - t * t) - 1);
}

float easeOutCubic(float t) {
	return 1.f - easeInCubic(1.f - t);
}

float easeInSine(float t) {
	return -1.f * cos(t / 1.f * (PI * 0.5f)) + 1.f;
}

float easeOutSine(float t) {
	return sin(t / 1 * (PI * 0.5));
}

// Range -1.f | 1.f
float easeInSineInput(float axisInput)
{
	axisInput = clamp(axisInput, -1.f, 1.f);

	float easeAbs = easeInSine(abs(axisInput));

	if (axisInput < 0.f)
		easeAbs *= -1.f;

	return easeAbs;
}

Vector2f readLookAroundInput()
{
	float mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -5.f;
	float my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * (LastInputMethodWasMouseAndKeyboard ? -5.f : 5.f);

	if (!LastInputMethodWasMouseAndKeyboard())  // if gamepad
	{
		mx *= 0.2f;
		my *= 0.2f;

		//showText(1, std::to_string(mx).c_str());
		//showText(2, std::to_string(my).c_str());


		// apply deadzone
		Vector2f stickInput = Vector2f(mx, my);
		if (stickInput.norm() < deadzone)
			stickInput = Vector2f(0.f, 0.f);
		else
			stickInput = stickInput.normalized() * ((stickInput.norm() - deadzone) / (1.f - deadzone));

		mx = stickInput.x();
		my = stickInput.y();


		// apply easing 
		if (gamepadAimEasing) {
			mx = easeInSineInput(mx);
			my = easeInSineInput(my);
		}

		// Scale (sensibility)
		mx *= 2.f * gamepadSensibility;
		my *= 2.f * gamepadSensibility;
	}
	else
	{
		mx *= mouseSensibility;
		my *= mouseSensibility;
	}

	return Vector2f(mx, my);
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

	const float f = 57.29578f; // 360.0f / 2.0f / 3.1415926535897f
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

// Projects a vector onto another vector.
Vector3f Project(Vector3f vector, Vector3f onNormal)
{
	float sqrMag = onNormal.dot(onNormal);
	if (sqrMag < FLT_EPSILON)
		return Vector3f();
	else
		return onNormal * vector.dot(onNormal) / sqrMag;
}

// Projects a vector onto a plane defined by a normal orthogonal to the plane.
Vector3f ProjectOnPlane(Vector3f vector, Vector3f planeNormal)
{
	return vector - Project(vector, planeNormal);
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
	UI::BEGIN_TEXT_COMMAND_THEFEED_POST("CELL_EMAIL_BCON");

	std::string strMsg(msg);
	const int maxStringLength = 99;

	for (int i = 0; i < strlen(msg); i += maxStringLength)
	{
		UI::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME((char*)strMsg.substr(i, min(maxStringLength, strlen(msg) - i)).c_str());
	}

	UI::END_TEXT_COMMAND_THEFEED_POST_TICKER(false, true);
}

void ReadSettings(bool byUser) 
{
	CSimpleIniA ini;
	//ini.SetUnicode();
	SI_Error res = ini.LoadFile("CustomCameraVPlus.ini");

	if (res == SI_Error::SI_OK) {
		reloadKey    = strdup(ini.GetValue("keyMappings", "reloadSettingsKey", "F10"));
		toggleModKey = strdup(ini.GetValue("keyMappings", "toggleModKey", "1"));
		lookLeftKey  = strdup(ini.GetValue("keyMappings", "lookLeftKey", "B"));
		lookRightKey = strdup(ini.GetValue("keyMappings", "lookRightKey", "N"));

		distanceOffset3p = (float)ini.GetDoubleValue("3rdPersonView", "distanceOffset", 0.0);
		heightOffset3p = (float) ini.GetDoubleValue("3rdPersonView", "heightOffset", 0.0);
		cameraAngle3p = clamp((float)ini.GetDoubleValue("3rdPersonView", "cameraAngle", 3.5), 0.f, 20.f);

		fov3P = (float) ini.GetDoubleValue("3rdPersonView", "fov", 77.5);
		fov1P = (float) ini.GetDoubleValue("1stPersonView", "fov", 75.0);

		LookLeftAngle3p = (float) ini.GetDoubleValue("3rdPersonView", "lookLeftAngle", 90.0);
		LookRightAngle3p = (float)ini.GetDoubleValue("3rdPersonView", "lookRightAngle", 90.0);
		
		//InertiaAffectsPitch3p = ini.GetLongValue("3rdPersonView", "InertiaAffectsPitch", 0) > 0;
		InertiaForce3p = (float)ini.GetDoubleValue("3rdPersonView", "InertiaForce", 1.0);

		SmartHeadingEnabled = ini.GetLongValue("3rdPersonView", "SmartHeading", 1l) > 0;
		SmartHeadingIntensity = (float)ini.GetDoubleValue("3rdPersonView", "SmartHeadingIntensity", 1.0);

		InertiaEffects1p = ini.GetLongValue("1stPersonView", "InertiaEffects", 1) > 0;

		LookLeftAngle1p = (float)ini.GetDoubleValue("1stPersonView", "lookLeftAngle", 75.0);
		LookRightAngle1p = (float)ini.GetDoubleValue("1stPersonView", "lookLeftAngle", 80.0);

		readInputFromMt = ini.GetLongValue("general", "GetInputFromGearsAsi", 1l) > 0;

		deadzone = clamp01((float)ini.GetDoubleValue("input", "gamepadDeadzone", 0.0));
		gamepadAimEasing = ini.GetLongValue("input", "gamepadEasing", 1l) > 0;

		gamepadSensibility = (float)ini.GetDoubleValue("input", "gamepadSensibility", 1.0);
		mouseSensibility = (float)ini.GetDoubleValue("input", "mouseSensibility", 1.0);

		if (!byUser)
		{
			currentCam = (int)ini.GetLongValue("general", "DefaultCamera", 2);
			currentCam = currentCam % camsLength;
		}

		if (byUser) {
			ShowNotification("CCVPlus: Settings reloaded");
			updateVehicleProperties();
			setupCurrentCamera();

			if (readInputFromMt && !MT::Present)
			{
				ShowNotification("CCVPlus: Manual Transmission integration was enabled, but no compatible Gears.asi found. (You need at least version 4.6.6). Looking left/right/back from steering wheel will not work.");
			}
		}
			
	}
	else
		ShowNotification("CCVPlus: Cannot load settings! Missing ini file?");
}


// showText() taken from https://github.com/E66666666/GTAVManualTransmission/
void showText(float x, float y, float scale, std::string text, int font, const Color &rgba, bool outline) {
	UI::SET_TEXT_FONT(font);
	UI::SET_TEXT_SCALE(scale, scale);
	UI::SET_TEXT_COLOUR(rgba.R, rgba.G, rgba.B, rgba.A);
	UI::SET_TEXT_WRAP(0.0, 1.0);
	UI::SET_TEXT_CENTRE(0);
	if (outline) UI::SET_TEXT_OUTLINE();
	UI::BEGIN_TEXT_COMMAND_DISPLAY_TEXT("STRING");
	UI::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME((char*)text.c_str());
	UI::END_TEXT_COMMAND_DISPLAY_TEXT(x, y);
}

void showText(int index, std::string text) {
	showText(0.03f, 0.03f * (float)index, 0.3f, (char*)text.c_str(), 0, solidWhite, true);
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
	setupCompatibility();

	ReadSettings(false);
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
	return AreFloatsSimilar(v1.x(), v2.x()) &&
		AreFloatsSimilar(v1.y(), v2.y()) &&
		AreFloatsSimilar(v1.z(), v2.z());
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

float damp(float a, float b, float lambda, float dt)
{
	return lerp(a, b, 1.f - exp(-lambda * dt));
}

Vector3f lerp(Vector3f& v1, Vector3f& v2, float t)
{
	return Vector3f(lerp(v1.x(), v2.x(), t),
		lerp(v1.y(), v2.y(), t),
		lerp(v1.z(), v2.z(), t));
}

Vector3f damp(Vector3f& a, Vector3f& b, float lambda, float dt)
{
	return lerp(a, b, 1 - exp(-lambda * dt));
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
	// Roll pitch and yaw in Radians (-90.0, 0.0, 0.0)
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

Quaternionf lookRotationEx(Vector3f dir, Vector3f upVector)
{
	Matrix3f lookM = matrixLookAt(dir, upVector);
	Quaternionf ret = Quaternionf(lookM);

	return ret;
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

Quaternionf lerpXZ(Quaternionf& q1, Quaternionf& q2, float tX, float tY, float tZ)
{
	Vector3f e1 = QuatToEuler(q1);
	Vector3f e2 = QuatToEuler(q2);

	float x = lerp(e1.x(), e2.x(), tX);
	float y = lerp(e1.y(), e2.y(), tY);
	float z = lerp(e1.z(), e2.z(), tZ);

	Vector3f ret = Vector3f(x, y, z);
	//Vector3f ret = Vector3f(x, 0.f, z);

	return QuatEuler(ret);
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
		playerHeadAltitude = 0.675f;
		playerHeadDistance = -0.56f;
	}
	else if (vehClass == eVehicleClass::VehicleClassSuper)
	{
		// super sport
		playerHeadAltitude = 0.645f;
		playerHeadDistance = -0.61f;

	}
	else
	{
		// anything else
		playerHeadAltitude = 0.695f;
		playerHeadDistance = -0.56f;
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

	calcLongitudeOffset3P = getVehicleLongitudeFromCenterBack(veh) + 0.7f;

	calcHeightOffset3P = clamp(getVehicleHeightFromCenterUp(veh) + 0.5f, 0.f, 2.0f);
	//ShowNotification(std::to_string(heightOffset3P).c_str());

	if (calcHeightOffset3P > 1.75f)
	{
		extraAngleCamHeight = clamp(lerp(0.1f, 2.0f, unlerp(1.75f, 2.00f, calcHeightOffset3P)), 0.f, 3.0f);
		calcHeightOffset3P += (extraAngleCamHeight * 0.5f);
		calcLongitudeOffset3P += extraAngleCamHeight;

		extraAngleCamHeight = clamp(extraAngleCamHeight, 0.f, 2.25f);
	}
	else
		extraAngleCamHeight = 0.0f;

	if (isBike) {
		calcLongitudeOffset3P += 1.f;
		calcHeightOffset3P = 1.38f;
	}

	calcLongitudeOffset3P += 1.45f + distanceOffset3p;

	heightIcrementCalc = calcLongitudeOffset3P * tan(cameraAngle3p * PI / 180.0);

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
	//smoothVelocity = lerp(smoothVelocity, vehSpeed > 1.25f ? vehVelocity : vehForwardVector, 5.0f * getDeltaTime());
	smoothVelocity = lerp(smoothVelocity, vehVelocity, 5.0f * getDeltaTime());

	// TODO: Refelcted velocity

	ultraSmoothVelocity = lerp(ultraSmoothVelocity, vehSpeed > 2.f ? vehVelocity : vehForwardVector, 3.f * getDeltaTime());

	if ((ENTITY::IS_ENTITY_IN_AIR(veh) || (ENTITY::GET_ENTITY_UPRIGHT_VALUE(veh) < 0.6f)) && smoothIsInAir < 0.001f)
	{
		smoothVelocity = getCameraForwardVector(customCam);
		//velocityQuat3P = lookRotation(smoothVelocity);
	}

	bool isInAir = ENTITY::IS_ENTITY_IN_AIR(veh) || ENTITY::IS_ENTITY_UPSIDEDOWN(veh);

	smoothIsInAir = lerp(smoothIsInAir, isInAir ? 1.f : 0.f, 12.f * getDeltaTime());
	smootherIsInAir = lerp(smootherIsInAir, isInAir ? 1.f : 0.f, 5.f * getDeltaTime());
	smootherIsInAirStep = lerp(smootherIsInAirStep, isInAir ? 1.f : 0.f, lerp(5.25f, 1.35f, smootherIsInAir) * getDeltaTime());
	smoothIsInAirNfs = lerp(smoothIsInAirNfs, (ENTITY::IS_ENTITY_IN_AIR(veh)) ? 1.f : 0.f, 0.75f * getDeltaTime());
	vehRightVector = toV3f(getRightVector(vehRot));
	vehUpVector = vehRightVector.cross(vehForwardVector);
	vehAcceleration = getVehicleAcceleration();
	vehRelativeSpeedVector = toV3f(ENTITY::GET_ENTITY_SPEED_VECTOR(veh, true));
	vehAngularVelocity = toV3f(ENTITY::GET_ENTITY_ROTATION_VELOCITY(veh));
	smoothVehAngularVelocity = lerp(smoothVehAngularVelocity, vehAngularVelocity, 2.f * getDeltaTime());
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
	else if (currentCam == eCamType::Racing3P) {
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.15f);
		CAM::SET_CAM_FAR_CLIP(customCam, 800.0f);
		CAM::SET_CAM_FOV(customCam, fov3P);
		viewLock = 0.f;

		timerResetLook = 0.25f;
		smoothIsMouseLooking = 1.0f;
		lookQuat = getEntityQuaternion(veh);
		dirQuat3P = lookQuat;
		smoothQuat3P = lookQuat;

		prevCamPos = (vehPos + (up * calcHeightOffset3P)) + (up * (0.14f + extraAngleCamHeight)) + ((lookQuat) * back * (calcLongitudeOffset3P + currentTowLongitudeIncrement));
		camPosSmooth = prevCamPos;
		prevVehPos = vehPos;
		smoothLatDist = 0.f;
		smoothCurveEval = 0.f;
//		smoothVehRightVector = vehRightVector;
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

	float distIncFinal = 0.f;

	if (InertiaEffects1p) {
		float accelScale = VEHICLE::GET_VEHICLE_ACCELERATION(veh);

		float vehDirectAccel = ((double)(vehAcceleration * accelScale)) * 1700.0;
		vehDelayedAccel1 = lerp(vehDelayedAccel1, vehDirectAccel, 1.725f * getDeltaTime());
		vehDelayedAccel2 = lerp(vehDelayedAccel2, vehDirectAccel, 0.765f * getDeltaTime());

		if (vehSpeed <= 0.02f && vehDelayedAccel1 > vehDelayedAccel2)
			vehDelayedAccel2 = lerp(vehDelayedAccel2, vehDelayedAccel1, 8.f * getDeltaTime());

		float accelThreshold = (vehDelayedAccel1 - vehDelayedAccel2);

		distIncFinal = accelThreshold + max(0.f, vehSpeed * 0.01295f) - 0.3f;

		distIncFinal *= 0.175f;
		distIncFinal = clamp(distIncFinal, -0.1f, 0.1f);
	}

	float btnLookingFactor = abs(RelativeLookFactor);

	Quaternionf finalQ;

	float wheelieFactor = 0.f;

	if (isBike) {
		CAM::STOP_CAM_POINTING(customCam);
		Vector3f rot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		rot[1] = rot.y() * 0.5f; // rot.y = rot.y * .5f

		//smoothRotSeat = Vector3fLerpAngle(smoothRotSeat, rot, clamp01(30.f * getDeltaTime()));
		smoothQuatSeat = getEntityQuaternion(veh);
		veloQuat3P = lookRotation(vehSpeed < 1.25f || vehVelocity.dot(vehForwardVector) <= 0.12f ? vehForwardVector : vehVelocity);

		smoothQuatSeat = slerp(veloQuat3P, smoothQuatSeat, smoothIsInAir);

		wheelieFactor = clamp(vehForwardVector.dot(up), 0.f, 0.5f) * (1.f - smoothIsInAir);

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

		finalQ = smoothQuatSeat * qLookLeftRight;
	
		//Vector3f veloPlane = ProjectOnPlane(vehSpeed > 0.05f ? vehVelocity : vehForwardVector, vehForwardVector.cross(up)).normalized();
		//float wheelieFactor = veloPlane.dot(vehForwardVector) * (1.f - smoothIsInAir) * 5.f;

		//float roll1 = wheelieFactor * DEG_TO_RAD, pitch1 = 0.f, yaw1 = 0.f;
		//Quaternionf wheelieCompensation;
		//wheelieCompensation = AngleAxisf(roll1, Vector3f::UnitX())
		//	* AngleAxisf(pitch1, Vector3f::UnitY())
		//	* AngleAxisf(yaw1, Vector3f::UnitZ());

		//finalQ *= wheelieCompensation;

		if (isAiming || hasInputThisFrame)
		{
			if (timerResetLook < 0.00001f)
			{
				lookQuat = finalQ;
			}
			timerResetLook = 2.f;

			Vector2f lookXY = readLookAroundInput();
			Vector3f vecLook = Vector3f(lookXY.y(), 0.f, lookXY.x());

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

			finalQ = smoothQuatSeat * qLookLeftRight;

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

	camPos = camPos + smoothQuatSeat * back * distIncFinal;

	if (isBike) 
	{
		camPos += wheelieFactor * 0.155f * up;
		camPos += wheelieFactor * 0.540f * -((vehSpeed < 1.25f || vehVelocity.dot(vehForwardVector) <= 0.12f ? vehForwardVector : vehVelocity).normalized());
	}

	CAM::SET_CAM_COORD(customCam, camPos.x(), camPos.y(), camPos.z());

	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, 1, 0);
}

void ProccessLookLeftRightOrBackInput()
{
	const float rotSpeed = 9.f;

	bool readFromMtApi = readInputFromMt && MT::Present;

	bool evalLeft = IsKeyDown(str2key(lookLeftKey)) || (readFromMtApi && MT::LookingBack());
	bool evalRight = IsKeyDown(str2key(lookRightKey)) || (readFromMtApi && MT::LookingRight());

	isLookingBack = CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlVehicleLookBehind) || (readFromMtApi && MT::LookingBack()) || (evalLeft && evalRight);

	if (evalLeft && !evalRight) {
		RelativeLookFactor += rotSpeed * getDeltaTime();
	}
	else if (evalRight || isLookingBack) {
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

Vector3f VectorReflect(Vector3f vector, Vector3f normal)
{
	// I is the original array
	// N is the normal of the incident plane
	// R = I - (2 * N * ( DotProduct[ I,N] ))
	// inline the dotProduct here instead of calling method
	double dotProduct = ((vector.x() * normal.x()) + (vector.y() * normal.y())) + (vector.z() * normal.z());

	Vector3f reflectedVector = Vector3f(
		reflectedVector.x() = vector.x() - (2.0f * normal.x()) * dotProduct,
		reflectedVector.y() = vector.y() - (2.0f * normal.y()) * dotProduct,
		reflectedVector.z() = vector.z() - (2.0f * normal.z()) * dotProduct
	);

	return reflectedVector;
}

float CalcSmoothAccel()
{
	float vehDirectAccel = vehAcceleration * 125.f;

	vehDelayedAccel1 = lerp(vehDelayedAccel1, clamp(vehDirectAccel, -0.46f, 0.46f), 3.0f * getDeltaTime());
	vehDelayedAccel2 = lerp(vehDelayedAccel2, clamp(vehDirectAccel, -0.46f, 0.46f), 2.0f * getDeltaTime());

	float accelThreshold = vehDelayedAccel2 - vehDelayedAccel1;

	float smoothAccel = clamp(accelThreshold, -0.50f, 0.50f)/* + max(0.f, vehSpeed * 0.01295f)*/ * (1.f - smoothIsInAir);

	//showText(1, std::to_string(smoothAccel).c_str());
	//distIncFinal *= 0.7f * InertiaForce3p;

	return smoothAccel;
}

float CalcSmoothAccelSlow()
{
	float vehDirectAccel = vehAcceleration * 125.f;

	vehDelayedAccel3 = lerp(vehDelayedAccel3, clamp(vehDirectAccel, -0.46f, 0.46f), 2.55f * getDeltaTime());
	vehDelayedAccel4 = lerp(vehDelayedAccel4, clamp(vehDirectAccel, -0.46f, 0.46f), 2.0f * getDeltaTime());

	float accelThreshold = vehDelayedAccel3 - vehDelayedAccel4;

	float smoothAccel = clamp(accelThreshold, -0.50f, 0.50f)/* + max(0.f, vehSpeed * 0.01295f)*/ * (1.f - smoothIsInAir);

	//showText(1, std::to_string(smoothAccel).c_str());
	//distIncFinal *= 0.7f * InertiaForce3p;

	return smoothAccel;
}

void updateCamRacing3P()
{
	float calcHeigthOffset = heightOffset3p + 0.15f + heightIcrementCalc;

	currentTowHeightIncrement = lerp(currentTowHeightIncrement, towHeightIncrement, 1.45f * getDeltaTime());
	currentTowLongitudeIncrement = lerp(currentTowLongitudeIncrement, towLongitudeIncrement, 1.75f * getDeltaTime());

	float airDistance = lerp(0.f, 2.5f, smoothIsInAirNfs * (lerp(0.6f, 1.2f, smoothIsInAirNfs)));
	float vehSpeedCoeff = 1.f - clamp01(vehSpeed * 0.01f);

	Vector3f posCenter = vehPos + (up * calcHeightOffset3P);

	float smoothAccel = CalcSmoothAccel();
	float smoothAccelSlow = CalcSmoothAccelSlow();

	float accelDiff = smoothAccelSlow - smoothAccel;

	smoothAccelSlower = lerp(smoothAccelSlower, smoothAccelSlow, 3.6f * getDeltaTime());

	accelDiffSlow = lerp(accelDiffSlow, accelDiff, 3.02f * getDeltaTime());

	Vector3f targetPos = vehPos + ((calcHeightOffset3P + currentTowHeightIncrement + calcHeigthOffset) * up);

	float speedFactor = clamp01(unlerp(8.f, 20.f, vehSpeed));
	float speedFactor2 = clamp01(unlerp(0.f, 15.f, vehSpeed));
	float speedFactor3 = clamp01(unlerp(0.f, 20.f, vehSpeed));

	Vector3f veloTarget = vehForwardVector;
	float veloForwardFactor = vehVelocity.dot(vehForwardVector);

	float veloTargetWeight = veloForwardFactor * 0.00025f * (1.f - smootherIsInAirStep);
	//float veloLerpFactor = unlerp(0.8f, 0.0f, clamp01(veloForwardFactor));

	//Vector3f targetB = lerp(targetPos, vehPos, speedFactor2);
	//Vector3f targetA = lerp(prevCamPos, prevVehPos, speedFactor2);

	// soft limit rotation speed
	float minMult = clamp01(0.0f);
	float maxMult = clamp01(10.f);
	float softLimitFactor = lerp(minMult, maxMult, easeInCubic(clamp01(unlerp(1.f, 0.95f, VeloRotDiff))));

	showText(1, std::to_string(VeloRotDiff).c_str());
	showText(2, std::to_string(softLimitFactor).c_str());

	//Vector3f targetB = lerp(vehPos, (Vector3f)(targetPos + (vehForwardVector * 2.f)), softLimitFactor);
	//Vector3f targetA = lerp((Vector3f)(prevVehPos + (PrevCamQuat * back * 0.33f)), prevVehPos, speedFactor2 * (1.f - smootherIsInAirStep));
	//targetA = lerp(targetA, (Vector3f)(prevCamPos + (vehForwardVector * 2.f)), softLimitFactor);

	Vector3f targetB = vehPos;
	Vector3f targetA = lerp((Vector3f)(prevVehPos + (PrevCamQuat * back * 0.33f)), prevVehPos, speedFactor2 * (1.f - smootherIsInAirStep));

	Vector3f auxVelocityDir = (targetB + (veloTarget * veloTargetWeight)) - (targetA - (veloTarget * veloTargetWeight));

	Vector3f airDir = (targetPos + (vehForwardVector * 2.f)) - (prevCamPos + (vehForwardVector * 2.f));

	velocityDir = lerp(auxVelocityDir, airDir, max(softLimitFactor * getDeltaTime(), smootherIsInAirStep));

	VeloRotDiff = prevVelocityDir.normalized().dot(velocityDir.normalized());

	prevVelocityDir = velocityDir;

	dirQuat3P = lookRotation(velocityDir, up);

	Vector3f vehVelocityNormalized = vehVelocity.normalized();

	Vector3f VeloEdit = vehVelocityNormalized;

	if (vehSpeed < 0.25f)
		VeloEdit = vehForwardVector;
	//else if (vehVelocityNormalized.dot(vehForwardVector) <= 0.025f)
	//	VeloEdit = -VectorReflect(-vehVelocityNormalized, -vehForwardVector);

	//float dot = VeloEdit.dot(vehForwardVector);

	//float lerpSpeedVelo = lerp(5.0f, 60.0f, speedFactor3);
	//float lerpSpeedDir = lerp(0.f, 10.0f, speedFactor3);
	
	
	float lerpSpeedDir = 60.f;


	//// soft limit rotation speed
	//float minMult = 1.0f;
	//float maxMult = 0.75f;

	//prevRotDiff = smoothVeloDir.normalized().dot(velocityDir.normalized());
	//float softLimitMult = lerp(minMult, maxMult, clamp01(unlerp(1.f, 0.975f, prevRotDiff)));

	//showText(1, std::to_string(prevRotDiff).c_str());
	//showText(2, std::to_string(softLimitMult).c_str());
	//showText(3, std::to_string(lerpSpeedDir * softLimitMult).c_str());

	//smoothVeloDir = lerp(smoothVeloDir, velocityDir, clamp01((lerpSpeedDir/* * softLimitMult*/) * getDeltaTime()));
	smoothVeloDir = velocityDir;

	//Vector3f finalDir = lerp(smoothVeloDir, smoothVeloEdit, speedFactor3 * 0.5f * (1.f - smootherIsInAirStep));
	Vector3f finalDir = smoothVeloDir;

	Quaternionf veloCompAux = lookRotation(finalDir);
	//veloCompQuat3P = slerp(veloCompQuat3P, veloCompAux, lerpSpeed * getDeltaTime());
	veloCompQuat3P = veloCompAux;

	//Quaternionf veloRotCompAux = lookRotation(veloRotComp);
	//veloRotQuat3P = slerp(veloRotQuat3P, veloRotCompAux, lerpSpeed * getDeltaTime());
	veloRotQuat3P = veloCompQuat3P;

	Vector3f V3CurrentTowHeightIncrement = up * currentTowHeightIncrement;

	bool lookBehind = false;
	if (CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind) || isLookingBack)
		lookBehind = true;

	float lookHorizontalAngle = 0.f;

	if (!lookBehind) {
		lookHorizontalAngle = RelativeLookFactor < 0.f ?
			lerp(0.f, -LookLeftAngle3p, -RelativeLookFactor)
			:
			lerp(0.f, LookRightAngle3p, RelativeLookFactor)
			;
	}
	else
	{
		lookHorizontalAngle = 180.f;
	}

	Quaternionf AirQuat3P = dirQuat3P;

	bool horizontalLooking = !AreFloatsSimilar(0.f, lookHorizontalAngle);

	if (isAiming || hasInputThisFrame || horizontalLooking)
	{
		if (timerResetLook <= 0.00001f)
		{
			lookQuat = AirQuat3P;
		}
		timerResetLook = 2.f;

		Vector2f inputXY = readLookAroundInput();
		Vector3f vecLook = Vector3f(inputXY.y(), 0.f, inputXY.x());

		Quaternionf result = lookQuat * QuatEuler(vecLook);
		Vector3f resultEuler = QuatToEuler(result);

		float rx = clamp(resultEuler[0], -62.f, 40.f);

		lookQuat = QuatEuler(Vector3f(rx, 0.f, resultEuler[2]));
	}

	if (horizontalLooking)
	{
		float leftRightRad = lookHorizontalAngle * DEG_TO_RAD;

		float roll = 0.f, pitch = 0.f, yaw = leftRightRad;
		Quaternionf qLookLeftRight;
		qLookLeftRight = AngleAxisf(roll, Vector3f::UnitX())
			* AngleAxisf(pitch, Vector3f::UnitY())
			* AngleAxisf(yaw, Vector3f::UnitZ());

		//lookQuat = smoothQuat3P * qLookLeftRight;
		lookQuat = veloCompQuat3P * qLookLeftRight;
	}

	bool switchBack = false;

	float factorLook = clamp01(timerResetLook + abs(RelativeLookFactor) + (lookBehind ? 1.f : 0.f));

	timerResetLook = clamp(timerResetLook - getDeltaTime(), 0.f, 2.f);

	if (smoothIsAiming > 0.00001f) {
		float currentFov = lerp(fov3P, fov3PAiming, smoothIsAiming);
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	float aimHeightIncrement = lerp(0.f, 0.22f, smoothIsAiming);
	float pivotInfluenceLook = lerp(finalPivotFrontOffset, -0.2f, clamp01(abs(lookHorizontalAngle * 0.00277f))) * 1.f - smootherIsInAirStep;

	Quaternionf compositeQuat = slerp(veloCompQuat3P, AirQuat3P, smootherIsInAirStep);

	if (timerResetLook > 0.001f || horizontalLooking)
		compositeQuat = slerp(compositeQuat, lookQuat, clamp01(timerResetLook));

	//Vector3f camPosCam = posCenter + V3CurrentTowHeightIncrement + ((compositeQuat)*back * (calcLongitudeOffset3P + currentTowLongitudeIncrement + (airDistance - finalPivotFrontOffset) + distIncFinal)) + (up * (aimHeightIncrement + calcHeigthOffset/* + heightInc */));
	Vector3f camPosCam = posCenter + V3CurrentTowHeightIncrement + ((compositeQuat)* back * (calcLongitudeOffset3P + currentTowLongitudeIncrement + (airDistance - finalPivotFrontOffset) /*+ distIncFinal*/)) + (up * (aimHeightIncrement + calcHeigthOffset/* + heightInc */));

	prevCamPos = camPosCam;
	Vector3f camPosFinal = camPosCam;

	 // calc rot

	Vector3f rotEuler;

	rotEuler = QuatToEuler(compositeQuat);
		PrevCamQuat = compositeQuat;


	rotEuler[1] = 0.f;

	//float angleInc = distIncFinal < 0.f ? distIncFinal * 10.f : distIncFinal * 6.f;

	Vector3f realFinalRot = Vector3f(rotEuler.x() /*+ angleInc*/ - cameraAngle3p, rotEuler.y(), rotEuler.z());

	Vector3f vehRotSmoothEuler = QuatToEuler(veloCompQuat3P);

	Vector3f camForward = veloCompQuat3P * front;

	//camPosSmooth = lerp(camPosSmooth, camPosFinal, 10.f * getDeltaTime());

	//float upDist = distanceOnAxisNoAbs(camPosFinal, camPosSmooth, camUp);
	//smoothUpDist = lerp(smoothUpDist, clamp(-upDist, -0.2f, 0.2f), 4.8f * getDeltaTime());

	//showText(1, std::to_string(smoothUpDist).c_str());

	//float aimFactor = 1.f - clamp01(timerResetLook);

	//Vector3f posOffset = (camUp * smoothUpDist * aimFactor);
	//Vector3f realSmoothPos = camPosFinal + posOffset;

	// Raycast //
	int ray = WORLDPROBE::_START_SHAPE_TEST_RAY(posCenter.x(), posCenter.y(), posCenter.z(), camPosFinal.x(), camPosFinal.y(), camPosFinal.z(), 1, veh, 7);

	Vector3 endCoords, surfaceNormal;
	BOOL hit;
	Entity entityHit = 0;

	WORLDPROBE::GET_SHAPE_TEST_RESULT(ray, &hit, &endCoords, &surfaceNormal, &entityHit);

	if (hit) {
		setCamPos(customCam, toV3f(endCoords) + (camForward * 0.1f));
	}
	else
	{
		setCamPos(customCam, camPosFinal);
	}
	// End raycast //


	CAM::SET_CAM_ROT(customCam, realFinalRot.x(), realFinalRot.y(), realFinalRot.z(), 2);

	prevVehPos = vehPos;
}

void updateCustomCamera() 
{
	if (currentCam == eCamType::Racing3P) {
		updateCamRacing3P();
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
	if (IsKeyJustUp(str2key(toggleModKey), true)) {
		customCamEnabled = !customCamEnabled;
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

			if (IsKeyJustUp(str2key(reloadKey), true)) {
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
