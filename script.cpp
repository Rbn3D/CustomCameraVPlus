/*
*  Custom Camera V Plus
*/
#include "script.h"


double smoothIsMouseLooking = 0.;
double viewLock = 0.;
double smoothViewLock = 0.;

BOOL modEnabled = true;
BOOL camInitialized = false;
Vehicle veh;
Ped playerPed;

Vector3d prevVehPos;
Vector3d vehPos;
Vector3d vehRot;
Vector3d vehVelocity;
double vehSpeed;
uint16_t vehGear;
double lastVelocityMagnitude;

Vector3d vehForwardVector;
Vector3d vehRightVector;
Vector3d vehUpVector;
Vector3d vehRelativeSpeedVector;
Vector3d vehAngularVelocity;
Vector3d smoothVehAngularVelocity;

//Vector3d smoothVehRightVector;

double vehAcceleration = 0.;
double smoothAccelerationFactor = 0.;
double prevVehAcceleration = 0.;

eVehicleClass vehClass;
bool vehHasTowBone = false;
bool vehHasTrailerBone = false;

//double pivotFrontOffset = 0.365;
//double pivotFrontOffsetHighSpeed = 0.0;
double pivotFrontOffsetStraight = 0.;
double pivotFrontOffsetTurn = 0.;
double finalPivotFrontOffset = 0.;

double smoothAngularDiff = 0.;
double smoothAuxLerpFactor = 0.;

Camera customCam = NULL;
//double fov3P = 85.;
//double fov1P = 90.;
double fov3P = 77.5;
double fov1P = 75.;
double fov1PAiming = 60.;
double fov3PAiming = 60.;
double distanceOffset3p = 0.;
double heightOffset3p = 0.;
double cameraAngle3p = 3.5;
double InertiaForce3p = 1.;
const double PI = 3.1415926535897932;
int lastVehHash = -1;
bool isBike = false;

bool isInVehicle = false;

double calcLongitudeOffset3P = 0.;
double calcHeightOffset3P = 0.;

double heightIcrementCalc = 0.;

double rotationSpeed3P = 4.75;

//bool useVariableRotSpeed3P = true;
//double minRotSpeed3P = 4.55;
//double maxRotSpeed3P = 7.6;
//double maxRotSpeedAngle3P = 90.0;
//
//double variableRotLerpSpeed = 1.9;

bool useVariableRotSpeed3P = true;
double RotSpeedFast3P = 4.7;
double RotSpeedSlow3P = 3.8;
double maxRotSpeedAngle3P = 90.0;

double smoothFactor = 0.;

//double variableRotLerpSpeed = 7.75;

double currentRotSpeed3P = 2.0;

double smoothSpeedFactor = 0.;

double useRoadSurfaceNormal = false;
Vector3d cachedSurfaceNormal;
Vector3d smoothSurfaceNormal;

double smoothCurveEval = 0.;
double smoothCurveEvalLat = 0.;

Vector3d smoothVehForwardVector = Vector3d();

Vector3d smoothVelocity = Vector3d();
Vector3d ultraSmoothVelocity = Vector3d();
Quaterniond dirQuat3P = Quaterniond();
Quaterniond veloQuat3P = Quaterniond();
Quaterniond veloCompQuat3P = Quaterniond();
Quaterniond smoothQuat3P = Quaterniond();
Quaterniond ultraSmoothVelocity3P = Quaterniond();
Vector3d smoothRotSeat = Vector3d();
Quaterniond smoothQuatSeat = Quaterniond();
Quaterniond PrevCamQuat = Quaterniond();

Quaterniond freeLookQuat = Quaterniond();

double smoothIsInAir = 0.;
double smootherIsInAir = 0.;
double smootherIsInAirStep = 0.;
double smoothIsInAirNfs = 0.;
double maxHighSpeed = 130.;
double maxHighSpeedDistanceIncrement = 1.45;
double accelerationCamDistanceMultiplier = 1.45;
Vector3d playerVehOffset;

double freeLookAtPreviousFrame = false;

Vector3d velocityDir = Vector3d();
Vector3d prevVelocityDir = Vector3d();

double VeloRotDiff = 0.;

Vector3d smoothVelocityDir;

Vector3d up(0.0, 0.0, 1.0);
Vector3d down(0.0, 0.0, -1.0);
Vector3d back(0.0, -1.0, 0.0);
Vector3d front(0.0, 1.0, 0.0);
Vector3d right(1.0, 0.0, 0.0);

Ewma forwInfX(0.01);
Ewma forwInfY(0.01);
Ewma forwInfZ(0.01);

Ewma smDirX(0.01);
Ewma smDirY(0.01);
Ewma smDirZ(0.01);

Ewma smPosX(0.01);
Ewma smPosY(0.01);
Ewma smPosZ(0.01);


Ewma smAccel(0.419);

Vector2i lastMouseCoords;
double mouseMoveCountdown = 0.;

bool isAiming = false;
double smoothIsAiming = 0.;

bool customCamEnabled = true;
float timeInVehicle = 0.f;

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

double towLongitudeIncrement = .0;
double currentTowLongitudeIncrement = .0;

double towHeightIncrement = .0;
double currentTowHeightIncrement = .0;

double lowUpdateTimerInterval = 1.0;
double lowUpdateTimerCurrentTime = lowUpdateTimerInterval;

Vehicle lastTowVehicle;
double lastTowVehicleLongitude = 0.;
Vehicle lastTrailer;
double lastTrailerLongitude = 0.;

double smoothRadarAngle = 0.;
double extraAngleCamHeight = 0.;
double relAngle3p = 0.;

const double DEG_TO_RAD = 0.0174532925;
const double RADIAN_TO_DEG = 57.29577951;

double LookLeftAngle1p = 75.0;
double LookRightAngle1p = 80.0;

double LookLeftAngle3p = 90.0;
double LookRightAngle3p = 90.0;

bool InertiaAffectsPitch3p = false;
bool SmartHeadingEnabled = true;
double SmartHeadingIntensity = 1.;
double smoothSmartHeading = 0.;
double smoothSmartHeading2 = 0.;

bool InertiaEffects1p = true;

double semiDelayedVehSpeed = 0.;
double delayedVehSpeed = 0.;

Vector3d prevCamPos = Vector3d();
Vector3d camPosSmooth = Vector3d();

double smoothLatFactor = 0.;

double smoothLatDist = 0.;
double smoothLongDist = 0.;
double smoothUpDist = 0.;

//double smoothTurnForce3P = 0.;
double smoothAngular1 = 0.;
double smoothAngular2 = 0.;
double smoothAngular3 = 0.;

bool isLookingBack = false;

double fixedLookTimer = 0.;
double freeLookTimer = 0.;
double delayFreeLookTimer = 0.;

Quaterniond lookQuat;

double mouseDeltaX = 0.;
double mouseDeltaY = 0.;

double deadzone = 0.;
bool gamepadAimEasing = true;

double gamepadSensibility = 1.;
double mouseSensibility = 1.;

// look left = -1.; lookRight = 1.; Center = 0. (For smooth transitions)
double RelativeLookFactor = 0.;

double prevLookHorizontalAngle = 0.;

bool readInputFromMt = true;
double vehDelayedAccel1 = 0.;
double vehDelayedAccel2 = 0.;
double vehDelayedAccel3 = 0.;
double vehDelayedAccel4 = 0.;
double smoothIsGoingForwardInc = 0.;

double smoothAccelSlower = 0.;
double accelDiffSlow = 0.;
double smoothAngVelFactor = 0.;

Vector3d smoothVehForward;
Vector3d smoothVeloEdit;

Vector3d smoothFinalDir;

char * reloadKey = "F10";
char * toggleModKey = "1";
char * lookLeftKey = "B";
char * lookRightKey = "N";

bool hasInputThisFrame = false;
bool hasMouseInputThisFrame = false;

bool AreFloatsSimilar(double a, double b)
{
	return fabs(a - b) < 0.01;
}

double getDeltaTime() {
	return SYSTEM::TIMESTEP();
	//return (double)GAMEPLAY::GET_FRAME_TIME();
}

std::string SubstringOfCString(const char *cstr,
	size_t start, size_t length)
{
	return std::string(cstr + start, length);
}

double isMouseLooking() {
	return mouseMoveCountdown > 0.001;
}

void ResetMouseLook() {
	mouseMoveCountdown = 0.;
	smoothIsMouseLooking = 0.;
}

double mathRepeat(double t, double length)
{
	return t - floor(t / length) * length;
}

double clamp01(double t) {
	if ((double)t < 0.0)
		return 0.0;
	if ((double)t > 1.0)
		return 1.;
	return t;
}

double clamp(double n, double lower, double upper) {
	return max(lower, min(n, upper));
}

double DeltaAngle(double current, double target)
{
	double num = mathRepeat(target - current, 360.);
	if (num > 180.)
	{
		num -= 360.;
	}
	return num;
}

double InertialDamp(double previousValue, double targetValue, double smoothTime)
{
	double x = previousValue - targetValue;
	double newValue = x + getDeltaTime() * (-1. / smoothTime * x);
	return targetValue + newValue;
}

double InertialDampAngle(double previousValue, double targetValue, double smoothTime)
{
	double x = DeltaAngle(previousValue, targetValue);
	double newValue = x + getDeltaTime() * (-1. / smoothTime * x);
	return targetValue + newValue;
}

double distanceOnAxis(Vector3d A, Vector3d B, Vector3d axis) {
	//The key is to normalize the arrow vector, so it doesn't scale your distances.
	axis.normalize();
	double ADistanceAlongAxis = axis.dot(A);
	double BDistanceAlongAxis = axis.dot(B);

	return abs(BDistanceAlongAxis - ADistanceAlongAxis);
}

double distanceOnAxisNoAbs(Vector3d A, Vector3d B, Vector3d axis) {
	//The key is to normalize the arrow vector, so it doesn't scale your distances.
	axis.normalize();
	double ADistanceAlongAxis = axis.dot(A);
	double BDistanceAlongAxis = axis.dot(B);

	return BDistanceAlongAxis - ADistanceAlongAxis;
}

double easeInCubic(double t) {
	return pow(t, 3.);
}

double easeInCirq(double t) {
	return -1 * (sqrt(1 - t * t) - 1);
}

double easeOutCubic(double t) {
	return 1. - easeInCubic(1. - t);
}

double easeInSine(double t) {
	return -1. * cos(t / 1. * (PI * 0.5)) + 1.;
}

double easeOutSine(double t) {
	return sin(t / 1 * (PI * 0.5));
}

// Range -1. | 1.
double easeInSineInput(double axisInput)
{
	axisInput = clamp(axisInput, -1., 1.);

	double easeAbs = easeInSine(abs(axisInput));

	if (axisInput < 0.)
		easeAbs *= -1.;

	return easeAbs;
}

Vector2d readLookAroundInput()
{
	double mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -5.;
	double my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * (LastInputMethodWasMouseAndKeyboard ? -5. : 5.);

	if (!LastInputMethodWasMouseAndKeyboard())  // if gamepad
	{
		mx *= 0.2;
		my *= 0.2;

		// apply deadzone
		Vector2d stickInput = Vector2d(mx, my);
		if (stickInput.norm() < deadzone)
			stickInput = Vector2d(0., 0.);
		else
			stickInput = stickInput.normalized() * ((stickInput.norm() - deadzone) / (1. - deadzone));

		mx = stickInput.x();
		my = stickInput.y();


		// apply easing 
		if (gamepadAimEasing) {
			mx = easeInSineInput(mx);
			my = easeInSineInput(my);
		}

		// Scale (sensibility)
		mx *= 2. * gamepadSensibility;
		my *= 2. * gamepadSensibility;
	}
	else
	{
		mx *= mouseSensibility;
		my *= mouseSensibility;
	}

	return Vector2d(mx, my);
}

Quaterniond negateQuat(Quaterniond q) {
	return Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
}

double Vector3Angle(Vector3d from, Vector3d to)
{
	double dot = from.normalized().dot(to.normalized());
	return (double)((acos(dot)) * (180.0 / PI));
}

Vector3 getRightVector(Vector3d rotation)
{
	double num = cos(rotation.y() * (PI / 180.0));
	Vector3 vec;

	vec.x = cos(-rotation.z() * (PI / 180.0)) * num;
	vec.y = sin(rotation.z() * (PI / 180.0)) * num;
	vec.z = sin(-rotation.y() * (PI / 180.0));

	return vec;
}

Vector3d QuatToEuler(Quaterniond q) 
{
	double r11 = -2 * (q.x() * q.y() - q.w() * q.z());
	double r12 = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
	double r21 = 2 * (q.y() * q.z() + q.w() * q.x());
	double r31 = -2 * (q.x() * q.z() - q.w() * q.y());
	double r32 = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();

	double ax = asinf(r21);
	double ay = atan2f(r31, r32);
	double az = atan2f(r11, r12);

	const double f = 360.0 / 2.0 / 3.1415926535897;
	ax *= f;
	ay *= f;
	az *= f;

	Vector3d ret = Vector3d(ax, ay, az);

	return ret;
}

void SET_CAM_QUATERNION(Camera c, Quaterniond q)
{
	//Matrix3d rot = q.toRotationMatrix();

	//double ax = asinf(rot(1,0));
	//double ay = atan2f(rot(2,0), rot(2,1));
	//double az = atan2f(rot(0,0), rot(0,1));

	Vector3d ret = QuatToEuler(q);

	CAM::SET_CAM_ROT(c, ret.x(), ret.y(), ret.z(), 2);
}

Vector3d toV3f(const Vector3 &vec) {
	return Vector3d((double)vec.x, (double)vec.y, (double)vec.z);
}

std::string V3ToStr(const Vector3d& vec) {
	return "(" + std::to_string(vec.x()) + ", " + std::to_string(vec.y()) + ", " + std::to_string(vec.z()) + ")";
}

Vector3 toV3(Vector3d vec) {
	Vector3 ret;
	ret.x = (float)vec.x();
	ret.y = (float)vec.y();
	ret.z = (float)vec.z();

	return ret;
}


double RadToDeg(const double &rad)
{
	return double(rad / M_PI * 180.0);
}

double DegToRad(const double &deg)
{
	return double(deg * M_PI / 180.0);
}

Quaterniond QuatEuler(Vector3d &euler)
{
	Vector3d rotVec = DegToRad(1.0) * euler;

	Matrix3d xRot = AngleAxisd(rotVec.x(), Vector3d(1.0, 0.0, 0.0)).matrix();
	Matrix3d yRot = AngleAxisd(rotVec.y(), Vector3d(0.0, 1.0, 0.0)).matrix();
	Matrix3d zRot = AngleAxisd(rotVec.z(), Vector3d(0.0, 0.0, 1.0)).matrix();

	Matrix3d rot = zRot * yRot * xRot;

	return Quaterniond(rot);
}

Quaterniond GET_CAM_QUATERNION(Camera c)
{
	Vector3d camVec = toV3f(CAM::GET_CAM_ROT(c, 0));
	return QuatEuler(camVec);
}

Vector2i getMouseCoords() {
	int mX = CONTROLS::GET_CONTROL_VALUE(0, 239) - 127 / 127. * 1280;
	int mY = CONTROLS::GET_CONTROL_VALUE(0, 240) - 127 / 127. * 720;

	return Vector2i(mX, mY);
}

void updateMouseState() 
{
	double x = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight);
	double y = CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown);

	hasInputThisFrame = 
		(abs(x) > 0.005)
		||
		(abs(y) > 0.005);
}

// Projects a vector onto another vector.
Vector3d Project(Vector3d vector, Vector3d onNormal)
{
	double sqrMag = onNormal.dot(onNormal);
	if (sqrMag < DBL_EPSILON)
		return Vector3d();
	else
		return onNormal * vector.dot(onNormal) / sqrMag;
}

// Projects a vector onto a plane defined by a normal orthogonal to the plane.
Vector3d ProjectOnPlane(Vector3d vector, Vector3d planeNormal)
{
	return vector - Project(vector, planeNormal);
}

void setGameplayCamRelativeRotation(double heading) {
	CAM::SET_GAMEPLAY_CAM_RELATIVE_HEADING(relAngle3p);
	CAM::SET_GAMEPLAY_CAM_RELATIVE_PITCH(0., 1.);

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

		distanceOffset3p = (double)ini.GetDoubleValue("3rdPersonView", "distanceOffset", 0.0);
		heightOffset3p = (double) ini.GetDoubleValue("3rdPersonView", "heightOffset", 0.0);
		cameraAngle3p = clamp((double)ini.GetDoubleValue("3rdPersonView", "cameraAngle", 3.5), 0., 20.);

		fov3P = (double) ini.GetDoubleValue("3rdPersonView", "fov", 77.5);
		fov1P = (double) ini.GetDoubleValue("1stPersonView", "fov", 75.0);

		LookLeftAngle3p = (double) ini.GetDoubleValue("3rdPersonView", "lookLeftAngle", 90.0);
		LookRightAngle3p = (double)ini.GetDoubleValue("3rdPersonView", "lookRightAngle", 90.0);
		
		//InertiaAffectsPitch3p = ini.GetLongValue("3rdPersonView", "InertiaAffectsPitch", 0) > 0;
		InertiaForce3p = (double)ini.GetDoubleValue("3rdPersonView", "InertiaForce", 1.0);

		SmartHeadingEnabled = ini.GetLongValue("3rdPersonView", "SmartHeading", 1l) > 0;
		SmartHeadingIntensity = (double)ini.GetDoubleValue("3rdPersonView", "SmartHeadingIntensity", 1.0);

		InertiaEffects1p = ini.GetLongValue("1stPersonView", "InertiaEffects", 1) > 0;

		LookLeftAngle1p = (double)ini.GetDoubleValue("1stPersonView", "lookLeftAngle", 75.0);
		LookRightAngle1p = (double)ini.GetDoubleValue("1stPersonView", "lookLeftAngle", 80.0);

		readInputFromMt = ini.GetLongValue("general", "GetInputFromGearsAsi", 1l) > 0;

		deadzone = clamp01((double)ini.GetDoubleValue("input", "gamepadDeadzone", 0.0));
		gamepadAimEasing = ini.GetLongValue("input", "gamepadEasing", 1l) > 0;

		gamepadSensibility = (double)ini.GetDoubleValue("input", "gamepadSensibility", 1.0);
		mouseSensibility = (double)ini.GetDoubleValue("input", "mouseSensibility", 1.0);

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
void showText(double x, double y, double scale, std::string text, int font, const Color &rgba, bool outline) {
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
	showText(0.02, 0.02 * (double)index, 0.225, (char*)text.c_str(), 0, solidWhite, true);
}

double getVehicleAcceleration() {
	double mag = vehVelocity.norm();
	double ret = (mag - lastVelocityMagnitude) * getDeltaTime();

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

Vector3d getCameraForwardVector(Camera cam) {
	Vector3d rotation = toV3f(CAM::GET_CAM_ROT(cam, 2));

	double num1 = (double)rotation.x() / (180.0 / PI);
	double num2 = (double)rotation.z() / (180.0 / PI);
	double num3 = abs(cos(num1));
	return Vector3d((double)-(sin(num2) * num3), (double)(cos(num2) * num3), (double)sin(num1));
}

Vector3d getCameraRightVector(Camera cam) {
	Vector3d rotation = toV3f(CAM::GET_CAM_ROT(cam, 2));

	return toV3f(getRightVector(rotation));
}

double dot(Vector3d a, Vector3d b)  //calculates dot product of a and b
{
	return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

double mag(Vector3d a)  //calculates magnitude of a
{
	return std::sqrt(a.x() * a.x() + a.y() * a.y() + a.z() * a.z());
}

double AreSameVector(Vector3d v1, Vector3d v2)
{
	return AreFloatsSimilar(v1.x(), v2.x()) &&
		AreFloatsSimilar(v1.y(), v2.y()) &&
		AreFloatsSimilar(v1.z(), v2.z());
}

double AngleBetweenVectors(Vector3d v1, Vector3d v2)
{
	if (AreSameVector(v1, v2))
		return 0.;

	return std::acos(dot(v1, v2) / (mag(v1)*mag(v2)));
}

double unlerp(double min, double max, double value) {
	return (value - min) / (max - min);
}

double lerp(double a, double b, double f)
{
	return a + f * (b - a);
}

double damp(double a, double b, double lambda, double dt)
{
	return lerp(a, b, 1. - exp(-lambda * dt));
}

Vector3d lerp(Vector3d& v1, Vector3d& v2, double t)
{
	return Vector3d(lerp(v1.x(), v2.x(), t),
		lerp(v1.y(), v2.y(), t),
		lerp(v1.z(), v2.z(), t));
}

Vector3d damp(Vector3d& a, Vector3d& b, double lambda, double dt)
{
	return lerp(a, b, 1 - exp(-lambda * dt));
}

double MAX_ANGLE = 360.;

double mod(double a, double b)
{
	return fmod((fmod(a,b) + b), b);
}

double NormalizeAngle(double angle) {
	//while (angle < 0)
	//	angle += MAX_ANGLE;
	//while (angle >= MAX_ANGLE)
	//	angle -= MAX_ANGLE;
	//return angle;

	return mod(angle, MAX_ANGLE);
}

double lerpAngle(double a, double b, double t)
{
	double num = mathRepeat(b - a, 360.);
	if (num > 180.)
	{
		num -= 360.;
	}
	return a + num * clamp01(t);
}

double lerpAngle360(double start, double end, double amount)
{
	double difference = abs(end - start);
	if (difference > 180.)
	{
		// We need to add on to one of the values.
		if (end > start)
		{
			// We'll add it on to start...
			start += 360.;
		}
		else
		{
			// Add it on to end.
			end += 360.;
		}
	}

	// Interpolate it.
	double value = (start + ((end - start) * amount));

	// Wrap it..
	double rangeZero = 360.;

	if (value >= 0 && value <= 360)
		return value;

	return fmod(value, rangeZero);
}

Vector3d Vector3dLerpAngle(Vector3d& v1, Vector3d& v2, double t)
{
	return Vector3d(lerpAngle(v1.x(), v2.x(), t),
		lerpAngle(v1.y(), v2.y(), t),
		lerpAngle(v1.z(), v2.z(), t));
}

Vector3d Vector3dInertialDamp(Vector3d& v1, Vector3d& v2, double t)
{
	return Vector3d(InertialDamp(v1.x(), v2.x(), t),
		InertialDamp(v1.y(), v2.y(), t),
		InertialDamp(v1.z(), v2.z(), t));
}

Vector3d Vector3dInertialDampAngle(Vector3d& v1, Vector3d& v2, double t)
{
	return Vector3d(InertialDampAngle(v1.x(), v2.x(), t),
		InertialDampAngle(v1.y(), v2.y(), t),
		InertialDampAngle(v1.z(), v2.z(), t));
}

Quaterniond slerp(Quaterniond& q1, Quaterniond& q2, double time)
{
	return q1.slerp(time, q2);
}

Quaterniond nlerp(Quaterniond& q1, Quaterniond& q2, double time)
{
	return QuatEuler(lerp(QuatToEuler(q1), QuatToEuler(q2), time));
}

double smoothStep(double from, double to, double t)
{
	t = clamp01(t);
	t = (double)(-2.0 * (double)t * (double)t * (double)t + 3.0 * (double)t * (double)t);
	return (double)((double)to * (double)t + (double)from * (1.0 - (double)t));
}

Matrix3d matrixLookAt(Vector3d& dir, Vector3d& up)
{
	dir.normalize();
	Vector3d s(dir.cross(up));
	s.normalize();
	Vector3d u(s.cross(dir));
	u.normalize();

	Matrix3d mat;

	mat <<
		s[0], u[0], -dir[0],
		s[1], u[1], -dir[1],
		s[2], u[2], -dir[2];

	return mat;
	//preMultTranslate(-eye);
}

Quaterniond lookRotation(Vector3d dir) 
{
	// Fix for gta coord system
	// Roll pitch and yaw in Radians (-90.0, 0.0, 0.0)
	double roll = -1.5707963267948966, pitch = 0., yaw = 0.;
	Quaterniond compensation;
	compensation = AngleAxisd(roll, Vector3d::UnitX())
		* AngleAxisd(pitch,  Vector3d::UnitY())
		* AngleAxisd(yaw, Vector3d::UnitZ());

	Matrix3d lookM = matrixLookAt(dir, up);
	Quaterniond ret = Quaterniond(lookM);

	return ret * compensation;
}

Quaterniond lookRotation(Vector3d dir, Vector3d upVector)
{
	// Fix for gta coord system
	//Roll pitch and yaw in Radians (-90.0, 0.0, 0.0)
	double roll = -1.5707963267948966, pitch = 0., yaw = 0.;
	Quaterniond compensation;
	compensation = AngleAxisd(roll, Vector3d::UnitX())
		* AngleAxisd(pitch, Vector3d::UnitY())
		* AngleAxisd(yaw, Vector3d::UnitZ());

	Matrix3d lookM = matrixLookAt(dir, upVector);
	Quaterniond ret = Quaterniond(lookM);

	return ret * compensation;
}

Quaterniond lookRotationEx(Vector3d dir, Vector3d upVector)
{
	Matrix3d lookM = matrixLookAt(dir, upVector);
	Quaterniond ret = Quaterniond(lookM);

	return ret;
}

Vector3d lookRotEuler(Vector3d dir, Vector3d upVector) {
	Matrix3d lookM = matrixLookAt(dir, upVector);

	return lookM.eulerAngles(2, 1, 0);
}

Quaterniond getEntityQuaternion(Entity entity) {

	float x, y, z, w;
	ENTITY::GET_ENTITY_QUATERNION(entity, &x, &y, &z, &w);

	return Quaterniond((double)w, (double)x, (double)y, (double)z);
}

Quaterniond lerpXZ(Quaterniond& q1, Quaterniond& q2, double tX, double tY, double tZ)
{
	Vector3d e1 = QuatToEuler(q1);
	Vector3d e2 = QuatToEuler(q2);

	double x = lerp(e1.x(), e2.x(), tX);
	double y = lerp(e1.y(), e2.y(), tY);
	double z = lerp(e1.z(), e2.z(), tZ);

	Vector3d ret = Vector3d(x, y, z);
	//Vector3d ret = Vector3d(x, 0., z);

	return QuatEuler(ret);
}

void setCamPos(Camera cam, Vector3d pos) {
	CAM::SET_CAM_COORD(cam, pos.x(), pos.y(), pos.z());
}

void camPointAt(Camera cam, Vector3d pos) {
	CAM::POINT_CAM_AT_COORD(cam, pos.x(), pos.y(), pos.z());
}

bool vehHasBone(char *boneName) {
	return (ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, boneName) != -1);
}

void updateVehicleProperties() 
{
	vehClass = (eVehicleClass)VEHICLE::GET_VEHICLE_CLASS(veh);

	Vector3d skelPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "seat_dside_f")));
	Vector3d wheelPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "steeringwheel")));
	Vector3d windscreenPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "windscreen")));

	double playerHeadAltitude;
	double playerHeadDistance;

	if (vehClass == eVehicleClass::VehicleClassCoupes || vehClass == eVehicleClass::VehicleClassSports || vehClass == eVehicleClass::VehicleClassSportsClassics || vehClass == eVehicleClass::VehicleClassMuscle)
	{
		// sports - coupes - muscles
		playerHeadAltitude = 0.675;
		playerHeadDistance = -0.56;
	}
	else if (vehClass == eVehicleClass::VehicleClassSuper)
	{
		// super sport
		playerHeadAltitude = 0.645;
		playerHeadDistance = -0.61;

	}
	else
	{
		// anything else
		playerHeadAltitude = 0.695;
		playerHeadDistance = -0.56;
	}

	skelPos += vehUpVector * playerHeadAltitude;
	skelPos += vehRightVector * distanceOnAxisNoAbs(skelPos, wheelPos, vehRightVector);
	skelPos += vehForwardVector * (distanceOnAxisNoAbs(skelPos, windscreenPos, vehForwardVector) + playerHeadDistance);

	double distSteeringWheel = distanceOnAxisNoAbs(skelPos, wheelPos, vehForwardVector);
	double minDistSteeringWheel = 0.285;

	if (distSteeringWheel < minDistSteeringWheel)
		skelPos += vehForwardVector * (distSteeringWheel - minDistSteeringWheel);


	playerVehOffset = toV3f(ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(veh, skelPos.x(), skelPos.y(), skelPos.z()));

	isBike = vehClass == eVehicleClass::VehicleClassCycles || vehClass == eVehicleClass::VehicleClassMotorcycles;
	isSuitableForCam = vehClass != eVehicleClass::VehicleClassTrains && vehClass != eVehicleClass::VehicleClassPlanes && vehClass != eVehicleClass::VehicleClassHelicopters && vehClass != eVehicleClass::VehicleClassBoats;

	calcLongitudeOffset3P = getVehicleLongitudeFromCenterBack(veh) + 0.7;

	calcHeightOffset3P = clamp(getVehicleHeightFromCenterUp(veh) + 0.5, 0., 2.0);
	//ShowNotification(std::to_string(heightOffset3P).c_str());

	if (calcHeightOffset3P > 1.75)
	{
		extraAngleCamHeight = clamp(lerp(0.1, 2.0, unlerp(1.75, 2.00, calcHeightOffset3P)), 0., 3.0);
		calcHeightOffset3P += (extraAngleCamHeight * 0.5);
		calcLongitudeOffset3P += extraAngleCamHeight;

		extraAngleCamHeight = clamp(extraAngleCamHeight, 0., 2.25);
	}
	else
		extraAngleCamHeight = 0.0;

	if (isBike) {
		calcLongitudeOffset3P += 1.;
		calcHeightOffset3P = 1.38;
	}

	calcLongitudeOffset3P += 1.45 + distanceOffset3p;

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
	vehSpeed = (double)ENTITY::GET_ENTITY_SPEED(veh);
	//smoothVelocity = lerp(smoothVelocity, vehSpeed > 1.25 ? vehVelocity : vehForwardVector, 5.0 * getDeltaTime());
	smoothVelocity = lerp(smoothVelocity, vehVelocity, 5.0 * getDeltaTime());

	// TODO: Refelcted velocity

	ultraSmoothVelocity = lerp(ultraSmoothVelocity, vehSpeed > 2. ? vehVelocity : vehForwardVector, 3. * getDeltaTime());

	if ((ENTITY::IS_ENTITY_IN_AIR(veh) || (ENTITY::GET_ENTITY_UPRIGHT_VALUE(veh) < 0.6)) && smoothIsInAir < 0.001)
	{
		smoothVelocity = getCameraForwardVector(customCam);
		//velocityQuat3P = lookRotation(smoothVelocity);
	}

	bool isInAir = ENTITY::IS_ENTITY_IN_AIR(veh) || ENTITY::IS_ENTITY_UPSIDEDOWN(veh);

	smoothIsInAir = lerp(smoothIsInAir, isInAir ? 1. : 0., 12. * getDeltaTime());
	smootherIsInAir = lerp(smootherIsInAir, isInAir ? 1. : 0., 5. * getDeltaTime());
	smootherIsInAirStep = lerp(smootherIsInAirStep, isInAir ? 1. : 0., lerp(5.25, 1.35, smootherIsInAir) * getDeltaTime());
	smoothIsInAirNfs = lerp(smoothIsInAirNfs, (ENTITY::IS_ENTITY_IN_AIR(veh)) ? 1. : 0., 0.75 * getDeltaTime());
	vehRightVector = toV3f(getRightVector(vehRot));
	vehUpVector = vehRightVector.cross(vehForwardVector);
	vehAcceleration = getVehicleAcceleration();
	vehRelativeSpeedVector = toV3f(ENTITY::GET_ENTITY_SPEED_VECTOR(veh, true));
	vehAngularVelocity = toV3f(ENTITY::GET_ENTITY_ROTATION_VELOCITY(veh));
	smoothVehAngularVelocity = lerp(smoothVehAngularVelocity, vehAngularVelocity, 2. * getDeltaTime());
}

void setupCurrentCamera() {
	if (currentCam == eCamType::DriverSeat1P) {
		ENTITY::SET_ENTITY_ALPHA(playerPed, 0, false);
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.05);
		CAM::SET_CAM_FAR_CLIP(customCam, 740.0);
		CAM::SET_CAM_FOV(customCam, fov1P);
		smoothRotSeat = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		smoothQuatSeat = getEntityQuaternion(veh);
		relAngle3p = 0.;
	}
	else if (currentCam == eCamType::Racing3P) {
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.15);
		CAM::SET_CAM_FAR_CLIP(customCam, 800.0);
		CAM::SET_CAM_FOV(customCam, fov3P);
		viewLock = 0.;

		freeLookTimer = 0.0;
		fixedLookTimer = 0.0;
		smoothIsMouseLooking = 1.0;
		lookQuat = getEntityQuaternion(veh);
		dirQuat3P = lookQuat;
		smoothQuat3P = lookQuat;

		prevCamPos = (vehPos + (up * calcHeightOffset3P)) + (up * (0.14 + extraAngleCamHeight)) + ((lookQuat) * back * (calcLongitudeOffset3P + currentTowLongitudeIncrement));
		camPosSmooth = prevCamPos;
		prevVehPos = vehPos;
		smoothLatDist = 0.;
		smoothCurveEval = 0.;
//		smoothVehRightVector = vehRightVector;
	}

	CAM::SET_FOLLOW_VEHICLE_CAM_VIEW_MODE(1);
}

void setupCustomCamera() {
	customCam = CAM::CREATE_CAM_WITH_PARAMS("DEFAULT_SCRIPTED_CAMERA", vehPos.x(), vehPos.y(), vehPos.z(), vehRot.x(), vehRot.y(), vehRot.z(), fov3P, true, 2);
	CAM::SET_CAM_ACTIVE(customCam, true);
	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, true, false);

	smoothRadarAngle = mathRepeat(CAM::GET_GAMEPLAY_CAM_ROT(2).z, 360.);

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
		smoothQuatSeat = slerp(smoothQuatSeat, getEntityQuaternion(veh), clamp01(30. * getDeltaTime()));

		return;
	}

	Vector3d seatPos;
	if (!isBike) 
	{
		seatPos = toV3f(ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(veh, playerVehOffset.x(), playerVehOffset.y(), playerVehOffset.z()));
		//seatPos = getGameplayCameraPos();
	}
	else
		seatPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "seat_f")));
		

	Vector3d camPos;
	if (isBike)
		camPos = seatPos + (vehUpVector * 0.4) + (vehForwardVector * 0.32); // bike
	else
		camPos = seatPos; // car; // car

	if (smoothIsAiming > 0.00001) {
		double currentFov = lerp(fov1P, fov1PAiming, smoothIsAiming);
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	double distIncFinal = 0.;

	if (InertiaEffects1p) {
		double accelScale = VEHICLE::GET_VEHICLE_ACCELERATION(veh);

		double vehDirectAccel = ((double)(vehAcceleration * accelScale)) * 1700.0;
		vehDelayedAccel1 = lerp(vehDelayedAccel1, vehDirectAccel, 1.725 * getDeltaTime());
		vehDelayedAccel2 = lerp(vehDelayedAccel2, vehDirectAccel, 0.765 * getDeltaTime());

		if (vehSpeed <= 0.02 && vehDelayedAccel1 > vehDelayedAccel2)
			vehDelayedAccel2 = lerp(vehDelayedAccel2, vehDelayedAccel1, 8. * getDeltaTime());

		double accelThreshold = (vehDelayedAccel1 - vehDelayedAccel2);

		distIncFinal = accelThreshold + max(0., vehSpeed * 0.01295) - 0.3;

		distIncFinal *= 0.175;
		distIncFinal = clamp(distIncFinal, -0.1, 0.1);
	}

	double btnLookingFactor = abs(RelativeLookFactor);

	Quaterniond finalQ;

	double wheelieFactor = 0.;

	if (isBike) {
		CAM::STOP_CAM_POINTING(customCam);
		Vector3d rot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		rot[1] = rot.y() * 0.5; // rot.y = rot.y * .5

		//smoothRotSeat = Vector3dLerpAngle(smoothRotSeat, rot, clamp01(30. * getDeltaTime()));
		smoothQuatSeat = getEntityQuaternion(veh);
		veloQuat3P = lookRotation(vehSpeed < 1.25 || vehVelocity.dot(vehForwardVector) <= 0.12 ? vehForwardVector : vehVelocity);

		smoothQuatSeat = slerp(veloQuat3P, smoothQuatSeat, smoothIsInAir);

		wheelieFactor = clamp(vehForwardVector.dot(up), 0., 0.5) * (1. - smoothIsInAir);

		double leftRightAngle = RelativeLookFactor < 0 ?
			lerp(0., -LookLeftAngle1p, -RelativeLookFactor)
			:
			lerp(0., LookRightAngle1p, RelativeLookFactor)
			;

		double leftRightRad = leftRightAngle * DEG_TO_RAD;

		double roll = 0., pitch = 0., yaw = leftRightRad;
		Quaterniond qLookLeftRight;
		qLookLeftRight = AngleAxisd(roll, Vector3d::UnitX())
			* AngleAxisd(pitch, Vector3d::UnitY())
			* AngleAxisd(yaw, Vector3d::UnitZ());

		finalQ = smoothQuatSeat * qLookLeftRight;
	
		//Vector3d veloPlane = ProjectOnPlane(vehSpeed > 0.05 ? vehVelocity : vehForwardVector, vehForwardVector.cross(up)).normalized();
		//double wheelieFactor = veloPlane.dot(vehForwardVector) * (1. - smoothIsInAir) * 5.;

		//double roll1 = wheelieFactor * DEG_TO_RAD, pitch1 = 0., yaw1 = 0.;
		//Quaterniond wheelieCompensation;
		//wheelieCompensation = AngleAxisd(roll1, Vector3d::UnitX())
		//	* AngleAxisd(pitch1, Vector3d::UnitY())
		//	* AngleAxisd(yaw1, Vector3d::UnitZ());

		//finalQ *= wheelieCompensation;

		if (isAiming || hasInputThisFrame)
		{
			if (freeLookTimer < 0.00001)
			{
				lookQuat = finalQ;
			}
			freeLookTimer = 2.;

			Vector2d lookXY = readLookAroundInput();
			Vector3d vecLook = Vector3d(lookXY.y(), 0., lookXY.x());

			Quaterniond result = lookQuat * QuatEuler(vecLook);
			Vector3d resultEuler = QuatToEuler(result);

			double rx = clamp(resultEuler[0], -62., 40.);

			lookQuat = QuatEuler(Vector3d(rx, 0., resultEuler[2]));
		}

		freeLookTimer = clamp(freeLookTimer - getDeltaTime(), 0., 2.);

		finalQ = slerp(finalQ, lookQuat, clamp01(freeLookTimer));

		SET_CAM_QUATERNION(customCam, finalQ);
	}
	else
	{
		CAM::STOP_CAM_POINTING(customCam);

		Vector3d rot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, 2));
		smoothQuatSeat = getEntityQuaternion(veh);


			double leftRightAngle = RelativeLookFactor < 0 ?
					lerp(0., -LookLeftAngle1p, -RelativeLookFactor)
				:
					lerp(0., LookRightAngle1p, RelativeLookFactor)
				;

			double leftRightRad = leftRightAngle * DEG_TO_RAD;

			double roll = 0., pitch = 0., yaw = leftRightRad;
			Quaterniond qLookLeftRight;
			qLookLeftRight = AngleAxisd(roll, Vector3d::UnitX())
				* AngleAxisd(pitch, Vector3d::UnitY())
				* AngleAxisd(yaw, Vector3d::UnitZ());

			finalQ = smoothQuatSeat * qLookLeftRight;

			if (isAiming || hasInputThisFrame)
			{
				if (freeLookTimer < 0.00001)
				{
					lookQuat = finalQ;
				}
				freeLookTimer = 2.;

				double mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -5.;
				double my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * (LastInputMethodWasMouseAndKeyboard ? -5. : 5.);

				if (!LastInputMethodWasMouseAndKeyboard())
				{
					mx *= 0.6;
					my *= 0.6;
				}

				Vector3d vecLook = Vector3d(my, 0., mx);

				Quaterniond result = lookQuat * QuatEuler(vecLook);
				Vector3d resultEuler = QuatToEuler(result);

				double rx = clamp(resultEuler[0], -62., 40.);

				lookQuat = QuatEuler(Vector3d(rx, 0., resultEuler[2]));
			}

			freeLookTimer = clamp(freeLookTimer - getDeltaTime(), 0., 2.);

			finalQ = slerp(finalQ, lookQuat, clamp01(freeLookTimer));

			SET_CAM_QUATERNION(customCam, finalQ);
	}

	camPos = camPos + smoothQuatSeat * back * distIncFinal;

	if (isBike) 
	{
		camPos += wheelieFactor * 0.155 * up;
		camPos += wheelieFactor * 0.540 * -((vehSpeed < 1.25 || vehVelocity.dot(vehForwardVector) <= 0.12 ? vehForwardVector : vehVelocity).normalized());
	}

	CAM::SET_CAM_COORD(customCam, camPos.x(), camPos.y(), camPos.z());

	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, 1, 0);
}

void ProccessLookLeftRightOrBackInput()
{
	bool readFromMtApi = readInputFromMt && MT::Present;

	bool evalLeft = IsKeyDown(str2key(lookLeftKey)) || (readFromMtApi && MT::LookingBack());
	bool evalRight = IsKeyDown(str2key(lookRightKey)) || (readFromMtApi && MT::LookingRight());

	isLookingBack = CONTROLS::IS_CONTROL_PRESSED(0, eControl::ControlLookBehind) || (readFromMtApi && MT::LookingBack()) || (evalLeft && evalRight);

	if (evalLeft)
		RelativeLookFactor = -1;
	else if (evalRight)
		RelativeLookFactor = 1;
	else
		RelativeLookFactor = 0;

	RelativeLookFactor = clamp(RelativeLookFactor, -1., 1.);
}

void lookBehind1p()
{
	Vector3d camPos;

	if (isBike)
		camPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "seat_f"))) + (vehUpVector * 0.3); // bike
	else {
		char *boneName;
		Vector3d offset;

		bool hasBone = false;
		if (vehHasBone("windscreen_r"))
		{
			boneName = "windscreen_r";
			offset = (vehUpVector * 0.08) + (-vehForwardVector * 0.08);
			hasBone = true;

			goto setBonePos;
		}

		if (vehHasBone("bumper_r"))
		{
			boneName = "bumper_r";
			offset = (vehUpVector * 0.18) + (-vehForwardVector * 0.08);
			hasBone = true;

			goto setBonePos;
		}

		if (vehHasBone("brakelight_m"))
		{
			boneName = "brakelight_m";
			offset = (vehUpVector * 0.08) + (-vehForwardVector * 0.08);
			hasBone = true;

			goto setBonePos;
		}

		if (vehHasBone("boot"))
		{
			boneName = "boot";
			offset = vehUpVector * 0.28;
			hasBone = true;

			goto setBonePos;
		}

		 setBonePos:

		if (hasBone) {

			if (VEHICLE::GET_VEHICLE_MOD(veh, 0) >= 0) // Has spoiler?
			{
				offset += vehUpVector * 0.30;
			}

			camPos = GetBonePos(veh, boneName) + offset;
		}
			
		else
		{
			char* boneName1 = "brakelight_l";
			char* boneName2 = "brakelight_l";

			Vector3d pos1 = GetBonePos(veh, boneName1);
			Vector3d pos2 = GetBonePos(veh, boneName2);

			Vector3d posCenter = lerp(pos1, pos2, 0.5);
			offset = (vehUpVector * 0.08) + (-vehForwardVector * 0.08);

			camPos = posCenter + offset; // car
		}
	}

	Quaterniond quat = getEntityQuaternion(veh);

	double roll = 0., pitch = 0., yaw = DegToRad(180.);
	Quaterniond invert180;
	invert180 = AngleAxisd(roll, Vector3d::UnitX())
		* AngleAxisd(pitch, Vector3d::UnitY())
		* AngleAxisd(yaw, Vector3d::UnitZ());

	setCamPos(customCam, camPos);
	CAM::STOP_CAM_POINTING(customCam);
	SET_CAM_QUATERNION(customCam, quat * invert180);
}

Vector3d GetBonePos(Entity entity, char * boneName)
{
	return toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(entity, boneName)));
}

bool LastInputMethodWasMouseAndKeyboard()
{
	return CONTROLS::_IS_INPUT_DISABLED(2);
}

Vector3d V3Reflect(Vector3d vector, Vector3d normal) 
{
	double dot = vector.dot(normal);
	Vector3d temp = normal * dot * 2.;
	return vector - temp;
}

Vector3d VectorReflect(Vector3d vector, Vector3d normal)
{
	// I is the original array
	// N is the normal of the incident plane
	// R = I - (2 * N * ( DotProduct[ I,N] ))
	// inline the dotProduct here instead of calling method
	double dotProduct = ((vector.x() * normal.x()) + (vector.y() * normal.y())) + (vector.z() * normal.z());

	Vector3d reflectedVector = Vector3d(
		reflectedVector.x() = vector.x() - (2.0 * normal.x()) * dotProduct,
		reflectedVector.y() = vector.y() - (2.0 * normal.y()) * dotProduct,
		reflectedVector.z() = vector.z() - (2.0 * normal.z()) * dotProduct
	);

	return reflectedVector;
}

double CalcSmoothAccel()
{
	double vehDirectAccel = vehAcceleration * 125.;

	vehDelayedAccel1 = lerp(vehDelayedAccel1, clamp(vehDirectAccel, -0.46, 0.46), 3.0 * getDeltaTime());
	vehDelayedAccel2 = lerp(vehDelayedAccel2, clamp(vehDirectAccel, -0.46, 0.46), 2.0 * getDeltaTime());

	double accelThreshold = vehDelayedAccel2 - vehDelayedAccel1;

	double smoothAccel = clamp(accelThreshold, -0.50, 0.50)/* + max(0., vehSpeed * 0.01295)*/ * (1. - smoothIsInAir);

	return smoothAccel;
}

double CalcSmoothAccelSlow()
{
	double vehDirectAccel = vehAcceleration * 125.;

	vehDelayedAccel3 = lerp(vehDelayedAccel3, clamp(vehDirectAccel, -0.46, 0.46), 2.55 * getDeltaTime());
	vehDelayedAccel4 = lerp(vehDelayedAccel4, clamp(vehDirectAccel, -0.46, 0.46), 2.0 * getDeltaTime());

	double accelThreshold = vehDelayedAccel3 - vehDelayedAccel4;

	double smoothAccel = clamp(accelThreshold, -0.50, 0.50)/* + max(0., vehSpeed * 0.01295)*/ * (1. - smoothIsInAir);

	return smoothAccel;
}

Vector3d buildMixedDir(Vector3d vehForwardVector, Vector3d dirV)
{
	//double mag = rawDir.norm();

	Vector3d dirN = dirV.normalized();
	Vector3d forwN = vehForwardVector.normalized();

	//forwN[2] = dirN.z();

	return forwN.normalized();
}

void updateCamRacing3P()
{
	double calcHeigthOffset = heightOffset3p + 0.15 + heightIcrementCalc;
	double aimHeightIncrement = lerp(0., 0.35, smoothIsAiming);

	// showText(1, "FPS: " + std::to_string(1.f / GAMEPLAY::GET_FRAME_TIME()));

	currentTowHeightIncrement = lerp(currentTowHeightIncrement, towHeightIncrement, 1.45 * getDeltaTime());
	currentTowLongitudeIncrement = lerp(currentTowLongitudeIncrement, towLongitudeIncrement, 1.75 * getDeltaTime());


	Vector3d posCenter = vehPos + (up * calcHeightOffset3P);
	Vector3d smoothPosCenter = Vector3d
	(
		(double)smPosX.filter(posCenter.x(), getDeltaTime()),
		(double)smPosY.filter(posCenter.y(), getDeltaTime()),
		(double)smPosZ.filter(posCenter.z(), getDeltaTime())
	);

	double distInc = clamp01((smoothPosCenter - posCenter).norm());

	double auxMult = lerp(0.000206, 0.000411, distInc);
	double accelInc = smAccel.filter(vehAcceleration, getDeltaTime());

	//showText(1, std::to_string(vehSpeed));
	//showText(2, std::to_string(distInc));


	Vector3d V3CurrentTowHeightIncrement = up * currentTowHeightIncrement;

	double airDistance = lerp(0., 2.5, smoothIsInAirNfs * (lerp(0.6, 1.2, smoothIsInAirNfs)));
	float speedMult = clamp01(unlerp(0.f, 5.0f, vehSpeed));

	Vector3d targetPos = vehPos + ((calcHeightOffset3P + currentTowHeightIncrement + calcHeigthOffset + aimHeightIncrement) * up);

	Vector3d rawDir = (targetPos - prevCamPos) * getDeltaTime() * 98.0025; // higher values "relax" rotation speed

	Vector3d veloTarget = buildMixedDir(vehForwardVector, (vehPos - (prevVehPos - (rawDir * 0.075 * speedMult))));

	Vector3d normVelo = vehVelocity.normalized();
	if (vehSpeed < 0.25f)
		normVelo = vehForwardVector;

	double veloForwardFactor = (double)normVelo.dot(veloTarget);

	//showText(4, std::to_string(auxMult));
	//showText(5, std::to_string(accelInc));
	//showText(6, std::to_string(auxMult + accelInc));

	double veloTargetWeight = veloForwardFactor * (auxMult + accelInc) * (1. - (smootherIsInAirStep)); // higher values increases influence on forward vector, when it's aligned to velocity (multiplier is usually small [around 0.00X])

	Vector3d targetB = vehPos;
	Vector3d targetA = prevVehPos + (-rawDir);

	Vector3d forwardInfluence = veloTarget * veloTargetWeight;

	Vector3d filteredForwInfluence = Vector3d
	(
		(double)forwInfX.filter(forwardInfluence.x(), getDeltaTime()),
		(double)forwInfY.filter(forwardInfluence.y(), getDeltaTime()),
		(double)forwInfZ.filter(forwardInfluence.z(), getDeltaTime())
	);

	Vector3d auxDir = (targetB + (filteredForwInfluence * speedMult)) - (targetA - (filteredForwInfluence * speedMult));

	Vector3d filteredVelocityDir = Vector3d
	(
		(double)smDirX.filter(auxDir.x(), getDeltaTime()),
		(double)smDirY.filter(auxDir.y(), getDeltaTime()),
		(double)smDirZ.filter(auxDir.z(), getDeltaTime())
	);

	Vector3d airDir = (targetPos + (-rawDir)) - (prevCamPos + (-rawDir));

	velocityDir = lerp(filteredVelocityDir, airDir, smootherIsInAirStep);

	dirQuat3P = lookRotation(velocityDir, up);

	veloCompQuat3P = lookRotation(velocityDir);

	double lookHorizontalAngle = 0.;

	if (isLookingBack) {
		lookHorizontalAngle = 180.;
	}
	else if(abs(RelativeLookFactor) > 0.01)
	{
		lookHorizontalAngle = RelativeLookFactor < 0. ?
			lerp(0., -LookLeftAngle3p, abs(RelativeLookFactor))
			:
			lerp(0., LookRightAngle3p, RelativeLookFactor)
			;
	}

	bool fixedLookAt = isLookingBack || !AreFloatsSimilar(0., RelativeLookFactor);
	bool freeLookAt = isAiming || hasInputThisFrame;
	bool freeLookAtDelay = freeLookAt || delayFreeLookTimer > 0.001;
	bool anyLookAt = fixedLookAt || freeLookAt;

	float mixedLookTimer = clamp01(max(fixedLookTimer, max(freeLookTimer, delayFreeLookTimer > 0.01 ? 1. : 0.)));


	Quaterniond lookAtQuat;
	double aimUpIncrement = 0.;

	if (fixedLookAt)
	{
		fixedLookTimer = 1.f;//clamp(fixedLookTimer + getDeltaTime() * 2., 0, 1);

		//if (fixedLookTimer < 0.05)
		//{
		//	smoothQuat3P = getEntityQuaternion(veh);
		//}
	}
	else if (freeLookAt)
	{
		if (!freeLookAtPreviousFrame)
		{
			freeLookQuat = veloCompQuat3P;
		}

		freeLookAtPreviousFrame = true;

		freeLookTimer = 1.f;//clamp(freeLookTimer + getDeltaTime() * 2., 0, 2);
		delayFreeLookTimer = 1.25;
	}
	else
	{
		fixedLookTimer = 0.; //clamp(fixedLookTimer - getDeltaTime() * 2., 0, 1);
		freeLookTimer = 0.; // clamp(freeLookTimer - getDeltaTime() * 2., 0, 2);
		delayFreeLookTimer = clamp(delayFreeLookTimer - getDeltaTime() * 2., 0, 1.25);
	}

	if (!freeLookAt)
	{
		freeLookAtPreviousFrame = false;
	}

	if (fixedLookAt)
	{
		float positiveAngle = lookHorizontalAngle;

		if (positiveAngle < 0.)
			positiveAngle += 360.;

		double leftRightRad = positiveAngle * DEG_TO_RAD;

		double roll = 0, pitch = 0, yaw = leftRightRad;
		Quaterniond qLookLeftRight;
		qLookLeftRight = AngleAxisd(roll, Vector3d::UnitX())
			* AngleAxisd(pitch, Vector3d::UnitY())
			* AngleAxisd(yaw, Vector3d::UnitZ());

		smoothQuat3P = getEntityQuaternion(veh);
		lookAtQuat = smoothQuat3P * qLookLeftRight;

		//smoothQuat3P = slerp(smoothQuat3P, getEntityQuaternion(veh), 25. * getDeltaTime());
	}
	else if (freeLookAt)
	{
		double mx = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookLeftRight)) * -1.;
		double my = (CONTROLS::GET_CONTROL_NORMAL(2, eControl::ControlLookUpDown)) * -1.;

		double inputMult = LastInputMethodWasMouseAndKeyboard() ? 12 : 3;
		inputMult *= lerp(1., 0.75f, smoothIsAiming);

		mx *= inputMult;
		my *= inputMult;

		Vector3d vecLook = Vector3d(my, 0., mx);
		Quaterniond lookResult = freeLookQuat * QuatEuler(vecLook);

		Vector3d resultEuler = QuatToEuler(lookResult);

		double rx = clamp(resultEuler[0], -62., 40.);

		double auxF = clamp01(unlerp(10., 30., rx));
		aimUpIncrement = lerp(0., 0.80, auxF) * lerp(0.4, 1.0, smoothIsAiming);

		lookResult = QuatEuler(Vector3d(rx, 0., resultEuler[2]));

		freeLookQuat = lookResult;

		lookAtQuat = freeLookQuat;
	}

	double currentFov = lerp(fov3P, fov3PAiming, smoothIsAiming);

	if (smoothIsAiming > 0.001 || currentFov != fov3P) 
	{	
		CAM::SET_CAM_FOV(customCam, currentFov);
	}

	if (isAiming) {
		UI::SHOW_HUD_COMPONENT_THIS_FRAME(eHudComponent::HudComponentReticle);
	}

	if (freeLookAtDelay)
		veloCompQuat3P = slerp(veloCompQuat3P, lookAtQuat, mixedLookTimer);

	
	Vector3d camPosCam = smoothPosCenter + V3CurrentTowHeightIncrement + ((veloCompQuat3P) * back * (calcLongitudeOffset3P + currentTowLongitudeIncrement + (airDistance - finalPivotFrontOffset) /*+ distIncFinal*/)) + (up * (aimHeightIncrement + calcHeigthOffset/* + heightInc */));

	prevCamPos = camPosCam;
	PrevCamQuat = veloCompQuat3P;

	if (!freeLookAt && fixedLookAt)
	{
		veloCompQuat3P = slerp(veloCompQuat3P, lookAtQuat, mixedLookTimer);
		camPosCam = smoothPosCenter + V3CurrentTowHeightIncrement + ((veloCompQuat3P)*back * (calcLongitudeOffset3P + currentTowLongitudeIncrement + (airDistance - finalPivotFrontOffset) /*+ distIncFinal*/)) + (up * (aimHeightIncrement + calcHeigthOffset/* + heightInc */));
	}

	Vector3d camPosFinal = camPosCam + (aimUpIncrement * up);

	Vector3d rotEuler = QuatToEuler(veloCompQuat3P);
	rotEuler[1] = 0.;

	Vector3d realFinalRot = Vector3d(rotEuler.x() - cameraAngle3p, rotEuler.y(), rotEuler.z());

	Vector3d camForward = veloCompQuat3P * front;

	// Raycast //
	int ray = WORLDPROBE::_START_SHAPE_TEST_RAY(posCenter.x(), posCenter.y(), posCenter.z(), camPosFinal.x(), camPosFinal.y(), camPosFinal.z(), 1, veh, 7);

	Vector3 endCoords, surfaceNormal;
	BOOL hit;
	Entity entityHit = 0;

	WORLDPROBE::GET_SHAPE_TEST_RESULT(ray, &hit, &endCoords, &surfaceNormal, &entityHit);

	if (hit) {
		setCamPos(customCam, toV3f(endCoords) + (camForward * 0.1));
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
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.15);
		CAM::SET_CAM_FAR_CLIP(customCam, 800.0);
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

double getVehicleLongitude(Vehicle vehicle) {

	Vector3d vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));
	Vector3d forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));

	double maxBackDistance = 0.;
	double maxFrontDistance = 0.;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3d bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			double currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, forward);
			double currFrontDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, -forward);

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

double getVehicleLongitudeFromCenterBack(Vehicle vehicle) {

	Vector3d vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));
	Vector3d forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));

	double maxBackDistance = 0.;

	//const char * maxFrontName;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3d bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			double currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, forward);

			if (currBackDistance > maxBackDistance) {
				maxBackDistance = currBackDistance;

				//maxFrontName = boneName;
			}
		}
	}

	//ShowNotification(maxFrontName);

	return maxBackDistance;
}

double getVehicleHeight(Vehicle vehicle) {

	Vector3d rotation = toV3f(ENTITY::GET_ENTITY_ROTATION(vehicle, false));

	Vector3d rightVector = toV3f(getRightVector(rotation));
	Vector3d forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));
	Vector3d upVector = rightVector.cross(forward);

	Vector3d vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));

	double maxBackDistance = 0.;
	double maxFrontDistance = 0.;

	//Vector3d backBonePos;
	//Vector3d frontBonePos;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3d bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			double currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, upVector);
			double currFrontDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, -upVector);

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

double getVehicleHeightFromCenterUp(Vehicle vehicle) {

	Vector3d rotation = toV3f(ENTITY::GET_ENTITY_ROTATION(vehicle, false));

	Vector3d rightVector = toV3f(getRightVector(rotation));
	Vector3d forward = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle));
	Vector3d upVector = rightVector.cross(forward);

	Vector3d vehiclePos = toV3f(ENTITY::GET_ENTITY_COORDS(vehicle, true));

	//double maxBackDistance = 0.;
	double maxFrontDistance = 0.;

	//Vector3d backBonePos;
	//Vector3d frontBonePos;
	const char * maxFrontName;

	for (const char *boneName : vehicleBones)
	{
		int boneIndex = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(vehicle, (char*)boneName);

		if (boneIndex != -1)
		{
			Vector3d bonePos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(vehicle, boneIndex));
			//double currBackDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, upVector);
			double currFrontDistance = distanceOnAxisNoAbs(bonePos, vehiclePos, -upVector);

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

double getTowedVehicleOrTrailerLongitude() {
	if (vehHasTowBone)
	{
		Vehicle towed = VEHICLE::GET_ENTITY_ATTACHED_TO_TOW_TRUCK(veh);

		if (towed != NULL) {
			if (towed == lastTowVehicle) {
				return lastTowVehicleLongitude;
			}
			else
			{
				towHeightIncrement = .75;
				double longitude = getVehicleLongitude(towed) + 2.0;
				lastTowVehicle = towed;
				lastTowVehicleLongitude = longitude;

				return longitude;
			}
		}
		else
		{
			towHeightIncrement = .0;
			double longitude = 0.;
			lastTowVehicle = NULL;
			lastTowVehicleLongitude = 0.;

			return 0.;
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
				towHeightIncrement = 1.45;
				double longitude = getVehicleLongitude(trailer) + 1.85;
				lastTrailer = trailer;
				lastTrailerLongitude = longitude;

				return longitude;
			}
		}
		else
		{
			towHeightIncrement = .0;
			double longitude = 0.;
			lastTrailer = NULL;
			lastTrailerLongitude = longitude;

			return longitude;
		}
	}
	else
	{
		towHeightIncrement = .0;
		double longitude = 0.;
		lastTrailer = NULL;
		lastTrailerLongitude = longitude;

		return longitude;
	}
}

void onLowTimeUpdate() {
	towLongitudeIncrement = getTowedVehicleOrTrailerLongitude();
}

void updateTimers() {
	if (lowUpdateTimerCurrentTime > 0.)
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
	smoothIsMouseLooking = lerp(smoothIsMouseLooking, isMouseLooking() ? 1. : 0., 8. * SYSTEM::TIMESTEP());

	isAiming = timeInVehicle > 0.4f && PLAYER::IS_PLAYER_FREE_AIMING(player);
	smoothIsAiming = lerp(smoothIsAiming, isAiming ? 1. : 0., 8. * SYSTEM::TIMESTEP());

	// check if player is in a vehicle
	if (PED::IS_PED_IN_ANY_VEHICLE(playerPed, FALSE))
	{
		timeInVehicle += getDeltaTime();
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
				smoothRadarAngle = lerpAngle(smoothRadarAngle, NormalizeAngle((double)rotCam.z - 180.), 1.);
			}
			else
				smoothRadarAngle = lerpAngle(smoothRadarAngle, NormalizeAngle((double)rotCam.z), lerp(6. * getDeltaTime(), 1., smoothIsMouseLooking));

			smoothRadarAngle = NormalizeAngle(smoothRadarAngle);

			UI::LOCK_MINIMAP_ANGLE(NormalizeAngle(smoothRadarAngle - (lBehind ? 180. : 0.)));
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
		timeInVehicle = 0.f;

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
