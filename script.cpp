/*
*  Custom Camera V Plus
*/
#include "script.h"
#include "utils.h"
#include <string>

#include "..\..\inc\GTAVMenuBase\menu.h"
#include "..\..\inc\GTAVMenuBase\menu.cpp"
#include "..\..\inc\GTAVMenuBase\menucontrols.h"
#include "..\..\inc\GTAVMenuBase\menucontrols.cpp"
#include "..\..\inc\GTAVMenuBase\menukeyboard.h"
#include "..\..\inc\GTAVMenuBase\menukeyboard.cpp"

using namespace Eigen;
using namespace NativeMenu;

BOOL modEnabled = true;
BOOL camInitialized = false;
Vehicle veh;
Ped playerPed;

Vector3f vehPos;
Vector3f vehRot;
Vector3f vehVelocity;
float vehSpeed;
int vehGear;

Camera customCam = NULL;
float fov = 75.;
const float PI = 3.1415926535897932f;
int lastVehHash = -1;
bool isBike = false;

float longitudeOffset3P = 0.f;
float heightOffset3P = 0.f;

float rotationSpeed3P = 4.75f;

bool useVariableRotSpeed3P = true;
float minRotSpeed3P = 2.0f;
float maxRotSpeed3P = 5.0f;
float maxRotSpeedAngle3P = 90.0f;

float currentRotSpeed3P = 2.0f;

Vector3f smoothVelocity = Vector3f();
Quaternionf velocityQuat3P = Quaternionf();
Quaternionf smoothQuat3P = Quaternionf();
float smoothIsInAir = 0.f;

Vector3f up(0.0f, 0.0f, 1.0f);
Vector3f back(0.0f, -1.0f, 0.0f);

enum eCamType {
	Smooth3P = 0,
	DriverSeat1P = 1
};

int camsLength = 2;
int currentCam = eCamType::Smooth3P;

NativeMenu::Menu menu;
NativeMenu::MenuControls menuControls;

bool showDebug = false;

Vector3 getRightVector(Vector3f rotation)
{
	float num = cos(rotation.y() * (PI / 180.0f));
	Vector3 vec;

	vec.x = cos(-rotation.z() * (PI / 180.0f)) * num;
	vec.y = sin(rotation.z() * (PI / 180.0f)) * num;
	vec.z = sin(-rotation.y() * (PI / 180.0f));

	return vec;
}

// taken from https://github.com/E66666666/GTAVManualTransmission/
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

// taken from https://github.com/E66666666/GTAVManualTransmission/
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

	ret.x = max.x - min.x;
	ret.y = max.y - min.y;
	ret.z = max.z - min.z;

	return toV3f(ret);
}

Vector3f getDimensions(Vehicle veh) {
	Hash modelHash = VEHICLE::GET_VEHICLE_LAYOUT_HASH(veh);
	return getDimensions(modelHash);
}

void displayBoundingBox(Vehicle veh) {
	Hash modelHash = VEHICLE::GET_VEHICLE_LAYOUT_HASH(veh);

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Lelft
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
	Vector3 min;
	Vector3 max;

	GAMEPLAY::GET_MODEL_DIMENSIONS(modelHash, &min, &max);
	//ENTITY::GET_ENTITY_MATRIX(veh, &rightVector, &forwardVector, &upVector, &position); //Blue or red pill

	forwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(veh);
	rightVector = getRightVector(vehRot);
	upVector = toV3(toV3f(rightVector).cross(toV3f(forwardVector)));
	position = toV3(vehPos);

																								//Calculate size
	dim.x = 0.5f*(max.x - min.x);
	dim.y = 0.5f*(max.y - min.y);
	dim.z = 0.5f*(max.z - min.z);

	FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
	FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
	FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
	//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, FUR.z, &(FUR.z), 0);
	//FUR.z += 2 * dim.z;

	BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
	BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
	BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
	//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

	showText(0.01f, 0.375f, 0.4f, std::to_string(FUR.x).c_str(), 4, solidWhite, true);
	showText(0.01f, 0.400f, 0.4f, std::to_string(FUR.y).c_str(), 4, solidWhite, true);
	showText(0.01f, 0.425f, 0.4f, std::to_string(FUR.z).c_str(), 4, solidWhite, true);

	showText(0.01f, 0.475f, 0.4f, std::to_string(BLL.x).c_str(), 4, solidWhite, true);
	showText(0.01f, 0.500f, 0.4f, std::to_string(BLL.y).c_str(), 4, solidWhite, true);
	showText(0.01f, 0.525f, 0.4f, std::to_string(BLL.z).c_str(), 4, solidWhite, true);

	//DEBUG

	Vector3 edge1 = BLL;
	Vector3 edge2;
	Vector3 edge3;
	Vector3 edge4;
	Vector3 edge5 = FUR;
	Vector3 edge6;
	Vector3 edge7;
	Vector3 edge8;

	edge2.x = edge1.x + 2.f * dim.y*rightVector.x;
	edge2.y = edge1.y + 2.f * dim.y*rightVector.y;
	edge2.z = edge1.z + 2.f * dim.y*rightVector.z;

	edge3.x = edge2.x + 2.f * dim.z*upVector.x;
	edge3.y = edge2.y + 2.f * dim.z*upVector.y;
	edge3.z = edge2.z + 2.f * dim.z*upVector.z;

	edge4.x = edge1.x + 2.f * dim.z*upVector.x;
	edge4.y = edge1.y + 2.f * dim.z*upVector.y;
	edge4.z = edge1.z + 2.f * dim.z*upVector.z;

	edge6.x = edge5.x - 2.f * dim.y*rightVector.x;
	edge6.y = edge5.y - 2.f * dim.y*rightVector.y;
	edge6.z = edge5.z - 2.f * dim.y*rightVector.z;

	edge7.x = edge6.x - 2.f * dim.z*upVector.x;
	edge7.y = edge6.y - 2.f * dim.z*upVector.y;
	edge7.z = edge6.z - 2.f * dim.z*upVector.z;

	edge8.x = edge5.x - 2.f * dim.z*upVector.x;
	edge8.y = edge5.y - 2.f * dim.z*upVector.y;
	edge8.z = edge5.z - 2.f * dim.z*upVector.z;

	GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge2.x, edge2.y, edge2.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge4.x, edge4.y, edge4.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge3.x, edge3.y, edge3.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge4.x, edge4.y, edge4.z, 0, 255, 0, 200);

	GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge6.x, edge6.y, edge6.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge6.x, edge6.y, edge6.z, edge7.x, edge7.y, edge7.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge7.x, edge7.y, edge7.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);

	GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge7.x, edge7.y, edge7.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge5.x, edge5.y, edge5.z, 0, 255, 0, 200);
	GRAPHICS::DRAW_LINE(edge4.x, edge4.y, edge4.z, edge6.x, edge6.y, edge6.z, 0, 255, 0, 200);
}

void nextCam() {
	currentCam++;
	currentCam = currentCam % camsLength;
}

void firstInit()
{

}

float AngleInRad(Vector3f vec1, Vector3f vec2)
{
	return atan2(vec2.y() - vec1.y(), vec2.x() - vec1.x());
}

//This returns the angle in degrees
float AngleInDeg(Vector3f vec1, Vector3f vec2)
{
	return AngleInRad(vec1, vec2) * 180.f / PI;
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

Quaternionf lerp(float t, const Quaternionf& a, const Quaternionf& b)
{
	return a.slerp(t, b);
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

float smoothStep(float from, float to, float t)
{
	t = clamp01(t);
	t = (float)(-2.0 * (double)t * (double)t * (double)t + 3.0 * (double)t * (double)t);
	return (float)((double)to * (double)t + (double)from * (1.0 - (double)t));
}

float getDeltaTime() {
	return SYSTEM::TIMESTEP();
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

float getVehLongitude3P(Vehicle veh, float inLongitude) {
	float longitude = inLongitude;
	float distanceAdd = 0.f;

	//if (height > 1.6f)
	//{
	//	distanceAdd = ((height - 1.6f));
	//}

	int bumperRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "bumper_r");
	if (bumperRBone != -1)
	{
		Vector3 rearBumperPos = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, bumperRBone);

		return (vehPos - toV3f(rearBumperPos)).size() + distanceAdd + 2.f;
	}

	if (!isBike) {
		int brakeLBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "brakelight_l");
		int brakeRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "brakelight_r");

		if (brakeLBone != -1 && brakeRBone != -1) {
			Vector3f brakeLPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, brakeLBone));
			Vector3f brakeRPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, brakeRBone));

			Vector3f brakePos = (brakeLPos + brakeRPos) * 0.5f;

			return (vehPos - brakePos).size() + distanceAdd + 2.f;
		}
	}

	int spoilerBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "spoiler");
	if (spoilerBone != -1)
	{
		Vector3 spoilerPos = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, spoilerBone);

		return (vehPos - toV3f(spoilerPos)).size() + distanceAdd + 2.f;
	}

	int neonBBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "neon_b");
	if (neonBBone != -1)
	{
		Vector3 rearNeonPos = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, neonBBone);

		return (vehPos - toV3f(rearNeonPos)).size() + distanceAdd + 1.f;
	}


	int BootBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "boot");
	if (BootBone != -1)
	{
		Vector3 bootPos = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, BootBone);

		return (vehPos - toV3f(bootPos)).size() + distanceAdd + 2.f;
	}

	int windscreenRBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "windscreen_r");
	if (windscreenRBone != -1)
	{
		Vector3 rearGlassPos = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, windscreenRBone);

		return (vehPos - toV3f(rearGlassPos)).size() + distanceAdd + 2.0f;
	}

	if (isBike)
	{
		longitude += 1.4f;
	}
	else
	{
		longitude += 3.6f;
	}

	return (longitude * 2.f) + distanceAdd;
}

float getVehHeight3P(Vehicle veh)
{
	int intLightBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "interiorlight");
	if (intLightBone != -1)
	{
		Vector3 lbp = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, intLightBone);

		return (vehPos - toV3f(lbp)).size() - 1.5f;
	}

	int roofBone = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "roof");
	if (roofBone != -1)
	{
		Vector3 roofPos = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, roofBone);

		return (vehPos - toV3f(roofPos)).size() - 1.5f;
	}

	int roofBone2 = ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, "roof2");
	if (roofBone2 != -1)
	{
		Vector3 roofPos2 = ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, roofBone2);

		return (vehPos - toV3f(roofPos2)).size() - 1.5f;
	}

	return 1.45f;
}

void updateVehicleProperties() 
{
	int vehClass = VEHICLE::GET_VEHICLE_CLASS(veh);
	isBike = vehClass == eVehicleClass::VehicleClassCycles || vehClass == eVehicleClass::VehicleClassMotorcycles;

	Vector3f dimensions = getDimensions(veh);

	float minHeight = 1.3f;
	float maxHeight = 4.75f;

	float heightFactor = max(0.f, dimensions.z() - minHeight) / maxHeight;

	heightOffset3P = getVehHeight3P(veh);
	longitudeOffset3P = getVehLongitude3P(veh, dimensions.y());
}

void updateVehicleVars() 
{
	vehPos = toV3f(ENTITY::GET_ENTITY_COORDS(veh, true));
	vehRot = toV3f(ENTITY::GET_ENTITY_ROTATION(veh, false));
	vehVelocity = toV3f(ENTITY::GET_ENTITY_VELOCITY(veh));
	vehSpeed = ENTITY::GET_ENTITY_SPEED(veh);
	smoothVelocity = lerp(smoothVelocity, vehVelocity, 10.f * getDeltaTime());
	smoothIsInAir = lerp(smoothIsInAir, (ENTITY::IS_ENTITY_IN_AIR(veh) || ENTITY::IS_ENTITY_UPSIDEDOWN(veh)) ? 1.f : 0.f, 2.f * getDeltaTime());
}

void setupCurrentCamera() {
	if (currentCam == eCamType::DriverSeat1P) {
		ENTITY::SET_ENTITY_ALPHA(playerPed, 0, false);
		CAM::SET_CAM_NEAR_CLIP(customCam, 0.1f);
		CAM::SET_CAM_FAR_CLIP(customCam, 750.0f);
	}
	else if (currentCam == eCamType::Smooth3P) {

	}

}

void setupCustomCamera() {
	customCam = CAM::CREATE_CAM_WITH_PARAMS("DEFAULT_SCRIPTED_CAMERA", vehPos.x(), vehPos.y(), vehPos.z(), vehRot.x(), vehRot.y(), vehRot.z(), fov, true, 2);
	CAM::SET_CAM_ACTIVE(customCam, true);
	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, true, false);

	camInitialized = true;

	setupCurrentCamera();
}

void updateCameraDriverSeat() {
	char *boneName = (isBike ? "seat_f" : "seat_dside_f");

	Vector3f seatPos = toV3f(ENTITY::GET_WORLD_POSITION_OF_ENTITY_BONE(veh, ENTITY::GET_ENTITY_BONE_INDEX_BY_NAME(veh, boneName)));
	Vector3f forwardVector = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(veh));
	Vector3f rightVector = toV3f(getRightVector(vehRot));
	Vector3f upVector = rightVector.cross(forwardVector);

	Vector3f camPos;
	if (isBike)
		camPos = seatPos + (upVector * 0.4f) + (forwardVector * 0.45f);
	else
		camPos = seatPos + (upVector * 0.69f);

	Vector3f pointAt = camPos + forwardVector;

	CAM::SET_CAM_COORD(customCam, camPos.x(), camPos.y(), camPos.z());
	CAM::POINT_CAM_AT_COORD(customCam, pointAt.x(), pointAt.y(), pointAt.z());
	CAM::RENDER_SCRIPT_CAMS(true, false, 3000, 1, 0);
}

void updateCameraSmooth3P() {
	Vector3f extraCamHeight = up * 0.11f;
	Vector3f posCenter = vehPos + (up * heightOffset3P);
	Vector3f vehForwardVector = toV3f(ENTITY::GET_ENTITY_FORWARD_VECTOR(veh));

	float rotSpeed = rotationSpeed3P;
	if (useVariableRotSpeed3P)
	{
		float desiredRootSpeed = smoothStep(minRotSpeed3P, maxRotSpeed3P, clamp01(AngleInDeg(vehForwardVector, vehVelocity) / 90.f));
		currentRotSpeed3P = lerp(currentRotSpeed3P, desiredRootSpeed, 2.0f * getDeltaTime());
	}

	if (vehSpeed > 1.f)
	{
		velocityQuat3P = lookRotation(smoothVelocity);
	}

	Quaternionf vehQuat = getEntityQuaternion(veh);

	if (isBike && vehSpeed >= 3.f) 
	{
		smoothQuat3P = smoothQuat3P.slerp(rotationSpeed3P * getDeltaTime(), velocityQuat3P);
	}
	else
	{
		smoothQuat3P = smoothQuat3P.slerp(rotationSpeed3P * getDeltaTime(), vehQuat);
	}

	//float speedLerpFactor = clamp(vehSpeed / 50.f, 1.f, 1.0005f) - 1.f;
	//float forwardSpeed = vehVelocity.dot(vehForwardVector);
	//if (forwardSpeed < 0.f)
	//	speedLerpFactor = 0.f;

	//Quaternionf finalQuat = lerp(max(speedLerpFactor, smoothIsInAir), smoothQuat3P, velocityQuat3P);
	Quaternionf finalQuat = lerp(smoothIsInAir, smoothQuat3P, velocityQuat3P);

	setCamPos(customCam, posCenter + extraCamHeight + (finalQuat * back * longitudeOffset3P));
	camPointAt(customCam, posCenter);
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

void drawDebug() 
{
	Vector3f dimensions = getDimensions(veh);
	showText(0.01f, 0.275f, 0.4f, std::to_string(dimensions.x()).c_str(), 4, solidWhite, true);
	showText(0.01f, 0.300f, 0.4f, std::to_string(dimensions.y()).c_str(), 4, solidWhite, true);
	showText(0.01f, 0.325f, 0.4f, std::to_string(dimensions.z()).c_str(), 4, solidWhite, true);

	displayBoundingBox(veh);
}

void update()
{
	if (IsKeyJustUp(str2key("F9"))) {
		showDebug = !showDebug;
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

	// check if player is in a vehicle
	if (PED::IS_PED_IN_ANY_VEHICLE(playerPed, FALSE))
	{
		Vehicle newVeh = PED::GET_VEHICLE_PED_IS_USING(playerPed);

		if (newVeh != veh) {
			veh = newVeh;
			updateVehicleProperties();
		}
		updateVehicleVars();

		if (!camInitialized) {
			setupCustomCamera();
		}

		if (CONTROLS::IS_CONTROL_JUST_PRESSED(2, eControl::ControlNextCamera)) {
			haltCurrentCamera();
			nextCam();
			setupCurrentCamera();
		}

		updateCustomCamera();

		if (showDebug) {
			drawDebug();
		}
	}
	else
	{
		veh = -1;
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
