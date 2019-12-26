//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include <cstdio>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>
#include <iostream>

#include <map>
#include <process.h>
#include <tchar.h>
#include <thread>

#include "Gamepad.h"

//#if defined( _WINDOWS )
#include <windows.h>
//#endif

using namespace vr;
using namespace std;

typedef struct _Controller
{
	double	X;
	double	Y;
	double	Z;
	double	Yaw;
	double	Pitch;
	double	Roll;
	unsigned short	Buttons;
	float	Trigger;
	float	AxisX;
	float	AxisY;
} TController, * PController;

bool running;    // Used to break the loop
Gamepad gamepad; // Gamepad instance
TController MyCtrl;


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

double DegToRad(double f) {
	return f * (3.14159265358979323846 / 180);
}

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CBarebonesControllerDriver : public vr::ITrackedDeviceServerDriver
{

	int32_t ControllerIndex = 0;
public:
	CBarebonesControllerDriver()
	{
		Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid;
		Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
	}

	virtual ~CBarebonesControllerDriver()
	{
	}

	void printDevicePositionalData(const char* deviceName, vr::HmdMatrix34_t posMatrix, vr::HmdVector3_t position, vr::HmdQuaternion_t quaternion)
	{
		LARGE_INTEGER qpc; // Query Performance Counter for Acquiring high-resolution time stamps.
						   // From MSDN: "QPC is typically the best method to use to time-stamp events and 
						   // measure small time intervals that occur on the same system or virtual machine.
		QueryPerformanceCounter(&qpc);

		// Print position and quaternion (rotation).
		DriverLog("\n%lld, %s, x = %.5f, y = %.5f, z = %.5f, qw = %.5f, qx = %.5f, qy = %.5f, qz = %.5f",
			qpc.QuadPart, deviceName,
			position.v[0], position.v[1], position.v[2],
			quaternion.w, quaternion.x, quaternion.y, quaternion.z);


		// Uncomment this if you want to print entire transform matrix that contains both position and rotation matrix.
		//dprintf("\n%lld,%s,%.5f,%.5f,%.5f,x: %.5f,%.5f,%.5f,%.5f,y: %.5f,%.5f,%.5f,%.5f,z: %.5f,qw: %.5f,qx: %.5f,qy: %.5f,qz: %.5f",
		//    qpc.QuadPart, whichHand.c_str(),
		//    posMatrix.m[0][0], posMatrix.m[0][1], posMatrix.m[0][2], posMatrix.m[0][3],
		//    posMatrix.m[1][0], posMatrix.m[1][1], posMatrix.m[1][2], posMatrix.m[1][3],
		//    posMatrix.m[2][0], posMatrix.m[2][1], posMatrix.m[2][2], posMatrix.m[2][3],
		//    quaternion.w, quaternion.x, quaternion.y, quaternion.z);

	}
	//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

	vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
		vr::HmdQuaternion_t q;

		q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
		q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
		q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
		return q;
	}
	//-----------------------------------------------------------------------------
	// Purpose: Extracts position (x,y,z).
	// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
	//-----------------------------------------------------------------------------

	vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
		vr::HmdVector3_t vector;

		vector.v[0] = matrix.m[0][3];
		vector.v[1] = matrix.m[1][3];
		vector.v[2] = matrix.m[2][3];

		return vector;
	}

	// generate a rotation quaternion around an arbitrary axis
	vr::HmdQuaternion_t rotate_around_axis(vr::HmdQuaternion_t v, const float& a)
	{
		// Here we calculate the sin( a / 2) once for optimization
		float factor = sinf(a * 0.01745329 / 2.0);

		// Calculate the x, y and z of the quaternion
		float x = v.x * factor;
		float y = v.y * factor;
		float z = v.z * factor;

		// Calcualte the w value by cos( a / 2 )
		float w = cosf(a * 0.01745329 / 2.0);

		float mag = sqrtf(w * w + x * x + y * y + z * z);

		vr::HmdQuaternion_t result = { w / mag, x / mag, y / mag, z / mag };
		return result;
	}

	// convert a 3x3 rotation matrix into a rotation quaternion
	static vr::HmdQuaternion_t CalculateRotation(float(*a)[3]) {

		vr::HmdQuaternion_t q;

		float trace = a[0][0] + a[1][1] + a[2][2];
		if (trace > 0) {
			float s = 0.5f / sqrtf(trace + 1.0f);
			q.w = 0.25f / s;
			q.x = (a[2][1] - a[1][2]) * s;
			q.y = (a[0][2] - a[2][0]) * s;
			q.z = (a[1][0] - a[0][1]) * s;
		}
		else {
			if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) {
				float s = 2.0f * sqrtf(1.0f + a[0][0] - a[1][1] - a[2][2]);
				q.w = (a[2][1] - a[1][2]) / s;
				q.x = 0.25f * s;
				q.y = (a[0][1] + a[1][0]) / s;
				q.z = (a[0][2] + a[2][0]) / s;
			}
			else if (a[1][1] > a[2][2]) {
				float s = 2.0f * sqrtf(1.0f + a[1][1] - a[0][0] - a[2][2]);
				q.w = (a[0][2] - a[2][0]) / s;
				q.x = (a[0][1] + a[1][0]) / s;
				q.y = 0.25f * s;
				q.z = (a[1][2] + a[2][1]) / s;
			}
			else {
				float s = 2.0f * sqrtf(1.0f + a[2][2] - a[0][0] - a[1][1]);
				q.w = (a[1][0] - a[0][1]) / s;
				q.x = (a[0][2] + a[2][0]) / s;
				q.y = (a[1][2] + a[2][1]) / s;
				q.z = 0.25f * s;
			}
		}
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;
		return q;
	}

	virtual void SetControllerIndex(int32_t CtrlIndex)
	{
		ControllerIndex = CtrlIndex;
	}

	/** This is called before an HMD is returned to the application. It will always be
	* called before any display or tracking methods. Memory and processor use by the
	* ITrackedDeviceServerDriver object should be kept to a minimum until it is activated.
	* The pose listener is guaranteed to be valid until Deactivate is called, but
	* should not be used after that point. */
	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		switch (ControllerIndex)
		{
		case 0:
			Ctrl1Index_t = unObjectId;
			Ctrl1Index_t = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t);
			break;
		case 1:
			Ctrl2Index_t = unObjectId;
			Ctrl2Index_t = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t);
			break;
		}

		switch (ControllerIndex)
		{
		case 0:
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL1Serial");
			break;
		case 1:
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL2Serial");
			break;
		}

		switch (ControllerIndex)
		{
		case 0:
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand);
			break;
		case 1:
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
			break;
		}
		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
		// retrieve the property container of the current HMD
		vr::PropertyContainerHandle_t ulContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(k_unTrackedDeviceIndex_Hmd);

		uint64_t supportedButtons = 0xA4;
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, supportedButtons);


		// retrieve and save the factory calibration data
		
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_LegacyInputProfile_String, "vive_controller");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");
		//vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
		//vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);
		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2218716);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		// our sample device isn't actually tracked, so set this property to avoid having the icon blink in the status window
		//vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_NeverTracked_Bool, true);

		// even though we won't ever track we want to pretend to be the right hand so binding will work as expected
		//vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);

		// this file tells the UI what to show the user for binding this controller as well as what default bindings should
		// be for legacy or other apps
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{barebones}/input/barebones_profile.json");

		bool bSetupIconUsingExternalResourceFile = false;
		if (!bSetupIconUsingExternalResourceFile)
		{
			// Setup properties directly in code.
			// Path values are of the form {drivername}\icons\some_icon_filename.png
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{barebones}/icons/barebones_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{barebones}/icons/barebones_status_ready.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{barebones}/icons/barebones_status_ready_alert.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{barebones}/icons/barebones_status_error.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{barebones}/icons/barebones_status_standby.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{barebones}/icons/barebones_status_ready_low.png");
		}

		// create all the input components
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x", &rightJoystickXInputHandle, EVRScalarType::VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
		vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y", &rightJoystickYInputHandle, EVRScalarType::VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);

		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trackpad/x", &leftJoystickXInputHandle,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
		);
		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trackpad/y", &leftJoystickYInputHandle,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
		);

		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trigger/value", &leftTriggerInputHandle,
			vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided
		);

		/*vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trigger1/value", &rightTriggerInputHandle,
			vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided
		);*/

		//  Buttons handles
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &m_start);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_back);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compA);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &m_compB);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &m_compX);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &m_compY);

		// create our haptic component
		//vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

	
		return VRInitError_None;

	}

	/** This is called when The VR system is switching from this Hmd being the active display
	* to another Hmd being the active display. The driver should clean whatever memory
	* and thread use it can when it is deactivated */
	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;

	}

	/** Handles a request from the system to put this device into standby mode. What that means is defined per-device. */
	virtual void EnterStandby()
	{
		DriverLog("Barebones: Entering Standby Mode\n");
	}

	/** Requests a component interface of the driver for device-specific functionality. The driver should return NULL
	* if the requested interface or version is not supported. */
	virtual void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** A VR Watchdog has made this debug request of the driver. The set of valid requests is entirely
	* up to the driver and the Watchdog to figure out, as is the format of the response. Responses that
	* exceed the length of the supplied buffer should be truncated and null terminated */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	// ------------------------------------
	// Tracking Methods
	// ------------------------------------
	virtual DriverPose_t GetPose()
	{
		//HmdVector3_t position;

		HmdQuaternion_t rot;
		DriverPose_t pose = { 0 };

		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		pose.qRotation = HmdQuaternion_Init(0, 0, 0, 0);

		TrackedDevicePose_t trackedDevicePose;
		VRControllerState_t controllerState;
		vr::TrackedDevicePose_t tracker;

		for (unsigned int deviceId = 0; deviceId < k_unMaxTrackedDeviceCount; deviceId++) {
			
			vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses(deviceId, devicePoses, vr::k_unMaxTrackedDeviceCount);
			tracker = devicePoses[deviceId];
			rot = GetRotation(tracker.mDeviceToAbsoluteTracking);
			if (deviceId == 0) {
				break;
			}
			/*VRControllerState_t controllerState;

			ETrackedDeviceClass class =	vr_pointer->GetTrackedDeviceClass(id);

			if (!vr_pointer->IsTrackedDeviceConnected(id))
				continue;

			if (class == ETrackedDeviceClass::) {
				vr_pointer->);

				//perform actions with pose struct (see next section)
			}
			if () {
				break;*/
		}

		float a = (tracker.mDeviceToAbsoluteTracking.m[0][0]);
		float b = tracker.mDeviceToAbsoluteTracking.m[1][0];
		float c = (tracker.mDeviceToAbsoluteTracking.m[2][0]);

		double offset[3] = { -0.90, -0.1, 0.16}; //x=dn-r, y=up, z=dn-l

		float X = MyCtrl.X + ((cos(c) * 0.6) + (sin(c) * 0.6));// +(cos(c) * 0.6) * (sin(c) * 0.6));
		float Y = MyCtrl.Y;
		float Z = MyCtrl.Z - ((cos(a) * 0.6) + (sin(a) * 0.6));// +(cos(a) * 0.6) * (sin(a) * 0.6));
	
		if (center == true) {
			MyCtrl.X = a;
			MyCtrl.Y = b;
			MyCtrl.Z = c;
			MyCtrl.Yaw = 0.0;
			MyCtrl.Roll = 0.0;
			MyCtrl.Pitch = 0.0;
			center = false;
		}

		pose.vecDriverFromHeadTranslation[0] = 0;
		pose.vecDriverFromHeadTranslation[1] = 0;
		pose.vecDriverFromHeadTranslation[2] = 0;
		pose.vecAcceleration[0] = -0.8;
		pose.vecAcceleration[1] = 0.0;
		pose.vecAcceleration[2] = -0.8;

		pose.vecPosition[0] = X + offset[0];
		pose.vecPosition[1] = Y + offset[1];
		pose.vecPosition[2] = Z + offset[2];

		if (cnt == 100) {
			DriverLog("HMD: %f %f %f / %f %f %f", a + X + offset[0], b + Y + offset[1], c + Z + offset[2], a, b, c);
			cnt = 0;
		}

		//Convert yaw, pitch, roll to quaternion
		float w = cos(DegToRad(MyCtrl.Yaw) * 0.5) * cos(DegToRad(MyCtrl.Roll) * 0.5) * cos(DegToRad(MyCtrl.Pitch) * 0.5) + sin(DegToRad(MyCtrl.Yaw) * 0.5) * sin(DegToRad(MyCtrl.Roll) * 0.5) * sin(DegToRad(MyCtrl.Pitch) * 0.5);
		float x = cos(DegToRad(MyCtrl.Yaw) * 0.5) * sin(DegToRad(MyCtrl.Roll) * 0.5) * cos(DegToRad(MyCtrl.Pitch) * 0.5) - sin(DegToRad(MyCtrl.Yaw) * 0.5) * cos(DegToRad(MyCtrl.Roll) * 0.5) * sin(DegToRad(MyCtrl.Pitch) * 0.5);
		float y = cos(DegToRad(MyCtrl.Yaw) * 0.5) * cos(DegToRad(MyCtrl.Roll) * 0.5) * sin(DegToRad(MyCtrl.Pitch) * 0.5) + sin(DegToRad(MyCtrl.Yaw) * 0.5) * sin(DegToRad(MyCtrl.Roll) * 0.5) * cos(DegToRad(MyCtrl.Pitch) * 0.5);
		float z = sin(DegToRad(MyCtrl.Yaw) * 0.5) * cos(DegToRad(MyCtrl.Roll) * 0.5) * cos(DegToRad(MyCtrl.Pitch) * 0.5) - cos(DegToRad(MyCtrl.Yaw) * 0.5) * sin(DegToRad(MyCtrl.Roll) * 0.5) * sin(DegToRad(MyCtrl.Pitch) * 0.5);

		//float W = w - rot.w;
		float Xr = x - rot.x;
		float Yr = y - rot.y;
		float Zr = z - rot.z;

		pose.qRotation.w = rot.w;
		pose.qRotation.x = rot.x;
		pose.qRotation.y = rot.y;
		pose.qRotation.z = rot.z;

		IVRServerDriverHost* pDriverHost = vr::VRServerDriverHost();

		if (pDriverHost == nullptr)
		{
			DriverLog("barebonse: CServerDriver::RunFrame() pDriverHost is NULL\n");
		}

		// Triggers

		if (float LftT = gamepad.LeftTrigger())
		{
			if (LftT > 0.6) {
				vr::VRDriverInput()->UpdateScalarComponent(leftTriggerInputHandle, LftT, 0);
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftTriggerInputHandle, 0.0, 0);
			}
		}

		if (float RghT = gamepad.RightTrigger())
		{
			if (RghT > 0.6) {
				vr::VRDriverInput()->UpdateScalarComponent(rightTriggerInputHandle, RghT, 0);
			}
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(rightTriggerInputHandle, 0.0, 0);
		}

		// Sticks

		if (float LftStX = gamepad.LeftStick_X())
		{
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0.0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 0, 0);
		}

		if (float LftStY = gamepad.LeftStick_Y())
		{
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0.0, 0);
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 0, 0);
		}

		if (float RghStX = gamepad.RightStick_X())
		{
			vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
			MyCtrl.Pitch = MyCtrl.Pitch - RghStX * 1.2;
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0.0, 0);
		}

		if (float RghStY = gamepad.RightStick_Y())
		{
			vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
			MyCtrl.Roll = MyCtrl.Roll + RghStY * 1.2;
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0.0, 0);
		}

		// Buttons

		if (gamepad.GetButtonPressed(xButtons.A))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 1, 0);
			pushed[12] = true;

		}
		else {
			if (pushed[12] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 0, 0);
				pushed[12] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.B))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 1, 0);
			pushed[11] = true;

		}
		else {
			if (pushed[11] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 0, 0);
				pushed[11] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.Y))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 1, 0);
			pushed[10] = true;

		}
		else {
			if (pushed[10] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 0, 0);
				pushed[10] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.X))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 1, 0);
			pushed[9] = true;
		}
		else {
			if (pushed[9] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 0, 0);
				pushed[9] = false;
			}
		}

		// Start/select

		if (gamepad.GetButtonPressed(xButtons.Start))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_start, 1, 0);
			pushed[0] = true;
		}
		else {
			if (pushed[0] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_start, 0, 0);
				pushed[0] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.Back))
		{

			vr::VRDriverInput()->UpdateBooleanComponent(m_back, 1, 0);
			pushed[13] = true;
			if (leftctrl == true) {
				DriverLog("Left ctrl");
				SetControllerIndex(0);
			}
			else {
				DriverLog("Right ctrl");
				SetControllerIndex(1);
			}

		}
		else {
			if (pushed[13] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_back, 0, 0);
				pushed[13] = false;
			}
		}

		// dPad

		if (gamepad.GetButtonPressed(xButtons.DPad_Left))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_dPadLeft, 1, 0);
			MyCtrl.X = MyCtrl.X - 0.01;
			if (diag == true) {
				MyCtrl.Z = MyCtrl.Z + 0.01;
			}
			pushed[8] = true;

		}
		else {
			if (pushed[8] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_dPadLeft, 0, 0);
				pushed[8] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Right))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_dPadRight, 1, 0); //Trackpad touch
			MyCtrl.X = MyCtrl.X + 0.01;
			if (diag == true) {
				MyCtrl.Z = MyCtrl.Z - 0.01;
			}
			pushed[1] = true;

		}
		else {
			if (pushed[1] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_dPadRight, 0, 0); //Trackpad touch
				pushed[1] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Up))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_dPadUp, 1, 0); //Trackpad touch
			MyCtrl.Z = MyCtrl.Z - 0.01;
			if (diag == true) {
				MyCtrl.X = MyCtrl.X - 0.01;
			}
			pushed[2] = true;

		}
		else {
			if (pushed[2] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_dPadUp, 0, 0); //Trackpad touch
				pushed[2] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Down))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_dPadDown, 1, 0); //Trackpad touch
			MyCtrl.Z = MyCtrl.Z + 0.01;
			if (diag == true) {
				MyCtrl.X = MyCtrl.X + 0.01;
			}
			pushed[3] = true;
		}
		else {
			if (pushed[3] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_dPadDown, 0, 0); //Trackpad touch
				pushed[3] = false;
			}
		}

		// Bummers

		if (gamepad.GetButtonPressed(xButtons.L_Shoulder))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_bumLeft, 1, 0); //Trackpad touch
			MyCtrl.Y = MyCtrl.Y + 0.01;
			pushed[4] = true;

		}
		else {
			if (pushed[4] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_bumLeft, 0, 0); //Trackpad touch
				pushed[4] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.R_Shoulder))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_bumRight, 1, 0); //Trackpad touc
			MyCtrl.Y = MyCtrl.Y - 0.01;
			pushed[5] = true;

		}
		else {
			if (pushed[5] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_bumRight, 0, 0); //Trackpad touch
				pushed[5] = false;
			}
		}

		// Sticks click

		if (gamepad.GetButtonPressed(xButtons.L_Thumbstick))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_stickL, 1, 0); //Trackpad touch
			if (pushed[6] == false) {
				center = true;
				DriverLog("Center true");
			}
			pushed[6] = true;

		}
		else {
			if (pushed[6] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_stickL, 0, 0); //Trackpad touch
				pushed[6] = false;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.R_Thumbstick))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_stickR, 1, 0); //Trackpad touch
			if (diag == true) {
				DriverLog("Diag false");
				diag = false;
			}
			else {
				DriverLog("Diag true");
				diag = true;
			}
			pushed[7] = true;

		}
		else {
			if (pushed[7] == true) {
				vr::VRDriverInput()->UpdateBooleanComponent(m_stickR, 0, 0); //Trackpad touch
				pushed[7] = false;
			}
		}

		//DriverLog("------ HMD - %f %f %f %f Cntrl - %f %f %f %f", rot.w, rot.x, rot.y, rot.z, Xr, Yr, Zr);
		cnt++;
		return pose;
	}

	void RunFrame()
	{

		if (ControllerIndex == 0) {

			if (Ctrl1Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl1Index_t, GetPose(), sizeof(DriverPose_t));
			}

		}

		if (ControllerIndex == 1) {

			if (Ctrl2Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl2Index_t, GetPose(), sizeof(DriverPose_t));
			}

		}
		
	}

	void ProcessEvent(const vr::VREvent_t & vrEvent)
	{
		switch (vrEvent.eventType)
		{
		case vr::VREvent_Input_HapticVibration:
		{
			if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
			{
				// This is where you would send a signal to your hardware to trigger actual haptic feedback
				DriverLog("BUZZ!\n");
			}
		}
		break;
		}
	}

	std::string GetSerialNumber() const {

		switch (ControllerIndex)
		{
		case 0:
			return "CTRL1Serial";
			break;
		case 1:
			return "CTRL2Serial";
			break;
		}
	}

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t;
	vr::TrackedDeviceIndex_t Ctrl2Index_t;

	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;
	vr::HmdQuaternion_t m_hmdRot;

	std::string m_sSerialNumber;

	vr::VRInputComponentHandle_t rightTriggerInputHandle;
	vr::VRInputComponentHandle_t leftTriggerInputHandle;

	vr::VRInputComponentHandle_t rightJoystickXInputHandle;
	vr::VRInputComponentHandle_t rightJoystickYInputHandle;
	vr::VRInputComponentHandle_t leftJoystickXInputHandle;
	vr::VRInputComponentHandle_t leftJoystickYInputHandle;

	vr::VRInputComponentHandle_t m_compHaptic;

	vr::VRInputComponentHandle_t m_compA;
	vr::VRInputComponentHandle_t m_compB;
	vr::VRInputComponentHandle_t m_compX;
	vr::VRInputComponentHandle_t m_compY;

	vr::VRInputComponentHandle_t m_dPadUp;
	vr::VRInputComponentHandle_t m_dPadDown;
	vr::VRInputComponentHandle_t m_dPadLeft;
	vr::VRInputComponentHandle_t m_dPadRight;

	vr::VRInputComponentHandle_t m_bumRight;
	vr::VRInputComponentHandle_t m_bumLeft;

	vr::VRInputComponentHandle_t m_start;
	vr::VRInputComponentHandle_t m_back;
	vr::VRInputComponentHandle_t m_stickL;
	vr::VRInputComponentHandle_t m_stickR;

	bool leftctrl;
	bool pushed[15];
	float m_hmdPos[3];
	bool diag = true;
	bool center = true;
	int cnt = 0;

};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class BareboneProvider : public IServerTrackedDeviceProvider
{
public:
	virtual const char * const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
	virtual bool ShouldBlockStandbyMode() override { return true; }
	virtual void EnterStandby() override {}
	virtual void LeaveStandby() override {}

private:
	CBarebonesControllerDriver * m_pController = nullptr;
	CBarebonesControllerDriver* m_pController2 = nullptr;

public:
	virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext) override
	{
		VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
		InitDriverLog(vr::VRDriverLog());

		DriverLog("barebones: Creating Controller Driver\n");
		DriverLog("barebones: Post- Creating Controller Driver\n");

		m_pController = new CBarebonesControllerDriver();
		m_pController->SetControllerIndex(0);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);


		m_pController2 = new CBarebonesControllerDriver();
		m_pController2->SetControllerIndex(1);
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController2->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController2);

		return VRInitError_None;
	}

	virtual void Cleanup() override
	{
		CleanupDriverLog();
		delete m_pController;
		m_pController = NULL;
	}

	virtual void RunFrame() override
	{
		if (gamepad.Connected()) {
			
			gamepad.Update(); // Update gamepad
			if (m_pController)
			{
				m_pController->RunFrame();
			}
			if (m_pController2)
			{
				m_pController2->RunFrame();
			}

			vr::VREvent_t vrEvent;
			while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
			{
				if (m_pController)
				{
					m_pController->ProcessEvent(vrEvent);
				}
				if (m_pController2)
				{
					m_pController2->ProcessEvent(vrEvent);
				}
			}
			gamepad.Refresh();
		}
		
	}
	
};
////-----------------------------------------------------------------------------
//// Purpose:
////-----------------------------------------------------------------------------

static BareboneProvider g_serverDriverNull;

HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_serverDriverNull;
	}
	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
