﻿//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include <cstdio>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <atlbase.h>

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
} TController, * PController;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

bool ctrl = true;
Gamepad gamepad; // Gamepad instance
TController MyCtrl[2];
int32_t Active = 1;

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
	int32_t ControllerIndex;
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
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t);
			break;
		case 1:
			Ctrl2Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t);
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

		uint64_t supportedButtons = 0xA4;
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, supportedButtons);


		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_LegacyInputProfile_String, "vive_controller");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");

		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2218716);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

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

		
		switch (ControllerIndex)
		{
			case 0:
				vr::VRDriverInput()->CreateScalarComponent(
					m_ulPropertyContainer, "/input/trackpad/x", &leftJoystickXInputHandle,
					vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
				);
				vr::VRDriverInput()->CreateScalarComponent(
					m_ulPropertyContainer, "/input/trackpad/y", &leftJoystickYInputHandle,
					vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
				);
				break;
			case 1:
				vr::VRDriverInput()->CreateScalarComponent(
					m_ulPropertyContainer, "/input/trackpad/x", &rightJoystickXInputHandle,
					vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
				);
				vr::VRDriverInput()->CreateScalarComponent(
					m_ulPropertyContainer, "/input/trackpad/y", &rightJoystickYInputHandle,
					vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
				);
				break;
		}
		

		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trigger/value", &TriggerInputHandle[ControllerIndex],
			vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided
		);

		//  Buttons handles
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &m_start);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_back);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compA);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &m_compB);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &m_compX);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &m_compY);

		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

		return VRInitError_None;

	}

	/** This is called when The VR system is switching from this Hmd being the active display
	* to another Hmd being the active display. The driver should clean whatever memory
	* and thread use it can when it is deactivated */
	virtual void Deactivate()
	{
		Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid;
		Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid;
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

	void Center() {
		double offset = 1.5;
		for (int i = 0; i < 2; i++) {
			if (center[i] == true) {
				MyCtrl[i].X = 0;
				MyCtrl[i].Y = offset;
				MyCtrl[i].Z = 0;
				MyCtrl[i].Yaw = 0;
				MyCtrl[i].Pitch = 0;
				MyCtrl[i].Roll = 0;
				center[i] = false;
			}
		}
	}

	// ------------------------------------
	// Tracking Methods
	// ------------------------------------
	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };
		TrackedDevicePose_t trackedDevicePose;
		VRControllerState_t controllerState;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		pose.qRotation = HmdQuaternion_Init(0, 0, 0, 0);

		vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
		vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, devicePoses, vr::k_unMaxTrackedDeviceCount);
		vr::TrackedDevicePose_t tracker = devicePoses[0];
		HmdQuaternion_t rot = GetRotation(tracker.mDeviceToAbsoluteTracking);

		HmdQuaternion_t r;

		float a = (tracker.mDeviceToAbsoluteTracking.m[0][0]);
		float b = tracker.mDeviceToAbsoluteTracking.m[1][0];
		float c = (tracker.mDeviceToAbsoluteTracking.m[2][0]);

		if (center[0] == true || center[1] == true) {
			Center();
			pose.qRotation = rot;
			pose.vecDriverFromHeadTranslation[2] = 3.5;
			pose.vecPosition[0] = c / 4;
			pose.vecPosition[2] = a * -1 / 4;
		}

		if (ControllerIndex == 0) {

			pose.vecDriverFromHeadTranslation[0] = MyCtrl[0].X;
			pose.vecDriverFromHeadTranslation[2] = MyCtrl[0].Z;

			r.w = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) + sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.x = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) - sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.y = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5) + sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.z = sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) - cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);

			pose.qRotation = rot;

			pose.qDriverFromHeadRotation = r;

			pose.vecPosition[0] = c / 4;
			pose.vecPosition[1] = MyCtrl[0].Y;
			pose.vecPosition[2] = a * -1 / 4;


			if (cnt == 100) {
				DriverLog("%d - HMD: %f %f %f / %f %f %f\n", ControllerIndex, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], rot.x, rot.y, rot.z);
			}
		}
		else {

			pose.vecDriverFromHeadTranslation[0] = MyCtrl[1].X;
			pose.vecDriverFromHeadTranslation[2] = MyCtrl[1].Z;
			

			r.w = cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5) + sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5);
			r.x = cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5) - sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5);
			r.y = cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5) + sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5);
			r.z = sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5) - cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5);

			pose.qRotation = rot;

			pose.qDriverFromHeadRotation = r;

			pose.vecPosition[0] = c/4;
			pose.vecPosition[1] = MyCtrl[1].Y;
			pose.vecPosition[2] = a * -1/4;
			
			if (cnt == 100) {
				DriverLog("%d - HMD: %f %f %f / %f %f %f\n", ControllerIndex, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], rot.x, rot.y, rot.z);
			}
		}

		cnt++;
		if (cnt == 101) {
			cnt = 0;
		}

		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		return pose;
	}

	void ProcessHandles(int32_t index) {

		// Sticks

		if (float LftStX = gamepad.LeftStick_X())
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].Pitch = MyCtrl[Active].Pitch - LftStX * 0.5;
				}
			}
			else {
				if (act[0] == 0) {
					MyCtrl[0].Pitch = MyCtrl[0].Pitch - LftStX * 0.5;
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
				}
			}
		}
		else {
			//vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0.0, 0);

		}

		if (float LftStY = gamepad.LeftStick_Y())
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].Roll = MyCtrl[Active].Roll + LftStY * 0.5;
				}
			}
			else {
				if (act[0] == 0) {
					MyCtrl[0].Roll = MyCtrl[0].Roll + LftStY * 0.5;
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
				}
			}
		}
		else {
				//vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0.0, 0);
		}

		if (float RghStX = gamepad.RightStick_X())
		{
			if (swap == 0) {
				if (index == Active) {
					if (Active == 1) {
						vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
					}
					else {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, RghStX, 0);
					}
				}
			}
			else {
				if (act[1] == 0) {
					MyCtrl[1].Pitch = MyCtrl[1].Pitch - RghStX * 0.5;
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
				}
			}
		}
		else {
			if (index == Active) {
				if (Active == 1) {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
				}
			}
		}

		if (float RghStY = gamepad.RightStick_Y())
		{
			if (swap == 0) {
				if (index == Active) {
					if (Active == 1) {
						vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
					}
					else {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, RghStY, 0);
					}
				}
			}
			else {
				if (act[1] == 0) {
					MyCtrl[1].Roll = MyCtrl[1].Roll + RghStY * 0.5;
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
				}
			}
		}
		else {
			if (index == Active) {
				if (Active == 1) {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
				}
			}
		}

		// Dpad

		if (gamepad.GetButtonPressed(xButtons.DPad_Left))
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].X = MyCtrl[Active].X - 0.005;
				}
			}
			else {
				MyCtrl[0].X = MyCtrl[0].X - 0.005;
				MyCtrl[1].X = MyCtrl[1].X - 0.005;
			}

		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Right))
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].X = MyCtrl[Active].X + 0.005;
				}
			}
			else {
				MyCtrl[0].X = MyCtrl[0].X + 0.005;
				MyCtrl[1].X = MyCtrl[1].X + 0.005;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Up))
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].Z = MyCtrl[Active].Z - 0.005;
				}
			}
			else {
				MyCtrl[0].Z = MyCtrl[0].Z - 0.005;
				MyCtrl[1].Z = MyCtrl[1].Z - 0.005;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Down))
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].Z = MyCtrl[Active].Z + 0.005;
				}
			}
			else {
				MyCtrl[0].Z = MyCtrl[0].Z + 0.005;
				MyCtrl[1].Z = MyCtrl[1].Z + 0.005;
			}
			
		}

		// Bummers

		if (gamepad.GetButtonPressed(xButtons.L_Shoulder))
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].Y = MyCtrl[Active].Y + 0.002;
				}
			}
			else {
				MyCtrl[0].Y = MyCtrl[0].Y + 0.002;
				MyCtrl[1].Y = MyCtrl[1].Y + 0.002;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.R_Shoulder))
		{
			if (swap == 0) {
				if (index == Active) {
					MyCtrl[Active].Y = MyCtrl[Active].Y - 0.002;
				}
			}
			else {
				MyCtrl[0].Y = MyCtrl[0].Y - 0.002;
				MyCtrl[1].Y = MyCtrl[1].Y - 0.002;
			}

		}

		if (gamepad.GetButtonPressed(xButtons.Back))
		{
			if (index == Active) {
				if (swap == 0) {
					center[Active] = true;
				}
				else {
					center[0] = true;
					center[1] = true;
				}
			}
		}

		// Singles

		if (index == Active) {

			// Triggers

			if (float LftT = gamepad.LeftTrigger())
			{
				if (LftT > 0.6) {
					vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[Active], LftT, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[Active], 0.0, 0);
				}
			}

			if (float RghT = gamepad.RightTrigger())
			{
				if (RghT > 0.6) {
					if (swap == 0 && check == 0) {
						DriverLog("swap true");
						swap = 1;
						check = 1;
					}
					if (swap == 1 && check == 0) {
						DriverLog("swap false");
						swap = 0;
						check = 1;
					}
				}
				else {
					check = 0;
				}
			}
			else {
				if (check == 1) {
					swap = 0;
				}
			}

			// Buttons

			if (gamepad.GetButtonPressed(xButtons.A))
			{
				vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 1, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 0, 0);
			}

			if (gamepad.GetButtonPressed(xButtons.B))
			{
				vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 1, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 0, 0);
			}

			if (gamepad.GetButtonPressed(xButtons.Y))
			{
				vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 1, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 0, 0);
			}

			if (gamepad.GetButtonPressed(xButtons.X))
			{
				vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 1, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 0, 0);
			}

			// Start/select

			if (gamepad.GetButtonPressed(xButtons.Start))
			{
				vr::VRDriverInput()->UpdateBooleanComponent(m_start, 1, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(m_start, 0, 0);
			}

			// Sticks click

			if (gamepad.GetButtonPressed(xButtons.L_Thumbstick))
			{
				if (ctrl == true && swap == 0) {
					Active = 0;
					DriverLog("%d - ctrl\n", ControllerIndex);
					ctrl = false;
				}
				if (chck[0] == 0 && swap == 1) {
					switch (act[0])
					{
					case 0:
						act[0] = 1;
						break;
					case 1:
						act[0] = 0;
						break;
					}
					chck[0] = 1;
				}
				else {
					chck[0] = 1;
				}

			} 
			else {
				if (chck[0] == 1 && swap == 1) {
					chck[0] = 0;
				}
			}

			if (gamepad.GetButtonPressed(xButtons.R_Thumbstick))
			{
				if (ctrl == false && swap == 0) {
					Active = 1;
					DriverLog("%d - ctrl\n", ControllerIndex);
					ctrl = true;
				}
				if (chck[1] == 0 && swap == 1) {
					switch (act[1])
					{
					case 0:
						act[1] = 1;
						break;
					case 1:
						act[1] = 0;
						break;
					}
					chck[1] = 1;
				}
				else {
					chck[1] = 1;
				}

			}
			else {
				if (chck[1] == 1 && swap == 1) {
					chck[1] = 0;
				}
			}
		}
	}
	void RunFrame()
	{
		switch (ControllerIndex)
		{
		case 0:
			ProcessHandles(0);
			if (Ctrl1Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl1Index_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		case 1:
			ProcessHandles(1);
			if (Ctrl2Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl2Index_t, GetPose(), sizeof(DriverPose_t));
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

	void ProcessEvent(const vr::VREvent_t& vrEvent)
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

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t;
	vr::TrackedDeviceIndex_t Ctrl2Index_t;

	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;

	vr::VRInputComponentHandle_t TriggerInputHandle[2];

	vr::VRInputComponentHandle_t rightJoystickXInputHandle;
	vr::VRInputComponentHandle_t rightJoystickYInputHandle;
	vr::VRInputComponentHandle_t leftJoystickXInputHandle;
	vr::VRInputComponentHandle_t leftJoystickYInputHandle;

	vr::VRInputComponentHandle_t m_compHaptic;

	vr::VRInputComponentHandle_t m_compA;
	vr::VRInputComponentHandle_t m_compB;
	vr::VRInputComponentHandle_t m_compX;
	vr::VRInputComponentHandle_t m_compY;

	vr::VRInputComponentHandle_t m_start;
	vr::VRInputComponentHandle_t m_back;

	int swap = 0;
	int act[2] = { 0, 0 };
	int check = 0;
	int chck[2] = { 0,0 };
	bool center[2] = { true, true };

	int cnt = 0;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class BareboneProvider : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
	virtual void RunFrame();
	virtual void Cleanup();

private:
	CBarebonesControllerDriver *m_pController = nullptr;
	CBarebonesControllerDriver *m_pController2 = nullptr;
};

static BareboneProvider g_serverDriverNull;

EVRInitError BareboneProvider::Init(vr::IVRDriverContext* pDriverContext)
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

void BareboneProvider::Cleanup()
{
	CleanupDriverLog();
	delete m_pController;
	m_pController = NULL;
	delete m_pController2;
	m_pController2 = NULL;
}

void BareboneProvider::RunFrame()
{
	if (gamepad.Connected()) {

		gamepad.Update();
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

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_serverDriverNull;
	}
	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
