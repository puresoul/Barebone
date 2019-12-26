#include "ControllerData.h"

namespace CustomController
{
	using namespace std;

	/*vr::HmdQuaternion_t ControllerData::rotate_around_axis(float axisX, float axisY, float axisZ, const float& a)
	{
		// Here we calculate the sin( a / 2) once for optimization
		float factor = sinf(a * 0.01745329 / 2.0);

		// Calculate the x, y and z of the quaternion
		float x = axisX * factor;
		float y = axisY * factor;
		float z = axisZ * factor;

		// Calcualte the w value by cos( a / 2 )
		float w = cosf(a * 0.01745329 / 2.0);

		float mag = sqrtf(w * w + x * x + y * y + z * z);

		vr::HmdQuaternion_t result = { w / mag, x / mag, y / mag, z / mag };
		return result;
	}*/

	ControllerData::ControllerData()
	{
		useControllerOrientation = false;
		leftState.ResetCenter();
		leftState.ResetZero();
		rightState.ResetCenter();
		rightState.ResetZero();
		currentHmdRotation.w = 1.0f;
		currentHmdRotation.x = 0.0f;
		currentHmdRotation.y = 0.0f;
		currentHmdRotation.z = 0.0f;
		//eg90_angleCorrection = rotate_around_axis(1.0f, 0.0f, 0.0f, 90.0f);
	}

	ControllerData::~ControllerData()
	{
	}

	vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
	{
		vr::HmdQuaternion_t tmp;

		tmp.w = (b.w * a.w) - (b.x * a.x) - (b.y * a.y) - (b.z * a.z);
		tmp.x = (b.w * a.x) + (b.x * a.w) + (b.y * a.z) - (b.z * a.y);
		tmp.y = (b.w * a.y) + (b.y * a.w) + (b.z * a.x) - (b.x * a.z);
		tmp.z = (b.w * a.z) + (b.z * a.w) + (b.x * a.y) - (b.y * a.x);

		return tmp;
	}

	void ControllerData::ParseStringSerial(char* serialString)
	{
		ButtonStates* targetState;

		stringstream sStream;
		sStream.str(serialString);
		string item;
		vector<string> parsed;
		int i = 0;
		while (getline(sStream, item, ';'))
		{
			parsed.push_back(item);
			i++;
		}

		// Identify the hand for the controller.
		if (parsed[0] == "L")
		{
			targetState = &leftState;
		}
		else if (parsed[0] == "R")
		{
			targetState = &rightState;
		}
		else
		{
			// Wrong packet format
			return;
		}

		// Buttons
		int buttonState = stoi(parsed[1]);

		targetState->btn_magicFar = (buttonState & 64) > 0;
		targetState->btn_magicMove = (buttonState & 32) > 0;
		targetState->btn_trigger = (buttonState & 16) > 0;
		targetState->btn_touchpadPress = (buttonState & 8) > 0;
		targetState->btn_menu = (buttonState & 4) > 0;
		targetState->btn_system = (buttonState & 2) > 0;
		targetState->btn_grip = (buttonState & 1) > 0;

		// Touchpad
		targetState->touchpadX = stof(parsed[2]);
		targetState->touchpadY = stof(parsed[3]);
		targetState->trackpad_touched = targetState->touchpadX != 0.f || targetState->touchpadY != 0.f;

		// Orientation
		targetState->orientationQuat = vr::HmdQuaternion_t();
		targetState->orientationQuat.w = stof(parsed[4]);
		targetState->orientationQuat.x = stof(parsed[5]);
		targetState->orientationQuat.y = stof(parsed[6]);
		targetState->orientationQuat.z = stof(parsed[7]);

		if ((buttonState & 128) > 0)
		{
			// Reset button
			targetState->SetAsCenter(currentHmdRotation);
			targetState->SetAsZero(targetState->orientationQuat);
		}

		// 90 degree angle correction for the controller
		targetState->orientationQuat = deg90_angleCorrection * targetState->orientationQuat;
	}

	void ControllerData::ParseStringUdp(string packetString)
	{
		ButtonStates* targetState;
		if (packetString[0] == 'L' && packetString[1] == '#')
		{
			targetState = &leftState;

		}
		else if (packetString[0] == 'R' && packetString[1] == '#')
		{
			targetState = &rightState;
		}
		else
		{
			// Wrong packet format
			return;
		}

		packetString.erase(0, 2);
		stringstream sStream;
		sStream.str(packetString);
		string item;
		vector<string> parsed;
		int i = 0;
		while (getline(sStream, item, ';'))
		{
			parsed.push_back(item);
			i++;
		}

		for (int i = 0; i < parsed.size(); i++)
		{
			if (parsed[i] == TIMESTAMP_TAG)
			{
				i++;
				targetState->timestamp = stoll(parsed[i]);
			}
			else if (parsed[i] == FUSED_ORIENTATION_TAG)
			{
				i++;
				targetState->orientation.x = stof(parsed[i]);
				i++;
				targetState->orientation.y = stof(parsed[i]);
				i++;
				targetState->orientation.z = stof(parsed[i]);

			}
			else if (parsed[i] == FUSED_QUATERNION_TAG)
			{
				i++;
				targetState->orientationQuat.w = stof(parsed[i]);
				i++;
				targetState->orientationQuat.x = stof(parsed[i]);
				i++;
				targetState->orientationQuat.y = stof(parsed[i]);
				i++;
				targetState->orientationQuat.z = stof(parsed[i]);
			}
			else if (parsed[i] == setCenter_TAG)
			{
				i++;
				bool reset = stoi(parsed[i]) == 1;
				if (reset)
				{
					targetState->SetAsCenter(currentHmdRotation);
					targetState->ResetZero();
				}
			}
			else if (parsed[i] == HMD_CORRECTION_QUAT_TAG)
			{
				//TODO: remove
				i++;
				i++;
				i++;
				i++;
			}
			else if (parsed[i] == BUTTON_TAG)
			{
				i++;
				int buttonState = stoi(parsed[i]);
				targetState->btn_trigger = (buttonState & 16) > 0;
				targetState->btn_touchpadPress = (buttonState & 8) > 0;
				targetState->btn_menu = (buttonState & 4) > 0;
				targetState->btn_system = (buttonState & 2) > 0;
				targetState->btn_grip = (buttonState & 1) > 0;
			}
			else if (parsed[i] == TRACKPAD_TAG)
			{
				i++;
				targetState->trackpad_touched = !(parsed[i] == "0");
				i++;
				targetState->touchpadX = stof(parsed[i]);
				i++;
				targetState->touchpadY = -stof(parsed[i]);
			}
			else if (parsed[i] == END_TAG)
			{
				//cout << "END tag found" << endl;
			}
			else
			{
				cout << "Unknown data foud: " << parsed[i];
			}
		}
	}

	vr::HmdQuaternion_t ControllerData::GetLeftOrientation()
	{
		return leftState.orientationQuat * leftState.controllerZeroQuaternion * leftState.hmdCenterQuaternion;
	}

	vr::HmdQuaternion_t ControllerData::GetRightOrientation()
	{
		return rightState.orientationQuat * rightState.controllerZeroQuaternion * rightState.hmdCenterQuaternion;
	}

	vr::HmdQuaternion_t* ControllerData::normalizeQauternion(vr::HmdQuaternion_t* quat)
	{
		double mag = quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z;
		quat->w /= mag;
		quat->x /= mag;
		quat->y /= mag;
		quat->z /= mag;
		return quat;
	}

	void ButtonStates::PrintToConsole()
	{
		cout << "Timestamp: " << timestamp << endl
			<< "Orientation: " << orientation.ToString() << endl
			<< "Toucpad: " << touchpadX << ", " << touchpadY << (trackpad_touched ? " touched" : " not touched") << endl
			<< "Trigger: " << btn_trigger << endl
			<< "Touchpad press: " << btn_touchpadPress << endl
			<< "Menu: " << btn_menu << endl
			<< "System: " << btn_system << endl
			<< "Grip: " << btn_grip << endl << endl;
	}

	void ButtonStates::ResetCenter()
	{
		hmdCenterQuaternion.w = 1.0f;
		hmdCenterQuaternion.x = 0.0f;
		hmdCenterQuaternion.y = 0.0f;
		hmdCenterQuaternion.z = 0.0f;
	}

	void ButtonStates::ResetZero()
	{
		controllerZeroQuaternion.w = 1.0f;
		controllerZeroQuaternion.x = 0.0f;
		controllerZeroQuaternion.y = 0.0f;
		controllerZeroQuaternion.z = 0.0f;
	}

	void ButtonStates::SetAsCenter(vr::HmdQuaternion_t hmdRot)
	{
		hmdCenterQuaternion = hmdRot;
		double d = hmdCenterQuaternion.w * hmdCenterQuaternion.w + hmdCenterQuaternion.x * hmdCenterQuaternion.x
			+ hmdCenterQuaternion.y * hmdCenterQuaternion.y + hmdCenterQuaternion.z * hmdCenterQuaternion.z;
		hmdCenterQuaternion.w = (-1.0f * hmdCenterQuaternion.w) / d;
		hmdCenterQuaternion.x = (-1.0f * hmdCenterQuaternion.x) / d;
		hmdCenterQuaternion.y = (-1.0f * hmdCenterQuaternion.y) / d;
		hmdCenterQuaternion.z = (-1.0f * hmdCenterQuaternion.z) / d;

		//hmdCenterQuaternion.w = hmdCenterQuaternion.w * -1.0f;
	}

	void ButtonStates::SetAsZero(vr::HmdQuaternion_t zeroQuat)
	{
		controllerZeroQuaternion = zeroQuat;
		double d = controllerZeroQuaternion.w * controllerZeroQuaternion.w + controllerZeroQuaternion.x * controllerZeroQuaternion.x
			+ controllerZeroQuaternion.y * controllerZeroQuaternion.y + controllerZeroQuaternion.z * controllerZeroQuaternion.z;
		controllerZeroQuaternion.w = (-1.0f * controllerZeroQuaternion.w) / d;
		controllerZeroQuaternion.x = (-1.0f * controllerZeroQuaternion.x) / d;
		controllerZeroQuaternion.y = (-1.0f * controllerZeroQuaternion.y) / d;
		controllerZeroQuaternion.z = (-1.0f * controllerZeroQuaternion.z) / d;

		controllerZeroQuaternion.w = controllerZeroQuaternion.w * -1.0f;
	}

	void ControllerData::SetCurrentHMDOrientation(vr::HmdQuaternion_t hmdRot)
	{
		currentHmdRotation = hmdRot;
	}

	/*void ControllerData::SetGripAngleOffset(float offset)
	{
		gripAngleCorrectionQuat = rotate_around_axis(1.0f, 0.0f, 0.0f, -offset);
	}*/

	void ControllerData::SetGripPositionOffset(float x, float y, float z)
	{
		controllerPositionOffset[0] = x;
		controllerPositionOffset[1] = y;
		controllerPositionOffset[2] = z;
	}

	double* ControllerData::GetGripPositionOffset()
	{
		return controllerPositionOffset;
	}

	std::string Vector3::ToString()
	{
		return to_string(x) + ", " + to_string(y) + ", " + to_string(z);
	}

}