#include "motorDriver.h"

motorDriver::motorDriver()
{
	char deviceName[maxStrSize] = "EPOS4";
	char protocolStackName[maxStrSize] = "MAXON SERIAL V2"; // Can be "MAXON_RS232", "MAXON SERIAL V2" OR "CANopen"
	char interfaceName[maxStrSize] = "USB"; // Can be "USB" "RS232"
	char portName[maxStrSize] = "USB0"; // Can also be COM1, COM2, ..., USB0, USB1, ..., CAN0, CAN1, ...

	// strDeviceName = "EPOS4";
	//strProtocolStackName = "CANopen"; // Can be "MAXON_RS232", "MAXON SERIAL V2" OR "CANopen"
	//strInterfaceName = "USB"; // Can be "USB" "RS232"
	//strPortName = "USB0"; // Can also be COM1, COM2, ..., USB0, USB1, ..., CAN0, CAN1, ...

	// get first device name
	if (VCS_GetDeviceNameSelection(TRUE, strDeviceName, maxStrSize, &endOfSel, &errorCode))
	{
		//get next device name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetDeviceNameSelection(FALSE, strDeviceName, maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;

	//get first protocol stack name
	if (VCS_GetProtocolStackNameSelection(strDeviceName, TRUE, strProtocolStackName,
		maxStrSize, &endOfSel, &errorCode))
	{
		//get next protocol stack name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetProtocolStackNameSelection(strDeviceName, FALSE, strProtocolStackName,
				maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;

	//get first interface name
	if (VCS_GetInterfaceNameSelection(strDeviceName, strProtocolStackName, TRUE, strInterfaceName,
		maxStrSize, &endOfSel, &errorCode))
	{
		//get next interface name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetInterfaceNameSelection(strDeviceName, strProtocolStackName, FALSE, strInterfaceName,
				maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;

	//get first port name
	if (VCS_GetPortNameSelection(strDeviceName, strProtocolStackName, strInterfaceName,
		TRUE, strPortName, maxStrSize, &endOfSel, &errorCode))
	{
		//get next port name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetPortNameSelection(strDeviceName, strProtocolStackName, strInterfaceName,
				FALSE, strPortName, maxStrSize, &endOfSel, &errorCode);
		}
	}	

	// Open Device
	//keyHandle = VCS_OpenDevice(strDeviceName, strProtocolStackName, strInterfaceName,
		//strPortName, &errorCode);
	keyHandle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName,
		portName, &errorCode);

	// Set Enable State
	VCS_SetEnableState(keyHandle, m_usNodeId, &m_ulErrorCode);

	// Set Operation Mode
	VCS_SetOperationMode(keyHandle, m_usNodeId, m_bMode, &m_ulErrorCode);
}

motorDriver::~motorDriver()
{
	// Set Disable State
	VCS_SetDisableState(keyHandle, m_usNodeId, &m_ulErrorCode);

	// Close Device
	VCS_CloseDevice(keyHandle, &errorCode);
}

long motorDriver::getStartPosition()
{
	return m_lStartPosition;
}

void motorDriver::setStartPosition(long startPosition)
{
	m_lStartPosition = startPosition;
}

BOOL motorDriver::getPositionProfile(DWORD* pProfileVelocity, DWORD* pProfileAcceleration, 
	DWORD* pProfileDeceleration, DWORD* pErrorCode)
{
	return VCS_GetPositionProfile(keyHandle, m_usNodeId, pProfileVelocity,
		pProfileAcceleration, pProfileDeceleration, pErrorCode);
}

BOOL motorDriver::setPositionProfile(DWORD ProfileVelocity, DWORD ProfileAcceleration, DWORD ProfileDeceleration, DWORD * pErrorCode)
{
	return VCS_SetPositionProfile(keyHandle, m_usNodeId, ProfileVelocity, 
		ProfileAcceleration, ProfileDeceleration, pErrorCode);
}

BOOL motorDriver::getPosition(long* pPosition)
{
	return VCS_GetPositionIs(keyHandle, m_usNodeId, pPosition, &m_ulErrorCode);
}

BOOL motorDriver::getPositionRad(float * pPositionRad)
{
	long posQC = 0;
	if (VCS_GetPositionIs(keyHandle, m_usNodeId, &posQC, &m_ulErrorCode))
	{
		*pPositionRad = qc2rad(posQC);
		return TRUE;
	}
	else
		return FALSE;
}


BOOL motorDriver::moveToPosition(long TargetPosition, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, TargetPosition, Absolute, Immediately, &m_ulErrorCode);
}

BOOL motorDriver::moveToPositionRad(float TargetPositionRad, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, rad2qc(TargetPositionRad), Absolute, Immediately, &m_ulErrorCode);
}

BOOL motorDriver::getMovementState(BOOL * pTargetReached, DWORD * pErrorCode)
{
	return VCS_GetMovementState(keyHandle, m_usNodeId, pTargetReached, pErrorCode);
}

BOOL motorDriver::getProtocolSettings(DWORD * pBaudrate, DWORD * pTimeOut, DWORD * pErrorCode)
{
	return VCS_GetProtocolStackSettings(keyHandle, pBaudrate, pTimeOut, pErrorCode);
}

long motorDriver::rad2qc(float angleInRadians)
{
	return 20000 / 2 / PI * (angleInRadians);
}

float motorDriver::qc2rad(long angleInQuadratureCount)
{
	return 2 * PI / 20000 * (angleInQuadratureCount);
}
