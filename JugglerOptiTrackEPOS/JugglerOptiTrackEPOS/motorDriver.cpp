#include "motorDriver.h"

motorDriver::motorDriver()
{
	// get first device name
	if (VCS_GetDeviceNameSelection(TRUE, strDeviceName, maxStrSize, &endOfSel, &errorCode))
	{
		//get next device name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetDeviceNameSelection(FALSE, strDeviceName, maxStrSize, &endOfSel, &errorCode);
		}
	}

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

	/* Or just use the following */
	// char* deviceName = "EPOS4";
	// char* protocolStackName = "MAXON_RS232"; // Can be "MAXON SERIAL V2" OR "CANopen"
	// char* interfaceName = "USB"; // Can also be "RS232"
	// char* portName = "COM3"; // Can also be COM1, COM2, ..., USB0, USB1, ..., CAN0, CAN1, ...

	// Open Device
	keyHandle = VCS_OpenDevice(strDeviceName, strProtocolStackName, strInterfaceName,
		strPortName, &errorCode);

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

BOOL motorDriver::getPosition(long* pPosition)
{
	return VCS_GetPositionIs(keyHandle, m_usNodeId, pPosition, &m_ulErrorCode);
}


BOOL motorDriver::moveToPosition(long TargetPosition, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, TargetPosition, Absolute, Immediately, &m_ulErrorCode);
}