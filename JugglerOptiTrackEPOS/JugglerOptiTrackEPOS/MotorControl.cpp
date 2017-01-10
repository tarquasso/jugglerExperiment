// MotorControl.cpp: Communicate with the motor through EPOS.

// #include "stdafx.h"
#include "Definitions.h"

#define PI	3.14159265358979;

int main()
{
	DWORD m_ulErrorCode;
	WORD m_usNodeId = 1;
	const WORD maxStrSize = 100;

	char* strDeviceName = "EPOS2";
	char* strProtocolStackName[maxStrSize];

	BOOL endOfSel;
	DWORD errorCode;

	//get first protocol stack name
	if (VCS_GetProtocolStackNameSelection(strDeviceName, TRUE, *strProtocolStackName,
		maxStrSize, &endOfSel, &errorCode))
	{
		//get next protocol stack name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetProtocolStackNameSelection(strDeviceName, FALSE, *strProtocolStackName,
				maxStrSize, &endOfSel, &errorCode);
		}
	}

	endOfSel = FALSE;
	errorCode = 0;
	char* strInterfaceName[maxStrSize];

	//get first interface name
	if (VCS_GetInterfaceNameSelection(strDeviceName, *strProtocolStackName, TRUE, *strInterfaceName,
		maxStrSize, &endOfSel, &errorCode))
	{
		//get next interface name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetInterfaceNameSelection(strDeviceName, *strProtocolStackName, FALSE, *strInterfaceName,
				maxStrSize, &endOfSel, &errorCode);
		}
	}


	endOfSel = FALSE;
	errorCode = 0;
	char* strPortName[maxStrSize];

	//get first port name
	if (VCS_GetPortNameSelection(strDeviceName, *strProtocolStackName, *strInterfaceName,
		TRUE, *strPortName, maxStrSize, &endOfSel, &errorCode))
	{
		//get next port name (as long as endOfSel == FALSE)
		while (!endOfSel)
		{
			VCS_GetPortNameSelection(strDeviceName, *strProtocolStackName, *strInterfaceName,
				FALSE, *strPortName, maxStrSize, &endOfSel, &errorCode);
		}
	}

	// Open Device
	HANDLE motorHandle;
	motorHandle = VCS_OpenDevice(strDeviceName, *strProtocolStackName, *strInterfaceName, 
		*strPortName, &errorCode);

	// Set Enable State
	VCS_SetEnableState(motorHandle, m_usNodeId, &m_ulErrorCode);

	// Set Operation Mode
	__int8 m_bMode = -1;
	VCS_SetOperationMode(motorHandle, m_usNodeId, m_bMode, &m_ulErrorCode);

	// Get the Current Position
	long m_lStartPosition = 0;
	VCS_GetPositionIs(motorHandle, m_usNodeId, &m_lStartPosition, &m_ulErrorCode);

	// Move To Position
	long m_lTargetPosition = 2000;
	BOOL m_oRadio = FALSE;
	BOOL m_oImmediately = TRUE;
	VCS_MoveToPosition(motorHandle, m_usNodeId, m_lTargetPosition, m_oRadio, m_oImmediately, &m_ulErrorCode);

	// Close Device
	VCS_CloseDevice(motorHandle, &errorCode);

	return 0;
}