#include "motorDriver.h"
#include <stdio.h>
#include <iostream>

MotorDriver::MotorDriver()
{
	zeta = 0.707;
	pGain = 0.75;
	dGain = 2 * zeta*pGain;
	iGain = 0.0;

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

	if (VCS_ClearFault(keyHandle, m_usNodeId, &errorCode))
	{
		printf("Faults cleared.\n");
	}

	// Set Enable State
	VCS_SetEnableState(keyHandle, m_usNodeId, &m_ulErrorCode);

	// Set Operation Mode
	if (VCS_SetOperationMode(keyHandle, m_usNodeId, m_bMode, &m_ulErrorCode))
		printf("Operation Mode Set.\n");
}

MotorDriver::~MotorDriver()
{
	// Set Disable State
	VCS_SetDisableState(keyHandle, m_usNodeId, &m_ulErrorCode);

	// Close Device
	VCS_CloseDevice(keyHandle, &errorCode);
}

long MotorDriver::getStartPosition()
{
	return m_lStartPosition;
}

void MotorDriver::setStartPosition(long startPosition)
{
	m_lStartPosition = startPosition;
}

double MotorDriver::getSampleTime()
{
	return DT;
}

void MotorDriver::setSampleTime(double sampleTime)
{
	DT = sampleTime;
}

double MotorDriver::getPsi()
{
	long dummy;
	DWORD dummyErrorCode;
	BOOL temp = VCS_GetPositionIs(keyHandle, m_usNodeId, &dummy, &dummyErrorCode);

	psi = qc2rad(dummy);
	return psi;
}

double MotorDriver::getPsiDot()
{
	long dummy;
	DWORD dummyErrorCode;
	BOOL temp = VCS_GetVelocityIs(keyHandle, m_usNodeId, &dummy, &dummyErrorCode);

	psiDot = qc2rad(dummy);
	return psiDot;
}

double MotorDriver::getpsiDesired()
{
	return psiDesired;
}

void MotorDriver::setpsiDesired(double a)
{
	psiDesired = a;
}

double MotorDriver::getpsiDotDesired()
{
	return psiDotDesired;
}

void MotorDriver::setpsiDotDesired(double a)
{
	psiDotDesired = a;
}

BOOL MotorDriver::getPositionProfile(DWORD* pProfileVelocity, DWORD* pProfileAcceleration, 
	DWORD* pProfileDeceleration, DWORD* pErrorCode)
{
	return VCS_GetPositionProfile(keyHandle, m_usNodeId, pProfileVelocity,
		pProfileAcceleration, pProfileDeceleration, pErrorCode);
}

BOOL MotorDriver::setPositionProfile(DWORD ProfileVelocity, DWORD ProfileAcceleration, DWORD ProfileDeceleration, DWORD * pErrorCode)
{
	return VCS_SetPositionProfile(keyHandle, m_usNodeId, ProfileVelocity, 
		ProfileAcceleration, ProfileDeceleration, pErrorCode);
}

BOOL MotorDriver::getPosition(long* pPosition)
{
	return VCS_GetPositionIs(keyHandle, m_usNodeId, pPosition, &m_ulErrorCode);
}

BOOL MotorDriver::getPositionRad(double * pPositionRad)
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


BOOL MotorDriver::moveToPosition(long TargetPosition, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, TargetPosition, Absolute, Immediately, &m_ulErrorCode);
}

BOOL MotorDriver::moveToPositionRad(double TargetPositionRad, BOOL Absolute, BOOL Immediately)
{
	return VCS_MoveToPosition(keyHandle, m_usNodeId, rad2qc(TargetPositionRad), Absolute, Immediately, &m_ulErrorCode);
}

BOOL MotorDriver::activatePositionMode(DWORD * pErrorCode)
{
	return VCS_ActivatePositionMode(keyHandle, m_usNodeId, pErrorCode);
}

BOOL MotorDriver::setPositionMust(long PositionMust, DWORD * pErrorCode)
{
	return VCS_SetPositionMust(keyHandle, m_usNodeId, PositionMust, pErrorCode);
}

BOOL MotorDriver::getPositionMust(long * pPositionMust, DWORD * pErrorCode)
{
	return VCS_GetPositionMust(keyHandle, m_usNodeId, pPositionMust, pErrorCode);
}

BOOL MotorDriver::activateAnalogPositionSetPoint(WORD AnalogInputNumber, float Scaling, long Offset, 
	DWORD * pErrorCode)
{
	return VCS_ActivateAnalogPositionSetpoint(keyHandle, m_usNodeId, AnalogInputNumber, Scaling, 
		Offset, pErrorCode);
}

BOOL MotorDriver::deactivateAnalogPositionSetPoint(WORD AnalogInputNumber, DWORD * pErrorCode)
{
	return VCS_DeactivateAnalogPositionSetpoint(keyHandle, m_usNodeId, AnalogInputNumber, pErrorCode);
}

BOOL MotorDriver::enableAnalogPositionSetPoint(DWORD * pErrorCode)
{
	return VCS_EnableAnalogPositionSetpoint(keyHandle, m_usNodeId, pErrorCode);
}

BOOL MotorDriver::disableAnalogPositionSetPoint(DWORD * pErrorCode)
{
	return VCS_DisableAnalogPositionSetpoint(keyHandle, m_usNodeId, pErrorCode);
}


BOOL MotorDriver::activateProfileVelocityMode(DWORD * pErrorCode)
{
	return VCS_ActivateProfileVelocityMode(keyHandle, m_usNodeId, pErrorCode);
}

BOOL MotorDriver::activateCurrentMode(DWORD * pErrorCode)
{
	return VCS_ActivateCurrentMode(keyHandle, m_usNodeId, pErrorCode);
}

BOOL MotorDriver::getCurrentMust(short * pCurrentMust, DWORD * pErrorCode)
{
	return VCS_GetCurrentMust(keyHandle, m_usNodeId, pCurrentMust, pErrorCode);
}

BOOL MotorDriver::setCurrentMust(short CurrentMust, DWORD * pErrorCode)
{
	return VCS_SetCurrentMust(keyHandle, m_usNodeId, CurrentMust, pErrorCode);
}


BOOL MotorDriver::getMovementState(BOOL * pTargetReached, DWORD * pErrorCode)
{
	return VCS_GetMovementState(keyHandle, m_usNodeId, pTargetReached, pErrorCode);
}

BOOL MotorDriver::getPositionIs(long* pPositionIs, DWORD* pErrorCode) 
{
	BOOL temp = VCS_GetPositionIs(keyHandle, m_usNodeId, pPositionIs, pErrorCode);
	psi = qc2rad(*pPositionIs);

	return temp;
}

BOOL MotorDriver::getVelocityIs(long* pVelocityIs, DWORD* pErrorCode)
{
	BOOL temp = VCS_GetVelocityIs(keyHandle, m_usNodeId, pVelocityIs, pErrorCode);
	psiDot = qc2rad(*pVelocityIs);

	return temp;
}

BOOL MotorDriver::getVelocityIsAveraged(long * pVelocityIsAveraged, DWORD * pErrorCode)
{
	BOOL temp = VCS_GetVelocityIsAveraged(keyHandle, m_usNodeId, pVelocityIsAveraged, pErrorCode);
	psiDot = qc2rad(*pVelocityIsAveraged);

	return temp;
}

BOOL MotorDriver::getProtocolSettings(DWORD * pBaudrate, DWORD * pTimeOut, DWORD * pErrorCode)
{
	return VCS_GetProtocolStackSettings(keyHandle, pBaudrate, pTimeOut, pErrorCode);
}

long MotorDriver::rad2qc(double angleInRadians)
{
	return 20000 / 2 / PI * (angleInRadians);
}

double MotorDriver::qc2rad(long angleInQuadratureCount)
{
	return 2 * PI / 20000 * (angleInQuadratureCount);
}

void MotorDriver::computePsiError()
{
	getPsi();
	psiError = psi - psiDesired;
}

void MotorDriver::computePsiErrorIntegral()
{
	psiErrorIntegral += psiError*DT;
}

void MotorDriver::computePsiDotError()
{
	getPsiDot();
	psiDotError = psiDot - psiDotDesired;
}

double MotorDriver::computeDIControl()
{
	computePsiError();
	computePsiDotError();
	computePsiErrorIntegral();

	std::cout << pGain << "\n";

	return -pGain*psiError - dGain*psiDotError - iGain*psiErrorIntegral;
}

double MotorDriver::torque2Current(double torqueValue)
{
	return torqueValue / torqueConstant;
}