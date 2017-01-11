#pragma once

#define maxStrSize 100
#define PI 3.14159265358979

#include "Definitions.h"

class motorDriver
{
private:
	HANDLE keyHandle;
	DWORD m_ulErrorCode;
	WORD m_usNodeId = 1;

	char strDeviceName[maxStrSize];
	char strProtocolStackName[maxStrSize];
	char strInterfaceName[maxStrSize];
	char strPortName[maxStrSize];

	BOOL endOfSel = FALSE;
	DWORD errorCode = 0;

	__int8 m_bMode = 1;
	long m_lStartPosition = 0;

public:
	//char* strDeviceName = "EPOS4";
	//char* strProtocolStackName = "CANopen";
	//char* strInterfaceName = "USB";
	//char* strPortName = "USB0";

	motorDriver();
	~motorDriver();

	long getStartPosition();
	void setStartPosition(long startPosition);

	BOOL getPositionProfile(DWORD* pProfileVelocity, DWORD* pProfileAcceleration,
		DWORD* pProfileDeceleration, DWORD* pErrorCode);
	BOOL setPositionProfile(DWORD ProfileVelocity, DWORD ProfileAcceleration,
		DWORD ProfileDeceleration, DWORD* pErrorCode);

	BOOL getPosition(long* pPosition);
	BOOL getPositionRad(float* pPositionRad);
	BOOL moveToPosition(long TargetPosition, BOOL Absolute, BOOL Immediately);
	BOOL moveToPositionRad(float TargetPositionRad, BOOL Absolute, BOOL Immediately);

	BOOL getMovementState(BOOL* pTargetReached, DWORD* pErrorCode);

	BOOL getProtocolSettings(DWORD* pBaudrate, DWORD* pTimeOut, DWORD* pErrorCode);

	long rad2qc(float angleInRadians);
	float qc2rad(long angleInQuadratureCount);

};