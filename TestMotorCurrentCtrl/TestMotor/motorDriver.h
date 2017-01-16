#pragma once

#define maxStrSize 100
#define PI 3.14159265358979

#include "Definitions.h"
#include <math.h>

class MotorDriver
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

	const double torqueConstant = 85.9 / 1000 / 1000; // Nm/mA

	double zeta = 0.707;	// Damping coefficient
	double pGain = 0.5;
	double dGain = 2*zeta*pGain;
	double iGain = 0.0;

	double psiDesired = 0.0;
	double psiDotDesired = 0.0;

	double psi = 0.0;
	double psiDot = 0.0;

	double psiError = 0.0;
	double psiDotError = 0.0;
	double psiErrorIntegral = 0.0;

	double DT = 0.0;

	__int8 m_bMode = 3;
	//int m_bMode = -3;
	long m_lStartPosition = 0;

public:
	//char* strDeviceName = "EPOS4";
	//char* strProtocolStackName = "CANopen";
	//char* strInterfaceName = "USB";
	//char* strPortName = "USB0";

	MotorDriver();
	~MotorDriver();

	long getStartPosition();
	void setStartPosition(long startPosition);

	double getSampleTime();
	void setSampleTime(double sampleTime);

	double getPsi();
	double getPsiDot();

	double getpsiDesired();
	void setpsiDesired(double a);
	double getpsiDotDesired();
	void setpsiDotDesired(double a);

	BOOL getPositionProfile(DWORD* pProfileVelocity, DWORD* pProfileAcceleration,
		DWORD* pProfileDeceleration, DWORD* pErrorCode);
	BOOL setPositionProfile(DWORD ProfileVelocity, DWORD ProfileAcceleration,
		DWORD ProfileDeceleration, DWORD* pErrorCode);

	BOOL getPosition(long* pPosition);
	BOOL getPositionRad(double* pPositionRad);
	BOOL moveToPosition(long TargetPosition, BOOL Absolute, BOOL Immediately);
	BOOL moveToPositionRad(double TargetPositionRad, BOOL Absolute, BOOL Immediately);

	BOOL activatePositionMode(DWORD* pErrorCode);
	BOOL setPositionMust(long PositionMust, DWORD* pErrorCode);
	BOOL getPositionMust(long* pPositionMust, DWORD* pErrorCode);
	BOOL activateAnalogPositionSetPoint(WORD AnalogInputNumber, float Scaling, long Offset, DWORD* pErrorCode);
	BOOL deactivateAnalogPositionSetPoint(WORD AnalogInputNumber, DWORD* pErrorCode);
	BOOL enableAnalogPositionSetPoint(DWORD* pErrorCode);
	BOOL disableAnalogPositionSetPoint(DWORD* pErrorCode);

	BOOL activateProfileVelocityMode(DWORD* pErrorCode);

	BOOL activateCurrentMode(DWORD* pErrorCode);
	BOOL getCurrentMust(short* pCurrentMust, DWORD* pErrorCode);
	BOOL setCurrentMust(short CurrentMust, DWORD* pErrorCode);

	BOOL getMovementState(BOOL* pTargetReached, DWORD* pErrorCode);
	BOOL getPositionIs(long* pPositionIs, DWORD* pErrorCode);
	BOOL getVelocityIs(long* pVelocityIs, DWORD* pErrorCode);
	BOOL getVelocityIsAveraged(long* pVelocityIsAveraged, DWORD* pErrorCode);

	BOOL getProtocolSettings(DWORD* pBaudrate, DWORD* pTimeOut, DWORD* pErrorCode);

	long rad2qc(double angleInRadians);
	double qc2rad(long angleInQuadratureCount);

	void computePsiError();
	void computePsiErrorIntegral();
	void computePsiDotError();

	double computeDIControl();

	double torque2Current(double torqueValue);
};