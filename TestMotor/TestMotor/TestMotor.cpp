#include <stdlib.h>     //for using the function sleep
#include <conio.h>
#include <iostream>
#include "motorDriver.h"

motorDriver motor;
long m_lStartPosition = motor.getStartPosition();
float TargetPositionInRadians = 20*PI;

float pPositionRad = 0.0;

BOOL Absolute = TRUE;
BOOL Immediately = TRUE;

int main()
{	
	int c;
	bool bExit = false;

	DWORD pBaudrate;
	DWORD pTimeOut;
	DWORD pErrorCode;
	BOOL pTargetReached = FALSE;

	DWORD pProfileVelocity;
	DWORD pProfileAcceleration;
	DWORD pProfileDeceleration;

	DWORD ProfileVelocity = 500;
	DWORD ProfileAcceleration = 5000;
	DWORD ProfileDeceleration = 5000;

	motor.getProtocolSettings(&pBaudrate, &pTimeOut, &pErrorCode);
	motor.getPositionProfile(&pProfileVelocity, &pProfileAcceleration, &pProfileDeceleration, &pErrorCode);
	motor.setPositionProfile(ProfileVelocity, ProfileAcceleration, ProfileDeceleration, &pErrorCode);


	motor.getPositionRad(&pPositionRad);

	std::cout << "Current position is: " << pPositionRad << "\n";
	std::cout << "Enter target position (in radians): ";
	std::cin >> TargetPositionInRadians;
	std::cout << "\n";

	while(!pTargetReached)
	{
	motor.getPositionRad(&pPositionRad);		// Read Motor Position
	//std::cout << pPositionRad << "\n";

	motor.moveToPositionRad(TargetPositionInRadians, Absolute, Immediately);

	motor.getMovementState(&pTargetReached, &pErrorCode);
	}

	std::cout << "Current position is: " << pPositionRad << "\n" << "Position reached, exiting... \n";
	
	return 0;
}
