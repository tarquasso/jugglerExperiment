#include <stdlib.h>     //for using the function sleep
#include <conio.h>
#include <iostream>
#include <math.h>
#include "motorDriver.h"

MotorDriver motor;
long m_lStartPosition = motor.getStartPosition();
double TargetPositionInRadians = 0.0;

double pPositionRad = 0.0;

double psiTolerance = 0.05;
double torqueToCommand = 0.0;
double currentToCommand = 0.0;

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

	DWORD ProfileVelocity = 10;
	DWORD ProfileAcceleration = 100;
	DWORD ProfileDeceleration = 100;

	short pCurrentMust = 0;
	short CurrentMust = 0;
	//WORD AnalogInputNumber = 1000;
	//float Scaling = 1;
	//long Offset = 0;

	motor.getProtocolSettings(&pBaudrate, &pTimeOut, &pErrorCode);
	motor.getPositionProfile(&pProfileVelocity, &pProfileAcceleration, &pProfileDeceleration, &pErrorCode);
	motor.setPositionProfile(ProfileVelocity, ProfileAcceleration, ProfileDeceleration, &pErrorCode);


	motor.getPositionRad(&pPositionRad);
	motor.getCurrentMust(&pCurrentMust, &pErrorCode);

	//std::cout << "Current position is: " << pPositionRad << "\n";
	//std::cout << "Enter target position (in radians): ";
	//std::cin >> TargetPositionInRadians;
	//std::cout << "\n";

	motor.setpsiDesired(TargetPositionInRadians);
	motor.setpsiDotDesired(0.0);

	std::cout << "Current is: " << pCurrentMust << "\n";
	std::cout << "Enter target current (in mA?): ";
	std::cin >> CurrentMust;
	std::cout << "\n";
	
	//if (motor.activatePositionMode(&pErrorCode))
	//	printf("Position Mode Activated.\n");
	if (motor.activateCurrentMode(&pErrorCode))
		printf("Current Mode Activated.\n");
	//if (motor.activateProfileVelocityMode(&pErrorCode))
	//	printf("Velocity Profile Mode Activated.\n");

	while (TRUE)
	{
		if (motor.setCurrentMust(CurrentMust, &pErrorCode))
		{
			printf("CurrentMust Set.\n");
			motor.getCurrentMust(&pCurrentMust, &pErrorCode);
		}

		//while (!pTargetReached)
		//{
			//motor.getPositionRad(&pPositionRad);		// Read Motor Position
			//std::cout << pPositionRad << "\n";

			 //motor.getPositionMust(&pPositionMust, &pErrorCode);
			 

			// motor.moveToPositionRad(TargetPositionInRadians, Absolute, Immediately);

			//motor.getMovementState(&pTargetReached, &pErrorCode);
		//}

		//while (abs(motor.getPsi() - TargetPositionInRadians) > psiTolerance)
		//{
		//	torqueToCommand = motor.computeDIControl();
		//	currentToCommand = motor.torque2Current(torqueToCommand);
		//	currentToCommand = floor(currentToCommand);
		//	
		//	if (abs(currentToCommand) < pow(2, 15))
		//		CurrentMust = (short)currentToCommand;
		//	else
		//		CurrentMust = 0.0;

		//	motor.setCurrentMust(CurrentMust, &pErrorCode);
		//}


		//pTargetReached = FALSE;
		//std::cout << "Current position is: " << pPositionRad << "\n" << "Position reached, starting over... \n";
		//std::cout << "Enter target position (in radians): ";
		//std::cin >> TargetPositionInRadians;
		//std::cout << "\n";

		std::cout << "Current is: " << pCurrentMust << "\n";
		std::cout << "Enter target current (in mA?): ";
		std::cin >> CurrentMust;
		std::cout << "\n";
	}

	return 0;
}
