/*
mainJugglerOptiTrackMbedEscon.cpp
*/

#include "OptiTrack.h"
#include "controller.h"

#pragma warning( disable : 4996 )

// int _tmain(int argc, _TCHAR* argv[])
int main()
{
	std::cout << "Main Thread :: ID = " << std::this_thread::get_id() << std::endl;

	OptiTrack optiTrack;

	int errorCode = optiTrack.initialize();
	if (errorCode != ErrorCode_OK)
	{
		printf("Something went wrong initializing OptiTrack.");
		return 1;
	}

	//start controller
	Controller mirrorLawController;
	mirrorLawController.initialize(&optiTrack);

	errorCode = optiTrack.enterMenuMode();
	if (errorCode != ErrorCode_OK)
	{
		printf("Something went wrong in menu mode of OptiTrack.");
		return 1;
	}


	return ErrorCode_OK;
}