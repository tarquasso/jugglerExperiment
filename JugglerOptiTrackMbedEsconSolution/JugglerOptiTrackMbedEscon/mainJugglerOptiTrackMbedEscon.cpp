/*
mainJugglerOptiTrackMbedEscon.cpp
*/

#include "OptiTrack.h"
#include "controller.h"
//#include <stdio.h>
#include <conio.h>
#pragma warning( disable : 4996 )


int overallMenuMode(OptiTrack* optiTrackPointer, Controller* controllerPointer)
{
	//void* response;
	//int nBytes;
	int c;
	int iResult = ErrorCode_OK;
	bool bExit = false;
	double k0,k1,k00,k01;
	double increaseFactor = 1.1;
	double decreaseFactor = 0.90909090909;
	while (c = _getch())
	{
		switch (c)
		{
		case 'y':
			controllerPointer->getGains(&k0,&k1,&k00,&k01);
			k0 = k0*increaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'h':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k0 = k0*decreaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'u':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k1 = k1*increaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'j':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k1 = k1*decreaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'i':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k00 = k00*increaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'k':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k00 = k00*decreaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'o':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k01 = k01*increaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'l':
			controllerPointer->getGains(&k0, &k1, &k00, &k01);
			k01 = k01*decreaseFactor;
			controllerPointer->setGains(k0, k1, k00, k01);
			break;
		case 'q':
			bExit = true;
			break;
		case 'r':
			optiTrackPointer->resetClient();
			break;
		//case 'p':
		//	sServerDescription ServerDescription;
		//	memset(&ServerDescription, 0, sizeof(ServerDescription));
		//	m_theClient->GetServerDescription(&ServerDescription);
		//	if (!ServerDescription.HostPresent)
		//	{
		//		printf("Unable to connect to server. Host not present. Exiting.");
		//		return 1;
		//	}
		//	break;
		//case 'f':
		//{
		//	sFrameOfMocapData* pData = m_theClient->GetLastFrameOfData();
		//	printf("Most Recent Frame: %d", pData->iFrame);
		//}
		break;
		//case 'm':	                        // change to multicast
		//	iConnectionType = ConnectionType_Multicast;
		//	iResult = optiTrackPointer->CreateClient(iConnectionType);
		//	if (iResult == ErrorCode_OK)
		//		printf("Client connection type changed to Multicast.\n\n");
		//	else
		//		printf("Error changing client connection type to Multicast.\n\n");
		//	break;
		//case 'u':	                        // change to unicast
		//	iConnectionType = ConnectionType_Unicast;
		//	iResult = CreateClient(iConnectionType);
		//	if (iResult == ErrorCode_OK)
		//		printf("Client connection type changed to Unicast.\n\n");
		//	else
		//		printf("Error changing client connection type to Unicast.\n\n");
		//	break;
		//case 'c':                          // connect
		//	iResult = CreateClient(iConnectionType);
		//	break;
		//case 'd':                          // disconnect
		//								   // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
		//	iResult = m_theClient->SendMessageAndWait("Disconnect", &response, &nBytes);
		//	if (iResult == ErrorCode_OK)
		//		printf("[SampleClient] Disconnected");
		//	break;
		default:
			break;
		}
		if (bExit)
			break;
		// ADDED A SLEEP HERE
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return iResult;
}

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
	Controller controller;
	errorCode = controller.initialize(&optiTrack);
	if (errorCode != ErrorCode_OK)
	{
		printf("Something went wrong initializing the Controller.");
		return 1;
	}

	errorCode = overallMenuMode(&optiTrack, &controller);//optiTrack.enterMenuMode();
	if (errorCode != ErrorCode_OK)
	{
		printf("Something went wrong in menu mode.");
		return 1;
	}

	return ErrorCode_OK;
}