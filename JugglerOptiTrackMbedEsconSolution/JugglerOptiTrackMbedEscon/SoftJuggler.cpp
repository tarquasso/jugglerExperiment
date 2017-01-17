/*
SampleClient.cpp
*/

// #define PI 3.14159265358979;


#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <thread>
using namespace std;

#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "motorDriver.h"
#include "controller.h"




#pragma warning( disable : 4996 )

void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
void resetClient();
int CreateClient(int iConnectionType);

void my_sleep(unsigned long milliseconds) {
	Sleep(milliseconds); // 100 ms
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;

		printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
			device.hardware_id.c_str());
	}
}

unsigned int MyServersDataPort = 3130;
//unsigned int MyServersDataPort = 3883;
unsigned int MyServersCommandPort = 3131;
int iConnectionType = ConnectionType_Multicast;
//int iConnectionType = ConnectionType_Unicast;

NatNetClient* theClient;
Controller mirrorLawController;


char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

int analogSamplesPerMocapFrame = 0;

double fRate = 0.0;
double expectedFramePeriod = 0.0;

long* pPosition = NULL;

BOOL Absolute = TRUE;
BOOL Immediately = TRUE;

DWORD pProfileVelocity;
DWORD pProfileAcceleration;
DWORD pProfileDeceleration;

DWORD ProfileVelocity = 500;
DWORD ProfileAcceleration = 5000;
DWORD ProfileDeceleration = 5000;

float pPositionRad = 0.0;

BOOL pTargetReached = FALSE;

DWORD pBaudrate;
DWORD pTimeOut;
DWORD pErrorCode;


double xPos = 0.0;
double zPos = 0.0;

double xPosOld = 0.0;
double zPosOld = 0.0;

double xVel = 0.0;
double zVel = 0.0;

double q0 = 1;
double q1 = 0;
double q2 = 0;
double q3 = 0;
double psi = 0;


// int _tmain(int argc, _TCHAR* argv[])
int main()
{
	mirrorLawController.init();

	// create a motor object
	//MotorDriver motorObj;

	std::cout << "Main Thread :: ID = " << std::this_thread::get_id() << std::endl;

	int iResult;

	// Create NatNet Client
	iResult = CreateClient(iConnectionType);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");
		return 1;
	}
	else
	{
		printf("Client initialized and ready.\n");
	}


	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if (!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
		printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
		for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
		{
			printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
			if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
			{
				// MarkerSet
				sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
				printf("MarkerSet Name : %s\n", pMS->szName);
				for (int i = 0; i < pMS->nMarkers; i++)
					printf("%s\n", pMS->szMarkerNames[i]);

			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
			{
				// RigidBody
				sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
				printf("RigidBody Name : %s\n", pRB->szName);
				printf("RigidBody ID : %d\n", pRB->ID);
				printf("RigidBody Parent ID : %d\n", pRB->parentID);
				printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
			}
			else
			{
				printf("Unknown data type.");
				// Unknown
			}
		}
	}


	//Init Motor Related thing

	//motor.getPositionProfile(&pProfileVelocity, &pProfileAcceleration, &pProfileDeceleration, &pErrorCode);
	//motor.setPositionProfile(ProfileVelocity, ProfileAcceleration, ProfileDeceleration, &pErrorCode);


	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	int c;
	bool bExit = false;
	while (c = _getch())
	{
		switch (c)
		{
		case 'q':
			bExit = true;
			break;
		case 'r':
			resetClient();
			break;
		case 'p':
			sServerDescription ServerDescription;
			memset(&ServerDescription, 0, sizeof(ServerDescription));
			theClient->GetServerDescription(&ServerDescription);
			if (!ServerDescription.HostPresent)
			{
				printf("Unable to connect to server. Host not present. Exiting.");
				return 1;
			}
			break;
		case 'f':
		{
			sFrameOfMocapData* pData = theClient->GetLastFrameOfData();
			printf("Most Recent Frame: %d", pData->iFrame);
		}
		break;
		case 'm':	                        // change to multicast
			iConnectionType = ConnectionType_Multicast;
			iResult = CreateClient(iConnectionType);
			if (iResult == ErrorCode_OK)
				printf("Client connection type changed to Multicast.\n\n");
			else
				printf("Error changing client connection type to Multicast.\n\n");
			break;
		case 'u':	                        // change to unicast
			iConnectionType = ConnectionType_Unicast;
			iResult = CreateClient(iConnectionType);
			if (iResult == ErrorCode_OK)
				printf("Client connection type changed to Unicast.\n\n");
			else
				printf("Error changing client connection type to Unicast.\n\n");
			break;
		case 'c':                          // connect
			iResult = CreateClient(iConnectionType);
			break;
		case 'd':                          // disconnect
			// note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
			iResult = theClient->SendMessageAndWait("Disconnect", &response, &nBytes);
			if (iResult == ErrorCode_OK)
				printf("[SampleClient] Disconnected");
			break;
		default:
			break;
		}
		if (bExit)
			break;
	}
	// End of OptiTrack Capture code.

		//sFrameOfMocapData* pData = theClient->GetLastFrameOfData();

		//for (int kk = 0; kk < pData->nLabeledMarkers; kk++)
		//{
		//	sMarker marker = pData->LabeledMarkers[kk];
		//	x += marker.x;
		//	z += marker.z;
		//}

		//x /= pData->nLabeledMarkers;
		//z /= pData->nLabeledMarkers;

		//std::cout << "x-position is: " << x << "\n\n";

	// Done - clean up: OptiTrack stuff.
	theClient->Uninitialize();

	return ErrorCode_OK;
}

// Establish a NatNet Client connection
int CreateClient(int iConnectionType)
{
	// release previous server
	if (theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// create NatNet client
	theClient = new NatNetClient(iConnectionType);


	// set the callback handlers
	theClient->SetVerbosityLevel(Verbosity_Warning);
	theClient->SetMessageCallback(MessageHandler);
	theClient->SetDataCallback(DataHandler, theClient);	// this function will receive data from the server
	// [optional] use old multicast group
	//theClient->SetMulticastAddress("224.0.0.1");

	// print version info
	unsigned char ver[4];
	theClient->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Init Client and connect to NatNet server
	// to use NatNet default port assignments
	int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	// to use a different port for commands and/or data:
	//int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// get # of analog samples per mocap frame of data
		void* pResult;
		int ret = 0;
		int nBytes = 0;
		ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			analogSamplesPerMocapFrame = *((int*)pResult);
			printf("Analog Samples Per Mocap Frame : %d", analogSamplesPerMocapFrame);
		}

		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		theClient->GetServerDescription(&ServerDescription);
		if (!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.");
			return 1;
		}
		printf("[SampleClient] Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
			ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2], ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		printf("Client IP:%s\n", szMyIPAddress);
		printf("Server IP:%s\n", szServerIPAddress);
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;

}

// DataHandler receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*)pUserData;

	int i = 0;

	// printf("FrameID : %d\n", data->iFrame);
	//printf("Timestamp :  %3.2lf\n", data->fTimestamp);
	//printf("Latency :  %3.2lf\n", data->fLatency);

	// FrameOfMocapData params
	bool bIsRecording = ((data->params & 0x01) != 0);
	bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
	if (bIsRecording)
		printf("RECORDING\n");
	if (bTrackedModelsChanged)
		printf("Models Changed.\n");

	// get frame rate from host
	void* pResult;
	int ret = 0;
	int nBytes = 0;
	ret = theClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
	if (ret == ErrorCode_OK)
	{
		fRate = *((float*)pResult);
		if (fRate != 0.0f)
			expectedFramePeriod = (1 / fRate);
	}
	if (expectedFramePeriod == 0.0)
		printf("Error establishing Frame Rate.");

	// xPos /= data->nLabeledMarkers;
	// zPos /= data->nLabeledMarkers;

	sDataDescriptions* pDataDefs = NULL;
	int nBodies = pClient->GetDataDescriptions(&pDataDefs);

	for (i = 0; i < pDataDefs->nDataDescriptions; i++)
	{
		if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
		{
			sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
			if (!strcmp("Puck", pRB->szName)) // Puck
			{
				xPos = data->RigidBodies[i].x;
				zPos = data->RigidBodies[i].z;

				xVel = (xPos - xPosOld) * fRate;
				zVel = (zPos - zPosOld) * fRate;

				xPosOld = xPos;
				zPosOld = zPos;
			}
			else if (!strcmp("Paddle", pRB->szName)) // Paddle
			{
				q0 = data->RigidBodies[i].qw;
				q1 = data->RigidBodies[i].qx;
				q2 = data->RigidBodies[i].qy;
				q3 = data->RigidBodies[i].qz;
				psi = atan2(-2 * (q1*q3 + q0*q2), pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2));
			}
		}
	}

	//printf("\t%3.3f\t%3.3f\t%3.2f\t%3.2f\t%3.3f\n",
	//	xPos, zPos, xVel, zVel, psi);


	mirrorLawController.setPaddlePosition(psi);
	mirrorLawController.setBallPosition(xPos, zPos);
	mirrorLawController.setBallVelocity(xVel, zVel);
}

// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = theClient->Uninitialize();
	if (iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if (iSuccess != 0)
		printf("error re-initting Client\n");


}