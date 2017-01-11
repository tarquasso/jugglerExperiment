//=============================================================================
// Copyright � 2014 NaturalPoint, Inc. All Rights Reserved.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

// #define PI 3.14159265358979;


#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <iostream>
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
void commandMotor(float x, float z);

unsigned int MyServersDataPort = 3130;
//unsigned int MyServersDataPort = 3883;
unsigned int MyServersCommandPort = 3131;
int iConnectionType = ConnectionType_Multicast;
//int iConnectionType = ConnectionType_Unicast;

NatNetClient* theClient;

char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

int analogSamplesPerMocapFrame = 0;

double fRate = 0.0;
double expectedFramePeriod = 0.0;

motorDriver motor;
long m_lStartPosition = motor.getStartPosition();
float TargetPositionRad = 0;

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


Controller mirrorLaw;
float xPos = 0.0;
float zPos = 0.0;

float xPosOld = 0.0;
float zPosOld = 0.0;

float xVel = 0.0;
float zVel = 0.0;

// int _tmain(int argc, _TCHAR* argv[])
int main()
{
    int iResult;

    // Create NatNet Client
    iResult = CreateClient(iConnectionType);
    if(iResult != ErrorCode_OK)
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
	if(!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
        }
	}


	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	int c;
	bool bExit = false;
	while(c =_getch())
	{
		switch(c)
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
                if(!ServerDescription.HostPresent)
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
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Multicast.\n\n");
                else
                    printf("Error changing client connection type to Multicast.\n\n");
                break;
            case 'u':	                        // change to unicast
                iConnectionType = ConnectionType_Unicast;
                iResult = CreateClient(iConnectionType);
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Unicast.\n\n");
                else
                    printf("Error changing client connection type to Unicast.\n\n");
                break;
            case 'c' :                          // connect
                iResult = CreateClient(iConnectionType);
                break;
            case 'd' :                          // disconnect
                // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
                iResult = theClient->SendMessageAndWait("Disconnect", &response, &nBytes);
                if (iResult == ErrorCode_OK)
                    printf("[SampleClient] Disconnected");
                break;
			default:
				break;
		}
		if(bExit)
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
    if(theClient)
    {
        theClient->Uninitialize();
        delete theClient;
    }

    // create NatNet client
    theClient = new NatNetClient(iConnectionType);



    // set the callback handlers
    theClient->SetVerbosityLevel(Verbosity_Warning);
    theClient->SetMessageCallback(MessageHandler);
    theClient->SetDataCallback( DataHandler, theClient );	// this function will receive data from the server
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
        if(!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
            ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
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
	NatNetClient* pClient = (NatNetClient*) pUserData;

    int i=0;

    // printf("FrameID : %d\n", data->iFrame);
    //printf("Timestamp :  %3.2lf\n", data->fTimestamp);
    //printf("Latency :  %3.2lf\n", data->fLatency);

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        printf("RECORDING\n");
    if(bTrackedModelsChanged)
        printf("Models Changed.\n");

	/* // Other Markers
	printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	for(i=0; i < data->nOtherMarkers; i++)
	{
		printf("Other Marker %d : %3.2f\t%3.2f\t%3.2f\n",
			i,
			data->OtherMarkers[i][0],
			data->OtherMarkers[i][1],
			data->OtherMarkers[i][2]);
	}*/


	// labeled markers
    bool bOccluded;     // marker was not visible (occluded) in this frame
    bool bPCSolved;     // reported position provided by point cloud solve
    bool bModelSolved;  // reported position provided by model solve
	printf("Labeled Markers [Count=%d]\n", data->nLabeledMarkers);
	for(i=0; i < data->nLabeledMarkers; i++)
	{
        bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);
        bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);
        bModelSolved = ((data->LabeledMarkers[i].params & 0x04)!=0);
		sMarker marker = data->LabeledMarkers[i];
        int modelID, markerID;
        theClient->DecodeID(marker.ID, &modelID, &markerID);
		printf("Labeled Marker [ModelID=%d, MarkerID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
            modelID, markerID, bOccluded, bPCSolved, bModelSolved,  marker.size, marker.x, marker.y, marker.z);

		xPos += marker.x;
		zPos += marker.z;
	}
	// End of data receipt.
	
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

	xPos /= data->nLabeledMarkers;
	zPos /= data->nLabeledMarkers;

	xVel = (xPos - xPosOld) * fRate;
	zVel = (zPos - zPosOld) * fRate;

	std::cout << expectedFramePeriod << "\n\n";



	commandMotor(xPos, zPos, xVel, zVel);
	xPosOld = xPos;
	zPosOld = zPos;
	xPos = 0.0;
	zPos = 0.0;
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
	if(iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if(iSuccess != 0)
		printf("error re-initting Client\n");


}


void commandMotor(float x, float z, float xp, float zp)
{
	mirrorLaw.setBallPosition(x, z);
	mirrorLaw.setBallVelocity(xp, zp);

	/**********************************************/
	/* Insert motor control commands from here on */

	// Get the Current Position
	motor.getPosition(pPosition);		// Read Motor Position

	motor.getPositionProfile(&pProfileVelocity, &pProfileAcceleration, &pProfileDeceleration, &pErrorCode);
	motor.setPositionProfile(ProfileVelocity, ProfileAcceleration, ProfileDeceleration, &pErrorCode);

	TargetPositionRad = mirrorLaw.computeDesiredPaddlePosition();		// Need conversion to ticks or something here.

	std::cout << "I'm commanding " << TargetPositionRad << "[rad] to the motor.\n";

//	while (!pTargetReached)
//	{
		motor.getPositionRad(&pPositionRad);		// Read Motor Position

		motor.moveToPositionRad(TargetPositionRad, Absolute, Immediately);

		motor.getMovementState(&pTargetReached, &pErrorCode);
//	}
	/* End of motor commands */
	/*************************/
}