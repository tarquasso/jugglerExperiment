#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>

typedef union {
	float floatingPoint;
	char binary[4];
} binaryFloat;


// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	printf("Welcome to the serial test app!\n\n");

	//Serial* SP = new Serial("\\\\.\\COM10");    // adjust as needed
	Serial* SP = new Serial("\\\\.\\COM3");    // adjust as needed


	if (SP->IsConnected())
		printf("We're connected");

	char incomingData[256] = "";			// don't forget to pre-allocate memory
											//printf("%s\n",incomingData);
	int dataLength = 255;
	int readResult = 0;

	//char buffer[10] = "";
	//sprintf_s(buffer, "%6.4f\n", 2.1000);
	// strcat_s(buffer, "\n");
	//int i = 1;

	binaryFloat hi;
	hi.floatingPoint = 1.1f;
	binaryFloat rec;


	// while (SP->IsConnected() && i < 5)
	while (SP->IsConnected())
	{
		SP->WriteData(hi.binary, 4);
		SP->WriteData("\n", 1);
	    //printf("%s", "I've successfully sent something.\n");

	    readResult = SP->ReadData(incomingData, 100);
		//printf("Bytes read: (0 means no data available) %i\n",readResult);
		//incomingData[readResult] = 0;
		//if()
		rec.binary[0] = incomingData[0];
			rec.binary[1] = incomingData[1];
			rec.binary[2] = incomingData[2];
			rec.binary[3] = incomingData[3];

		printf("%f\n", rec.floatingPoint);

			//i += 1;
	
		Sleep(400);
	}


	return 0;
}