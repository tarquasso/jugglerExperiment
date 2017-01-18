#ifndef OPTITRACK_H
#define OPTITRACK_H

#define PUCK_NAME "PuckHeavy"
//#define PUCKNAME "Puck"

#include "NatNetTypes.h"
#include "NatNetClient.h"
#include <mutex>
#include <Eigen/Dense>
using namespace Eigen;

class OptiTrack
{
public:
	OptiTrack();
	~OptiTrack();
	int initialize();
	void resetClient();
	int enterMenuMode();

	double getPaddlePosition();
	Vector2d getBallPosition();
	Vector2d getBallVelocity();

	void dataCallback(sFrameOfMocapData* data);

protected:
	int CreateClient(int iConnectionType);

	void setPaddlePosition(const double&);
	void setBallPosition(const double&, const double&);
	void setBallVelocity(const double&, const double&);

private:
	bool m_initialized;
	unsigned int MyServersDataPort = 3130;
	//unsigned int MyServersDataPort = 3883;
	unsigned int MyServersCommandPort = 3131;
	int iConnectionType = ConnectionType_Multicast; //ConnectionType_Unicast;

	NatNetClient* m_theClient;

	char szMyIPAddress[128] = "";
	char szServerIPAddress[128] = "";

	int analogSamplesPerMocapFrame = 0;

	double fRate = 0.0;
	double expectedFramePeriod = 0.0;

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

	// DONT TOUCH THOSE OTHER THAN BY GET SET FUNCTIONS
	Vector2d m_ballPosOptiTrack;
	Vector2d m_ballVelOptiTrack;
	double m_paddlePositionOptiTrack = 0.0;
	 
	std::mutex m_mutexPaddlePos, m_mutexBallPos, m_mutexBallVel;

};

//void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
//void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages

void __cdecl dataCallback(sFrameOfMocapData *, void *);
void __cdecl messageCallback(int, char*);

#endif //OPTITRACK_H
