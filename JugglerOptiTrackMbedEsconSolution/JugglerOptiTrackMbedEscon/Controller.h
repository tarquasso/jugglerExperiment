#ifndef CONTROLLER_H
#define CONTROLLER_H

#define PI 3.14159265358979
#define M_PI 3.14159265358979323846f
#define FLOATSIZE 4
#define LOOP_PERIOD_MS 3

#include <math.h>
#include <Eigen/Dense>
//#include "motorDriver.h"
#include "OptiTrack.h"

#include <iostream>
//#include <string>
//#include <cstdio>
//#include <windows.h>

#include "serial/serial.h"
#include "SerialCommunicator.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using namespace Eigen;

typedef union {
	float floatingPoint;
	uint8_t binary[FLOATSIZE];
} binaryFloat;

class Controller
{
public:
	Controller();
	//Controller(double x, double z, double xp, double zp);
	~Controller();

	int initialize(OptiTrack* optiTrackPointer);
	// double computeIncline();

protected:
	double getReferenceEnergy();
	void setReferenceEnergy(double referenceEnergy);

	void updateReferencePosition();
	void updateReferenceVelocity();

	void computeJacobianInverse();

	double computeVerticalEnergy();

	double computeDesiredPaddlePosition();
	double computeDesiredPaddleVelocity();

	void controlArmThread();

private:
	bool m_initialized;

	double r = 0.086/2;	// Radius of the ball
	double ox = 0.275;	// Offset from the world frame to the paddle frame in the x-direction
	double oz = -0.256;		// Offset from the world frame to the paddle frame in the z-direction

	double x, z, xp, zp;

	double psiRef;
	double sigmaRef;

	double psipRef;
	double sigmapRef;

	Matrix2d JRigidInv;
	// VectorXd xi;
	Vector2d xip;
	// VectorXd qRef;
	Vector2d qpRef;

	double H;		// Vertical energy of the ball
	double Href;		// Reference vertical energy of the ball
	double Htilde;	// Error in vertical energy of the ball

	double g = 9.81;	// Gravitational acceleration
	double beta = PI / 180 * 36;		// Inclination angle of the table

	double kappa0;
	double kappa1;
	double kappa00;
	double kappa01;


	double m_currentPaddlePositionRad;
	double m_desiredPaddlePositionRad;
	double m_desiredPaddleVelocityRad;

	//control thread
	float m_motorVelMeasRadS = 0;
	
	std::mutex m_mutexPaddlePos, m_mutexBallPos, m_mutexBallVel;

	double m_positionGain = 250;

	// DONT TOUCH THOSE OTHER THAN BY GET SET FUNCTIONS
	Vector2d m_ballPosOptiTrack;
	Vector2d m_ballVelOptiTrack;
	double m_paddlePositionOptiTrack = 0.0;
	uint64_t m_controlThreadCounter = 0;
	
	SerialCommunicator* m_serialComm;
	OptiTrack* m_optiTrackPointer;
};

#endif //CONTROLLER_H
