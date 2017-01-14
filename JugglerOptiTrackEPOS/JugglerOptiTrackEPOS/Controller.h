#pragma once

#define PI 3.14159265358979
#define FLOATSIZE 4


#include <math.h>
#include <Eigen/Dense>
#include "motorDriver.h"

#include <iostream>
#include <string>
#include <cstdio>
#include <windows.h>

#include "serial/serial.h"

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
private:
	double r = 0.131;	// Radius of the ball
	double ox = -0.145;	// Offset from the world frame to the paddle frame in the x-direction
	double oz = 0.0;		// Offset from the world frame to the paddle frame in the z-direction

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


	binaryFloat sentNumber;
	binaryFloat receivedNumber;
	uint8_t incomingData[FLOATSIZE];			// don't forget to pre-allocate memory

	string port = "COM5";
	unsigned long baud = 9600;
	int timeOut = 1;

	serial::Serial my_serial;

	Vector2d puckPos;

public:

	MotorDriver motorObj;

public:
	Controller();
	//Controller(double x, double z, double xp, double zp);
	~Controller();

	void init();

	double computeDesiredPaddlePosition();

	Vector2d getBallPosition();
	void setBallPosition(double, double);

	Vector2d getBallVelocity();
	void setBallVelocity(double, double);

	double getReferenceEnergy();
	void setReferenceEnergy(double referenceEnergy);

	void updateReferencePosition();
	void updateReferenceVelocity();

	void computeJacobianInverse();

	double computeVerticalEnergy();
	
	void controlArm();

};