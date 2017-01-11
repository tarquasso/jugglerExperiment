#pragma once

#define PI 3.14159265358979


#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

class Controller
{
private:
	float r = 0.131;	// Radius of the ball
	float ox = -0.145;	// Offset from the world frame to the paddle frame in the x-direction
	float oz = 0.0;		// Offset from the world frame to the paddle frame in the z-direction

	float x, z, xp, zp;

	float psiRef;
	float sigmaRef;

	float psipRef;
	float sigmapRef;

	Matrix2d JRigidInv;
	// VectorXd xi;
	Vector2d xip;
	// VectorXd qRef;
	Vector2d qpRef;

	float H;		// Vertical energy of the ball
	float Href;		// Reference vertical energy of the ball
	float Htilde;	// Error in vertical energy of the ball

	float g = 9.81;	// Gravitational acceleration
	float beta = PI / 180 * 36;		// Inclination angle of the table

	float kappa0;
	float kappa1;
	float kappa00;
	float kappa01;

public:
	Controller();
	Controller(float x, float z, float xp, float zp);
	~Controller();

	float computeDesiredPaddlePosition();

	Vector2d getBallPosition();
	void setBallPosition(float, float);

	Vector2d getBallVelocity();
	void setBallVelocity(float, float);

	float getReferenceEnergy();
	void setReferenceEnergy(float referenceEnergy);

	void updateReferencePosition();
	void updateReferenceVelocity();

	void computeJacobianInverse();

	float computeVerticalEnergy();

};