#include "Controller.h"

Controller::Controller()
{
	sigmaRef = 0.0;
	psiRef = 0.0;

	JRigidInv << 0, 0,
				 0, 0;
	
	xip(0) = 0.0;
	xip(1) = 0.0;

	qpRef = JRigidInv*xip;

	psipRef = qpRef(0);
	sigmapRef = qpRef(1);

	H = 1 / 2 * pow(0.0, 2) + g*sin(beta)*0.0;
	Href = getReferenceEnergy();
	Htilde = H - Href;

}

Controller::Controller(float x, float z, float xp, float zp)
{
	//JRigidInv.resize(2, 2);
	//xip.resize(2, 1);
	//qpRef.resize(2, 1);
	
	sigmaRef = sqrt(-pow(r, 2) + pow(ox - x, 2) + pow(oz - z, 2));
	psiRef = atan2((oz - z)*sigmaRef + r*(x - ox), (x - ox)*sigmaRef + r*(z - oz));

	JRigidInv(0,0) = (ox*r - r*x + sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2))*(-oz + z)) /
		((pow(ox - x, 2) + pow(oz - z, 2))*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)));

	JRigidInv(0, 1) = (oz*r + (ox - x)*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)) - r*z) /
		((pow(ox - x, 2) + pow(oz - z, 2))*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)));

	JRigidInv(1, 0) = (-ox + x) / sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2));

	JRigidInv(1,1) = (-oz + z) / sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2));

	xip(0) = xp;
	xip(1) = zp;

	qpRef = JRigidInv*xip;

	psipRef = qpRef(0);
	sigmapRef = qpRef(1);

	H = 1 / 2 * pow(zp, 2) + g*sin(beta)*z;
	Href = getReferenceEnergy();
	Htilde = H - Href;
}

Controller::~Controller()
{

}


Vector2d Controller::getBallPosition()
{
	return Vector2d(x, z);
}

void Controller::setBallPosition(float a, float b)
{
	x = a;
	z = b;
}

Vector2d Controller::getBallVelocity()
{
	return Vector2d(xp, zp);
}

void Controller::setBallVelocity(float a, float b)
{
	xp = a;
	zp = b;
}


float Controller::getReferenceEnergy()
{
	return Href;
}

void Controller::setReferenceEnergy(float referenceEnergy)
{
	Href = referenceEnergy;
}

float Controller::computeVerticalEnergy()
{
	return 1 / 2 * pow(zp, 2) + g*sin(beta)*z;
}

void Controller::computeJacobianInverse()
{
	JRigidInv(0, 0) = (ox*r - r*x + sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2))*(-oz + z)) /
		((pow(ox - x, 2) + pow(oz - z, 2))*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)));

	JRigidInv(0, 1) = (oz*r + (ox - x)*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)) - r*z) /
		((pow(ox - x, 2) + pow(oz - z, 2))*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)));

	JRigidInv(1, 0) = (-ox + x) / sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2));

	JRigidInv(1, 1) = (-oz + z) / sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2));
}


void Controller::updateReferencePosition()
{
	sigmaRef = sqrt(-pow(r, 2) + pow(ox - x, 2) + pow(oz - z, 2));
	psiRef = atan2((oz - z)*sigmaRef + r*(x - ox), (x - ox)*sigmaRef + r*(z - oz));
}


void Controller::updateReferenceVelocity()
{
	computeJacobianInverse();
	Vector2d xip(xp, zp);
	Vector2d refVel = JRigidInv*xip;

	psipRef = refVel(0);
	sigmapRef = refVel(1);
}



float Controller::computeDesiredPaddlePosition()
{
	float rhoBar = ox;
	float rhoRef = 0.0;
	float rhopRef = 0.0;

	H = computeVerticalEnergy();
	Htilde = H - Href;

	updateReferencePosition();
	updateReferenceVelocity();

	rhoRef = sigmaRef*cos(psiRef);
	rhopRef = sigmapRef*cos(psiRef) - sigmapRef*psipRef*sin(psiRef);

	return  -(kappa0 + kappa1*Htilde)*psiRef + kappa00*(rhoRef - rhoBar) + kappa01*rhopRef;
}