#include "Controller.h"
#include <iostream>

Controller::Controller():
	m_ballPosOptiTrack(0,0),
	m_ballVelOptiTrack(0, 0),
	m_initialized(false)
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

Controller::~Controller()
{

}


void Controller::init()
{
// create a motor object

std::cout << "Controller Init, Thread :: ID = " << std::this_thread::get_id() << std::endl;

int motorSetPosition = 0;
//std::cout << "In Main Thread : Before Thread Start motorSetPosition = " << motorSetPosition << std::endl;

std::cout << "Start Motor Thread" << std::endl;

// ToDo: get a handle on that thread
m_initialized = true;

std::thread threadObj(&Controller::controlArmThread, this);
if (threadObj.joinable())
{
	//threadObj.join();
	//std::cout << "Joined Thread " << std::endl;
	std::cout << "Detaching Control Arm Thread " << std::endl;
	threadObj.detach();
}
}

// OPTITRACK
double Controller::getPaddlePosition()
{
	std::lock_guard<std::mutex> guard(m_mutexPaddlePos);
	return m_paddlePositionOptiTrack;
}

void Controller::setPaddlePosition(double paddlePos)
{
	std::lock_guard<std::mutex> guard(m_mutexPaddlePos);
	m_paddlePositionOptiTrack = paddlePos;
}

Vector2d Controller::getBallPosition()
{
	std::lock_guard<std::mutex> guard(m_mutexBallPos);
	return m_ballPosOptiTrack;
}

void Controller::setBallPosition(double x, double z)
{
	std::lock_guard<std::mutex> guard(m_mutexBallPos);
	m_ballPosOptiTrack(0) = x;
	m_ballPosOptiTrack(1) = z;
}

Vector2d Controller::getBallVelocity()
{
	std::lock_guard<std::mutex> guard(m_mutexBallVel);
	return m_ballVelOptiTrack;
}

void Controller::setBallVelocity(double xp, double zp)
{
	std::lock_guard<std::mutex> guard(m_mutexBallVel);
	m_ballVelOptiTrack(0) = xp;
	m_ballVelOptiTrack(1) = zp;
}


double Controller::getReferenceEnergy()
{
	return Href;
}

void Controller::setReferenceEnergy(double referenceEnergy)
{
	Href = referenceEnergy;
}

double Controller::computeVerticalEnergy()
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



double Controller::computeDesiredPaddlePosition()
{
	double rhoBar = ox;
	double rhoRef = 0.0;
	double rhopRef = 0.0;

	H = computeVerticalEnergy();
	Htilde = H - Href;

	updateReferencePosition();
	updateReferenceVelocity();

	rhoRef = sigmaRef*cos(psiRef);
	rhopRef = sigmapRef*cos(psiRef) - sigmapRef*psipRef*sin(psiRef);

	return  -(kappa0 + kappa1*Htilde)*psiRef + kappa00*(rhoRef - rhoBar) + kappa01*rhopRef;
}



double Controller::computeDesiredPaddleVelocity()
{
	return  -m_positionGain * (m_currentPaddlePositionRad - m_desiredPaddlePositionRad);
}

void Controller::controlArmThread()
{
	while (true)
	{
		// Ensure it was initialized!
		if (!m_initialized)
		{
			printf("please init() controller!");
			return;
		}

		// Velocity of Motor from Mbed
		m_motorVelMeasRadS = serialComm.readMotorRadPerSec();

		//Position of Paddle as set by OptiTrack
		m_currentPaddlePositionRad = this->getPaddlePosition();

		//Position and Velocity of Puck
		Vector2d ballPos = this->getBallPosition();
		x = ballPos(0);
		z = ballPos(1);
		Vector2d ballVel = this->getBallVelocity();
		xp = ballVel(0);
		zp = ballVel(1);

		m_desiredPaddlePositionRad = this->computeDesiredPaddlePosition();

		m_desiredPaddleVelocityRad = this->computeDesiredPaddleVelocity();

		this->serialComm.sendMotorRadPerSec((float)m_desiredPaddleVelocityRad);
		m_controlThreadCounter++;
		std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
		if(m_controlThreadCounter % 333 == 0)
			printf("ctrl thread alive!\n");
	}
}
