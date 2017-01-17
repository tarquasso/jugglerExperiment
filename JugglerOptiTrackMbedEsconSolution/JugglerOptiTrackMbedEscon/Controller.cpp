#include "Controller.h"
#include <iostream>

Controller::Controller() :
	m_ballPosOptiTrack(0, 0),
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

	setReferenceEnergy(0.0);
	H = 1 / 2 * pow(0.0, 2) + g*sin(beta)*0.0;
	Href = getReferenceEnergy();
	Htilde = H - Href;

	kappa0 = 0.075; //reflection gain of mirror law 0.075
	kappa1 = 0.01; // restoring of the energy href 0.01
	kappa00 = 0.0;
	kappa01 = 0.0;
}

Controller::~Controller()
{

}


int Controller::initialize(OptiTrack* optiTrackPointer)
{
	if (m_initialized)
	{
		printf("Controller already initialized.  Exiting");
	}

	m_optiTrackPointer = optiTrackPointer; //ToDo: check if pointer was initilaized
	try {
		m_serialComm = new SerialCommunicator();
	}
	catch(...)
	{
		// catch any serial errors!
		printf("COM Port not open!");
		return 1;
	}

	// create a motor object

	std::cout << "Controller Init, Thread :: ID = " << std::this_thread::get_id() << std::endl;

	int motorSetPosition = 0;
	//std::cout << "In Main Thread : Before Thread Start motorSetPosition = " << motorSetPosition << std::endl;

	// ToDo: get a handle on that thread
	m_initialized = true;

	// Start the Thread!

	std::thread threadObj(&Controller::controlArmThread, this);
	if (threadObj.joinable())
	{
		//threadObj.join();
		//std::cout << "Joined Thread " << std::endl;
		std::cout << "Detaching Control Arm Thread " << std::endl;
		threadObj.detach();
	}
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
	double ballDistanceSquaredFromHinge = pow(ox - x, 2) + pow(oz - z, 2);
	double sigma = sqrt(-pow(r, 2) + ballDistanceSquaredFromHinge);

	JRigidInv(0, 0) = (r*(x - ox) + (z - oz)*sigma) / ballDistanceSquaredFromHinge / sigma;
	JRigidInv(0, 1) = (r*(z - oz) + (ox - x)*sigma) / ballDistanceSquaredFromHinge / sigma;
	JRigidInv(1, 0) = (ox - x) / sigma;
	JRigidInv(1, 1) = (oz - z) / sigma;
}


void Controller::updateReferencePosition()
{
	sigmaRef = -sqrt(-pow(r, 2) + pow(ox - x, 2) + pow(oz - z, 2));
	psiRef = atan2(-(oz - z)*sigmaRef + r*(ox - x), (ox - x)*sigmaRef - r*(oz - z)) - M_PI;
	psiRef = remainder(-psiRef, (2 * M_PI));
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

		// printf("I'm in the controller thread.\n");

		// Velocity of Motor from Mbed
		m_motorVelMeasRadS = this->m_serialComm->readMotorRadPerSec();

		//Position of Paddle as set by OptiTrack
		m_currentPaddlePositionRad = m_optiTrackPointer->getPaddlePosition();

		//Position and Velocity of Puck

		Vector2d ballPos = m_optiTrackPointer->getBallPosition();
		x = ballPos(0);
		z = ballPos(1);
		Vector2d ballVel = m_optiTrackPointer->getBallVelocity();
		xp = ballVel(0);
		zp = ballVel(1);

		m_desiredPaddlePositionRad = this->computeDesiredPaddlePosition();

		m_desiredPaddleVelocityRad = this->computeDesiredPaddleVelocity();

		this->m_serialComm->sendMotorRadPerSec((float)m_desiredPaddleVelocityRad);
		m_controlThreadCounter++;
		std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_PERIOD_MS));
		if (m_controlThreadCounter % LOOPCOUNTS_INT == 0)
			printf("sigmaRef = %3.2f, \t psiRef = %3.2f, \t psiDes = %3.2f, \t psipDes = %3.2f\n",
				sigmaRef, psiRef, m_desiredPaddlePositionRad, m_desiredPaddleVelocityRad);
	}
}
