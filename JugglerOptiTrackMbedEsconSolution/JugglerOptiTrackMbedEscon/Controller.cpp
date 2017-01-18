#include "Controller.h"
#include <iostream>
#include <chrono>

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::duration<double, std::milli> millisec_t;

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
	//reflection gain of mirror law 0.075, // restoring of the energy href 0.01
	this->setGains(0.075, 0.1, 0.05, 0.05);

	/* Following set of gains seems to work alright
	this->setGains(0.05, 0.001, 0.1, 0.05) // also k1 = 1e-4
	*/

	// For heavier puck
	this->setGains(0.3, 0.0, 0.9, 0.5);
}

Controller::~Controller()
{
	delete m_serialComm;
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

void Controller::getGains(double* gainVert, double* gainVerticalDer, double* gainHoriz, double* gainHorizDer)
{
	std::lock_guard<std::mutex> guard(m_mutexGains);
	*gainVert = kappa0;
	*gainVerticalDer = kappa1;
	*gainHoriz = kappa00;
	*gainHorizDer = kappa01;
}

void Controller::setGains(const double& gainVert, const double& gainVerticalDer, const double& gainHoriz, const double& gainHorizDer)
{
	std::lock_guard<std::mutex> guard(m_mutexGains);
	kappa0 = gainVert;
	kappa1 = gainVerticalDer;
	kappa00 = gainHoriz;
	kappa01 = gainHorizDer;
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

void Controller::computeSigmaRef()
{
	ballDistanceSquaredFromHinge = pow(ox - x, 2) + pow(oz - z, 2);
	sigmaRef = sqrt(-pow(r + rz, 2) + ballDistanceSquaredFromHinge) - rx;
}

void Controller::updateReferencePosition()
{
	/* Old calculation
	psiRef = atan2(-(oz - z)*sigmaRef + r*(ox - x), (ox - x)*sigmaRef - r*(oz - z)) - M_PI;
	psiRef = remainder(-psiRef, (2 * M_PI));
	*/
	//psiRef = atan2( (r+rz)*(ox - x) + (sigmaRef+rx)*(oz-z), (sigmaRef+rx)*(ox-x) - (r+rz)*(oz-z) );
	psiRef = atan2((r + rz)*(ox - x) + (sigmaRef + rx)*(oz - z), (sigmaRef + rx)*(ox - x) - (r + rz)*(oz - z));
}


void Controller::updateReferenceVelocity()
{
	computeJacobianInverse();
	Vector2d xip(xp, zp);
	Vector2d refVel = JRigidInv*xip;

	psipRef = refVel(0);
	sigmapRef = refVel(1);
}

void Controller::computeJacobianInverse()
{
	/*Old Jinv
	JRigidInv(0, 0) = (r*(x - ox) + (z - oz)*sigmaRef) / ballDistanceSquaredFromHinge / sigmaRef;
	JRigidInv(0, 1) = (r*(z - oz) + (ox - x)*sigmaRef) / ballDistanceSquaredFromHinge / sigmaRef;
	JRigidInv(1, 0) = (ox - x) / sigmaRef;
	JRigidInv(1, 1) = (oz - z) / sigmaRef;
	*/

	JRigidInv(0, 0) = ((r + rz)*(ox - x) + (sigmaRef + rx)*(oz - z)) / ballDistanceSquaredFromHinge / (sigmaRef + rx);
	JRigidInv(0, 1) = ((r + rz)*(oz - z) - (sigmaRef + rx)*(ox - x)) / ballDistanceSquaredFromHinge / (sigmaRef + rx);
	JRigidInv(1, 0) = -(ox - x) / (sigmaRef + rx);
	JRigidInv(1, 1) = -(oz - z) / (sigmaRef + rx);
}


double Controller::computeDesiredPaddlePosition()
{
	// control vertically
	H = computeVerticalEnergy();
	//Htilde = H - Href;
	Htilde = Href - H;

	computeSigmaRef();
	updateReferencePosition();
	updateReferenceVelocity();

	// Control horizontally
	double rhoBar = rubberLength / 2; //in the rubber frame
	double rhoRef = sigmaRef*cos(psiRef);
	double rhopRef = sigmapRef*cos(psiRef) - sigmapRef*psipRef*sin(psiRef);

	std::lock_guard<std::mutex> guard(m_mutexGains);
	
	return  -(kappa0 + kappa1*Htilde)*psiRef - ( kappa00*(rhoRef - rhoBar) + kappa01*rhopRef);
}



double Controller::computeDesiredPaddleVelocity()
{
	return  -m_positionGain * (m_currentPaddlePositionRad - m_desiredPaddlePositionRad);
}

void Controller::controlArmThread()
{
	double gaink0, gaink1, gaink00, gaink01;

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
		if (m_debug)
		{
			if (m_controlThreadCounter % LOOPCOUNTS_INT == 0)
			{
				printf("sigRef= %3.2f, psiRef= %3.2f, psiDes= %3.2f, psipDes= %3.2f\n",
					sigmaRef, psiRef, m_desiredPaddlePositionRad, m_desiredPaddleVelocityRad);
				//printf("H = %3.2f, \t Href = %3.2f \t Htilde = %3.2f \t \n", H, Href, Htilde);
			}
			if (m_controlThreadCounter % LOOPCOUNTS_INT == 150)
			{
				this->getGains(&gaink0, &gaink1, &gaink00, &gaink01);
				printf("k0 = %4.3f, k1 = %4.3f, k00 = %4.3f, k01 = %4.3f\n",
					gaink0, gaink1, gaink00, gaink01);
			}
		}
		//double incline = computeIncline();
		//printf("The inline is %3.2f degrees.\n", incline);
	}
}

double sgn(double d) {
	double eps = 1e-16;
	return d<-eps ? -1 : d>eps;
}

//double Controller::computeIncline()
//{
//	Vector2d ballPos = m_optiTrackPointer->getBallPosition();
//	z = ballPos(1);
//
//	while (abs(z) > 0.25) {
//		m_optiTrackPointer->getBallPosition();
//		z = ballPos(1);
//		printf("z-coordinate is = %3.2f\n", z);
//	}
//
//	auto t1 = Clock::now();
//	while (abs(z) < 0.25) {
//		m_optiTrackPointer->getBallPosition();
//		z = ballPos(1);
//	}
//	auto t2 = Clock::now();
//
//	millisec_t clkDuration(std::chrono::duration_cast<millisec_t>(t2 - t1));
//	
//	double deltat= clkDuration.count();;
//	double deltaz = 0.5;
//	double incline = ( asin((-2 * deltaz) / g / deltat / deltat) ) * 180 / M_PI;
//
//	return incline;
//}
