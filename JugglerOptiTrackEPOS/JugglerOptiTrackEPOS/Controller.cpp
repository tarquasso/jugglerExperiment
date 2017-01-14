#include "Controller.h"
#include <iostream>

Controller::Controller():
	my_serial(port, baud, serial::Timeout::simpleTimeout(timeOut))
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

	this->init();

}

// Controller::Controller(double x, double z, double xp, double zp)
// {
	//JRigidInv.resize(2, 2);
	//xip.resize(2, 1);
	//qpRef.resize(2, 1);
	
	//sigmaRef = sqrt(-pow(r, 2) + pow(ox - x, 2) + pow(oz - z, 2));
	//psiRef = atan2((oz - z)*sigmaRef + r*(x - ox), (x - ox)*sigmaRef + r*(z - oz));

	//JRigidInv(0,0) = (ox*r - r*x + sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2))*(-oz + z)) /
	//	((pow(ox - x, 2) + pow(oz - z, 2))*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)));

	//JRigidInv(0, 1) = (oz*r + (ox - x)*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)) - r*z) /
	//	((pow(ox - x, 2) + pow(oz - z, 2))*sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2)));

	//JRigidInv(1, 0) = (-ox + x) / sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2));

	//JRigidInv(1,1) = (-oz + z) / sqrt(pow(ox, 2) - pow(r, 2) - 2 * ox*x + pow(x, 2) + pow(oz - z, 2));

	//xip(0) = xp;
	//xip(1) = zp;

	//qpRef = JRigidInv*xip;

	//psipRef = qpRef(0);
	//sigmapRef = qpRef(1);

	//H = 1 / 2 * pow(zp, 2) + g*sin(beta)*z;
	//Href = getReferenceEnergy();
	//Htilde = H - Href;


//}

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
std::thread threadObj(&MotorDriver::motor_control_thread_function, &this->motorObj, std::ref(motorSetPosition));
if (threadObj.joinable())
{
	//threadObj.join();
	//std::cout << "Joined Thread " << std::endl;
	std::cout << "Detaching Thread " << std::endl;
	threadObj.detach();
}

//// port, baudrate, timeout in milliseconds
//my_serial.setBaudrate(baud);
//my_serial.setPort(port);
//my_serial.setTimeout(serial::Timeout::max(), timeOut, 0, timeOut, 0);

cout << "Is the serial port open?";
if (my_serial.isOpen())
cout << " Yes." << endl;
else
cout << " No." << endl;

//std::cout << "In Main Thread : After Thread Joins motorSetPosition = " << motorSetPosition << std::endl;

}
Vector2d Controller::getBallPosition()
{
	return Vector2d(x, z);
}

void Controller::setBallPosition(double a, double b)
{
	x = a;
	z = b;
}

Vector2d Controller::getBallVelocity()
{
	return Vector2d(xp, zp);
}

void Controller::setBallVelocity(double a, double b)
{
	xp = a;
	zp = b;
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


void Controller::controlArm()
{

	/**********************************************/
	/* Insert motor control commands from here on */

	// Get the Current Position
	//motor.getPosition(pPosition);		// Read Motor Position

	double TargetPositionRad = this->computeDesiredPaddlePosition();		// Need conversion to ticks or something here.

																		// std::cout << "I'm commanding " << TargetPositionRad << "[rad] to the motor.\n";

																		//	while (!pTargetReached)
																		//	{
																		//motor.getPositionRad(&pPositionRad);		// Read Motor Position

																		//motor.moveToPositionRad(TargetPositionRad, Absolute, Immediately);

																		//motor.getMovementState(&pTargetReached, &pErrorCode);

	motorObj.setDesiredMotorPosition(TargetPositionRad);

	sentNumber.floatingPoint = 90.12f;
	uint8_t endMessage = 0x0A;					//New Line Character in Hex.
	size_t bytes_wrote;
	size_t bytes_read = 0;

	//for (int count = 0; count < 1000; count++) {
		bytes_wrote = my_serial.write(sentNumber.binary, FLOATSIZE);
		my_serial.write(&endMessage, 1);

		bytes_read = my_serial.read(incomingData, FLOATSIZE);

		 for (int i = 1; i < FLOATSIZE; i++)
			receivedNumber.binary[i] = incomingData[i];

		cout << "Bytes written: ";
		cout << bytes_wrote << ", What is sent: " << sentNumber.floatingPoint << ", Bytes read: ";
		cout << bytes_read << ", What is read: " << receivedNumber.floatingPoint << endl;
	//}


	//std::cout << "Exiting commandMotor(double, double, double, double)\n";

	//	}
	/* End of motor commands */
	/*************************/
}
