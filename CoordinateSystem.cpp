#include "CoordinateSystem.h"

// default constructor
CoordinateSystem::CoordinateSystem()
{
	position.set(-1750.0, 0.0, 0.0);
	forward.set(1.0, 0.0, 0.0);
	up.set(0.0, 1.0, 0.0);
	right = forward.crossProduct(up);
	//rotationalAngle = 0.0;

	moveRate = 2;
	turnRate = 0.1;
}

// initializing constructor with only position
CoordinateSystem::CoordinateSystem(Vector3 position)
{
	this->position = position;
	forward.set(1.0, 0.0, 0.0);
	up.set(0.0, 1.0, 0.0);
	right = forward.crossProduct(up);
	//rotationalAngle = 0.0;

	moveRate = 2;
	turnRate = 0.1;
}

// initializing constructor without the rates
CoordinateSystem::CoordinateSystem(Vector3 position, Vector3 forward, Vector3 up)
{
	this->position = position;
	this->forward = forward;
	this->up = up;
	right = forward.crossProduct(up);
	//this->rotationalAngle = rotationalAngle;

	moveRate = 2.0;
	turnRate = 0.1;
}

// initializing constructor with rates
CoordinateSystem::CoordinateSystem(Vector3 position, Vector3 forward, Vector3 up, double moveRate, double turnRate, float rotationalAngle)
{
	this->position = position;
	this->forward = forward;
	this->up = up;
	right = forward.crossProduct(up);
	//this->rotationalAngle = rotationalAngle;

	this->moveRate = moveRate;
	this->turnRate = turnRate;
}

// destructor
CoordinateSystem::~CoordinateSystem()
{
	// there is no dynamically allocated memory to deal with
}

// getter for the position Vector3
const Vector3& CoordinateSystem::getPosition() const
{
	return position;
}

// getter for the forward Vector3
const Vector3& CoordinateSystem::getForward() const
{
	return forward;
}

// getter for the up Vector3
const Vector3& CoordinateSystem::getUp() const
{
	return up;
}

// getter for the right Vector3
const Vector3& CoordinateSystem::getRight() const
{
	return right;
}

// getter for the move rate
const double CoordinateSystem::getMoveRate()
{
	return moveRate;
}

// getter for the turn rate
const double CoordinateSystem::getTurnRate()
{
	return turnRate;
}

// getter for the rotational angle
const float CoordinateSystem::getArbitraryRotationalAngle()
{
	return arbitraryRotationalAngle;
	//return 0.0;
}


//initializes the camera properly if the camera is a coordinate system
void CoordinateSystem::setupCamera() const  // calls gluLookAt
{
	Vector3 look_at = position + forward;
	gluLookAt(position.x, position.y, position.z,
		look_at.x, look_at.y, look_at.z,
		up.x, up.y, up.z);

	return;
}

// setter for position
void CoordinateSystem::setPosition(const Vector3& position)
{
	this->position = position;

	return;
}

//sets the full orientation(forward, up, and right) using the provided forward and up, right ends up the cross product
void CoordinateSystem::setOrientation(const Vector3& forward, const Vector3& up)
{
	this->forward = forward;
	this->up = up;
	right = forward.crossProduct(up);

	return;
}

// setter for move rate
void CoordinateSystem::setMoveRate(double rate)
{
	moveRate = rate;
}

// setter for turn rate
void CoordinateSystem::setTurnRate(double rate)
{
	turnRate = rate;
}

/****************************************************

smooth members are designed to be repeaditly called
and make use of this classes moveRate and turnRate.

jump members can be called once with a given rate
or be called repeaditly like the smooth functions
but with a different rate without altering this
classes rates.

****************************************************/

// jumps the camera in its local forwards direction
void CoordinateSystem::jumpForward(double distance)
{
	position += forward * distance;

	return;
}

// moves the camera in its local forwards direction, intended to be repeatidly called to acheive smooth movement
void CoordinateSystem::smoothMoveForward(bool moveForward)
{
	if (moveForward)
		position += forward * moveRate;
	else
		position -= forward * moveRate;

	return;
}

// jumpss the camera in its local upwards direction
void CoordinateSystem::jumpUp(double distance)
{
	position += up * distance;

	return;
}

// moves the camera in its local upwards direction, intended to be repeatidly called to acheive smooth movement
void CoordinateSystem::smoothMoveUp(bool moveUp)
{
	if (moveUp)
		position += up * moveRate;
	else
		position -= up * moveRate;

	return;
}

// jumpss the camera in its local right direction
void CoordinateSystem::jumpRight(double distance)
{
	position += right * distance;

	return;
}

// moves the camera in its local right direction, intended to be repeatidly called to acheive smooth movement
void CoordinateSystem::smoothMoveRight(bool moveRight)
{
	if (moveRight)
		position += right * moveRate;
	else
		position -= right * moveRate;

	return;
}

// rotates the camera around its local forward
void CoordinateSystem::rotationalJumpAroundForward(double radians)
{
	up.rotateArbitrary(forward, radians);
	right.rotateArbitrary(forward, radians);

	return;
}

// rotates the camera around its local forward, intended to be called repeatidly to acheive smooth movement
void CoordinateSystem::smoothRotateAroundForward(bool clockwise)
{
	if (clockwise)
	{
		up.rotateArbitrary(forward, turnRate);
		right.rotateArbitrary(forward, turnRate);
	}
	else
	{
		up.rotateArbitrary(forward, -turnRate);
		right.rotateArbitrary(forward, -turnRate);
	}

	return;
}

// rotates the camera around its local up
void CoordinateSystem::rotationalJumpAroundUp(double radians)
{
	forward.rotateArbitrary(up, radians);
	right.rotateArbitrary(up, radians);

	return;
}

// rotates the camera around its local up, intended to be called repeatidly to acheive smooth movement
void CoordinateSystem::smoothRotateAroundUp(bool lookLeft)
{
	if (lookLeft)
	{
		forward.rotateArbitrary(up, turnRate);
		right.rotateArbitrary(up, turnRate);
	}
	else
	{
		forward.rotateArbitrary(up, -turnRate);
		right.rotateArbitrary(up, -turnRate);
	}

	return;
}

// rotates the camera around its local right
void CoordinateSystem::CoordinateSystem::rotationalJumpAroundRight(double radians)
{
	forward.rotateArbitrary(right, radians);
	up.rotateArbitrary(right, radians);

	return;
}

// rotates the camera around its local right, intended to be called repeatidly to acheive smooth movement
void CoordinateSystem::smoothRotateAroundRight(bool lookUp)
{
	if (lookUp)
	{
		forward.rotateArbitrary(right, turnRate);
		up.rotateArbitrary(right, turnRate);
	}
	else
	{
		forward.rotateArbitrary(right, -turnRate);
		up.rotateArbitrary(right, -turnRate);
	}

	return;
}

// rotates the camera around a given axis
void CoordinateSystem::rotationalJumpAroundArbitrary(Vector3 axis, double radians)
{
	axis.normalize();
	forward.rotateArbitrary(axis, radians);
	up.rotateArbitrary(axis, radians);
	right.rotateArbitrary(axis, radians);
	arbitraryRotationalAngle += radians;
	arbitraryRotationalAngle = fmod(arbitraryRotationalAngle, 6.238);

	return;
}

// rotates the camera around a given axis, intended to be called repeatidly to acheive smooth movement
void CoordinateSystem::smoothRotatearoundArbitrary(Vector3 axis, bool positiveTurnRate)
{
	if (positiveTurnRate)
	{
		forward.rotateArbitrary(axis, turnRate);
		up.rotateArbitrary(axis, turnRate);
		right.rotateArbitrary(axis, turnRate);
		arbitraryRotationalAngle += turnRate;
		arbitraryRotationalAngle = fmod(arbitraryRotationalAngle, 6.238);
	}
	else
	{
		forward.rotateArbitrary(axis, -turnRate);
		up.rotateArbitrary(axis, -turnRate);
		right.rotateArbitrary(axis, -turnRate);
		arbitraryRotationalAngle -= turnRate;
		arbitraryRotationalAngle = fmod(arbitraryRotationalAngle, 6.238);
	}

	return;
}

//rotates the camera towards given vector using a max of given turn rate
void CoordinateSystem::rotateToVectorWithTurnRate(const Vector3& desired_forward, double max_radians)
{
	if (desired_forward.isZero())
		return;

	Vector3 axis = forward.crossProduct(desired_forward);
	if (axis.isZero())
		axis = up;
	else
		axis.normalize();

	double radians = forward.getAngleSafe(desired_forward);
	if (radians > max_radians)
		radians = max_radians;

	forward.rotateArbitrary(axis, radians);
	up.rotateArbitrary(axis, radians);
	right.rotateArbitrary(axis, radians);

	return;
}

//rotates the camera towards given vector using a max of the cameras turn rate
void CoordinateSystem::rotateToVector(const Vector3& desired_forward)
{
	if (desired_forward.isZero())
		return;

	Vector3 axis = forward.crossProduct(desired_forward);
	if (axis.isZero())
		axis = up;
	else
		axis.normalize();

	double radians = forward.getAngleSafe(desired_forward);
	if (radians > turnRate)
		radians = turnRate;

	forward.rotateArbitrary(axis, radians);
	up.rotateArbitrary(axis, radians);
	right.rotateArbitrary(axis, radians);

	return;
}

void CoordinateSystem::applyDrawTransformations() const
{
	// code for translation will go here
	glTranslated(position.x, position.y, position.z);
	// code for rotation will go here
	double matrix[16];

	calculateOrientationMatrix(matrix);
	glMultMatrixd(matrix);
}

void CoordinateSystem::calculateOrientationMatrix(double a_matrix[]) const
{
	a_matrix[0] = forward.x;
	a_matrix[1] = forward.y;
	a_matrix[2] = forward.z;
	a_matrix[3] = 0.0;
	a_matrix[4] = up.x;
	a_matrix[5] = up.y;
	a_matrix[6] = up.z;
	a_matrix[7] = 0.0;
	a_matrix[8] = right.x;
	a_matrix[9] = right.y;
	a_matrix[10] = right.z;
	a_matrix[11] = 0.0;
	a_matrix[12] = 0.0;
	a_matrix[13] = 0.0;
	a_matrix[14] = 0.0;
	a_matrix[15] = 1.0;
}

ObjLibrary::Vector3 CoordinateSystem::localToWorld(const ObjLibrary::Vector3& local) const
{
	return
		forward * local.x +
		up * local.y +
		right * local.z;
}

ObjLibrary::Vector3 CoordinateSystem::worldToLocal(const ObjLibrary::Vector3& world) const
{
	Vector3 row1(forward.x, up.x, right.x);
	Vector3 row2(forward.y, up.y, right.y);
	Vector3 row3(forward.z, up.z, right.z);

	//Vector3 offset = world - position;
	return row1 * world.x +
		row2 * world.y +
		row3 * world.z;
}