#include "Spaceship.h"
#include <iostream>
using namespace std;

Spaceship::Spaceship()
{
	Vector3 direction;
	direction = Vector3::getRandomUnitVector();
	direction = direction.getRejectionSafe(localCoordinateSystem->getPosition() * -1.0);
	direction.normalizeSafe();
	velocity = direction * (calculateOrbitSpeed() * ((rand() % 11 + 5) / 10.0));


	mass = 1000.0;
	localCoordinateSystem = new CoordinateSystem();

	radius = 2;

	skybox.load("Skybox.obj");
}


void Spaceship::rotateAroundForward(float radians, const microseconds DELTA_TIME)
{
	localCoordinateSystem->rotationalJumpAroundForward(radians);
}

void Spaceship::rotateAroundUp(float radians, const microseconds DELTA_TIME)
{
	localCoordinateSystem->rotationalJumpAroundUp(radians);
}

void Spaceship::rotateAroundRight(float radians, const microseconds DELTA_TIME)
{
	localCoordinateSystem->rotationalJumpAroundRight(radians);
}

void Spaceship::accelerateInDirection(Vector3 accelerationDirection, bool useFastAcceleration, const microseconds DELTA_TIME)
{
	int accelerationMagnitude;
	if (useFastAcceleration)
		accelerationMagnitude = 10000;
	else
		accelerationMagnitude = 200;
	Vector3 acceleration = accelerationDirection * accelerationMagnitude;
	

      	velocity = velocity + acceleration * (DELTA_TIME.count() / 1e6);
}

//Holds the glut matrix functions for drawing the Spaceship, overides virtual draw from Entity parent
void Spaceship::draw(const microseconds DELTA_TIME)
{
	Vector3 cameraPosition = drawCamera();
	drawSkyBox(cameraPosition);
	drawPredictedPath(DELTA_TIME);

	Vector3 position = localCoordinateSystem->getPosition();

	glPushMatrix();
	glTranslated(position.x, position.y, position.z);


	double  a_matrix[16];
	localCoordinateSystem->calculateOrientationMatrix(a_matrix);


	glMultMatrixd(a_matrix);

	glScaled(2, 2, 2);
	//glScaled(.5, .5, .5);

	entityDisplayList.draw();

	glPopMatrix();
}

Vector3 Spaceship::getCameraPosition()
{
	//Vector3 cameraPosition;
	//Vector3 shipPosition = localCoordinateSystem->getPosition();
	//Vector3 shipsbackwards = -1.0 * localCoordinateSystem->getForward();
	//Vector3 shipsUp = localCoordinateSystem->getUp();
	//cameraPosition = shipPosition + shipsbackwards * 20.0;
	//cameraPosition += shipsUp * 5.0;

	//return cameraPosition;

	return localCoordinateSystem->getPosition() + (-1.0 * localCoordinateSystem->getForward() * 20.0) + (localCoordinateSystem->getUp() * 5.0);
}

double Spaceship::getRadius()
{
	return radius;
}

//called by the main files update callback function, is an override of Entity parent Update but also calls parants update
void Spaceship::update(const microseconds DELTA_TIME)
{
	Entity::update(DELTA_TIME);
	if (velocity.getNorm() > MAX_VELOCITY_MAGNITUDE)
		capVelocity(velocity);
}

Vector3 Spaceship::drawCamera()
{
	Vector3 cameraPosition = getCameraPosition();
	Vector3 look_at = cameraPosition + localCoordinateSystem->getForward();
	Vector3 up = localCoordinateSystem->getUp();

	gluLookAt(cameraPosition.x, cameraPosition.y, cameraPosition.z,  // (x, y, z) camera eye position //*******************************
		look_at.x, look_at.y, look_at.z,  // (x, y, z) camera look at position
		up.x, up.y, up.z); // (x, y, z) camera up direction

	return cameraPosition;
}

void Spaceship::drawPredictedPath(const microseconds DELTA_TIME)
{
	Vector3 predictedPosition = localCoordinateSystem->getPosition();
	Vector3 predictedVelocity = velocity;
	Vector3 p;

	glPushMatrix();
	glBegin(GL_LINE_STRIP);

	for (int i = 0; i < 3000; i++)
	{
		glColor3d(0, 0, 1 - i/6000); // blue getting lighter
		p = predictedPosition;
		glVertex3d(predictedPosition.x, predictedPosition.y, predictedPosition.z);
		futurePhysics(predictedPosition, predictedVelocity, DELTA_TIME);  // updates predicted position and predicted velocity
		if(predictedVelocity.getNorm() > MAX_VELOCITY_MAGNITUDE)
			capVelocity(predictedVelocity);
		glVertex3d(predictedPosition.x, predictedPosition.y, predictedPosition.z);
		//if(crossesPoint(p, predictedPosition, Vector3(0,0,0), 1)) // if spaceship is going to colide with blackhole stop drawing
		//	i = 500;
	}

	glEnd();
	glPopMatrix();

}

void Spaceship::drawSkyBox(Vector3 cameraPosition)
{
	glPushMatrix();
	glTranslated(cameraPosition.x, cameraPosition.y, cameraPosition.z);
	glScaled(10.0, 10.0, 10.0);
	glDepthMask(GL_FALSE);
	skybox.draw();
	glDepthMask(GL_TRUE);
	glPopMatrix();
}

void Spaceship::moveUp(bool moveUp, const microseconds DELTA_TIME)
{
	//localCoordinateSystem->smoothMoveUp(moveUp);
	if(moveUp)
		accelerateInDirection(localCoordinateSystem->getUp(), false, DELTA_TIME);
	else
		accelerateInDirection(localCoordinateSystem->getUp() * -1.0, false, DELTA_TIME);
}

void Spaceship::moveRight(bool moveRight, const microseconds DELTA_TIME)
{
	//localCoordinateSystem->smoothMoveRight(moveRight);
	if (moveRight)
		accelerateInDirection(localCoordinateSystem->getRight(), false, DELTA_TIME);
	else
		accelerateInDirection(localCoordinateSystem->getRight() * -1.0, false, DELTA_TIME);
}

void Spaceship::moveForward(bool moveForward, bool fastSpeed, const microseconds DELTA_TIME)
{
	/*if (!fastSpeed)
		localCoordinateSystem->smoothMoveForward(moveForward);
	else
		localCoordinateSystem->jumpForward(100.0);*/
    if (moveForward)
		accelerateInDirection(localCoordinateSystem->getForward(), fastSpeed, DELTA_TIME);
	else
		accelerateInDirection(localCoordinateSystem->getForward() * -1.0, fastSpeed, DELTA_TIME);
}

void Spaceship::rotateAroundForward(bool clockwise, const microseconds DELTA_TIME)
{
	localCoordinateSystem->smoothRotateAroundForward(clockwise);
}

void Spaceship::rotateAroundUp(bool lookLeft, const microseconds DELTA_TIME)
{
	localCoordinateSystem->smoothRotateAroundUp(lookLeft);
}

void Spaceship::rotateAroundRight(bool lookUp, const microseconds DELTA_TIME)
{
	localCoordinateSystem->smoothRotateAroundRight(lookUp);
}


void Spaceship::futurePhysics(Vector3 & position, Vector3 & velocity, const microseconds DELTA_TIME)
{
	const double G = 0.000000000067408;// gravitiaional constant for black hole
	const double m = 5e16 * 100;// mass of a black hole
	const double d = position.getNorm(); //distance to the black hole, the norm works to find this because the black hole is at (0,0,0)

	const double accelerationDueToGravityMagnitude = G * m / (d * d);
	Vector3 accelerationDirection = Vector3(0, 0, 0) - position;
	accelerationDirection.normalize();
	const Vector3 acceleration = accelerationDirection * accelerationDueToGravityMagnitude;

	const double changeInTime = (DELTA_TIME.count() / 1e6) / 2.0;

	velocity = velocity + acceleration * changeInTime;
	//if (velocity.getNorm() > 1000)
	//	velocity /= 2;

	position = position + velocity * changeInTime;
	//localCoordinateSystem->setPosition(localCoordinateSystem->getPosition() + velocity * changeInTime);
}


void Spaceship::capVelocity(Vector3& velocity)
{
	// if velocity magnitude > max velocity then set it to max velocity 
	//         (      velocity direction     ) * Max velocity
	velocity = (velocity / velocity.getNorm()) * MAX_VELOCITY_MAGNITUDE;
}

void Spaceship::applyBrake(const microseconds DELTA_TIME)
{
	float magnitude = velocity.getNorm();
	Vector3 breakDirection = velocity / magnitude * -1.0;
	if (magnitude > 10)
		velocity = velocity + (magnitude * (0.3) * breakDirection);
	else
		velocity = Vector3(0,0,0);
}

void Spaceship::setModel(ObjModel baseModel)
{
	ObjModel model = baseModel;
	ObjLibrary::DisplayList spaceshipDisplayList(model.getDisplayList());
	entityDisplayList = spaceshipDisplayList;
}

