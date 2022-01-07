#include "Crystal.h"

Crystal::Crystal(int id, ObjLibrary::ObjModel baseModel, Vector3 position, Vector3 asteroidVelocity, Vector3 centerToSpawnPoint)
{
	debugID = id;
	// Velocity of the crystal should be the velocity of the asteroid it broke off from
	// plus the unit vector from the asteroids center to the break off point times some value y
	// plus some value x in some random directionthat is perpedicular to the unitvector from the asteroids center
	// sqrt(x2 + y2) = 10 m/s
	//this will cause crystals to bounce away from the asteroid irregardless of the asteroids velocity
	int intx = rand() % 81 + 10; //intx is random between 10 and 90
	float x = (float)intx * .1; // x is random between 1.0 and 9.0
	float y = sqrt(100 - x*x); // x and y such that sqrt(x^2 + y^2) = 10

	//velocity of the asteroid  + y * unit vector direction from asteroid to crystal spawn point      + x * unit vector direction that is perpedicular to direction from asteroid to spawn point
	velocity = asteroidVelocity + y*10.0 * (centerToSpawnPoint / centerToSpawnPoint.getNorm())             + x*10.0 * (Vector3::getRandomUnitVector().getRejectionSafe(centerToSpawnPoint));
	// I also added another *10 but this is to give the crystals more oomph when they break off the asteroid

	mass = 1;
	localCoordinateSystem = new CoordinateSystem(position, Vector3(1,0,0), Vector3(0,1,0));

	ObjLibrary::DisplayList crystalDisplayList(baseModel.getDisplayList());
	entityDisplayList = crystalDisplayList;
	radius = 2; // I thought 2 was too small for crystal radius so i used 20 for testing

	rotationalAxis = Vector3::getRandomUnitVector();
	float rotationSpeed = (float)(rand() % 25 + 1);
	rotationSpeed /= 100;
	localCoordinateSystem->setTurnRate(rotationSpeed);

	// give an initial random angle in radians around the random rotatinal axis
	localCoordinateSystem->rotationalJumpAroundArbitrary(rotationalAxis, ((float)(rand() % 6283)) / 1000.0);

}

void Crystal::update(const microseconds DELTA_TIME)
{
	Entity::update(DELTA_TIME);

	// rotate asteroid based off its own rotational momentum
	localCoordinateSystem->smoothRotatearoundArbitrary(rotationalAxis, true);
}

void Crystal::draw(const microseconds DELTA_TIME)
{

	glPushMatrix();
	Vector3 position = localCoordinateSystem->getPosition();
	glTranslated(position.x, position.y, position.z);
	// convert the coordinaT systems arbitrary angle from radians to degrees
	glRotated(localCoordinateSystem->getArbitraryRotationalAngle() * (180.0 / 3.141592653589793238463), rotationalAxis.x, rotationalAxis.y, rotationalAxis.z);
	glScaled(radius/0.7, radius / 0.7, radius / 0.7);

	entityDisplayList.draw();
	glPopMatrix();
}

double Crystal::getRadius()
{
	return radius;
}
