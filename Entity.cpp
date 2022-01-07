#include "Entity.h"


Entity::Entity()
{
	velocity = Vector3(0, 0, 0);
	mass = 1.0;
	localCoordinateSystem = new CoordinateSystem();
}

Entity::~Entity()
{
	delete localCoordinateSystem;
}

void Entity::setVelocity(Vector3 velocity)
{
	this->velocity = velocity;
}

void Entity::setMass(float mass)
{
	this->mass = mass;
}

void Entity::setCoordinateSystem(CoordinateSystem newCoordinateSystem)
{
	delete localCoordinateSystem;
	*localCoordinateSystem = newCoordinateSystem;
}

Vector3 Entity::getVelocity()
{
	return velocity;
}

float Entity::getMass()
{
	return mass;
}

CoordinateSystem Entity::getLocalCoordinateSystem()
{
	return *localCoordinateSystem;
}

void Entity::update(const microseconds DELTA_TIME)
{
	const double G = 0.000000000067408;// gravitiaional constant for black hole
	const double m = 5e16 * 100;// mass of a black hole
	const double d = localCoordinateSystem->getPosition().getNorm(); //distance to the black hole, the norm works to find this because the black hole is at (0,0,0)

	const double accelerationDueToGravityMagnitude = G * m / (d * d);
	Vector3 accelerationDirection = Vector3(0, 0, 0) - localCoordinateSystem->getPosition();
	accelerationDirection.normalizeSafe();
	const Vector3 acceleration = accelerationDirection * accelerationDueToGravityMagnitude;

	const double changeInTime = DELTA_TIME.count() / 1e6;


	velocity = velocity + acceleration * changeInTime;
	localCoordinateSystem->setPosition(localCoordinateSystem->getPosition() + velocity * changeInTime);
}

void Entity::draw(const microseconds DELTA_TIME)
{

}

float Entity::calculateOrbitSpeed()
{
	const double G = 0.000000000067408;// gravitiaional constant for black hole
	const double m = 5e16 * 100;// mass of a black hole
	const double d = localCoordinateSystem->getPosition().getNorm(); //distance to the black hole, the norm works to find this because the black hole is at (0,0,0)
	//s = sqrt(G * m / d)
	return sqrt(G * m / d);
}

void Entity::elasticCollision(Entity *e2)
{
	Entity* e1 = this;
	float m1 = e1->getMass(), m2 = e2->getMass();
	Vector3 v1 = e1->getVelocity(), v2 = e2->getVelocity();


	// momentum -> p
	// center of entity -> c
	// p = m*v
	Vector3 c1 = e1->getLocalCoordinateSystem().getPosition(), c2 = e2->getLocalCoordinateSystem().getPosition();
	//Vector3 p1 = m1 * v1, p2 = m2 * v2;

	//direction from e2 to e1
	Vector3 direction = c1 - c2;
	direction.normalizeSafe();
	// direction is now a unit vector

	//avg velocity of the system va= (m1 / (m1 + m2))v1 + (m2 / (m1 + m2))v2
	Vector3 va = ((m1 / (m1+m2))*v1) + ((m2/ (m1+m2))*v2);

	//relative velocity rv = v - va
	Vector3 rv1 = v1 - va, rv2 = v2 - va;
	// are they moving towards eachother
	if (rv2.dotProduct(direction) < 0)
	{
		return;
	}


	// momentum to be transfered to the other enitity pt = (v - va).project(d) m
	Vector3 p1 = rv1.getProjectionSafe(direction) * m1, p2 = rv2.getProjectionSafe(direction) * m2;
	
	// set the velocitys to va + (p other / m)
	e1->setVelocity(va + (p2 / m1));
	e2->setVelocity(va + (p1 / m2));
}