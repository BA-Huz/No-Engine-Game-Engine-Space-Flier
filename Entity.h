#pragma once
#include "CoordinateSystem.h"
#include <chrono>

using namespace std;
using namespace chrono;

// an abstract class that will be a parent for objects that have coordinates, velocities, and mass
class Entity
{
protected:
	Vector3 velocity;
	float mass;
	CoordinateSystem * localCoordinateSystem;
	//ObjModel model;
	ObjLibrary::DisplayList entityDisplayList;
	float calculateOrbitSpeed();
	double radius;


public:

	Entity();
	~Entity();

	void setVelocity(Vector3 velocity);
	void setMass(float mass);
	void setCoordinateSystem(CoordinateSystem newCoordinateSystem);

	Vector3 getVelocity();
	float getMass();
	CoordinateSystem getLocalCoordinateSystem();
	virtual double getRadius() = 0;

	virtual void update(const microseconds DELTA_TIME);
	//virtual functions to be derived by child classes
	virtual void draw(const microseconds DELTA_TIME);

	void elasticCollision(Entity *e2);
	
};