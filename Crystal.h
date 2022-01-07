#pragma once
#include "Entity.h"

class Crystal : public Entity
{
	// from enitiy it inherits
	/*
	Vector3 velocity;
	float mass;
	CoordinateSystem * localCoordinateSystem;
	ObjModel model;
	float calculateOrbitSpeed();
	double radius;

	Entity();
	~Entity();

	void setVelocity(Vector3 velocity);
	void setMass(float mass);
	void setCoordinateSystem(CoordinateSystem newCoordinateSystem);

	Vector3 getVelocity();
	float getMass();
	CoordinateSystem getLocalCoordinateSystem();
	double getRadius();

	virtual void update(const microseconds DELTA_TIME);
	//pure virtual functions to be derived by child classes
	virtual void draw(const microseconds DELTA_TIME) = 0;
	virtual double getRadius() = 0;

	void elasticCollision(Entity *e2);
	*/
private:

	Vector3 rotationalAxis;

public:
	Crystal(int id, ObjLibrary::ObjModel baseModel, Vector3 position, Vector3 velocity, Vector3 centerToSpawnPoint);
	int debugID;

	void update(const microseconds DELTA_TIME);
	void draw(const microseconds DELTA_TIME);

	double getRadius();

};
