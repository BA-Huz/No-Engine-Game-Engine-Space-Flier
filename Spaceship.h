#pragma once
#include "Entity.h"

class Spaceship : public Entity
{
private:
	const int MAX_VELOCITY_MAGNITUDE = 1100;
	ObjModel skybox;

	Vector3 drawCamera();
	void drawPredictedPath(const microseconds DELTA_TIME);
	void drawSkyBox(Vector3 cameraPosition);
	void futurePhysics(Vector3 & position, Vector3 & velocity, const microseconds DELTA_TIME);
	//bool crossesPoint(Vector3 currentPosition, Vector3 futurePosition, Vector3 point, float radiusAroundPoint);
	void capVelocity(Vector3 & velocity);
	
	/***********************************************
		also inherits

		float mass
		float velocity
		CoordinateSystem localCoordinateSystem

		void setVelocity(float velocity);
		void setMass(float mass);
		void setCoordinateSystem(CoordinateSystem newCoordinateSystem);

		float getVelocity();
		float getMass();
		CoordinateSystem getLocalCoordinateSystem();

		virtual void update() = 0;
		virtual void draw() = 0;
		virtual double gerRadius() = 0;

		from Entity parent
	***********************************************/
public:

	Spaceship();
	//Spaceship();

	void rotateAroundForward(float radians, const microseconds DELTA_TIME);
	void rotateAroundUp(float radians, const microseconds DELTA_TIME);
	void rotateAroundRight(float radians, const microseconds DELTA_TIME);
	void accelerateInDirection(Vector3 Direction, bool useFastAcceleration, const microseconds DELTA_TIME);

	//Holds the glut matrix functions for drawing the Spaceship, overides virtual draw from Entity parent
	void draw(const microseconds DELTA_TIME);

	//called by the main files update callback function, is an override of Entity parent Update but also calls parants update
	void update(const microseconds DELTA_TIME);

	Vector3 getCameraPosition();
	double getRadius();

	void moveUp(bool moveUp, const microseconds DELTA_TIME);
	void moveRight(bool moveRight, const microseconds DELTA_TIME);
	void moveForward(bool moveForward, bool fastSpeed, const microseconds DELTA_TIME);
	void rotateAroundForward(bool clockwise, const microseconds DELTA_TIME);
	void rotateAroundUp(bool lookLeft, const microseconds DELTA_TIME);
	void rotateAroundRight(bool lookUp, const microseconds DELTA_TIME);
	void applyBrake(const microseconds DELTA_TIME);

	void setModel(ObjModel baseModel);
};