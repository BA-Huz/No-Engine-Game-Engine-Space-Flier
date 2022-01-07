#pragma once
#include "Asteroid.h"

class Drone : public Entity
{
	/* 
	Vector3 velocity;
	float mass;
	CoordinateSystem* localCoordinateSystem;
	//ObjModel model;
	ObjLibrary::DisplayList entityDisplayList;
	float calculateOrbitSpeed();
	double radius;*/
	// inherited as protected from 

/*
	Entity();
	~Entity();

	void setVelocity(Vector3 velocity);
	void setMass(float mass);
	void setCoordinateSystem(CoordinateSystem newCoordinateSystem);

	Vector3 getVelocity();
	float getMass();
	CoordinateSystem getLocalCoordinateSystem();
	void elasticCollision(Entity* e2);

		//virtual functions to be derived by child classes
	virtual double getRadius() = 0;
	virtual void update(const microseconds DELTA_TIME);
	virtual void draw(const microseconds DELTA_TIME);

	*/
	// inherited as public

private:
	
	Entity * player;
	Entity * targetCrystal;
	Asteroid * asteroids[100];
	Asteroid* asteroidToAvoid;
	int numOfAsteroids;
	int currentAIState;
	int droneColour;
	float formationAngle;
	Vector3 formationDisplacement;
	Vector3 futureTargetPosition;
	int updatesSpentAvoiding;
	const int UPDATES_TOREASSES_AVOIDING = 30;
	bool closePursueDistance;

	float mainAcceleration;
	float maneuveringAcceleration;
	float* lastUsedAcceleration;
	float turnRateRadians;
	bool showPositions;

	void displayAIState();
	void drawEscortPosition();
	void drawPursuePositions();
	void drawAvoidSphere();

	void drawPredictedPath(const microseconds DELTA_TIME);
	void futurePhysics(Vector3& position, Vector3& velocity, const microseconds DELTA_TIME);
	float calculateArrivalTime(float distance, float accelerationMagnitude, float arrivalSpeed);
	float calculateSafeSpeed(float distance, float accelerationMagnitude, float arrivalSpeed);
	Vector3 desiredRelativeVelocity(float distance, Vector3 targetPosition, Vector3 futurePosition, float arrivalSpeed);
	Vector3 calculateOverallDesiredVelocity();
	float calculateMinimumSafeDistance(Asteroid * asteroid);

	void engageEngines(Vector3 desiredVelocity, const microseconds DELTA_TIME);

	void executeAI(const microseconds DELTA_TIME);
	Vector3 escort(float accelerationMagnitude);
	Vector3 pursue(float accelerationMagnitude);
	Vector3 avoid();


public:
	const static int DRONE_AI_STATE_ESCORT = 0;
	const static int DRONE_AI_STATE_PURSUE = 1;
	const static int DRONE_AI_STATE_AVOID = 2;

	Drone(Vector3 startingPosition, Vector3 initialVelocity, float formationAngle, ObjModel baseModel, const int COLOUR, Entity * player, Asteroid *asteroids[], int numOfAsteroids);
	~Drone();

	void draw(const microseconds DELTA_TIME);
	void update(Entity * crystal, const microseconds DELTA_TIME);
	double getRadius();

	void setShowPositions(bool show);
	void nullifyTargetCrystal();
};