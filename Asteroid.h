#pragma once
#include "CoordinateSystem.h"
#include "ObjLibrary/ObjModel.h"
#include "ObjLibrary/DisplayList.h"
#include "PerlinNoiseField.h"
#include "Entity.h"
#include <string>
//#include <chrono>


using namespace ObjLibrary;
using namespace std;
//using namespace chrono;

class Asteroid : public Entity
{
private:
	double innerRadius;
	double avgRadius;
	Vector3 rotationalAxis;
	bool showAxesAndMarkers = false;
	bool hasCrystals;
	double perlinAmplitude;
	Vector3 perlinOffset;
	PerlinNoiseField * noise;

	//ObjLibrary::DisplayList asteroidDisplayList;

	float calculateMass();
	void drawMarker(Vector3 direction, int colourSelector);
	void drawAllMarkers(Vector3 playerPosition);


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

		virtual void update();
		virtual void draw() = 0;
		virtual double gerRadius() = 0;

		from Entity parent
	***********************************************/

public:

	Asteroid(ObjLibrary::ObjModel baseModel);

	Asteroid(ObjLibrary::ObjModel baseModel, Vector3 position, Vector3 direction, double radius, double innerRadius);

	~Asteroid();

	Vector3 getPosition();

	bool getShowAxesAndMarkers();

	const CoordinateSystem getCoordinateSystem();

	bool getHasCrystals();

	double getRadius();

	void setPosition(float x, float y, float z);

	void setShowAxesAndMarkers(bool showAxesAndMarkers);

	//Holds the glut matrix functions for drawing the asteroid, overides virtual draw from Entity parent
	void draw(const microseconds DELTA_TIME, Vector3 playerPosition);

	//makes use of perlin noise to deform the asteroid
	void deform(ObjModel & model);

	//called by the main files update callback function, is an override of Entity parent Update but also calls parants update
	void update(const microseconds DELTA_TIME);

	Vector3 breakOffCrystals(Vector3 playerPosition, Vector3 displacement);

	float calculateSurfaceDistance(Vector3 surfacePoint);





};