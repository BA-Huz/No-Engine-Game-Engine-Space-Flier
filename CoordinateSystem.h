#pragma once
#include "GetGlut.h"
#include "ObjLibrary/ObjModel.h"
#include "ObjLibrary/DisplayList.h"
using namespace ObjLibrary;

//class for managing and moving a camera within a 3d environment
class CoordinateSystem
{
private:
	Vector3 position;
	Vector3 forward;
	Vector3 up;
	Vector3 right;


	double moveRate;
	double turnRate;
	double arbitraryRotationalAngle = 0;

public:
	// default constructor
	CoordinateSystem();

	// initializing constructor with only position
	CoordinateSystem(Vector3 position);

	// initializing constructor without the rates
	CoordinateSystem(Vector3 position, Vector3 forward, Vector3 up);

	// initializing constructor with rates
	CoordinateSystem(Vector3 position, Vector3 forward, Vector3 up, double moveRate, double turnRate, float rotationalAngle);

	// destructor
	~CoordinateSystem();

	// getter for the position Vector3
	const Vector3& getPosition() const;

	// getter for the forward Vector3
	const Vector3& getForward() const;

	// getter for the up Vector3
	const Vector3& getUp() const;

	// getter for the right Vector3
	const Vector3& getRight() const;

	// getter for the move rate
	const double getMoveRate();

	// getter for the turn rate
	const double getTurnRate();

	// getter for the arbitrary rotational angle
	const float getArbitraryRotationalAngle();


	//initializes the camera properly
	void setupCamera() const;  // calls gluLookAt

	// setter for position
	void setPosition(const Vector3& position);

	//sets the full orientation(forward, up, and right) using the provided forward and up, right ends up the cross product
	void setOrientation(const Vector3& forward, const Vector3& up);

	// setter for move rate
	void setMoveRate(double rate);

	// setter for turn rate
	void setTurnRate(double rate);

	/****************************************************

	smooth members are designed to be repeaditly called
	and make use of this classes moveRate and turnRate.

	jump members can be called once with a given rate
	or be called repeaditly like the smooth functions
	but with a different rate without altering this
	classes rates.

	****************************************************/

	// jumps the camera in its local forwards direction
	void jumpForward(double distance);

	// moves the camera in its local forwards direction, intended to be repeatidly called to acheive smooth movement
	void smoothMoveForward(bool moveForward);

	// jumpss the camera in its local upwards direction
	void jumpUp(double distance);

	// moves the camera in its local upwards direction, intended to be repeatidly called to acheive smooth movement
	void smoothMoveUp(bool moveUp);

	// jumpss the camera in its local right direction
	void jumpRight(double distance);

	// moves the camera in its local right direction, intended to be repeatidly called to acheive smooth movement
	void smoothMoveRight(bool moveRight);

	// rotates the camera around its local forward
	void rotationalJumpAroundForward(double radians);

	// rotates the camera around its local forward, intended to be called repeatidly to acheive smooth movement
	void smoothRotateAroundForward(bool clockwise);

	// rotates the camera around its local up
	void rotationalJumpAroundUp(double radians);

	// rotates the camera around its local up, intended to be called repeatidly to acheive smooth movement
	void smoothRotateAroundUp(bool lookLeft);

	// rotates the camera around its local right
	void rotationalJumpAroundRight(double radians);

	// rotates the camera around its local right, intended to be called repeatidly to acheive smooth movement
	void smoothRotateAroundRight(bool lookUp);

	// rotates the camera around a given axis
	void rotationalJumpAroundArbitrary(Vector3 axis, double radians);

	// rotates the camera around a given axis, intended to be called repeatidly to acheive smooth movement
	void smoothRotatearoundArbitrary(Vector3 axis, bool positiveTurnRate);

	//rotates the camera towards given vector using a max of given turn rate
	void rotateToVectorWithTurnRate(const Vector3& desired_forward, double max_radians);

	//rotates the camera towards given vector using a max of the cameras turn rate
	void rotateToVector(const Vector3& desired_forward);

	void applyDrawTransformations() const;

	void calculateOrientationMatrix(double a_matrix[]) const;

	ObjLibrary::Vector3 localToWorld(const ObjLibrary::Vector3& local) const;

	ObjLibrary::Vector3 worldToLocal(const ObjLibrary::Vector3& world) const;
};