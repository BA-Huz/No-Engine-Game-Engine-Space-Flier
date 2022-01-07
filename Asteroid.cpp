#include "Asteroid.h"
#include <math.h>

void drawAxes(CoordinateSystem cs);


// default constructor
Asteroid::Asteroid(ObjLibrary::ObjModel baseModel) : Entity()
{
	Vector3 unitVector = Vector3::getRandomUnitVector();
	double r = rand() % 6001 + 2000;
	localCoordinateSystem = new CoordinateSystem(Vector3(r * unitVector.x, r * unitVector.y, r * unitVector.z), Vector3(1, 0, 0), Vector3(0, 1, 0));

	Vector3 direction;
	direction = Vector3::getRandomUnitVector();
	direction = direction.getRejectionSafe(localCoordinateSystem->getPosition() * -1.0);
	direction.normalizeSafe();
	velocity = direction * (calculateOrbitSpeed() * ((rand() % 11 + 5) / 10.0));

	radius = rand() % 350 + 50.0;
	innerRadius = radius - radius * ((float)(rand() % 41 + 10)) / 100.0;
	avgRadius = 0;

	mass = calculateMass();

	hasCrystals = true;

	rotationalAxis = Vector3::getRandomUnitVector();
	float rotationSpeed = (float)(rand() % 11);
	rotationSpeed /= 100;
	localCoordinateSystem->setTurnRate(rotationSpeed);

	// give an initial random angle in radians around the random rotatinal axis
	localCoordinateSystem->rotationalJumpAroundArbitrary(rotationalAxis, ((float)(rand() % 6283)) / 1000.0);

	showAxesAndMarkers = false;

	//perlinAmplitude = (((double)(rand() % 76 + 25)) / 100.0);
	perlinAmplitude = (((double)(rand() % 100 + 100)) / 100.0);
	//perlinOffset = (float)(rand() % 50);
	//const double NOISE_OFFSET_MAX = 1.0e4;
	Vector3 perlinOffset(Vector3::getRandomSphereVector());// *NOISE_OFFSET_MAX);

	ObjModel model = baseModel;
	deform(model);

	ObjLibrary::DisplayList asteroidDisplayList(model.getDisplayList());
	entityDisplayList = asteroidDisplayList;


}

//constructor
Asteroid::Asteroid(ObjLibrary::ObjModel baseModel, Vector3 position, Vector3 direction, double radius, double innerRadius)
{
	localCoordinateSystem = new CoordinateSystem(position, Vector3(1,0,0), Vector3(0,1,0));
	this->radius = radius;
	this->innerRadius = innerRadius;
	avgRadius = 0;
	mass = calculateMass();
	hasCrystals = true;
	perlinAmplitude = (((double)(rand() % 100 + 100)) / 100.0);
	const double NOISE_OFFSET_MAX = 1.0e4;
	Vector3 perlinOffset(Vector3::getRandomSphereVector() * NOISE_OFFSET_MAX);

	this->rotationalAxis = Vector3::getRandomUnitVector();
	//this->rotationalAngle = rotationalAngle;
	float rotationSpeed = (float)(rand() % 11);
	rotationSpeed /= 100;
	localCoordinateSystem->setTurnRate(rotationSpeed);
	//this->orbitalDirection = orbitalDirection;

	//Vector3 direction;
	//direction = Vector3::getRandomUnitVector();
	//direction = direction.getRejectionSafe(localCoordinateSystem->getPosition() * -1.0);
	direction.normalizeSafe();
	velocity = direction * (calculateOrbitSpeed() * ((rand() % 11 + 5) / 10.0));

	// give an initial random angle in radians around the random rotatinal axis
	localCoordinateSystem->rotationalJumpAroundArbitrary(rotationalAxis, ((float)(rand() % 6283)) / 1000.0);

	ObjModel model = baseModel;
	deform(model);

	ObjLibrary::DisplayList asteroidDisplayList(model.getDisplayList());
	entityDisplayList = asteroidDisplayList;
}

// destructoor
Asteroid::~Asteroid()
{
	delete noise;
}


Vector3 Asteroid::getPosition()
{
	return localCoordinateSystem->getPosition();
}

bool Asteroid::getShowAxesAndMarkers()
{
	return showAxesAndMarkers;
}

const CoordinateSystem Asteroid::getCoordinateSystem()
{
	return *localCoordinateSystem;
}

bool Asteroid::getHasCrystals()
{
	return hasCrystals;
}

void Asteroid::setPosition(float x, float y, float z)
{
	//position.x = x;
	//position.y = y;
	//position.z = z;
	localCoordinateSystem->setPosition(Vector3(x, y, z));
}

double Asteroid::getRadius()
{
	//return avgRadius;
	return radius;
}

void Asteroid::setShowAxesAndMarkers(bool showAxesAndMarkers)
{
	this->showAxesAndMarkers = showAxesAndMarkers;
}

//Holds the glut matrix functions for drawing the asteroid
void Asteroid::draw(const microseconds DELTA_TIME, Vector3 playerPosition)
{
	glPushMatrix();
	Vector3 position = localCoordinateSystem->getPosition();
	glTranslated(position.x, position.y, position.z);

	// ***** Draw a sphere on the bounding shpere
	//glColor3d(0.8, 0.1, 0.5);
	//glPushMatrix();
	//glutSolidSphere(avgRadius, 30, 30);
	//glTranslated(localCoordinateSystem->getPosition().x, localCoordinateSystem->getPosition().y, localCoordinateSystem->getPosition().z);
	//glPopMatrix();
	// *****

	

	if (showAxesAndMarkers)
		drawAxes(*localCoordinateSystem);

	                                                    // convert the coordinaT systems arbitrary angle from radians to degrees
	glRotated(localCoordinateSystem->getArbitraryRotationalAngle() * (180.0 / 3.141592653589793238463), rotationalAxis.x, rotationalAxis.y, rotationalAxis.z);


	glScaled(radius, radius, radius);
	entityDisplayList.draw();


	glPopMatrix();

	//distance from ship to asteroid is asteroid - ship
	if(showAxesAndMarkers && (localCoordinateSystem->getPosition() - playerPosition).getNorm() < 2000)
		drawAllMarkers(playerPosition);
}

//makes use of perlin noise to deform the asteroid
void Asteroid::deform(ObjModel & model)
{
	int vertexCount = model.getVertexCount();
	Vector3 oldVertex;
	noise = new PerlinNoiseField(0.6, perlinAmplitude);
	//PerlinNoiseField perlinNoiseField(0.6, perlinAmplitude);
	float value;

	

	float newRadiusSum = 0;
	for (int i = 0; i < vertexCount; i++)
	{
		oldVertex = model.getVertexPosition(i);


		Vector3 offset_vertex = oldVertex + perlinOffset;

		//noise
		value = noise->perlinNoise(oldVertex.x, oldVertex.y, oldVertex.z) + 1.0;
		// my perlin amplitude is left large enaugh that values can go beyond the disired bounds
		// though i want a larger amplitude so I readjust them to be at the bounds
		if (value < 0.01)
			value = 0.01;
		else if (value > 2.0)
			value = 2.0;

		//value = noise->perlinNoise(oldVertex.x, oldVertex.y, oldVertex.z) + 1.0;
		//value is between 0 and 2
		// now map value to between inner radius and radius and call that output

		float output = ((radius - innerRadius) / (2.0 / value)) + innerRadius;
		//if (output < innerRadius)
		//	cout << "you should never see this" << endl;
		//else if (output > radius)
		//	cout << "sus if you see this" << endl;
		//cout << "inner: " << innerRadius << " outer: " << radius << " actual: " << output << endl;

		// new vertex is the direction of old radius with the magnitude of output
		Vector3 newVertex = (oldVertex/radius) * output;

		newRadiusSum += newVertex.getNorm()*radius;
		model.setVertexPosition(i, newVertex);

	}
	avgRadius = newRadiusSum / ((float)vertexCount);

	
}

//called by the main files update callback function, is an override of Entity parent Update but also calls parants update
void Asteroid::update(const microseconds DELTA_TIME)
{
	Entity::update(DELTA_TIME);
	
	// rotate asteroid based off its own rotational momentum
	localCoordinateSystem->smoothRotatearoundArbitrary(rotationalAxis, true);
}

float Asteroid::calculateMass()
{
	return (3.1415 / 6.0) * radius * radius * innerRadius * 2710.0; // 2710 is density of rocky s type asteroids
}

Vector3 Asteroid::breakOffCrystals(Vector3 playerPosition, Vector3 displacement)
{
	hasCrystals = false;
	
	//find the point on the bounding sphere closest to the player position

	float distance = displacement.getNorm();
	Vector3 direction = displacement / distance;
	Vector3 centerToEdge = direction*calculateSurfaceDistance(direction);
	return localCoordinateSystem->getPosition() + centerToEdge;

}

float Asteroid::calculateSurfaceDistance(Vector3 direction)
{
	// this negative on the rotational axis is the solution that ended my 6 hour nightmare, I love coding
	direction.rotateArbitrary(-rotationalAxis, localCoordinateSystem->getArbitraryRotationalAngle());

	//float value = noise->perlinNoise(offset_vertex.x,offset_vertex.y,offset_vertex.z);
	float value = noise->perlinNoise(direction.x, direction.y, direction.z) + 1.0;
	// my perlin amplitude is left large enaugh that values can go beyond the disired bounds
	// though i want a larger amplitude so I readjust them to be at the bounds
	if (value < 0.01)
		value = 0.01;
	else if (value > 2.0)
		value = 2.0;

	float surfaceDistance = ((radius - innerRadius) / (2.0 / value)) + innerRadius;

	return surfaceDistance;
}

void Asteroid::drawMarker(Vector3 direction, int colourSelector)
{
	if (colourSelector == 0)
		glColor3f(1, 0.0, 0.0);
	else if (colourSelector == 1)
		glColor3f(0.0, 0.0, 1);
	else
		glColor3f(0.4, 0.0, 0.6);
	glPushMatrix();



	Vector3 position = direction * calculateSurfaceDistance(direction) + localCoordinateSystem->getPosition();

	glTranslated(position.x, position.y, position.z);
	glScaled(5, 5, 5);
	glutSolidOctahedron();

	glPopMatrix();
}

void Asteroid::drawAllMarkers(Vector3 playerPosition)
{
	const float PI = 3.1415927;
	//int colourSelector = 0;

	// along XY plane
	for (int a = 0; a <= 40; a += 1)
		drawMarker(Vector3(cos(a * PI / 20.0), sin(a * PI / 20.0), 0), a % 2);

	// along YZ plane
	for (int a = 0; a <= 40; a += 1)
		drawMarker(Vector3(0, cos(a * PI / 20.0), sin(a * PI / 20.0)), a % 2);

	// along ZX plane
	for (int a = 0; a <= 40; a += 1)
		drawMarker(Vector3(sin(a * PI / 20.0), 0, cos(a * PI / 20.0)), a % 2);

	// also dray a marker on the asteroid surface point closest to the player
	 drawMarker((playerPosition - localCoordinateSystem->getPosition()).getNormalized(), 2);
}



// draws the axes on the asteroid
void drawAxes(CoordinateSystem cs)
{
	Vector3 position = cs.getPosition();
	float axisLength = 475.0;
	Vector3 forward = cs.getForward();
	Vector3 up = cs.getUp();
	Vector3 right = cs.getRight();


	glBegin(GL_LINES);
	

		glColor3d(1.0, 0.0, 0.0);
		glVertex3d(0,0, 0);///0,0,0
		glVertex3d(forward.x * axisLength, forward.y * axisLength, forward.z * axisLength);
		glColor3d(0.0, 1.0, 0.0);
		glVertex3d(0, 0, 0);
		glVertex3d(up.x * axisLength, up.y * axisLength, up.z * axisLength);
		glColor3d(0.0, 0.0, 1.0);
		glVertex3d(0,0, 0);
		glVertex3d(right.x * axisLength, right.y * axisLength, right.z * axisLength);

	//else // showaxes can only be 2 here
	//{
	//	glColor3d(1.0, 0.0, 0.0);
	//	glVertex3d(-forward.x * axisLength, -forward.y * axisLength, -forward.z * axisLength);///0,0,0
	//	glVertex3d(forward.x * axisLength, forward.y * axisLength, forward.z * axisLength);
	//	glColor3d(0.0, 1.0, 0.0);
	//	glVertex3d(-up.x * axisLength, -up.y * axisLength, -up.z * axisLength);
	//	glVertex3d(up.x * axisLength, up.y * axisLength, up.z * axisLength);
	//	glColor3d(0.0, 0.0, 1.0);
	//	glVertex3d(-right.x * axisLength, -right.y * axisLength, -right.z * axisLength);
	//	glVertex3d(right.x * axisLength, right.y * axisLength, right.z * axisLength);
	//}

	
	
	

	glEnd();
}
