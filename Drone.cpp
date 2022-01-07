#include "Drone.h"

Drone::Drone(Vector3 startingPosition, Vector3 initialVelocity, float formationAngle, ObjModel baseModel, const int COLOUR, Entity * player, Asteroid * asteroids[], int numOfAsteroids)
{
	currentAIState = DRONE_AI_STATE_ESCORT;
	velocity = initialVelocity;
	mass = 100;
	localCoordinateSystem = new CoordinateSystem(startingPosition);
	formationDisplacement = startingPosition;
	radius = 2;
	this->formationAngle = formationAngle;
	droneColour = COLOUR;
	this->player = player;
	for (int i = 0; i < numOfAsteroids; i++)
		this->asteroids[i] = asteroids[i];
	this->numOfAsteroids = numOfAsteroids;
	showPositions = false;
	lastUsedAcceleration = &mainAcceleration;
	updatesSpentAvoiding = 0;
	closePursueDistance = false;

	ObjModel model = baseModel;
	switch (droneColour)
	{
	case 0:
		{
		ObjLibrary::DisplayList droneDisplayList(model.getDisplayListMaterial("grapple_body_red"));
		entityDisplayList = droneDisplayList;
		break;
		}

	case 1:
		{
		ObjLibrary::DisplayList droneDisplayList(model.getDisplayListMaterial("grapple_body_orange"));
		entityDisplayList = droneDisplayList;
		break;
		}
	case 2:
		{
		ObjLibrary::DisplayList droneDisplayList(model.getDisplayListMaterial("grapple_body_yellow"));
		entityDisplayList = droneDisplayList;
		break;
		}
	case 3:
		{
		ObjLibrary::DisplayList droneDisplayList(model.getDisplayListMaterial("grapple_body_green"));
		entityDisplayList = droneDisplayList;
		break;
		}
	case 4:
		{
		ObjLibrary::DisplayList droneDisplayList(model.getDisplayListMaterial("grapple_body_cyan"));
		entityDisplayList = droneDisplayList;
		break;
		}
	}
	


	mainAcceleration = 2500;
	maneuveringAcceleration = 250;
	turnRateRadians = 1;
}

Drone::~Drone()
{

}


void Drone::draw(const microseconds DELTA_TIME)
{
	Vector3 position = localCoordinateSystem->getPosition();
	drawPredictedPath(DELTA_TIME);

	glPushMatrix();
	glTranslated(position.x, position.y, position.z);
	
	double  a_matrix[16];
	localCoordinateSystem->calculateOrientationMatrix(a_matrix);

	glMultMatrixd(a_matrix);


	glScaled(radius, radius, radius);
		
	entityDisplayList.draw();
	glPopMatrix();

	if (showPositions)
		displayAIState();
}

void Drone::update(Entity * crystal, const microseconds DELTA_TIME)
{
	targetCrystal = crystal;

	//if (updatesSpentAvoiding == 0 && (targetCrystal == NULL && currentAIState == DRONE_AI_STATE_ESCORT))
	if(updatesSpentAvoiding == 0)
	{
		//Vector3 playerPos = player->getLocalCoordinateSystem().getPosition();
		Vector3 dronePos = localCoordinateSystem->getPosition();
		for (int i = 0; i < numOfAsteroids; i++)
		{
			Vector3 asterPos = asteroids[i]->getLocalCoordinateSystem().getPosition();

			// if the distance between the player and asteroid i is less then the min safe distance to asteroid i
			if (asterPos.getDistance(dronePos) < calculateMinimumSafeDistance(asteroids[i]))
			{
				currentAIState = DRONE_AI_STATE_AVOID;// currentAIState = DRONE_AI_STATE_AVOID;
				asteroidToAvoid = asteroids[i];
			}

		}
	}
	// if a crystal is identified and I am currently escorting then switch to pursue
	if (targetCrystal != NULL && currentAIState == DRONE_AI_STATE_ESCORT)
		currentAIState = DRONE_AI_STATE_PURSUE;
	// else if I was pursuing but now their are no crystals then switch to escort
	else if (targetCrystal == NULL && currentAIState == DRONE_AI_STATE_PURSUE)
		currentAIState = DRONE_AI_STATE_ESCORT;

	
	Entity::update(DELTA_TIME);
	executeAI(DELTA_TIME);
}

double Drone::getRadius()
{
	return radius;
}

void Drone::drawPredictedPath(const microseconds DELTA_TIME)
{
	Vector3 predictedPosition = localCoordinateSystem->getPosition();
	Vector3 predictedVelocity = velocity;
	Vector3 p;

	glPushMatrix();
	glBegin(GL_LINE_STRIP);

	for (int i = 0; i < 750; i++)
	{
		switch (droneColour)
		{ 
		case 0: glColor3d(1, 0, 0); break; // red
		case 1: glColor3d(0.9961, 0.6445, 0); break; // orange
		case 2: glColor3d(0.9961, 0.9961, 0); break; // yellow
		case 3: glColor3d(0, 1, 0); break; // green
		case 4: glColor3d(0, 0.9961, 0.9961); break; // cyan
		} 

		p = predictedPosition;
		glVertex3d(predictedPosition.x, predictedPosition.y, predictedPosition.z);
		futurePhysics(predictedPosition, predictedVelocity, DELTA_TIME);  // updates predicted position and predicted velocity
		//if (predictedVelocity.getNorm() > MAX_VELOCITY_MAGNITUDE)
		//	capVelocity(predictedVelocity);
		glVertex3d(predictedPosition.x, predictedPosition.y, predictedPosition.z);
	}

	glEnd();
	glPopMatrix();

}

void Drone::futurePhysics(Vector3& position, Vector3& velocity, const microseconds DELTA_TIME)
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

	position = position + velocity * changeInTime;
}

void Drone::displayAIState()
{
	if (currentAIState == DRONE_AI_STATE_ESCORT)
		drawEscortPosition();
	else if (currentAIState == DRONE_AI_STATE_PURSUE)
		drawPursuePositions();
	else // currentAIState == DRONE_AI_STATE_AVOID
		drawAvoidSphere();
}

void Drone::drawEscortPosition()
{
	glPushMatrix();
	switch (droneColour)
	{
	case 0: glColor3d(1, 0, 0); break; // red
	case 1: glColor3d(0.9961, 0.6445, 0); break; // orange
	case 2: glColor3d(0.9961, 0.9961, 0); break; // yellow
	case 3: glColor3d(0, 1, 0); break; // green
	case 4: glColor3d(0, 0.9961, 0.9961); break; // cyan
	}
	
	Vector3 position = formationDisplacement + player->getLocalCoordinateSystem().getPosition();

	glTranslated(position.x, position.y, position.z);
	glScaled(1.5, 1.5, 1.5);
	glutWireOctahedron();
	glPopMatrix();


	glPushMatrix();

	glTranslated(futureTargetPosition.x, futureTargetPosition.y, futureTargetPosition.z);
	glScaled(1.5, 1.5, 1.5);
	glutWireOctahedron();
	glPopMatrix();
}

void Drone::drawPursuePositions()
{
	// beacuse we draw after update, when a drone catches a crystal its target crystal becomes null, then we draw, then we assign it a new target crystal
	// meaning once it catches its crystal their is 1 draw where will crash if we dont test if its null
	if (targetCrystal == NULL)
		return;

	glPushMatrix();
	switch (droneColour)
	{
	case 0: glColor3d(1, 0, 0); break; // red
	case 1: glColor3d(0.9961, 0.6445, 0); break; // orange
	case 2: glColor3d(0.9961, 0.9961, 0); break; // yellow
	case 3: glColor3d(0, 1, 0); break; // green
	case 4: glColor3d(0, 0.9961, 0.9961); break; // cyan
	}
	// draw at crystal position
	Vector3 position = targetCrystal->getLocalCoordinateSystem().getPosition();
	glTranslated(position.x, position.y, position.z);
	glScaled(1.5, 1.5, 1.5);
	glutWireOctahedron();
	glPopMatrix();

	// draw at future cystal position
	glPushMatrix();
	glTranslated(futureTargetPosition.x, futureTargetPosition.y, futureTargetPosition.z);
	glScaled(1.5, 1.5, 1.5);
	glutWireOctahedron();
	glPopMatrix();
}

void Drone::drawAvoidSphere()
{
	glPushMatrix();
	switch (droneColour)
	{
	case 0: glColor3d(1, 0, 0); break; // red
	case 1: glColor3d(0.9961, 0.6445, 0); break; // orange
	case 2: glColor3d(0.9961, 0.9961, 0); break; // yellow
	case 3: glColor3d(0, 1, 0); break; // green
	case 4: glColor3d(0, 0.9961, 0.9961); break; // cyan
	}

	Vector3 position = asteroidToAvoid->getLocalCoordinateSystem().getPosition();

	glTranslated(position.x, position.y, position.z);
	glScaled(1, 1, 1);
	glutWireSphere(calculateMinimumSafeDistance(asteroidToAvoid), 15 + droneColour, 15 - droneColour); // we are add/subtractingdrone colour into slices and stacks
	                                                                                                   // so if multiple drones are avoiding the same asteroid the
	                                                                                                   // wre shperes dont overlap. now we can see which colours are avoiding
	glPopMatrix();
}

void Drone::setShowPositions(bool show)
{
	showPositions = show;
}

void Drone::nullifyTargetCrystal()
{
	targetCrystal = NULL;
	closePursueDistance = false;
}


float Drone::calculateArrivalTime(float distance, float accelerationMagnitude, float arrivalSpeed)
{
	//t = (sqrt((sf)^2 + 2*a*d) – sf) / a
	return (sqrt(arrivalSpeed * arrivalSpeed + 2.0 * accelerationMagnitude * distance) - arrivalSpeed) / accelerationMagnitude;
}

float Drone::calculateSafeSpeed(float distance, float accelerationMagnitude, float arrivalSpeed)
{
	//sd = sf + a * t
	return arrivalSpeed + (accelerationMagnitude * calculateArrivalTime(distance, accelerationMagnitude, arrivalSpeed));
}

Vector3 Drone::desiredRelativeVelocity(float distance, Vector3 targetPosition, Vector3 futurePosition,  float arrivalSpeed)
{
	// for now I am assuming the mainAcceleration and an sf of my choice, I choose the players velocity magnittude for sf
	float safeSpeed = calculateSafeSpeed(distance, maneuveringAcceleration, arrivalSpeed);

	//determine direction from drone in future to escort position in future = target pos - drone future pos
	Vector3 direction = targetPosition - futurePosition;
	direction.normalize();

	//determine a vector of magnitude safe speed in the direction
	return safeSpeed * direction;
}

float Drone::calculateMinimumSafeDistance(Asteroid * asteroid)
{
	//ra + 10 * || vd - va|| / ad,
	return 1.3*asteroid->getRadius() + (10.0 * (velocity - asteroid->getVelocity()).getNorm() / mainAcceleration);
}

Vector3 Drone::calculateOverallDesiredVelocity()
{
	if (currentAIState == DRONE_AI_STATE_ESCORT)
		return escort(maneuveringAcceleration);
	else if (currentAIState == DRONE_AI_STATE_PURSUE)
		return pursue(maneuveringAcceleration);
	else // currentAIState == DRONE_AI_STATE_AVOID
		return avoid();

}

void Drone::engageEngines(Vector3 desiredVelocity, const microseconds DELTA_TIME)
{
	const double changeInTime = DELTA_TIME.count() / 1e6;
	Vector3 steeringDir = (desiredVelocity - velocity).getNormalized();


	if ((velocity - desiredVelocity).getNorm() < 50.0)
	{
		localCoordinateSystem->rotateToVectorWithTurnRate(velocity.getNormalized(), 3.1415 / 30.0);
		// using this if to go at Velocity difference magnitude rather then manuevering acceleration stops drones from shaking
		// because maneuvering acceleration can sometimes be overcorrective which caused the shake
		float velocityDifference = (velocity - desiredVelocity).getNorm();
		if (maneuveringAcceleration * changeInTime > velocityDifference)
			velocity += (steeringDir * velocityDifference * changeInTime);
		else
			velocity += (steeringDir * maneuveringAcceleration * changeInTime);
		lastUsedAcceleration = & maneuveringAcceleration;
	}
	else // (velocity - desiredVelocity).getNorm() >= 50.0
	{
		localCoordinateSystem->rotateToVectorWithTurnRate(steeringDir, 3.1415 / 15.0);

		if(localCoordinateSystem->getForward().getAngle(steeringDir) < 0.1)
		{
			velocity += localCoordinateSystem->getForward() * (mainAcceleration * changeInTime);
			lastUsedAcceleration = & mainAcceleration;
		}
	}

}

void Drone::executeAI(const microseconds DELTA_TIME)
{
		Vector3 desiredVelocity = calculateOverallDesiredVelocity(); // decision as to which AI algo to use is in here
		if (velocity != desiredVelocity)
			engageEngines(desiredVelocity, DELTA_TIME);
}

Vector3 Drone::escort(float accelerationMagnitude)
{
	Vector3 playerPosition = player->getLocalCoordinateSystem().getPosition();
	Vector3 playerVelocity = player->getVelocity();


	formationDisplacement = player->getLocalCoordinateSystem().getUp() * 5.0;
	formationDisplacement.rotateArbitrary(player->getLocalCoordinateSystem().getForward(), formationAngle);


	float estimatedTime = calculateArrivalTime(localCoordinateSystem->getPosition().getDistance(playerPosition), accelerationMagnitude, 0);

	//Estimate the position of the agent after that much time, assuming it moves with its current velocity.
	Vector3 esitmatedPosition = localCoordinateSystem->getPosition() + (velocity * estimatedTime);

	// Calculate the future escort position after the same time, assuming the player spaceship continues to move with its own current velocity.
	futureTargetPosition = (playerPosition + playerVelocity * estimatedTime) + formationDisplacement;


	// Calculate the desired velocity to move between those two positions.
	//To do this, call the function from the previous step and then add the player spaceship’s current velocity.
	float futureDistance = esitmatedPosition.getDistance(futureTargetPosition);
	return desiredRelativeVelocity(futureDistance, futureTargetPosition, esitmatedPosition, 0) + playerVelocity;
}

Vector3 Drone::pursue(float accelerationMagnitude)
{
	Vector3 crystalPosition = targetCrystal->getLocalCoordinateSystem().getPosition();
	Vector3 crystalVelocity = targetCrystal->getVelocity();

	
	float relativeArrivalSpeed;
	
	if (!closePursueDistance)
	{
		relativeArrivalSpeed = (crystalPosition - localCoordinateSystem->getPosition()).getNorm();
		if (relativeArrivalSpeed < 1.0)
			closePursueDistance = true;
	}
	else
		relativeArrivalSpeed = 0;



	//Convert the relative escort position to world coordinates to give the current escort position.
	//convert to local of spaceship so it rotates with spaceship
	//float forwardAngle = player->getLocalCoordinateSystem().getForward().getAngle(Vector3(1, 0, 0));
	//float upAngle = player->getLocalCoordinateSystem().getUp().getAngle(Vector3(0, 1, 0));
	//float rightAngle = player->getLocalCoordinateSystem().getRight().getAngle(Vector3(0, 0, 1));

	//Vector3 correctedFormationDisplacement = formationDisplacement.getRotatedArbitrary(player->getLocalCoordinateSystem().getForward(), forwardAngle);
	//Vector3 correctedFormationDisplacement = formationDisplacement.getRotatedArbitrary(player->getLocalCoordinateSystem().getUp(), upAngle);
	//correctedFormationDisplacement.rotateArbitrary(player->getLocalCoordinateSystem().getUp(), upAngle);
	//correctedFormationDisplacement.rotateArbitrary(player->getLocalCoordinateSystem().getRight(), rightAngle);



	//Estimate the time tfuture it would take the agent to reach the current escort position,
	//if it matched velocities(i.e.relative speed 0) as it reaches that position.
	//Use the function for computing the arrival time from step 3, with a final speed of 0.
	float estimatedTime = calculateArrivalTime(localCoordinateSystem->getPosition().getDistance(crystalPosition), accelerationMagnitude, relativeArrivalSpeed);

	//Estimate the position of the agent after that much time, assuming it moves with its current velocity.
	Vector3 esitmatedPosition = localCoordinateSystem->getPosition() + (velocity * estimatedTime);


	// Calculate the future target position after the same time, assuming the crystal continues to move with its own current velocity.
	futureTargetPosition = (crystalPosition + crystalVelocity * estimatedTime);

	// Calculate the desired velocity to move between those two positions.
	//To do this, call the function from the previous step and then add the crystals current velocity.
	float distance = esitmatedPosition.getDistance(crystalPosition);
	return desiredRelativeVelocity(distance, futureTargetPosition, esitmatedPosition, relativeArrivalSpeed) + crystalVelocity;
}

Vector3 Drone::avoid()
{
	// if weve spent sufficient time avoiding switch out of avoid mode to be re assesed next update
	if (++updatesSpentAvoiding == UPDATES_TOREASSES_AVOIDING)
	{
		updatesSpentAvoiding = 0;
		currentAIState = DRONE_AI_STATE_ESCORT;
	}

	// return the vector whos direction is from the asteroid to the drone with magnitude of maneuvering engines
	// **************This is what the assignment outline suggests but this causes speedy drones to slow down then still crash ************************
	//return (localCoordinateSystem->getPosition() - asteroidToAvoid->getLocalCoordinateSystem().getPosition()).getNormalized() * maneuveringAcceleration;


	Vector3 velocityDirection = velocity.getNormalized();
	Vector3 awayFromAsteroidDirection = (localCoordinateSystem->getPosition() - asteroidToAvoid->getLocalCoordinateSystem().getPosition()).getNormalized();

	float angle = velocityDirection.getAngleNormal(awayFromAsteroidDirection);

	// drones will avoid an asteroid by attempting to veer away, rather then just a full break
	return velocityDirection.getRotatedTowards(awayFromAsteroidDirection, angle * 2.0/3.0) * maneuveringAcceleration;
}
