#pragma once
#include "ObjLibrary/Vector2.h"
#include "ObjLibrary/Vector3.h"

using namespace ObjLibrary;

class PerlinNoiseField
{
private:
	float grid_size = 8.0f;
	float amplitude = 1.0;

	unsigned int seed_X1 = 1273472206;
	unsigned int seed_X2 = 4278162623;
	unsigned int seed_Y1 = 1440014778;
	unsigned int seed_Y2 = 524485263;
	unsigned int seed_Z1 = 2813546167;
	unsigned int seed_Z2 = 3305132234;
	unsigned int seed_Q0 = 1498573726;
	unsigned int seed_Q1 = 3476519523;
	unsigned int seed_Q2 = 3905844518;

	unsigned int pseudorandom(int x, int y, int z) const;
	float unsignedIntToPM1(unsigned int n) const;
	float unsignedIntTo01(unsigned int n) const;
	unsigned int interpolate(unsigned int v0, unsigned int v1, float fraction) const;
	float interpolate(float v0, float v1, float fraction) const;
	float fade(float n) const;
	Vector3 lattice(int x, int y, int z) const;

public:

	PerlinNoiseField(float gridSize, float amplitude, unsigned int seedX1, unsigned int seedX2,
		unsigned int seedY1, unsigned int seedY2,
		unsigned int seedZ1, unsigned int seedZ2,
		unsigned int seedQ0, unsigned int seedQ1, unsigned int seedQ2);

	PerlinNoiseField(float gridSize, float amplitude);

	float valueNoise(float x, float y, float z);
	float perlinNoise(float x, float y, float z);

};