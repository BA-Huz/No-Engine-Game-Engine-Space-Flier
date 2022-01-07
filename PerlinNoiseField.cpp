#include "PerlinNoiseField.h"
#include <cmath>


PerlinNoiseField::PerlinNoiseField(float gridSize, float amplitude, unsigned int seedX1, unsigned int seedX2,
	unsigned int seedY1, unsigned int seedY2,
	unsigned int seedZ1, unsigned int seedZ2,
	unsigned int seedQ0, unsigned int seedQ1, unsigned int seedQ2)
{
	grid_size = gridSize;
	this->amplitude = amplitude;
	seed_X1 = seedX1;
	seed_X2 = seedX2;
	seed_Y1 = seedY1;
	seed_Y2 = seedY2;
	seed_Z1 = seedZ1;
	seed_Z2 = seedZ2;
	seed_Q0 = seedQ0;
	seed_Q1 = seedQ1;
	seed_Q2 = seedQ2;
}

PerlinNoiseField::PerlinNoiseField(float gridSize, float amplitude)
{
	grid_size = gridSize;
	this->amplitude = amplitude;
}

float PerlinNoiseField::valueNoise(float x, float y, float z)
{

	// calculate noise here
	int x0 = (int)(floor(x / grid_size));
	int y0 = (int)(floor(y / grid_size));
	int z0 = (int)(floor(z / grid_size));
	int x1 = x0 + 1;
	int y1 = y0 + 1;
	int z1 = z0 + 1;

	float x_frac = x / grid_size - x0;
	float y_frac = y / grid_size - y0;
	float z_frac = z / grid_size - z0;

	float x_fade = fade(x_frac);
	float y_fade = fade(y_frac);
	float z_fade = fade(z_frac);

	unsigned int value000 = pseudorandom(x0, y0, z0);
	unsigned int value001 = pseudorandom(x0, y0, z1);
	unsigned int value010 = pseudorandom(x0, y1, z0);
	unsigned int value011 = pseudorandom(x0, y1, z1);
	unsigned int value100 = pseudorandom(x1, y0, z0);
	unsigned int value101 = pseudorandom(x1, y0, z1);
	unsigned int value110 = pseudorandom(x1, y1, z0);
	unsigned int value111 = pseudorandom(x1, y1, z1);

	unsigned int value00 = interpolate(value000, value001, z_fade);
	unsigned int value01 = interpolate(value010, value011, z_fade);
	unsigned int value10 = interpolate(value100, value101, z_fade);
	unsigned int value11 = interpolate(value110, value111, z_fade);
	unsigned int value0 = interpolate(value00, value01, y_fade);
	unsigned int value1 = interpolate(value10, value11, y_fade);
	unsigned int value = interpolate(value0, value1, x_fade);

	return unsignedIntToPM1(value) * amplitude;
}

float PerlinNoiseField::perlinNoise(float x, float y, float z)
{
	// calculate noise here
	int x0 = (int)(floor(x / grid_size));
	int y0 = (int)(floor(y / grid_size));
	int z0 = (int)(floor(z / grid_size));

	float x_frac = x / grid_size - x0;
	float y_frac = y / grid_size - y0;
	float z_frac = z / grid_size - z0;
	float x_fade = fade(x_frac);
	float y_fade = fade(y_frac);
	float z_fade = fade(z_frac);

	int x1 = x0 + 1;
	int y1 = y0 + 1;
	int z1 = z0 + 1;

	Vector3 lattice000 = lattice(x0, y0, z0);
	Vector3 lattice001 = lattice(x0, y0, z1);
	Vector3 lattice010 = lattice(x0, y1, z0);
	Vector3 lattice011 = lattice(x0, y1, z1);
	Vector3 lattice100 = lattice(x1, y0, z0);
	Vector3 lattice101 = lattice(x1, y0, z1);
	Vector3 lattice110 = lattice(x1, y1, z0);
	Vector3 lattice111 = lattice(x1, y1, z1);

	Vector3 direction000(-x_frac, -y_frac, -z_frac);
	Vector3 direction001(-x_frac, -y_frac, 1.0f -z_frac);
	Vector3 direction010(-x_frac, 1.0f -y_frac, -z_frac);
	Vector3 direction011(-x_frac, 1.0f -y_frac, 1.0f -z_frac);
	Vector3 direction100(1.0f -x_frac, -y_frac, -z_frac);
	Vector3 direction101(1.0f -x_frac, -y_frac, 1.0f -z_frac);
	Vector3 direction110(1.0f -x_frac, 1.0f -y_frac, -z_frac);
	Vector3 direction111(1.0f -x_frac, 1.0f -y_frac, 1.0f -z_frac);

	float value000 = (float)(lattice000.dotProduct(direction000));
	float value001 = (float)(lattice001.dotProduct(direction001));
	float value010 = (float)(lattice010.dotProduct(direction010));
	float value011 = (float)(lattice011.dotProduct(direction011));
	float value100 = (float)(lattice100.dotProduct(direction100));
	float value101 = (float)(lattice101.dotProduct(direction101));
	float value110 = (float)(lattice110.dotProduct(direction110));
	float value111 = (float)(lattice111.dotProduct(direction111));


	float value0 = interpolate(value000, value001, z_fade);
	float value1 = interpolate(value010, value011, z_fade);
	float value2 = interpolate(value100, value101, z_fade);
	float value3 = interpolate(value110, value111, z_fade);

	float valueA = interpolate(value0, value1, y_fade);
	float valueB = interpolate(value2, value3, y_fade);

	float value = interpolate(valueA, valueB, x_fade);
	
	//float value0 = interpolate2D(value00, value01, y_fade);
	//float value1 = interpolate2D(value10, value11, y_fade);
	//float value = interpolate2D(value0, value1, x_fade);

	return value * amplitude;

}


unsigned int PerlinNoiseField::pseudorandom(int x, int y, int z) const
{
	unsigned int n = (seed_X1 * x) +
		(seed_Y1 * y) + (seed_Z1 * z);
	unsigned int quad_term = seed_Q2 * n * n +
		seed_Q1 * n +
		seed_Q0;
	return quad_term +
		(seed_X2 * x) +
		(seed_Y2 * y) +
		(seed_Z2 * z);
}

float PerlinNoiseField::unsignedIntToPM1(unsigned int n) const
{
	return ((float)(n) / UINT_MAX) * 2.0f - 1.0f;
}

float PerlinNoiseField::unsignedIntTo01(unsigned int n) const
{
	return ((float)(n) / UINT_MAX);
}

unsigned int  PerlinNoiseField::interpolate(unsigned int v0, unsigned int v1, float fraction) const
{
	return (unsigned int)(v0 * (1.0f - fraction)) +
		(unsigned int)(v1 * fraction);
}

float  PerlinNoiseField::interpolate(float v0, float v1, float fraction) const
{
	return (v0 * (1.0f - fraction)) + (v1 * fraction);
}

float PerlinNoiseField::fade(float n) const
{
	//return (1 - cos(n * 3.14159265f)) * 0.5f;
	//return (-2 * pow(n, 3) + 3 * pow(n, 2));
	//return(6 * pow(n, 5) - 15 * pow(n, 4) + 10 * pow(n, 3));
	return (1 - cos(n * 3.14159265f)) * 0.5f;
}


Vector3 PerlinNoiseField::lattice(int x, int y, int z) const
{
	unsigned int value = pseudorandom(x, y, z);
	unsigned int value2 = pseudorandom(x + 1, y + 1, z + 1);
	//float radians = (float)(value);  // very random
	//return Vector2(cos(radians), sin(radians));
	return Vector3::getPseudorandomUnitVector(unsignedIntTo01(value), unsignedIntTo01(value2)); // crashes in this function

}