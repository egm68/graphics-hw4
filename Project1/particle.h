#pragma once
/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/


#include "math3d.h"

enum IntegrationScheme
{
	Euler,
	EulerCromer,
	Verlet
};

class Particle
{
private:
	vector3f force;
	vector3f velocity;
	vector3f position;
	vector3f oldPosition;
	IntegrationScheme is;
	vector3f color; //RGB

public:
	Particle()
	{
		clear();
		is = Euler;
	}

	Particle(vector3f f, vector3f v, vector3f p)
	{
		force = f;
		velocity = v;
		position = p;
		is = Euler;
	}

	//Setters and getters
	vector3f getForce() { return force; }
	vector3f getVelocity() { return velocity; }
	vector3f getPosition() { return position; }
	vector3f getOldPosition() { return oldPosition; }
	IntegrationScheme getIntegration() { return is; }
	vector3f getColor() { return color; }

	void setForce(vector3f f) { force = f; }
	void setVelocity(vector3f v) { velocity = v; }
	void setPosition(vector3f p) {
		position = p;
	}
	void setOldPosition(vector3f p) { oldPosition = p; }
	void setIntegration(IntegrationScheme isc) { is = isc; }
	void setColor(vector3f c) { color = c; }

	//Important functions
	void clear()
	{
		force.clear();
		velocity.clear();
		position.clear();
	}
	void clearForce() { force.clear(); }
	void clearVelocity() { velocity.clear(); }
	void clearPosition() { position.clear(); }
	void applyForce(vector3f f) { this->force += f; }
};
