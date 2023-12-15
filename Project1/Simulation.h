#pragma once
/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/


#include "particle.h"
#include <vector>
using namespace std;



class Simulation
{
public:
	float timestep;
	int noPt; // Number Of Particles
	vector<Particle> particleList;
	float commonMass;
	float kinEn;
	float potEn;

	float worldColldConst;
	vector3f world;
	vector3f halfWorld;

	Simulation()
	{
		particleList.clear();
		noPt = 20;
		commonMass = 1.0f;
		timestep = 0.0005f;

		world.setUp(20, 20, 20);
		halfWorld = world * 0.5f;
		initializeParticles();

		worldColldConst = 0.5f;
	}

	void initializeParticles();
	virtual void simulateEuler(Particle* p);
	virtual void simulateEulerCromer(Particle* p);
	virtual void simulateVerlet(Particle* p);

	virtual void simulate()
	{
		clearForces();
		applyForces();


		solveWorldCollision();

		for (int j = 0; j < noPt; j++)
		{
			switch (particleList[j].getIntegration())
			{
			case Euler:
				simulateEuler(&particleList[j]);
				break;
			case EulerCromer:
				simulateEulerCromer(&particleList[j]);
				break;
			case Verlet:
				simulateVerlet(&particleList[j]);
				break;

			}
		}

		computeSystemEnergies();

	}

	void clearForces();
	virtual void applyForces() {};
	void destroyParticles();
	void solveWorldCollision();
	void computeSystemEnergies();

};

//Movement under gravity
class SimBasic : public Simulation
{
public:
	vector3f gravity;

	SimBasic(vector3f v) : Simulation()
	{
		gravity = v;
	}

	virtual void applyForces()
	{
		for (int i = 0; i < noPt; i++)
		{
			particleList[i].applyForce(gravity*commonMass);
		}
	}
};
