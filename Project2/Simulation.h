/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/

#pragma once
#include "particle.h"

using namespace std;



class Simulation
{
public:
	float timestep;
	CubeMesh mesh;
	float commonMass;
	float kinEn;
	float potEn;
	float sprEn;
	IntegrationScheme is;

	float spring_constant;
	float damping_constant;

	float worldColldConst;
	vector3f world;
	vector3f halfWorld;

	Simulation()
	{
		commonMass = 1.0f;
		timestep = 0.0005f;

		world.setUp(20, 20, 20);
		halfWorld = world * 0.5f;
		initializeMesh();

		worldColldConst = 0.5f;

        //TO DO - Try different values and integration schemes
		is = EulerCromer;

		spring_constant = 1000;
		damping_constant = 10;
	}

	void initializeMesh();
	virtual void simulateEuler(Particle* p);
	virtual void simulateEulerCromer(Particle* p);
	virtual void simulateVerlet(Particle* p);

	virtual void simulate()
	{
		clearForces();
		applyForces();


		solveWorldCollision();

		for (int j = 0; j < mesh.particles.size(); j++)
		{
			switch (is)
			{
			case Euler:
				simulateEuler(&mesh.particles[j]);
				break;
			case EulerCromer:
				simulateEulerCromer(&mesh.particles[j]);
				break;
			case Verlet:
				simulateVerlet(&mesh.particles[j]);
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

//Spring Mesh movement
class SimSpring : public Simulation
{
public:

	SimSpring() : Simulation()
	{
	}

	virtual void applyForces()
	{

        //TO DO - Apply gravity
		vector3f gravity = vector3f(0.0, -9.8, 0.0);
        for (int i = 0; i < mesh.particles.size(); i++)
		{
			mesh.particles[i].applyForce(gravity);
		}

        //TO DO - Apply spring forces
		for (int i = 0; i < mesh.springs.size(); i++)
		{
			Particle& p1 = mesh.particles[mesh.springs[i].p1];
			Particle& p2 = mesh.particles[mesh.springs[i].p2];
			float spring_init_length = mesh.springs[i].initLength;

                        //Add spring Force
			float hookes_law_x = spring_init_length - (pow(p2.getPosition().getX() - p1.getPosition().getX(), 2) + pow(p2.getPosition().getY() - p1.getPosition().getY(), 2) + pow(p2.getPosition().getZ() - p1.getPosition().getZ(), 2));
			float hookes_law_F_s = spring_constant * hookes_law_x;
			vector3f p1_spring_force = vector3f(p1.getPosition().getX() - p2.getPosition().getX(), p1.getPosition().getY() - p2.getPosition().getY(), p1.getPosition().getZ() - p2.getPosition().getZ()).returnUnit().operator*(hookes_law_F_s);
			vector3f p2_spring_force = vector3f(p2.getPosition().getX() - p1.getPosition().getX(), p2.getPosition().getY() - p1.getPosition().getY(), p2.getPosition().getZ() - p1.getPosition().getZ()).returnUnit().operator*(hookes_law_F_s);
			p1.applyForce(p1_spring_force);
			p2.applyForce(p2_spring_force);

                        //Add damping Force
			//vector3f damping_force = (vector3f(p1.getVelocity().getX() - p2.getVelocity().getX(), p1.getVelocity().getY() - p2.getVelocity().getY(), p1.getVelocity().getZ() - p2.getVelocity().getZ())).operator*(-1 * damping_constant);
			vector3f p1_damping_force = p1.getVelocity().operator*(-1 * damping_constant);
			vector3f p2_damping_force = p2.getVelocity().operator*(-1 * damping_constant);
			p1.applyForce(p1_damping_force);
			p2.applyForce(p2_damping_force);
		}
	}
};
