/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/

#include "pch.h"
#include "Simulation.h"
#define SIGN(x) (x >= 0 ? 1.0 : -1.0)
#include <stdlib.h>
#include "math3d.h"


void Simulation::initializeParticles()
{
	srand(0);

	float x, y, z;
	for (int i = 0; i < noPt; i++)
	{
		Particle  particle;

		//TO DO: Set particle positions and velocities using srand and world positions
		particle.setPosition(vector3f(rand(), rand(), rand()));
		particle.setVelocity(vector3f(rand(), rand(), rand()));


		particle.clearForce();

		//TO DO: Compute particle's old position for Verlet integration scheme
		//We've just cleared external forces so old position only depends on velocity
		particle.setOldPosition(vector3f(particle.getPosition().getX() - (timestep * particle.getVelocity().getX()), particle.getPosition().getY() - (timestep * particle.getVelocity().getY()), particle.getPosition().getZ() - (timestep * particle.getVelocity().getZ())));
		//particle.setOldPosition(particle.getPosition());
			
		particle.setIntegration(Euler);
		particle.setColor(vector3f(0, 0, 255));
		particle.setPosition(vector3f(particle.getPosition().getX(),
			(particle.getPosition().getY() + 0.1, 0), particle.getPosition().getZ()));
		particle.setOldPosition(vector3f(particle.getOldPosition().getX(),
			(particle.getOldPosition().getY() + 0.1, 0), particle.getOldPosition().getZ()));


		particleList.push_back(particle);

	}
}

void Simulation::simulateEuler(Particle* p)
{
	//TO DO Task 1
	//get current info about particle
	vector3f position_t = p->getPosition();
	vector3f velocity_t = p->getVelocity();
	vector3f force_t = p->getForce();

	//update position
	p->setPosition(position_t.addition(velocity_t.operator*(timestep)));

	//update velocity
	p->setVelocity(velocity_t.addition(force_t.operator*(timestep/commonMass)));

}

void Simulation::simulateEulerCromer(Particle* p)
{
	//TO DO Task 1
	//get current info about particle
	vector3f position_t = p->getPosition();
	vector3f velocity_t = p->getVelocity();
	vector3f force_t = p->getForce();

	//update velocity
	vector3f velocity_t_plus_h = velocity_t.addition(force_t.operator*(timestep / commonMass));
	p->setVelocity(velocity_t_plus_h);

	//update position
	p->setPosition(position_t.addition(velocity_t_plus_h.operator*(timestep)));

}

void Simulation::simulateVerlet(Particle* p)
{
	//TO DO Task 1
	//get current info about particle
	vector3f position_t = p->getPosition();
	vector3f old_position_t = p->getOldPosition();
	vector3f velocity_t = p->getVelocity();
	vector3f force_t = p->getForce();

	//update position
	vector3f new_position_t = (position_t.operator*(2)).operator-(old_position_t).addition((force_t.operator/(commonMass)).operator*(timestep * timestep)).addition(vector3f(pow(timestep, 4), pow(timestep, 4), pow(timestep, 4)));
		//+ O_h_4;
	p->setPosition(new_position_t);

	//update velocity
	p->setVelocity((new_position_t.operator-(position_t)).operator/(timestep));

	//update old position
	p->setOldPosition(position_t);

}

void Simulation::clearForces()
{
	for (int i = 0; i < noPt; i++)
		particleList[i].clearForce();
}

void Simulation::destroyParticles()
{
	particleList.clear();
}

void Simulation::solveWorldCollision()
{
	vector3f tempVel;
	vector3f tempPos;

	for (int i = 0; i < noPt; i++)
	{
		tempVel = particleList[i].getVelocity();
		tempPos = particleList[i].getPosition();

		if (particleList[i].getPosition().getX() <= -halfWorld.getX() || particleList[i].getPosition().getX() >= halfWorld.getX())
		{
			tempVel.setX(tempVel.getX() * -worldColldConst);
			tempPos.setX(SIGN(tempPos.getX()) * halfWorld.getX());
		}

		if (particleList[i].getPosition().getY() <= -halfWorld.getY() || particleList[i].getPosition().getY() >= halfWorld.getY())
		{
			tempVel.setY(tempVel.getY() * -worldColldConst);
			tempPos.setY(SIGN(tempPos.getY()) * halfWorld.getY());
		}

		if (particleList[i].getPosition().getZ() <= -halfWorld.getZ() || particleList[i].getPosition().getZ() >= halfWorld.getZ())
		{
			tempVel.setZ(tempVel.getZ() * -worldColldConst);
			tempPos.setZ(SIGN(tempPos.getZ()) * halfWorld.getZ());
		}

		particleList[i].setVelocity(tempVel);
		particleList[i].setPosition(tempPos);
	}
}

void Simulation::computeSystemEnergies()
{
	//TO DO Task 2
	//Kinetic energy: 1/2 * mass * velocity^2
	//Potential energy: mass * gravity * height 
	//height = particlePosition.y + halfWorld.getY()
	//System total energy = kinetic + potential
	
	kinEn = 0;
	potEn = 0;

	for (int i = 0; i < noPt; i++)
	{
		vector3f tempVel = particleList[i].getVelocity();
		vector3f tempPos = particleList[i].getPosition();

		//update kinetic energy
		kinEn = kinEn + ((1.0 / 2.0) * commonMass * tempVel.lengthSquare());
		
		//update potential energy
		potEn = potEn + (commonMass * 9.8 * tempPos.getY() + halfWorld.getY());
	
	}

}
