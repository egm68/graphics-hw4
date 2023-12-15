#include "Simulation.h"
#define SIGN(x) (x >= 0 ? 1.0 : -1.0)
#include <stdlib.h>



void Simulation::initializeMesh()
{
	srand(0);

	float cornerx=-1, cornery=-1, cornerz=4;
	mesh.particles.clear();
	mesh.springs.clear();
	mesh.surfaceTriangles.clear();

	for (int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			for(int k = 0; k < 3; k++)
			{
				Particle  particle;
				particle.clearForce();
				particle.setPosition(vector3f(cornerx + 2*i, cornery + 2*j, cornerz + 2*k));
				particle.setColor(vector3f(1, 0, 0));

				//particle.setOldPosition(particle.getPosition().operator-(particle.getVelocity().operator*(timestep)));
				particle.setOldPosition(vector3f(particle.getPosition().getX() - (timestep * particle.getVelocity().getX()), particle.getPosition().getY() - (timestep * particle.getVelocity().getY()), particle.getPosition().getZ() - (timestep * particle.getVelocity().getZ())));
				mesh.particles.push_back(particle);
			}

	Spring s;
	// each pair shows the id of the particles in the cube for each spring
	int springset[304] = {0,1, 1,2, 3,4, 4,5, 6,7, 7,8, 0,3, 3,6, 1,4, 4,7, 2,5, 5,8, 0,4, 4,2, 3,7, 7,5,
	                      9,10, 10,11, 12,13, 13,14, 15,16, 16,17, 9,12, 12,15, 10,13, 13,16, 11,14, 14,17, 9,13, 13,11, 12,16, 16,14,
	                      18,19, 19,20, 21,22, 22,23, 24,25, 25,26, 18,21, 21,24, 19,22, 22,25, 20,23, 23,26, 18,22, 22,20, 21,25, 25,23,
	                      2,11, 11,20, 5,14, 14,23, 8,17, 17,26, 2,14, 14,20, 5,17, 17,23,
	                      1,10, 10,19, 4,13, 13,22, 7,16, 16,25, 1,13, 13,19, 4,16, 16,22,
	                      0,9, 9,18, 3,12, 12, 21, 6,15, 15,24, 0,12, 12,18, 3,15, 15, 21,
		                  8,5, 5,2, 17,14, 14,11, 26,23, 23,20,7,4, 4,1, 16,13, 13, 10, 25, 22, 22, 19, 6,3, 3,0, 15,12, 12,9, 24,21, 21,18,
	                      6,15, 15,24, 7,16, 16,25, 8,17, 17,26, 6,7, 7,8, 15,16, 16,17, 24,25, 25,26, 6,16, 16,24, 7,17, 17,25,
	                      3,12, 12,21, 4,13, 13,22, 5,14, 14,23, 3,4, 4,5, 12,13, 13,14, 21,22, 22,23, 3,13, 13,21, 4,14, 14,22,
	                      0,9, 9,18, 1,10, 10,19, 2,11, 11,20, 0,1, 1,2, 9,10, 10,11, 18,19, 19,20, 0,10, 10,18, 1,11, 11,19,
	                      5,16, 3,16, 23,16, 21,16, 5,10, 10,22, 10,3, 10,21};

	//particles for each triangle on outer faces, 48 triangles in total
	int triSet[144] = { 17,8,5, 14,17,5, 26,17,14, 23,26,14, 14,5,2, 1,14,2, 23,14,20, 14,11,20,
						26,23,22, 25,26,22, 24,25,21, 25,22,21, 22,23,19, 23,20,19, 22,22,18, 22,19,18,
						15,6,7, 16,15,7, 24,15,16, 25,24,16, 8,16,7, 17,16,8, 17,25,16, 26,25,17,
						7,6,3, 4,7,3, 8,7,4, 5,8,4, 4,3,0, 1,4,0, 5,4,1, 2,5,1,
						21,15,24, 12,15,21, 6,5,12, 3,6,12, 18,12,21, 9,12,18, 3,12,9, 0,3,9,
						11,2,1, 10,11,1, 20,11,10, 19,20,10, 0,10,1, 9,10,0, 19,10,9, 18,19,9 };

	for (int i = 0; i < 304; i += 2)
	{
		Spring s;
		s.p1 = springset[i];
		s.p2 = springset[i + 1];
		vector3f pos1 = mesh.particles[s.p1].getPosition();
		vector3f pos2 = mesh.particles[s.p2].getPosition();
		s.initLength = (pos1 - pos2).length();
		mesh.springs.push_back(s);
	}

	for (int i = 0; i < 144; i += 3)
	{
		Triangle t;
		t.p1 = triSet[i];
		t.p2 = triSet[i + 1];
		t.p3 = triSet[i + 2];

		vector3f pos1 = mesh.particles[t.p1].getPosition();
		vector3f pos2 = mesh.particles[t.p2].getPosition();
		vector3f pos3 = mesh.particles[t.p3].getPosition();

		vector3f u = pos2 - pos1;
		vector3f v = pos3 - pos1;
		vector3f normal = u.computeSurfaceNormal(v);

		normal.makeUnit();
		t.normal = normal;
		mesh.surfaceTriangles.push_back(t);
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
	p->setVelocity(velocity_t.addition(force_t.operator*(timestep / commonMass)));

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
	vector3f new_position_t = (position_t.operator*(2)).operator-(old_position_t).addition((force_t.operator/(commonMass)).operator*(timestep * timestep));
	//.addition(vector3f(pow(timestep, 4), pow(timestep, 4), pow(timestep, 4)));
	//+ O_h_4;
	p->setPosition(new_position_t);

	//update velocity
	p->setVelocity((new_position_t.operator-(position_t)).operator/(timestep));

	//update old position
	p->setOldPosition(position_t);

}

void Simulation::clearForces()
{
	for (int i = 0; i < mesh.particles.size(); i++)
		mesh.particles[i].clearForce();
}

void Simulation::destroyParticles()
{
	mesh.particles.clear();
}

void Simulation::solveWorldCollision()
{
	vector3f tempVel;
	vector3f tempPos;

	for (int i = 0; i < mesh.particles.size(); i++)
	{
		tempVel = mesh.particles[i].getVelocity();
		tempPos = mesh.particles[i].getPosition();

		if (mesh.particles[i].getPosition().getX() <= -halfWorld.getX() || mesh.particles[i].getPosition().getX() >= halfWorld.getX())
		{
			tempVel.setX(tempVel.getX() * -worldColldConst);
			tempPos.setX(SIGN(tempPos.getX()) * halfWorld.getX());
		}

		if (mesh.particles[i].getPosition().getY() <= -halfWorld.getY() || mesh.particles[i].getPosition().getY() >= halfWorld.getY())
		{
			tempVel.setY(tempVel.getY() * -worldColldConst);
			tempPos.setY(SIGN(tempPos.getY()) * halfWorld.getY());
		}

		if (mesh.particles[i].getPosition().getZ() <= -halfWorld.getZ() || mesh.particles[i].getPosition().getZ() >= halfWorld.getZ())
		{
			tempVel.setZ(tempVel.getZ() * -worldColldConst);
			tempPos.setZ(SIGN(tempPos.getZ()) * halfWorld.getZ());
		}

		mesh.particles[i].setVelocity(tempVel);
		mesh.particles[i].setPosition(tempPos);
	}
}

void Simulation::computeSystemEnergies()
{
	kinEn = 0.0f;
	potEn = 0.0f;
	sprEn = 0.0f;

        //TO DO - Compute kinetic and potential energy
	for (int i = 0; i < mesh.particles.size(); i++)
	{
		vector3f tempVel = mesh.particles[i].getVelocity();
		vector3f tempPos = mesh.particles[i].getPosition();

		//update kinetic energy
		kinEn = kinEn + ((1.0 / 2.0) * commonMass * tempVel.lengthSquare());

		//update potential energy
		potEn = potEn + (commonMass * 9.8 * tempPos.getY() + halfWorld.getY());
    }

         //TO DO - Compute spring energy
	for (int s = 0; s < mesh.springs.size(); s++)
	{
		Particle& p1 = mesh.particles[mesh.springs[s].p1];
		Particle& p2 = mesh.particles[mesh.springs[s].p2];
		float spring_init_length = mesh.springs[s].initLength;
		float hookes_law_x = mesh.springs[s].initLength - (pow(p2.getPosition().getX() - p1.getPosition().getX(), 2) + pow(p2.getPosition().getY() - p1.getPosition().getY(), 2) + pow(p2.getPosition().getZ() - p1.getPosition().getZ(), 2));
		sprEn = sprEn + ((1.0 / 2.0) * spring_constant * pow(hookes_law_x, 2));
	}


}
