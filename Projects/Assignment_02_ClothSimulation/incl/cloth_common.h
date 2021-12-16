#ifndef CLOTH_COMMON_H
#define CLOTH_COMMON_H

// Ext Headers
#include "ext/glm/glm.hpp"

// ======== Particles ========
enum pt_state
{
	FREE = 0, FIXED
};

// Particles map 1:1 to indexed vertices. 
struct Particle
{
	Particle(const glm::vec3 &p, std::size_t idx)
		: P(p), V(glm::vec3(0.f)), F(glm::vec3(0.f)), id(idx), spring_count(0), N(glm::vec3(0.f)), state(pt_state::FREE), mass(1.f) {}

	glm::vec3 P, V, F, N;
	pt_state state;
	std::size_t id, spring_count;
	float mass;
};

// ======== Spring ========
enum spring_type
{
	STRUCT = 0, SHEAR, BEND
};

struct Spring
{
	Spring(Particle *P0, Particle *P1, float RL) :
		pt_0(P0), pt_1(P1), rest(RL) {}
	Particle *pt_0, *pt_1; // Two Particles / point masses of spring. 
	spring_type type;
	float k, c;  // Stiffness and Damping coeffs
	float rest;  // Rest Length
};


// Typedefs 

using ParticleTriList = std::vector<std::vector<glm::ivec3*>>;

#endif