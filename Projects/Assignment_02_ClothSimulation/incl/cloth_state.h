// COMP5823M A2 : Niall Horn 201486968 - cloth_state.h
#ifndef CLOTH_STATE_H
#define CLOTH_STATE_H

// Std Headers
#include <vector>
#include <string>
#include <fstream>

// Project Headers
#include "mesh.h"

// Ext Headers
#include "ext/glm/glm.hpp"

// Struct FDs
struct Particle;
struct Spring; 

class Cloth_State
{
public:
	Cloth_State(const char *path); 
	~Cloth_State();

	void load_obj(std::ifstream &in);

	void build_cloth(); 

	void render(const glm::mat4x4 &view, const glm::mat4x4 &persp);

private:

	// Obj Mesh
	std::string file_path; 

	// Cloth Sim Data
	std::vector<Particle> *particles; 
	std::vector<Spring>   *springs; 

	// Cloth Render Data
	Mesh cloth_mesh; 

	friend class Cloth_Solver; 
};



// =================================== Simulation Primitive Objects ===================================

// ======== Particles ========
enum pt_state
{
	FREE = 0, FIXED
};

struct Particle
{
	glm::vec3 P, V, F, N; 
	std::size_t id; 
	pt_state state; 
	float mass; 
};

// ======== Spring ========
enum spring_type
{
	STRUCT = 0, SHEAR, BEND
};

struct Spring
{
	Particle *pt_0, *pt_1; // Two Particles / point masses of spring. 
	spring_type type; 
	float k, c;  // Stiffness and Damping coeffs
	float rest;  // Rest Length
};


#endif