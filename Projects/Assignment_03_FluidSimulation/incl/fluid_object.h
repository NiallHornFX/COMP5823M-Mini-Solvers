// COMP5823M - A3 : Niall Horn - fluid_object.h
#ifndef FLUID_OBJECT_H
#define FLUID_OBJECT_H

// Std Headers
#include <vector>

// Project Headers
#include "primitive.h"

// Ext Headers 
#include "ext/glm/glm.hpp" // GLM

struct Particle; 

class Fluid_Object
{
public:
	Fluid_Object(); 
	~Fluid_Object() = default; 

	// ======== Emit ========


	// ======== Render ========
	enum render_type {POINT_VERTS = 0, FRAGMENT = 1};
	void render();

public: 
	std::vector<Particle> Particles; 


private:

	// Point Rendering via Primitive
	Primitive *ren_points; 
	Shader Shad_Points; 

	// Rendering within Fragment Shader
	Primitive *fsQuad; // Fullscreen quad
	Shader *Shad_Frag; 

	// Marching Squares ... 

	friend class Fluid_Solver; 
	friend class Fluid_Collider; 
};










struct Particle
{
	Particle(const glm::vec3 &p, std::size_t idx)
		: P(p), V(glm::vec3(0.f)), F(glm::vec3(0.f)), id(idx), mass(1.f) {}

	glm::vec3 P, V, F;
	std::size_t id;
	float mass;
};



#endif