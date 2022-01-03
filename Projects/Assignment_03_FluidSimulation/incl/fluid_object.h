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
	~Fluid_Object(); 

	void reset_fluid();

	// ======== Emit Fluid ========
	void emit_square(const glm::vec2 &P, const glm::vec2 &Dim, float h);


	// ======== Render Fluid ========
	enum render_type {POINT_VERTS = 0, FRAGMENT = 1};
	void render_setup();
	void render();

public: 
	std::vector<Particle> particles; 


private:

	// ======== Render Primitives ========
	// Point Rendering via Primitive
	Primitive *ren_points; 

	// Rendering within Fragment Shader
	Primitive *fsQuad; 

	// Marching Squares ... 



	friend class Fluid_Solver; 
	friend class Fluid_Collider; 
};


// ======== Particle Struct ========

struct Particle
{
	Particle(const glm::vec3 &p, std::size_t idx)
		: P(p), V(glm::vec3(0.f)), F(glm::vec3(0.f)), id(idx), mass(1.f), radius(1.f), density(0.f) {}

	glm::vec3 P, V, F;
	std::size_t id;
	float density, mass, radius;
};

#endif