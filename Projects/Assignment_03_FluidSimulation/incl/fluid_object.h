// COMP5823M - A3 : Niall Horn - fluid_object.h
#ifndef FLUID_OBJECT_H
#define FLUID_OBJECT_H

// Std Headers
#include <vector>
#include <random>

// Project Headers
#include "primitive.h"

// Ext Headers 
#include "ext/glm/glm.hpp" // GLM

#define INLINE __forceinline 

struct Particle; 

class Fluid_Object
{
public:
	Fluid_Object(const glm::vec2 &P = glm::vec2(1.5f, 4.f), const glm::vec2 &Dim = glm::vec2(2.f, 3.f), float Spc = 0.25f);
	~Fluid_Object(); 

	// ======== Emit + Reset ========
	void emit_square();
	void reset_fluid();

	// ======== Render ========
	enum render_type {POINT_VERTS = 0, FRAGMENT = 1};
	void render_setup();
	void render(const glm::mat4 &ortho);

public: 
	enum Colour_Viz {Standard = 0, Density, GridCell, Velocity};
	Colour_Viz particle_colour;
	// ======== Fluid Data ========
	std::vector<Particle> particles; 
	glm::vec2 pos, dim; 
	float spc; 

	// View State
	bool ptColour_ = false;

	// ======== Render Primitives ========
	// Point Rendering via Primitive
	Primitive *ren_points; 

	// Rendering within Fragment Shader
	Primitive *fsQuad; 

	// Marching Squares ... 
};


// ================ Particle Struct ================
struct Particle
{
	Particle(const glm::vec3 &p, float Mass, std::size_t idx)
		: P(p), mass(Mass), rest(P), V(glm::vec3(0.f)), F(glm::vec3(0.f)), id(idx), cell_idx(-1),
		   radius(1.f), density(0.f), pressure(0.f) {}

	glm::vec3 P, V, F;
	glm::vec3 rest; 
	std::size_t id, cell_idx;
	float density, pressure, mass, radius;
};

// ================ Util Functions ================
INLINE float fitRange(float val, float a_min, float a_max, float b_min, float b_max)
{
	return b_min + (val - a_min)*(b_max - b_min) / (a_max - a_min);
}

INLINE glm::vec3 randRange(std::size_t seed, float min, float max)
{
	std::mt19937_64 rng;
	std::uniform_real_distribution<float> dist(min, max);
	rng.seed(seed);        float x = dist(rng);
	rng.seed(seed + 124);  float y = dist(rng);
	rng.seed(seed + 321);  float z = dist(rng);
	return glm::vec3(x, y, z);
}

#endif