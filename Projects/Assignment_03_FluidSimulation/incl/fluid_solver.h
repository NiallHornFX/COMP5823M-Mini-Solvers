// COMP5823M - A3 : Niall Horn - fluid_solver.h
#ifndef FLUID_SOLVER_H
#define FLUID_SOLVER_H

// Std Headers
#include <vector>

// Ext Headers 
// GLM
#include "ext/glm/glm.hpp"

class Fluid_Object; 
class Fluid_Collider; 

struct Particle; 

using kernel_func = float(*) (glm::vec3 v);

class Fluid_Solver
{
public: 
	Fluid_Solver(float Sim_Dt, Fluid_Object *Data);
	~Fluid_Solver() = default; 

	// ======= Operations =======
	void reset();

	void tick(float viewer_Dt);

	void step();

	void render_colliders(const glm::mat4 &ortho);

	// ======= Solver =======
	void eval_colliders();

	void eval_forces();

	void integrate();


	// ======= Eval Fluid Quanities =======
	float eval_fluid_density(const glm::vec3 P,  kernel_func w);
	float eval_fluid_pressure(const glm::vec3 P, kernel_func w);

	// ======= Smoothing Kernel Functions =======
	float kernel_poly6(glm::vec3 v); 
	float kernel_poly6_laplacian(glm::vec3 v);
	float kernel_spiky(glm::vec3 v);
	float kernel_spiky_gradient(glm::vec3 v);

public:
	// ======= Solver Intrinsics =======
	bool simulate;
	float at, dt; // Acumulated Time, Fixed Physics timestep. 
	std::size_t frame, timestep;

	// ======= Forces =======
	float gravity, viscosity; 
	float force_coeff; 

	float kernel_radius;

	// Fluid Colliders
	std::vector<Fluid_Collider*> colliders;

	// Fluid State Object
	Fluid_Object *fluidData; 
};

#endif