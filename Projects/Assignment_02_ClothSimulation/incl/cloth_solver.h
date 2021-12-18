// COMP5823M - A2 : Niall Horn - cloth_solver.h
#ifndef CLOTH_SOLVER_H
#define CLOTH_SOLVER_H

// Std Headers

// Ext Headers
#include "ext/glm/glm.hpp"

// Project Headers
#include "cloth_state.h"


class Cloth_Solver
{
public:
	Cloth_Solver(Cloth_State &ClothState, float Sim_Dt); 
	~Cloth_Solver() = default; 

	// ======= Operations =======
	void reset(); 

	void tick(float viewer_Dt);

	// ======= Solver =======
	void step();

	void eval_springs();

	void integrate_euler();

	// Collide 

public:
	// Solver Intrinsics
	bool simulate; 
	float at, dt; // Acumulated Time, Fixed Physics timestep. 
	std::size_t frame, timestep;

	// Forces
	glm::vec3 wind; 
	float gravity; 

	// Coeffs
	float K_s, K_c; // Stiffness and Damping Coeffs
	float coeff_force; 
	float coeff_fric; 

	// Cloth_State Ref
	Cloth_State &clothData; 
};

#endif
