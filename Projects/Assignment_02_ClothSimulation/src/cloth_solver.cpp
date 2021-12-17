// COMP5823M - A2 : Niall Horn - cloth_solver.cpp
// Implements
#include "cloth_solver.h"

// Std Headers
#include <iostream>


// ================================== Cloth_Solver Class Implementation ===============================

Cloth_Solver::Cloth_Solver(Cloth_State &ClothState, float Sim_Dt)
	: clothData(ClothState), dt(Sim_Dt), at(0.f)
{
	// Init 
	gravity = -9.8f;
	wind = glm::vec3(0.f);
	k = 10.f, c = 1.f; 
	coeff_force = 1.f; 
	coeff_fric  = 1.f; 
	frame = 0, timestep = 0; 
	simulate = false; 
}

// Info : Tick Simulation for number of timesteps determined by viewer Dt.
// Uses Hybrid Timestepping approach purposed by Glenn Fiedler : Physics Dt is constant while using Viewer Dt as input.
void Cloth_Solver::tick(float viewer_Dt)
{
	if (!simulate || !clothData.built_state) return; 

	// Reset Timestep counter (per tick)
	timestep = 0; 

	// Subdivide Accumulated Viewer Timestep into Solver Substeps
	at += viewer_Dt;
	while (at > dt)
	{
		step(); // Single Timestep
		at -= dt;
		timestep++;
	}
	frame++;

	// Debug
	std::cout << "DEBUG::ClothSolver::Frame_" << frame << " Number of SolveSteps = " << timestep << "\n";
}

// Info : Reset Cloth_State and Solver Time State
void Cloth_Solver::reset()
{
	// Reset Time state
	frame = 0, timestep = 0; 

	// Reset Cloth_State
	clothData.reset_cloth();
}

// Info : Single Simulation Timestep of Solver 
void Cloth_Solver::step()
{
	// Test
	for (std::size_t p = 0; p < clothData.particles.size(); ++p)
	{
		Particle &curPt = clothData.particles[p]; 
		curPt.P.y += (0.01f * coeff_force); 
	}
}