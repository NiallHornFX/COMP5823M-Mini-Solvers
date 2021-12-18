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
	gravity = -1.0f;
	wind = glm::vec3(0.f);
	K_s = 1.f, K_c = 1.f; 
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

// Info : Eval Springs and apply the resulting forces to their particles. 
void Cloth_Solver::eval_springs()
{
	for (Spring &s : clothData.springs)
	{
		glm::vec3 p1p0 = s.pt_0->P - s.pt_1->P;
		glm::vec3 p1p0_n = glm::normalize(p1p0);
		glm::vec3 v1v0 = s.pt_0->V - s.pt_1->V;
		float cl = glm::length(p1p0);

		// Spring Force (-K_s * (||p1-p0|| - rest) * (p1-p0 / ||p1-p0||))
		glm::vec3 spring = (-K_s) * (cl - s.rest) * p1p0_n;
		// Damper Force 
		glm::vec3 damper = (-K_c) * glm::dot(v1v0, p1p0_n) * p1p0_n;

		glm::vec3 sprdmp = spring + damper; 

		//glm::vec3 comb = glm::dot((K_s*(s.rest - glm::length(p1p0)) - K_c * v1v0), p1p0_n) * p1p0_n; 

		// Apply equal opposite forces to particles of spring
		s.pt_0->F += sprdmp, s.pt_1->F += -sprdmp;
	}
}

// Info : Using Semi Implicit Forward Euler, Integrate forces to particle positions for (n+1)
void Cloth_Solver::integrate_euler()
{
	for (Particle &curPt : clothData.particles)
	{
		if (curPt.state == pt_state::FIXED) continue; // Skip Fixed Particles 

		glm::vec3 forces = curPt.F + wind;
		// if Particle Mass is 1.0 we need not do A = F / M. 
		glm::vec3 accel = forces / curPt.mass; 
		// Gravity invaraint to mass.
		accel += glm::vec3(0.f, gravity, 0.f);

		// ==== Integrate (Semi-Implicit Euler) ====
		// v_(n+1) = v(n) + a(n) * dt; 
		curPt.V = curPt.V + accel * dt;
		// x_(n+1) = x(n) + v(n) * dt; 
		curPt.P = curPt.P + curPt.V * dt;
	}
}

// Info : Single Simulation Timestep of Solver 
void Cloth_Solver::step()
{
	eval_springs();

	integrate_euler();
}