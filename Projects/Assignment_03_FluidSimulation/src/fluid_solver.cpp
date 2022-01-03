// COMP5823M - A3 : Niall Horn - fluid_solver.cpp
// Implements
#include "fluid_solver.h"

// Std Headers

// Project Headers
#include "fluid_object.h"
#include "fluid_collider.h"

Fluid_Solver::Fluid_Solver(float Sim_Dt, Fluid_Object *Data)
	: fluidData(Data), dt(Sim_Dt), at(0.f)
{
	// Init 
	gravity = -1.0f;
	kernel_radius = 1.f; 
	viscosity = 0.f; 
	force_coeff = 1.f; 
	frame = 0, timestep = 0; 
	simulate = false;

	// Allocate Tank Collider Planes
	Fluid_Collider_Plane *floor = new Fluid_Collider_Plane("Tank_Floor", glm::vec3(-0.5f, -1.0f, 0.f), glm::vec3(0.f, 1.f, 0.f),  glm::vec2(1.0f, 0.f));
	Fluid_Collider_Plane *left  = new Fluid_Collider_Plane("Tank_Left",  glm::vec3(-0.5f, -1.0f, 0.f), glm::vec3(1.f, 0.f, 0.f),  glm::vec2(0.0f, 1.0f));
	Fluid_Collider_Plane *right = new Fluid_Collider_Plane("Tank_Right", glm::vec3(0.5f,  -1.0f, 0.f), glm::vec3(-1.f, 0.f, 0.f), glm::vec2(0.0f, 1.0f));
	colliders.push_back(floor), colliders.push_back(left), colliders.push_back(right);
}

// Info : Tick Simulation for number of timesteps determined by viewer Dt. Uses Hybrid Timestepping approach purposed by Glenn Fiedler
// Physics Dt is constant (based on UI value), while using Viewer app Dt to define number of solve substeps.
void Fluid_Solver::tick(float viewer_Dt)
{
	if (!simulate || fluidData->particles.size() == 0) return;

	// Reset Timestep (solvestep) counter (per tick)
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
}

// Info : Reset Fluid Object State and Solver Time State
void Fluid_Solver::reset()
{
	// Reset Time state
	frame = 0, timestep = 0;

	// Reset Cloth_State
	fluidData->reset_fluid();
}

// Info : Single Simulation Step of Cloth Solver 
void Fluid_Solver::step()
{
	// Get Particle Neighbours (HashGrid)
	// Compute Particle Pressure
	// Compute Particle Forces
	// Integrate Particle Forces
	// Eval Particle Collisions

}

// Info : Evaulate Collisions using passed colliders.
void Fluid_Solver::eval_colliders()
{
	if (!colliders.size()) return;

	for (Fluid_Collider *col : colliders)
	{
		if (!col) continue;
		col->eval_collision(fluidData->particles);
	}
}

void Fluid_Solver::render_colliders()
{
	for (Fluid_Collider *col : colliders)
	{
		col->prim->render();
	}
}