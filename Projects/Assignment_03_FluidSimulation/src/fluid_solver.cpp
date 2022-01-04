// COMP5823M - A3 : Niall Horn - fluid_solver.cpp
// Implements
#include "fluid_solver.h"

// Std Headers
#define _USE_MATH_DEFINES
#include <math.h>
#include <functional>

// Project Headers
#include "fluid_object.h"
#include "fluid_collider.h"
#include "hash_grid.h"


// ================================== Fluid_Solver Class Implementation ===============================

Fluid_Solver::Fluid_Solver(float Sim_Dt, float RestDens, float KernelRad, Fluid_Object *Data)
	: fluidData(Data), rest_density(RestDens), kernel_radius(KernelRad), dt(Sim_Dt), at(0.f)
{
	// ===== Init Solver =====
	gravity = -1.0f;
	viscosity = 0.f; 
	force_coeff = 1.f; 
	frame = 0, timestep = 0; 
	simulate = false;
	kernel_radius_sqr = kernel_radius * kernel_radius;
	rest_density = 10.f; 
	stiffness_coeff = 1e03f; 

	// ===== Setup Tank Collider Planes =====
	Fluid_Collider_Plane *left  = new Fluid_Collider_Plane("Tank_Left", glm::vec3(2.5f, 0.0f, 0.f),   glm::vec3(1.f, 0.f, 0.f),  glm::vec2(0.0f, 5.0f));
	Fluid_Collider_Plane *right = new Fluid_Collider_Plane("Tank_Right", glm::vec3(7.5f, 0.f, 0.f),  glm::vec3(-1.f, 0.f, 0.f), glm::vec2(0.0f, 5.0f));
	Fluid_Collider_Plane *floor = new Fluid_Collider_Plane("Tank_Floor", glm::vec3(2.5f, 0.0f, 0.f), glm::vec3(0.f, 1.f, 0.f),  glm::vec2(5.0f, 0.f));
	colliders.push_back(std::move(left)), colliders.push_back(std::move(right)), colliders.push_back(std::move(floor));

	// ===== Pre Compute Kernel + Derivative Scalar Coeffecints =====
	// Poly 6
	poly6_s = 4.f / M_PI  * std::powf(kernel_radius, 8.f);
	poly6_grad_s = -24.f / M_PI * std::powf(kernel_radius, 8.f);
	// Spiky
	spiky_s = 10.f / M_PI * std::powf(kernel_radius, 5.f);
	spiky_grad_s = -30.f / M_PI * std::powf(kernel_radius, 5.f);
	// Viscosity
	visc_lapl_s = -20.f / M_PI * std::powf(kernel_radius, 5.f);
}

// Info : Tick Simulation for number of timesteps determined by viewer Dt. Uses Hybrid Timestepping approach purposed by Glenn Fiedler
// Physics Dt is constant (based on UI value), while using Viewer app Dt to define number of solve substeps.
void Fluid_Solver::tick(float viewer_Dt)
{
	if (!simulate || fluidData->particles.size() == 0) return;

	// Reset Timestep (solvestep) counter (per tick)
	timestep = 0;

	// Debug use single time/substep 
	step(); 
	frame++;

	/*
	// Subdivide Accumulated Viewer Timestep into Solver Substeps
	at += viewer_Dt;
	while (at > dt)
	{
		step(); // Single Timestep
		at -= dt;
		timestep++;
	}
	frame++;
	*/
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
	get_neighbours();
	integrate();
	compute_dens_pres(&Fluid_Solver::kernel_poly6);
	eval_colliders();

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

void Fluid_Solver::render_colliders(const glm::mat4 &ortho)
{
	for (Fluid_Collider *col : colliders)
	{
		col->prim->shader.setMat4("proj", ortho);
		col->prim->render();
	}
}

void Fluid_Solver::integrate()
{
	for (Particle &p : fluidData->particles)
	{
		p.V += glm::vec3(0.f, -4.5f, 0.f) * dt; 
		p.P += p.V * dt; 
	}
}

void Fluid_Solver::get_neighbours()
{
	if (hg) delete hg;
	hg = new Hash_Grid(fluidData, 10, kernel_radius * 1.2f);
	hg->hash();
}

// ================================== Eval Attrib Functions ===============================
// Loop over particle neighbours (within hash cell) calc density using passed smoothing kernel func ptr. 
void Fluid_Solver::compute_dens_pres(kernel_func w)
{
	for (std::size_t p_i = 0; p_i < fluidData->particles.size(); ++p_i)
	{
		Particle &Pt_i = fluidData->particles[p_i];
		float dens_tmp = 0.f; 

		std::vector<Particle*> *neighbours = hg->grid[Pt_i.cell_idx];
		//std::cout << "Particle_" << Pt_i.id << "Neighbour Cell Count = " << neighbours->size() << "\n";
		for (std::size_t p_j = 0; p_j < neighbours->size(); ++p_j)
		{
			const Particle &Pt_j = *((*neighbours)[p_j]);
			dens_tmp += (this->*w)(Pt_i.P - Pt_j.P);
		}
		Pt_i.density = dens_tmp; 

		// Calc Pressure
		Pt_i.pressure = stiffness_coeff * (Pt_i.density - rest_density);
	}
}


// ================================== Kernel Functions ===============================

// Assumes r = |r - r_j|^2
float Fluid_Solver::kernel_poly6(const glm::vec3 &r)
{
	float r_sqr = glm::dot(r, r);

	if (r_sqr >= kernel_radius_sqr) return 0.f;
	
	return poly6_s * std::powf((kernel_radius_sqr - r_sqr), 3.f);
}