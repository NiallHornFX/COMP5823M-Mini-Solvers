// COMP5823M - A3 : Niall Horn - fluid_solver.cpp
// Implements
#include "fluid_solver.h"

// Std Headers
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <cassert>

// Project Headers
#include "fluid_object.h"
#include "fluid_collider.h"
#include "hash_grid.h"
#include "Spatial_Grid.h"


// ================================== Fluid_Solver Class Implementation ===============================

Fluid_Solver::Fluid_Solver(float Sim_Dt, float RestDens, float KernelRad, Fluid_Object *Data)
	: fluidData(Data), rest_density(RestDens), kernel_radius(KernelRad), dt(Sim_Dt), at(0.f)
{
	// ===== Init Solver =====
	gravity     = -10.0f;
	air_resist  = 0.5f;
	k_viscosity = 0.f; 
	k_surftens  = 0.f; 
	frame       = 0; 
	timestep    = 0;
	simulate    = false;
	got_neighbours = false; 
	kernel_radius_sqr = kernel_radius * kernel_radius;
	rest_density = 8.f; 
	stiffness_coeff = 1.f; 
	min_dens = 0.f, max_dens = 0.f, min_pres = 0.f, max_pres = 0.f, min_force = 0.f, max_force = 0.f; 

	// ===== Setup Tank Collider Planes =====
	Fluid_Collider_Plane *left  = new Fluid_Collider_Plane("Tank_Left",  glm::vec3(2.5f, 0.0f, 0.f), glm::vec3(1.f, 0.f, 0.f),  glm::vec2(0.0f, 5.0f));
	Fluid_Collider_Plane *right = new Fluid_Collider_Plane("Tank_Right", glm::vec3(7.5f, 0.f, 0.f),  glm::vec3(-1.f, 0.f, 0.f), glm::vec2(0.0f, 5.0f));
	Fluid_Collider_Plane *floor = new Fluid_Collider_Plane("Tank_Floor", glm::vec3(2.5f, 0.0f, 0.f), glm::vec3(0.f, 1.f, 0.f),  glm::vec2(5.0f, 0.f));
	colliders.push_back(std::move(left));
	colliders.push_back(std::move(right));
	colliders.push_back(std::move(floor));

	// ===== Pre Compute Kernel + Derivative Scalar Coeffecints =====
	// Poly 6
	poly6_s      =  315.f  / (64.f * M_PI  * std::powf(kernel_radius, 9.f));
	poly6_grad_s = -945.f  / (32.f * M_PI  * std::powf(kernel_radius, 9.f));
	// Spiky
	spiky_s      = 15.f  / (M_PI * std::powf(kernel_radius, 6.f));
	spiky_grad_s = -45.f / (M_PI * std::powf(kernel_radius, 6.f));
	// Viscosity
	visc_lapl_s  = 45.f  / (M_PI * std::powf(kernel_radius, 5.f));
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

	// DEBUG - auto reset
	//if (frame > 30) { simulate = false; reset(); return; }

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

	// Reset Attrib Ranges
	min_dens = 0.f, max_dens = 0.f, min_pres = 0.f, max_pres = 0.f; min_force = 0.f, max_force = 0.f; 

	// Reset Fluid State
	fluidData->reset_fluid();
}

// Info : Single Simulation Step of Cloth Solver 
void Fluid_Solver::step()
{
	get_neighbours();
	compute_dens_pres(&Fluid_Solver::kernel_poly6);
	eval_colliders();
	integrate();

	// Get Particle Neighbours (HashGrid)
	// Compute Particle Density + Pressure
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
	// Chosen Kernel Functions
	kernel_func      kernel = &Fluid_Solver::kernel_poly6;
	//kernel_grad_func grad = &Fluid_Solver::kernel_poly6_gradient;
	kernel_grad_func grad = &Fluid_Solver::kernel_spiky_gradient;

	// Ext Forces 
	glm::vec3 g(0.f, gravity, 0.f); 

	/*
	// Semi Implicit Euler Integration (testing)
	for (Particle &p : fluidData->particles)
	{
		// Eval RHS forces 0 
		glm::vec3 a_0 = g + eval_forces(p, kernel, grad);

		// Integrate Accel and Vel
		p.V += a_0 * dt; 
		p.P += p.V * dt; 
	} 
	*/

	// Leapfrog integration
	for (Particle &pt : fluidData->particles)
	{
		// Pre-calc
		float r_dens = 1.f / pt.density; 
		glm::vec3 air_res = -air_resist * pt.V; 
		
		// Eval RHS forces 0 
		glm::vec3 a_0 = (eval_forces(pt, kernel, grad) * r_dens) + g + air_res;
		// Integrate P 
		pt.P += (pt.V * dt) + (0.5f * a_0 * (dt*dt));

		// Eval RHS forces 1
		glm::vec3 a_1 = (eval_forces(pt, kernel, grad) * r_dens) + g + air_res;
		// Integrate V
		pt.V += 0.5f * (a_0 + a_1) * dt; 

		// Store Min/Max sqr_force
		float f_s = glm::dot(a_1, a_1);
		if (f_s < min_force) min_force = f_s; 
		if (f_s > max_force) max_force = f_s; 
	}
}

void Fluid_Solver::get_neighbours()
{
	/* =========== Spatial Accel : Hash Grid ===========
	// Delete old Hash Grid
	got_neighbours = false; 
	if (hg) delete hg;

	// Cell Size based on Kernel Radius
	float cs = kernel_radius * 1.5f; 

	// New Hash Grid 
	hg = new Hash_Grid(fluidData, 10, cs);
	hg->hash();
	got_neighbours = true; 

	// Test : Adjacent Hash of single particle, viz adj cells. 
	std::size_t testPt = 35; 
	auto pts = hg->get_adjacent_cells(fluidData->particles[testPt]);
	std::size_t idx = fluidData->particles[testPt].cell_idx;
	// Rm all pts cell indices first
	for (Particle &pt : fluidData->particles) pt.cell_idx = 0; 
	// Fill Adj particle list cell_idx with same idx for viz debgging.
	for (Particle *pt : pts)
	{
		pt->cell_idx = idx; 
	}
	fluidData->particles[testPt].cell_idx = 5;
	*/

	// =========== Spatial Accel : Uniform Grid ===========
	// Delete Grid
	got_neighbours = false; 
	if (accel_grid) delete accel_grid;
	float cs = kernel_radius;
	accel_grid = new Spatial_Grid(fluidData, kernel_radius * 1.f, 10.f);
	accel_grid->gather_particles();
	got_neighbours = true; 

	// Cache Particle Neighbours into array per particle. 
	fluidData->particle_neighbours.clear();
	fluidData->particle_neighbours.resize(fluidData->particles.size());
	for (std::size_t p = 0; p < fluidData->particles.size(); ++p)
	{
		fluidData->particle_neighbours[p] = accel_grid->get_adjcell_particles(fluidData->particles[p]);
	} 

	/*
	// Test : Adjacent Hash of single particle, viz adj cells. 
	Particle &testPt = fluidData->particles[128];
	auto pts = accel_grid->get_adjcell_particles(testPt);
	// Rm all pts cell indices first
	for (Particle &pt : fluidData->particles) pt.cell_idx = 0;
	// Fill Adj particle list cell_idx with same idx for viz debgging.
	for (Particle *pt : pts)
	{
		pt->cell_idx = 3;
	}
	testPt.cell_idx = 20; 
	*/
}

// ================================== Eval Attrib Functions ===============================
// Loop over particle neighbours (within hash cell) calc density using passed smoothing kernel func ptr. 
// Can be safely multithreaded 

void Fluid_Solver::compute_dens_pres(kernel_func w)
{
	// Check Hash Grid has been evaulated
	if (!got_neighbours)
	{
		std::cerr << "ERROR::Frame::" << frame << " Fluid_Solver::Particles Neighbours not caluclated via Hash Grid" << std::endl;
		return; 
	}

	min_dens = 1e06f, max_dens = 0.f, min_pres = 1e06f, max_pres = 0.f;
	for (std::size_t p_i = 0; p_i < fluidData->particles.size(); ++p_i)
	{
		Particle &Pt_i = fluidData->particles[p_i];
		float dens_tmp = 0.f; 

		std::vector<Particle*> &neighbours = fluidData->particle_neighbours[p_i];
		for (std::size_t p_j = 0; p_j < neighbours.size(); ++p_j)
		{
			const Particle &Pt_j = *(neighbours[p_j]);
			dens_tmp += Pt_j.mass * (this->*w)(Pt_i.P - Pt_j.P);
		}
		Pt_i.density = dens_tmp; 

		// Calc Pressure using equation of state : pres_i = k (rho - rho_0). No Negative Pressure (Use Surf Tens force).
		Pt_i.pressure = std::max((stiffness_coeff * (Pt_i.density - rest_density)), 0.f); 

		// Store Min/Max Dens (Debug) 
		if (Pt_i.density  < min_dens) min_dens = Pt_i.density;
		if (Pt_i.pressure < min_pres) min_pres = Pt_i.pressure;
		if (Pt_i.density  > max_dens) max_dens = Pt_i.density;
		if (Pt_i.pressure > max_pres) max_pres = Pt_i.pressure;
	}
	if (frame == 0) rest_density = max_dens * 1.025f; // Use as inital rest_dens
}

glm::vec3 Fluid_Solver::eval_forces(Particle &Pt_i, kernel_func w, kernel_grad_func w_g)
{
	// Check Hash Grid has been evaulated
	if (!got_neighbours)
	{
		std::cerr << "ERROR::Frame::" << frame << " Fluid_Solver::Particles Neighbours not caluclated via Hash Grid" << std::endl;
		return glm::vec3(0.f);
	}

	glm::vec3 force_pressure (0.f); 
	//glm::vec3 force_surftension;
	//glm::vec3 force_viscosity; 

	// Reset Particle forces 
	Pt_i.F.x = 0.f, Pt_i.F.y = 0.f, Pt_i.F.z = 0.f;

	// Cache neighbour cell ptr
	std::vector<Particle*> &neighbours = fluidData->particle_neighbours[Pt_i.id];

	// =============== Compute Pressure Gradient --> Pressure Force ===============
	glm::vec2 pressure_grad(0.f);
	for (std::size_t p_j = 0; p_j < neighbours.size(); ++p_j)
	{
		const Particle &Pt_j = *(neighbours[p_j]);
		if (Pt_j.id == Pt_i.id) continue;  // Skip Self 
		glm::vec2 tmp =  Pt_j.mass * ((Pt_i.pressure + Pt_j.pressure) / (2.f * Pt_j.density)) * (this->*w_g)(Pt_i.P - Pt_j.P);
		pressure_grad += tmp; 
	}
	force_pressure = -glm::vec3(pressure_grad.x, pressure_grad.y, 0.f);

	// =============== Compute Viscosity Laplacian --> Viscosity Force ===============


	// =============== Compute Surface Gradient --> Surface Tension Force ===============


	// Accumulate forces
	// ret instead of write to Pt.F
	return force_pressure; 
}
// ================================== Util Functions ===============================
// Info : Calc an estimate of the rest density from the maxium density value * some offset. 
//        This can be called external of solve step, to get inital estimate of rest_density. 
void Fluid_Solver::calc_restdens()
{
	get_neighbours();
	compute_dens_pres(&Fluid_Solver::kernel_poly6);
	rest_density = max_dens * 1.0f; 
}

// ================================== Kernel Functions ===============================

float Fluid_Solver::kernel_poly6(const glm::vec3 &r)
{
	float r_sqr = glm::dot(r, r);

	if (std::isnan(r_sqr)) throw std::runtime_error("nan");

	if (r_sqr > kernel_radius_sqr) return 0.f;
	
	return poly6_s * std::powf((kernel_radius_sqr - r_sqr), 3.f);
}

glm::vec2 Fluid_Solver::kernel_poly6_gradient(const glm::vec3 &r)
{
	glm::vec2 r_n2 = glm::normalize(glm::vec2(r.x, r.y));
	float r_sqr = glm::dot(r, r);

	// Outside Smoothing Radius or zero vector ? Ret 0.
	if (r_sqr > kernel_radius_sqr || r_sqr == 0.f) return glm::vec2(0.f);

	glm::vec2 val = poly6_grad_s * std::powf((kernel_radius - r_sqr), 2.f) * r_n2;

	if (std::isnan(glm::dot(val, val))) throw std::runtime_error("nan");

	return val;
}

float Fluid_Solver::kernel_spiky(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	// Outside Smoothing Radius ? Ret 0.
	if (r_l > kernel_radius) return 0.f; 

	return spiky_s * std::powf((kernel_radius - r_l), 3.f);
}

glm::vec2 Fluid_Solver::kernel_spiky_gradient(const glm::vec3 &r)
{
	float r_l = glm::length(r);

	// Outside Smoothing Radius or zero vector ? Ret 0.
	//if (r_l > kernel_radius || r_l == 0.f) return glm::vec2(0.f);

	if (r_l > kernel_radius) return glm::vec2(0.f);

	// Use orgn vector if zero legnth (oppose to ret 0.f)
	glm::vec2 r_n2 = r_l == 0.f ? r : glm::normalize(glm::vec2(r.x, r.y));

	glm::vec2 val = spiky_grad_s * std::powf((kernel_radius - r_l), 3.f) * r_n2; 

	if (std::isnan(glm::dot(val, val))) throw std::runtime_error("nan");

	return val; 
}

float Fluid_Solver::kernel_visc_laplacian(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	// Outside Smoothing Radius ? Ret 0.
	if (r_l > kernel_radius) return 0.f; 
	return visc_lapl_s * (1.f - (r_l / kernel_radius));
}