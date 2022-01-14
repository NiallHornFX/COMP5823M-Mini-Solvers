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

#define INTEGRATE_LEAPFROG  1
#define DEBUG_TEST_ADJACENT 0


// ================================== Fluid_Solver Class Implementation ===============================

Fluid_Solver::Fluid_Solver(float Sim_Dt, float KernelRad, Fluid_Object *Data)
	: fluidData(Data), kernel_radius(KernelRad), dt(Sim_Dt), at(0.f)
{
	// ===== Init Solver =====
	rest_density = 100.f; 
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
	stiffness_coeff = 500.f; 

	// Default Kernels 
	pressure_kernel  = kernel::SPIKY; // (Should be default poly6 for assignment demo, then switch to spiky)
	surftens_kernel  = kernel::POLY6;
	
	// ===== Setup Tank Collider Planes =====
	Fluid_Collider_Plane *left  = new Fluid_Collider_Plane("Tank_Left",  glm::vec3(2.5f, 0.0f, 0.f), glm::vec3(1.f, 0.f, 0.f),  glm::vec2(0.0f, 5.0f));
	Fluid_Collider_Plane *right = new Fluid_Collider_Plane("Tank_Right", glm::vec3(7.5f, 0.f, 0.f),  glm::vec3(-1.f, 0.f, 0.f), glm::vec2(0.0f, 5.0f));
	Fluid_Collider_Plane *floor = new Fluid_Collider_Plane("Tank_Floor", glm::vec3(2.5f, 0.0f, 0.f), glm::vec3(0.f, 1.f, 0.f),  glm::vec2(5.0f, 0.f));
	colliders.push_back(std::move(left)); colliders.push_back(std::move(right)); colliders.push_back(std::move(floor));

	// ===== Pre Compute Kernel + Derivative Scalar Coeffecints =====
	// Poly6
	poly6_s      =  315.f  / (64.f * M_PI  * std::powf(kernel_radius, 9.f));
	poly6_grad_s = -945.f  / (32.f * M_PI  * std::powf(kernel_radius, 9.f));
	// Spiky
	spiky_s      = 15.f  / (M_PI * std::powf(kernel_radius, 6.f));
	spiky_grad_s = -45.f / (M_PI * std::powf(kernel_radius, 6.f));
	// Viscosity
	visc_lapl_s  = 45.f  / (M_PI * std::powf(kernel_radius, 6.f));
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

	// Reset Fluid State
	fluidData->reset_fluid();
}

// Info : Single simulation step of SPH Solver
void Fluid_Solver::step()
{
	// Build Acceleration Grid and cache particle neighbours
	get_neighbours();
	// Compute Density and from this pressure using some smoothing kernel.
	compute_dens_pres(&Fluid_Solver::kernel_poly6);
	// Check for collisions and project offending particles. 
	eval_colliders();
	// Calculcate resulting forces and integrate
	integrate();
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
	// Chosen Kernel Functions based on GUI 
	kernel_func      attr_kernel = &Fluid_Solver::kernel_poly6;          // Attribs always sampled using Poly6. 
	kernel_lapl_func visc_lapl_k = &Fluid_Solver::kernel_visc_laplacian; // Viscosity always uses Visc Kernel Laplacian. 
	// Pressure and Surface Tension may use ethier Poly6 or Spiky Gradient kernel functions. 
	kernel_grad_func pres_grad = pressure_kernel == kernel::POLY6 ? &Fluid_Solver::kernel_poly6_gradient : &Fluid_Solver::kernel_spiky_gradient;
	kernel_grad_func surf_grad = surftens_kernel == kernel::POLY6 ? &Fluid_Solver::kernel_poly6_gradient : &Fluid_Solver::kernel_spiky_gradient;

	// Ext Forces 
	glm::vec3 g(0.f, gravity, 0.f); 

#if INTEGRATE_LEAPFROG == 0
	// Semi Implicit Euler Integration (testing)
	for (Particle &p : fluidData->particles)
	{
		float test = glm::dot(p.P, p.P);
		if (std::isnan(glm::dot(test, test))) throw std::runtime_error("nan");
		// Eval RHS forces 0 
		glm::vec3 a_0 = g + eval_forces(p, kernel, grad);
		//glm::vec3 a_0 = g;
		// Integrate Accel and Vel
		p.V += a_0 * dt; 
		p.P += p.V * dt; 
	} 
#else	
	// Leapfrog integration
	for (Particle &pt : fluidData->particles)
	{
		// Pre-calc
		float r_dens = 1.f / pt.density; 
		glm::vec3 air_res = -air_resist * pt.V; 

		// Debug 
		float test = glm::dot(pt.P, pt.P);
		if (std::isnan(glm::dot(test, test))) throw std::runtime_error("nan");
		
		// Eval RHS forces 0 
		glm::vec3 a_0 = (eval_forces(pt, pres_grad, surf_grad, visc_lapl_k) * r_dens) + g + air_res;
		// Integrate P 
		pt.P += (pt.V * dt) + (0.5f * a_0 * (dt*dt));

		// Eval RHS forces 1
		glm::vec3 a_1 = (eval_forces(pt, pres_grad, surf_grad, visc_lapl_k) * r_dens) + g + air_res;
		// Integrate V
		pt.V += 0.5f * (a_0 + a_1) * dt; 

		// Store Min/Max Attribs, post integration
		float f_s = glm::dot(a_1, a_1);
		if (f_s < fluidData->min_force) fluidData->min_force = f_s; 
		if (f_s > fluidData->max_force) fluidData->max_force = f_s; 
	} 
#endif
}

void Fluid_Solver::get_neighbours()
{
	// =========== Spatial Accel : Uniform Grid ===========
	// Delete old grid
	got_neighbours = false; 
	if (accel_grid) delete accel_grid;
	// Allocate New Grid
	accel_grid = new Spatial_Grid(fluidData, kernel_radius, 10.f);
	accel_grid->gather_particles();
	got_neighbours = true; 

	// Cache Particle Neighbours into array per particle. 
	fluidData->particle_neighbours.clear();
	fluidData->particle_neighbours.resize(fluidData->particles.size());
	for (std::size_t p = 0; p < fluidData->particles.size(); ++p)
	{
		fluidData->particle_neighbours[p] = accel_grid->get_adjcell_particles(fluidData->particles[p]);
	} 

#if DEBUG_TEST_ADJACENT == 1
	// Test : Adjacent Hash of single particle, viz adj cells. 
	Particle &testPt = fluidData->particles[100];
	auto pts = accel_grid->get_adjcell_particles(testPt);
	// Rm all pts cell indices first
	for (Particle &pt : fluidData->particles) pt.cell_idx = 0;
	// Fill Adj particle list cell_idx with same idx for viz debgging.
	for (Particle *pt : pts)
	{
		pt->cell_idx = 1;
	}
	testPt.cell_idx = 2; 
#endif
}

// ================================== Eval Attrib Functions ===============================
// Loop over particle neighbours (within hash cell) calc density using passed smoothing kernel func ptr. 
// Can be safely multithreaded 

void Fluid_Solver::compute_dens_pres(kernel_func w)
{	
	// Check Hash Grid has been evaulated
	if (!got_neighbours) { std::cerr << "ERROR::Frame::" << frame << " Fluid_Solver::Particles Neighbours not retrived from grid" << std::endl; return; }

	fluidData->min_dens = 1e06f, fluidData->max_dens = 0.f, fluidData->min_pres = 1e06f, fluidData->max_pres = 0.f;
	for (std::size_t p_i = 0; p_i < fluidData->particles.size(); ++p_i)
	{
		Particle &Pt_i = fluidData->particles[p_i];
		float dens_tmp = 0.0f; // Start w small min density. 

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
		if (Pt_i.density  < fluidData->min_dens) fluidData->min_dens = Pt_i.density;
		if (Pt_i.pressure < fluidData->min_pres) fluidData->min_pres = Pt_i.pressure;
		if (Pt_i.density  > fluidData->max_dens) fluidData->max_dens = Pt_i.density;
		if (Pt_i.pressure > fluidData->max_pres) fluidData->max_pres = Pt_i.pressure;
	}
	if (frame == 0) rest_density = fluidData->max_dens * 1.01f; // Use as inital rest_dens

	//for (Particle &pt : fluidData->particles) if (pt.density == 0.f) throw std::runtime_error("0 Dens");
}

glm::vec3 Fluid_Solver::eval_forces(Particle &Pt_i, kernel_grad_func w_pres_grad, kernel_grad_func w_surf_grad, kernel_lapl_func w_visc_lapl)
{
	// Check Hash Grid has been evaulated
	if (!got_neighbours) { std::cerr << "ERROR::Frame::" << frame << " Fluid_Solver::Particles Neighbours not retrived from grid" << std::endl;  return glm::vec3(0.f); }

	// Reset Particle forces 
	Pt_i.F.x = 0.f, Pt_i.F.y = 0.f, Pt_i.F.z = 0.f;

	glm::vec3 force_pressure    (0.f); 
	glm::vec3 force_viscosity   (0.f);
	glm::vec3 force_surftension (0.f);

	// Get Neighbour list
	std::vector<Particle*> &neighbours = fluidData->particle_neighbours[Pt_i.id];

	// Use sep loops for now...

	// =============== Compute Pressure Gradient --> Pressure Force ===============
	glm::vec2 pressure_grad(0.f);
	for (std::size_t p_j = 0; p_j < neighbours.size(); ++p_j)
	{
		const Particle &Pt_j = *(neighbours[p_j]);
		if (Pt_j.id == Pt_i.id) continue;  // Skip Self 

		// Compute Symmetric Pressure gradient for particle pair
		pressure_grad += Pt_j.mass * ((Pt_i.pressure + Pt_j.pressure) / (2.f * Pt_j.density)) * (this->*w_pres_grad)(Pt_i.P - Pt_j.P);
	}
	force_pressure = -glm::vec3(pressure_grad.x, pressure_grad.y, 0.f);

	/*
	// =============== Compute Viscosity Laplacian --> Viscosity Force ===============
	glm::vec3 visc_vec(0.f);
	for (std::size_t p_j = 0; p_j < neighbours.size(); ++p_j)
	{
		const Particle &Pt_j = *(neighbours[p_j]);
		if (Pt_j.id == Pt_i.id) continue;  // Skip Self 

		// Compute Symmetric Viscosity for particle pair
		visc_vec += Pt_j.mass * ((Pt_j.V - Pt_i.V) / Pt_j.density) * (this->*w_visc_lapl)(Pt_i.P - Pt_j.P);
	}
	force_viscosity = k_viscosity * visc_vec;

	// =============== Compute Surface Gradient --> Surface Tension Force ===============
	// tbd ...

	// Accumulate forces
	glm::vec3 acc_force = force_pressure + force_viscosity;
	// ret instead of write to Pt.F
	*/
	return force_pressure;
}
// ================================== Util Functions ===============================
// Info : Calc an estimate of the rest density from the maxium density value * some offset. 
//        This can be called external of solve step, to get inital estimate of rest_density. 
void Fluid_Solver::calc_restdens()
{
	get_neighbours();
	compute_dens_pres(&Fluid_Solver::kernel_poly6);
	rest_density = fluidData->max_dens * 1.01f; 
}

// ================================== Kernel Functions ===============================
// Info : Smoothing Kernel functions and their derivatives, using pre-caluclated scalar coefficents, based on
// kernel radius h.

// ========== Smooting Kernel - Poly6 ==========
float Fluid_Solver::kernel_poly6(const glm::vec3 &r)
{
	float r_sqr = glm::dot(r, r);
	if (std::isnan(r_sqr)) throw std::runtime_error("nan");

	// Outside smoothing radius ? Ret 0f
	if (r_sqr > kernel_radius_sqr) return 0.f;
	return poly6_s * std::powf((kernel_radius_sqr - r_sqr), 3.f);
}

glm::vec2 Fluid_Solver::kernel_poly6_gradient(const glm::vec3 &r)
{
	glm::vec2 r_n2 = glm::normalize(glm::vec2(r.x, r.y));
	float r_sqr = glm::dot(r, r);

	// Outside Smoothing Radius or zero vector ? Ret 0 vec
	if (r_sqr == 0.f || r_sqr > kernel_radius_sqr ) return glm::vec2(0.f);
	glm::vec2 val = poly6_grad_s * std::powf((kernel_radius - r_sqr), 2.f) * r_n2;
	if (std::isnan(glm::dot(val, val))) throw std::runtime_error("nan");
	return val;
}

// ========== Smooting Kernel - Spiky ==========
float Fluid_Solver::kernel_spiky(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	// Outside Smoothing Radius ? Ret 0f
	if (r_l > kernel_radius) return 0.f; 
	return spiky_s * std::powf((kernel_radius - r_l), 3.f);
}

glm::vec2 Fluid_Solver::kernel_spiky_gradient(const glm::vec3 &r)
{
	float r_l = glm::length(r);

	// Outside Smoothing Radius or zero vector ? Ret 0.
	if (r_l == 0.f || r_l > kernel_radius) return glm::vec2(0.f);
	glm::vec2 r_n2 = glm::normalize(glm::vec2(r.x, r.y));
	glm::vec2 val = spiky_grad_s * std::powf((kernel_radius - r_l), 3.f) * r_n2; 

	if (std::isnan(glm::dot(val, val))) throw std::runtime_error("nan");
	return val; 
}

// ========== Smooting Kernel - Viscosity ==========
float Fluid_Solver::kernel_visc_laplacian(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	// Outside Smoothing Radius ? Ret 0.
	if (r_l > kernel_radius) return 0.f; 
	//return visc_lapl_s * (1.f - (r_l / kernel_radius));
	return visc_lapl_s * (kernel_radius - r_l);
}