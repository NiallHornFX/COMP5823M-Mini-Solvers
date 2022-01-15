// COMP5823M - A3 : Niall Horn - fluid_object.cpp
// Implements
#include "fluid_object.h"
#include "fluid_solver.h"

// Std Headers

Fluid_Object::Fluid_Object(const glm::vec2 &P, const glm::vec2 &Dim, float Spc, float Jitter)
	: pos(P), dim(Dim), spc(Spc), jitter(Jitter)
{
	// Emission
	emit_square();

	// Setup
	render_setup();

	// Init 
	particle_colour = Colour_Viz::Standard;
	min_dens = 0.f, max_dens = 0.f;
	min_pres = 0.f, max_pres = 0.f;
	//min_force = 0.f, max_force = 0.f;
}

Fluid_Object::~Fluid_Object()
{
	// Delete Primitives
	if (ren_points) delete ren_points; 
	if (fsQuad) delete fsQuad;
}

// Info : Reset fluid to inital state.
void Fluid_Object::reset_fluid()
{
	for (Particle &pt : particles)
	{
		pt.P = pt.rest; float mass = pt.mass;
		pt = Particle(pt.P, mass, pt.id);
	}

	// Reset Attrib Ranges
	min_dens = 0.f, max_dens = 0.f, min_pres = 0.f, max_pres = 0.f; min_force = 0.f, max_force = 0.f;
}

// Info : Emit Fluid in square at P, defined by Dim with spacing h.
void Fluid_Object::emit_square()
{
	std::size_t n_j = std::size_t(dim.x / spc); 
	std::size_t n_i = std::size_t(dim.y / spc);
	float h_dim_x = dim.x * 0.5f, h_dim_y = dim.y * 0.5f; 

	// Compute particle mass
	float mass = 100.f / std::sqrtf(n_i * n_j);
	mass = 1.f; // Debug.

	for (std::size_t i = 0; i < n_i; ++i)
	{
		for (std::size_t j = 0; j < n_j; ++j)
		{
			float xx = (float(j) / float(n_j-1)) * dim.x;
			float yy = (float(i) / float(n_i-1)) * dim.y;
			//xx -= h_dim_x, yy -= h_dim_y; // Center
			xx += pos.x, yy += pos.y; 
			std::size_t id = i * n_j + j;
			glm::vec3 r = randRange((id+1), -jitter, jitter); r.z = 0.f; 
			particles.emplace_back(glm::vec3(xx,yy,0.f) + r, mass, id);
		}
	}

}

void Fluid_Object::render_setup()
{
	// =========== Point Vertices Render Setup ===========
	ren_points = new Primitive("Render Fluid Points");
	ren_points->set_shader("../../shaders/fluid_points.vert", "../../shaders/fluid_points.frag");
	ren_points->mode = Render_Mode::RENDER_POINTS;
}


void Fluid_Object::render(const glm::mat4 &ortho)
{
	// =========== Point Vertices Render ===========

	// Set projection matrix from viewer passed ortho
	ren_points->shader.setMat4("proj", ortho);

	if (!ren_points->flags.data_set) // Create GPU resources
	{
		std::vector<vert> data(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			data[p].pos = particles[p].P;
			data[p].normal = particles[p].V; // Store Pt vel in vertex normal attrib.
			data[p].col = glm::vec3(0.1f, 0.1f, 1.f);
		}
		ren_points->set_data_mesh(data);
	}
	else // Update Only
	{
		std::vector<glm::vec3> pos, norm, col;
		pos.resize(particles.size()), norm.resize(particles.size()), col.resize(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			// Particle Colour : 
			switch (particle_colour)
			{
				case Colour_Viz::GridCell:
				{
					// Colour particles by grid cell. 
					col[p] = randRange(particles[p].cell_idx, 0.f, 1.f);
					break;
				}
				case Colour_Viz::Pressure:
				{
					// Colour Particles by Pressure 
					col[p] = glm::vec3(fitRange(particles[p].pressure, 0.f, max_pres, 0.f, 1.f));
					break;
				}
				case Colour_Viz::Density:
				{
					// Colour Particles by Density 
					//col[p] = glm::vec3(fitRange(particles[p].density, 0.f, max_dens, 0.f, 1.f));
					col[p] = glm::vec3(fitRange(particles[p].cf, min_cf, max_cf, 0.f, 1.f)); // CF Test
					break;
				}
				case Colour_Viz::Velocity: 
				{
					break;
				}
				case Colour_Viz::Standard:
				{
					col[p] = glm::vec3(0.1f, 0.1f, 0.95f);
				}
			}
			// Pos and Normal (vel)
			pos[p]  = particles[p].P;
			norm[p] = particles[p].V;
		}
		// Pass updated attribute arrays to Primitve::update_data... 
		ren_points->update_data_position_normals_col(pos, norm, col);
	}

	// Render
	ren_points->point_size = spc * 75.f;
	ren_points->render();
}

std::pair<glm::vec2, glm::vec2> Fluid_Object::get_fluid_bounds() const
{
	glm::vec2 min(particles[0].P.x, particles[0].P.y); 
	glm::vec2 max(particles[particles.size()-1].P.x, particles[particles.size()-1].P.y);
	return std::pair<glm::vec2, glm::vec2>(min, max); 
}