// COMP5823M - A3 : Niall Horn - fluid_object.cpp
// Implements
#include "fluid_object.h"

Fluid_Object::Fluid_Object()
{
	particles.reserve(1000);
	render_setup();
}

Fluid_Object::~Fluid_Object()
{
	// Delete Primitives
	if (ren_points) delete ren_points; 
	if (fsQuad) delete fsQuad;
}

void Fluid_Object::reset_fluid()
{
	for (Particle &pt : particles)
	{
		pt.P = pt.rest;
		pt = Particle(pt.P, pt.id);
	}
}

// Info : Emit Fluid in square at P, defined by Dim with spacing h.
void Fluid_Object::emit_square(const glm::vec2 &P, const glm::vec2 &Dim, float h)
{
	std::size_t n_x = std::size_t(Dim.x / h); 
	std::size_t n_y = std::size_t(Dim.y / h);
	float h_dim_x = Dim.x * 0.5f, h_dim_y = Dim.y * 0.5f; 

	for (std::size_t i = 0; i < n_x; ++i)
	{
		for (std::size_t j = 0; j < n_y; ++j)
		{
			float xx = (float(i) / float(n_x-1)) * Dim.x;
			float yy = (float(j) / float(n_y-1)) * Dim.y;
			//xx -= h_dim_x, yy -= h_dim_y; // Center
			xx += P.x, yy += P.y; 
			
			particles.emplace_back(glm::vec3(xx,yy,0.f), (i*n_x+j));
		}
	}
}

void Fluid_Object::render_setup()
{
	// ==== Point Render Setup ====
	ren_points = new Primitive("Render Fluid Points");
	ren_points->set_shader("../../shaders/fluid_points.vert", "../../shaders/fluid_points.frag");
	ren_points->mode = Render_Mode::RENDER_POINTS;
}


void Fluid_Object::render(const glm::mat4 &ortho)
{
	// ==== Point Render ====
	// (Re)-Set Mesh Data, not ideal as new GPU resources per frame...
	// Store Particle Velocites in Normal attrib.
	ren_points->shader.setMat4("proj", ortho);
	if (!ren_points->flags.data_set) // Do Inital Vert Data Alloc
	{
		std::vector<vert> data(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			data[p].pos = particles[p].P;
			data[p].normal = particles[p].V;
			data[p].col = glm::vec3(0.1f, 0.1f, 1.f);
		}
		ren_points->set_data_mesh(data);
	}
	else // Update Pos and Normals Only
	{
		std::vector<glm::vec3> pos, norm;
		pos.resize(particles.size()), norm.resize(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			pos[p] = particles[p].P;
			norm[p] = particles[p].V;
		}
		ren_points->update_data_position_normals(pos, norm);
	}

	// Render
	ren_points->render();
}