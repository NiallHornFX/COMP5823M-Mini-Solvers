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
	particles.clear();
}

// Info : Emit Fluid in square at P, defined by Dim with spacing h.
void Fluid_Object::emit_square(const glm::vec2 &P, const glm::vec2 &Dim, float h)
{
	std::size_t n_x = std::size_t(Dim.x / h); 
	std::size_t n_y = std::size_t(Dim.y / h);

	for (std::size_t i = 0; i < n_x; ++i)
	{
		for (std::size_t j = 0; j < n_y; ++j)
		{
			float x = (float(i) / float(n_x)) * Dim.x;
			float y = (float(j) / float(n_y)) * Dim.y;
			
			particles.emplace_back(glm::vec3(x, y, 0.f), (i*n_x+j));
		}
	}
}

void Fluid_Object::render_setup()
{
	// ==== Point Render Setup ====
	ren_points = new Primitive("Render Fluid Points");
	ren_points->set_shader("../../shaders/fluid_points.vert", "../../shaders/fluid_points.frag");
	ren_points->mode = Render_Mode::RENDER_POINTS;

	ren_points->scale(glm::vec3(0.1f)); // Scale into CCS for Rendering. 
}


void Fluid_Object::render()
{
	// ==== Point Render ====
	// (Re)-Set Mesh Data, not ideal as new GPU resources per frame...
	// Store Particle Velocites in Normal attrib.

	if (!ren_points->flags.data_set) // Do Inital Vert Data Alloc
	{
		std::vector<vert> data(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			data[p].pos = particles[p].P;
			data[p].normal = particles[p].V;
			data[p].col = glm::vec3(0.1f, 0.1f, 0.75f);
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