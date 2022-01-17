// COMP5823M - A3 : Niall Horn - fluid_object.cpp
// Implements
#include "fluid_object.h"
#include "fluid_solver.h"

// Std Headers

Fluid_Object::Fluid_Object(const glm::vec2 &P, const glm::vec2 &Dim, float Spc, float Jitter)
	: pos(P), dim(Dim), spc(Spc), jitter(Jitter), grid_data("FluidData", this, 0.05f, 10.f)
{
	// Emission
	emit_square();

	// Setup
	render_setup();

	// Init 
	particle_colour = Colour_Viz::Velocity;
	min_dens = 0.f, max_dens = 0.f;
	min_pres = 0.f, max_pres = 0.f;
	min_cf   = 0.f, max_cf   = 0.f;
}

Fluid_Object::~Fluid_Object()
{
	// Delete Primitives
	if (ren_points) delete ren_points; 
	if (ren_quad)   delete ren_quad;
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

// Info : Called once (on construction of Fluid Object) to setup render paths. 
void Fluid_Object::render_setup()
{
	// Setup both render paths 

	// =========== Point Vertices Render Setup ===========
	ren_points = new Primitive("Render Fluid Points");
	ren_points->set_shader("../../shaders/fluid_points.vert", "../../shaders/fluid_points.frag");
	ren_points->mode = Render_Mode::RENDER_POINTS;

	// Allocate GPU Resources
	std::vector<vert> data(particles.size());
	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		data[p].pos = particles[p].P;
		data[p].col = glm::vec3(0.1f, 0.1f, 1.f);
	}
	ren_points->set_data_mesh(data);

	// =========== Grid Render Setup ===========
	ren_quad = new Primitive("Render Fluid Quad");
	ren_quad->set_shader("../../shaders/fluid_quad.vert", "../../shaders/fluid_quad.frag");
	ren_quad->mode = Render_Mode::RENDER_MESH;
	float quad_verts[36] =
	{
		// Tri 0
		-1.f, -1.f, 0.f, 0.f, 0.f, 0.f,
		1.f,  -1.f, 0.f, 0.f, 0.f, 0.f, 
		1.f,  1.f,  0.f, 0.f, 0.f, 0.f, 
		// Tri 1
		1.f,   1.f, 0.f, 0.f, 0.f, 0.f, 
		-1.f,  1.f, 0.f, 0.f, 0.f, 0.f, 
		-1.f, -1.f, 0.f, 0.f, 0.f, 0.f
	};
	ren_quad->set_data_mesh(quad_verts, 6);
}


void Fluid_Object::render(Render_Type mode, const glm::mat4 &ortho)
{
	if (mode == Render_Type::POINT_VERTS)
	{   // =================== Point Vertices Render ===================

		// Set Proj Mat
		ren_points->shader.setMat4("proj", ortho);

		// Particle Attribs to Vert Data update
		std::vector<glm::vec3> pos, col;
		pos.resize(particles.size()), col.resize(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			// Particle Colour : 
			switch (particle_colour)
			{
			case Colour_Viz::Velocity: // Colour Particles by Velocity - speed
			{
				float speed = fitRange(glm::length(particles[p].V), 0.f, 5.f, 0.f, 1.f);
				col[p] = lerp_vec(glm::vec3(0.02f, 0.02f, 0.9f), glm::vec3(1.f, 1.f, 1.f), speed);
				break;
			}
			case Colour_Viz::Pressure: // Colour Particles by Pressure 
			{
				col[p] = glm::vec3(fitRange(particles[p].pressure, 0.f, max_pres, 0.f, 1.f));
				break;
			}
			case Colour_Viz::Density: // Colour Particles by Density 
			{
				col[p] = glm::vec3(fitRange(particles[p].density, 0.f, max_dens, 0.f, 1.f));
				break;
			}
			case Colour_Viz::Colour: // Colour Particles by Colour Field
			{
				col[p] = glm::vec3(fitRange(particles[p].cf, min_cf, max_cf, 0.f, 1.f));
				break;
			}
			case Colour_Viz::GridCell: // Colour particles by grid cell. 
			{
				col[p] = randRange(particles[p].cell_idx, 0.f, 1.f);
				break;
			}
			}
			// Update Position
			pos[p] = particles[p].P;
		}

		// Pass updated attribute arrays to Primitve::update_data... 
		ren_points->update_data_position_col(pos, col);

		// Call Render Points as Verts
		ren_points->point_size = spc * 75.f;
		ren_points->render();
	}
	else if (mode == Render_Type::METABALL)
	{ // =================== Fragment Shader Metaballs Render ===================
	
		// Set Uniform Particle Array
		Shader &shad = ren_quad->shader;


		for (std::size_t i = 0; i < particles.size(); ++i)
		{
			const Particle &pt = particles[i];
			glm::vec2 pos_2(pt.P.x, pt.P.y);
			glm::vec2 vel_2(pt.V.x, pt.V.y);

			// Loc String
			std::string pt_str = "pts[" + std::to_string(i) + "].";
			shad.use();
			// Pos
			GLuint pos_loc = glGetUniformLocation(shad.ID, (pt_str + "pos").c_str());
			glUniform2fv(pos_loc, 1, glm::value_ptr(pos_2));
			// Vel
			GLuint vel_loc = glGetUniformLocation(shad.ID, (pt_str + "vel").c_str());
			glUniform2fv(vel_loc, 1, glm::value_ptr(vel_2));
			// Dens
			GLuint dens_loc = glGetUniformLocation(shad.ID, (pt_str + "dens").c_str());
			glUniform1f(dens_loc, pt.density); 
		}
		glUseProgram(0);
		// As Uniform array is fixed size, set current particle count uniform : 
		shad.setInt("pt_count", particles.size());

		// Pass min and max density range
		shad.setFloat("min_dens", min_dens);
		shad.setFloat("max_dens", max_dens);

		// Render
		ren_quad->render();
	}
	
}

