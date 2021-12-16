// COMP5823M - A2 : Niall Horn - cloth_mesh.cpp
// Implements
#include "cloth_mesh.h"

// Std Headers
#include <cmath>
#include <cassert>

// ================================== Cloth_Mesh Class Implementation ===============================

// Info : Construction of Cloth_Mesh, Pass Refereces to Cloth_State instances : particle and index data arrays. 
// Initalize Base Primtiive, Calc Inital Attributes of Particle-Verts and indices.
Cloth_Mesh::Cloth_Mesh(const std::vector<Particle> &array_particles, const std::vector<glm::ivec3> &array_triInds, const ParticleTriList &array_ptTris)
	: Primitive("Cloth_Mesh"), particles(array_particles), tri_indices(array_triInds), particle_tris(array_ptTris)
{
	// ========== Primitive Setup Cals ==========
	set_shader("../../shaders/cloth.vert", "../../shaders/cloth.frag");
	mode = Render_Mode::RENDER_MESH;
	
	// ========== Calc Attributes from Particles for Vert Data ==========
	// Calculate Normals Per Particle-Vert
	std::vector<glm::vec3> normals = calc_normals();
	// Get Inital UVs (assumes mesh is 2D Grid) 
	std::vector<glm::vec2> uv = calc_uvs();

	// ========== Serailize Particle Data to Vert Data array (Inital Data) ==========
	// Serailize Particle Attributes into float array in Attribute Order (P.xyz|N.xyz|C.rgb|UV.uv)
	std::vector<float> vert_data; 
	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		// Position (x,y,z) 3f 12 bytes
		const Particle &curPt = particles[p];
		vert_data.push_back(curPt.P.x), vert_data.push_back(curPt.P.y), vert_data.push_back(curPt.P.z);
		// Normal (x,y,z) 3f 12 bytes
		const glm::vec3 &normal = normals[p];
		vert_data.push_back(normal.x), vert_data.push_back(normal.y), vert_data.push_back(normal.z);
		// Colour (r,g,b) 3f 12 bytes
		const glm::vec3 col(1.f, 0.f, 0.f);
		vert_data.push_back(col.r), vert_data.push_back(col.g), vert_data.push_back(col.b);
		// UV (u,v) 2f 8 bytes
		const glm::vec2 &t_uv = uv[p];
		vert_data.push_back(t_uv.x), vert_data.push_back(t_uv.y);
	}
	assert(vert_data.size() == (particles.size() * 11)); // Make sure 11 floats per particle.
	// Pass VertData to Primitive::set_data_mesh
	set_data_mesh(vert_data.data(), particles.size());

	// ========== Serailize Tri Indices to Index Array ==========
	// only done once here, cloth topo thus indices don't change.
	for (const glm::ivec3 &tri : tri_indices)
	{
		indices.push_back(tri[0]), indices.push_back(tri[1]), indices.push_back(tri[2]);
	}

	// ========== GL Buffer Setup / Alloc ==========
	// Buffer Setup (Primitive: VAO,VBO. Cloth_Mesh: EBO)
	create_buffers();
}

// ==============================================================================================
//                              Primitive Virtual Overloads
// ==============================================================================================
// Info : Override Primitive::create_buffers() to setup Element Buffer, while still calling Primtiive::CreateBuffers for VAO,VBO.
void Cloth_Mesh::create_buffers()
{
	Primitive::create_buffers(); // VAO,VBO Setup

	// EBO Setup  
	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, (sizeof(std::size_t) * indices.size()), indices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

// Info : Override Primitive::render() to draw cloth mesh using indexed drawing. 
void Cloth_Mesh::render()
{
	// Check for state to render
	if (!check_state())
	{
		std::cerr << "ERROR::Cloth_Mesh::" << name << "::Render called, with incorrectly set state." << std::endl;
		std::terminate();
	}

	// Bind Primitive State
	shader.use();
	// Update Modfied Uniforms
	shader.setMat4("model", model);

	// Render in set mode
	glBindVertexArray(VAO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	switch (mode)
	{
		case (RENDER_POINTS): // Draw Particles/Verts as Points
		{
			glDrawArrays(GL_POINTS, 0, vert_count);
			break;
		}
		case (RENDER_LINES): // Indexed Cloth Wireframe
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawElements(GL_LINES, 0, indices.size(), 0);
			break;
		}
		case (RENDER_MESH): // Indexed Cloth Mesh 
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glDrawElements(GL_TRIANGLES, 0, indices.size(), 0);
			break;
		}
	}

	// Clear State
	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

// ==============================================================================================
//                              Particle - Vertex Update Functions 
// ==============================================================================================
// Info :  Update per tick particle data via cloth_state particle array reference. 
void Cloth_Mesh::update_fromParticles()
{

}

// ==============================================================================================
//                              Attribute Setup Functions
// ==============================================================================================
// Info : Calc per Particle->Vertex Normals, Using per particle neighbours from tri indices. 
std::vector<glm::vec3> Cloth_Mesh::calc_normals()
{
	std::vector<glm::vec3> pt_normal(particles.size());

	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		const Particle &curPt = particles[p];

		// Get Particle neighbours via indices of first tri of its particle_tri list, who are not itsself. 
		std::vector<Particle> neighbours; neighbours.reserve(2);
		glm::ivec3 *first_tri = particle_tris[p][0];
		for (std::size_t i = 0; i < 3; ++i) if ((*first_tri)[i] != p) neighbours.push_back(particles[(*first_tri)[i]]);

		// From these form basis for normal. 
		glm::vec3 tang   = neighbours[0].P - curPt.P;
		glm::vec3 bitang = neighbours[1].P - curPt.P;
		glm::vec3 normal = glm::cross(glm::normalize(tang), glm::normalize(bitang));

		// Set Normal
		pt_normal[p] = normal; 

		// Debug
		std::cout << "particle_" << p << " ID = " << curPt.id << " Normal = " << normal.x << "," << normal.y << "," << normal.z << "\n";
	}
	return pt_normal;
}

// Assumes mesh is a uniform 2D Grid, uses 2D indexing to calculate UVs as such. 
std::vector<glm::vec2> Cloth_Mesh::calc_uvs()
{
	// Defines single dimension size of grid. 
	std::size_t m = std::sqrt(particles.size());
	float m_r = 1.f / float(m);

	auto idx_1dto2D = [](std::size_t i, std::size_t m) -> glm::ivec2
	{
		return glm::ivec2((i / m), (i % m));
	};

	// Approximate UV Coords over 2D grid.  
	std::vector<glm::vec2> pt_uv(particles.size());
	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		const Particle &curPt = particles[p];
		glm::ivec2 ij = idx_1dto2D(p, m);

		float u = float(ij[0]) * m_r; 
		float v = float(ij[1]) * m_r; 
		pt_uv[p] = std::move(glm::vec2(u, v));

		std::cout << "particle_" << p << " UV = " << u << "," << v << "\n";
	}
	return pt_uv;
}




