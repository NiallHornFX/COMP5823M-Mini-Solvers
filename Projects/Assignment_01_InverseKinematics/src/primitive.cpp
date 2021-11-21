// Implements 
#include "primitive.h"

// Std Headers
#include <iostream>

// Ext Headers 
// GLEW
#include "ext/GLEW/glew.h" 
// GLFW
#include "ext/GLFW/glfw3.h" 
// GLM
#include "ext/glm/glm.hpp"

Primitive::Primitive(const char *Name)
	: name(Name), vert_count(0), mode(Render_Mode::RENDER_POINTS)
{
	flags.buffers_set  = false;
	flags.camTrs_set   = false;
	flags.data_set     = false; 
	flags.shader_set   = false;

	// Init World to ident
	model = glm::mat4(1);
}

Primitive::~Primitive()
{
	if (VAO) glDeleteVertexArrays(1, &VAO);
	if (VBO) glDeleteBuffers(1, &VBO);
}

void Primitive::render()
{
	// Check for state to render
	bool set = (flags.buffers_set & flags.camTrs_set & flags.data_set & flags.shader_set);
	if (!set)
	{
		std::cerr << "ERROR::Primitive::" << name << " Render called, with incorrectly set state." << std::endl;
		std::terminate();
	}

	// Bind Primitive State
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBindVertexArray(VAO);
	shader.use();

	switch (mode)
	{
		case (RENDER_POINTS) :
		{
			glPointSize(1.f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
			glDrawArrays(GL_POINTS, 0, vert_count);
			break;
		}
		case (RENDER_LINES) :
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawArrays(GL_LINES, 0, vert_count);
			break;
		}
		case (RENDER_MESH) :
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glDrawArrays(GL_TRIANGLES, 0, vert_count);
		}
	}
	
	// Clear State
	glUseProgram(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	
}

// ======= Info : Mesh Data  =======
// Mesh Data Assumptions 11 * 4 byte floats. 
// Stride :    0         3         6       9
// Data   : (x,y,z) | (x,y,z) | (r,g,b) | (u,v)
// ================================

void Primitive::set_data_mesh(const std::vector<vert> &data)
{
	// Copy Mesh Data
	vert_data.clear();
	vert_count = data.size();

	// Serailize into float array
	for (std::size_t v = 0; v < vert_count; ++v)
	{
		const vert &vert = data[v];
		// Position 
		vert_data.push_back(vert.pos.x), vert_data.push_back(vert.pos.y), vert_data.push_back(vert.pos.z);
		// Normal
		vert_data.push_back(vert.normal.x), vert_data.push_back(vert.normal.y), vert_data.push_back(vert.normal.z);
		// Colour
		vert_data.push_back(vert.col.r), vert_data.push_back(vert.col.g), vert_data.push_back(vert.col.b);
		// UV
		vert_data.push_back(vert.uv.x),  vert_data.push_back(vert.uv.y);
	}

	flags.data_set = true;

	// Setup Buffers
	create_buffers();
}
void Primitive::set_data_mesh(const float *data, std::size_t vert_n)
{
	flags.data_set = false; 
	// Copy Mesh Data
	vert_data.clear();
	vert_data.resize(vert_n * 11);
	vert_count = vert_n;
	std::memcpy(vert_data.data(), data, (vert_n * 11 * sizeof(float)));
	flags.data_set = true; 

	// Setup Buffers
	create_buffers();
}


void Primitive::create_buffers()
{
	// Gen VAO and VBO
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	// Fill with mesh data (Assumes Mesh is in correct layout within mesh_data float array)
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, (vert_count * 11 * sizeof(float)), reinterpret_cast<void*>(vert_data.data()), GL_STATIC_DRAW);

	// Vertex Attribute 
	// Position (0)
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), reinterpret_cast<void*>(0));
	glEnableVertexAttribArray(0);
	// Normals (1)
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), reinterpret_cast<void*>(sizeof(float) * 3));
	glEnableVertexAttribArray(1);
	// Colours (2)
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), reinterpret_cast<void*>(sizeof(float) * 6));
	glEnableVertexAttribArray(2);
	// UVs (3)
	glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 11 * sizeof(float), reinterpret_cast<void*>(sizeof(float) * 9));
	glEnableVertexAttribArray(3);

	// Clear bound state
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	flags.buffers_set = true;
}

// Only Sets Position of vertices (allows for updating positions per tick)
void Primitive::update_data_position(const std::vector<glm::vec3> &posData)
{
	for (std::size_t v = 0; v < vert_count; ++v)
	{
		std::size_t i = v * 11; // Vert Index, Position. 
		vert_data[i++] = posData[v].x;
		vert_data[i++] = posData[v].y;
		vert_data[i++] = posData[v].z;
	}
}

void Primitive::set_shader(const char *vert_path, const char *frag_path)
{
	std::string shader_name = name + "_Shader";
	shader = Shader(shader_name.c_str(), vert_path, frag_path);

	// Set Model Matrix
	shader.setMat4("model", model);

	if (shader.valid_state) flags.shader_set = true; 
}

void Primitive::set_cameraTransform(const glm::mat4x4 &view, const glm::mat4x4 &persp)
{
	if (!flags.shader_set) return; 
	// Set Shader Camera Matrices (assume unfiorms exist)
	shader.setMat4("view", view);
	shader.setMat4("proj", persp);
	flags.camTrs_set = true;
}