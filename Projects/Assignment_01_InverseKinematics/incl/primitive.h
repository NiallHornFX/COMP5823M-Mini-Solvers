#ifndef PRIMITIVE_H
#define _PRIMITIVE_H

// Std Headers
#include <string>
#include <vector>

// Ext Headers 
// GLM
#include "ext/glm/glm.hpp"
// GLEW
#include "ext/GLEW/glew.h" 

// Project Headers
#include "shader.h"
#include "texture.h"

struct vert; 

enum Render_Mode
{
	RENDER_MESH = 0,
	RENDER_LINES,
	RENDER_POINTS
};

// Info : Base Primitive object for rendering within the viewer app. 

class Primitive
{
public:
	Primitive() {}
	Primitive(const char *Name);
	~Primitive();

	// Virtual Methods 
	virtual void render();

	// Core
	void create_buffers();

	// Setters
	void set_data_mesh(const std::vector<vert> &data);
	void set_data_mesh(const float *data, std::size_t vert_n);

	void update_data_position(const std::vector<glm::vec3> &posData);

	void set_cameraTransform(const glm::mat4x4 &view, const glm::mat4x4 &persp);

	void set_shader(const char *vert_path, const char *frag_path);

public:
	// Intrinsics
	std::string name;
	glm::mat4 model; 

	// Shader
	Shader shader; 
	glm::mat4x4 s_view, s_persp; 

	// Data
	std::vector<float> vert_data; 
	std::size_t vert_count;

	GLuint VBO, VAO; 

	// State
	Render_Mode mode;

	struct
	{
		int32_t data_set : 1;
		int32_t shader_set : 1;
		int32_t buffers_set : 1;
		int32_t camTrs_set : 1;
	} flags;
};


// ==== Struct for Vertex Data ====
struct vert
{
	glm::vec3 pos;
	glm::vec3 normal;
	glm::vec3 col;
	glm::vec2 uv;
};

#endif