#ifndef PRIMITIVE_H
#define _PRIMITIVE_H

// Std Headers
#include <string>
#include <vector>

// Ext Headers 
#include "ext/glm/glm.hpp"

// Project Headers
#include "shader.h"

struct vert; 

enum Render_Mode
{
	RENDER_MESH = 0,
	RENDER_LINES,
	RENDER_POINTS
};

// Info : Base class that defines base primitive 

class Primitive
{
public:
	Primitive(const char *name);
	~Primitive();

	virtual void setup_data(); 

	virtual void render(Render_Mode &mode);

	void set_cameraTransform(glm::mat4x4 &view, glm::mat4x4 &persp);

	void set_positionData(const std::vector<glm::vec3> &posData);

public:
	// Intrinsics
	std::string name;
	glm::mat4x4 world; 
	
	// Shader
	Shader shader; 

	// Data
	std::vector<vert> vert_data; 
	std::size_t VBO, VAO; 

	// State
	Render_Mode active_mode;
};


// Utility Structs

struct vert
{
	glm::vec3 pos;
	glm::vec3 normal;
	glm::vec2 uv;
	glm::vec3 col;
};

#endif