#ifndef SHADER_H
#define SHADER_H

// Ext Headers
#include "ext/glm/glm.hpp"

// Std Headers
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

// Info : Basic Shader class, that defines a shader program containg a Vertex and Fragment stage. 

class Shader
{
public:
	Shader(const char *name, const char *vertparth, const char *fragpath);
	~Shader();

	void use();

	// Set Shader Uniforms
	void setBool(const std::string &name, bool value) const;
	void setInt(const std::string &name, int value) const;
	void setFloat(const std::string &name, float value) const;
	void setVec(const std::string &name, glm::vec3 value) const;

	// Debug
	void check_compile(unsigned int &shader, std::string type);
	void check_link();
	void debug_vert();
	void debug_frag();

public:
	std::string name;
	unsigned int ID; // Shader Program id.

private:
	std::string vert_shader_code;
	std::string frag_shader_code; 
};

#endif 

