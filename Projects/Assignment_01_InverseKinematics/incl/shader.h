#ifndef SHADER_H
#define SHADER_H

// Ext Headers
#include <GLM/glm.hpp>

// Std Headers
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader
{
public:
	Shader(const char *vertparth, const char *fragpath);
	~Shader();

	void use();

	// Set Shader Uniforms
	// Const functions to explictly stop modfication of object. 
	void setBool(const std::string &name, bool value) const;
	void setInt(const std::string &name, int value) const;
	void setFloat(const std::string &name, float value) const;
	void setVec(const std::string &name, glm::vec3 value) const;

	//Shader Compile/Link Error Check Functions (MFs) - 
	void check_compile(unsigned int &shader, char *type);
	void check_link();
	void debug_vert();
	void debug_frag();
	unsigned int get_id();

private:
	unsigned int ID; // Shader Program id.
	std::string vert_shader_code;
	std::string frag_shader_code; 
};

#endif 

