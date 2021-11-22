#ifndef MESH_H
#define MESH_H


// Std Headers
#include <string>
#include <vector>

// Ext Headers 
// GLM
#include "ext/glm/glm.hpp"

// Project Headers
#include "primitive.h"

// Info : Primitive based class for rendering meshes based on loaded .obj file, along with textures. 

class Mesh : public Primitive
{
public:
	Mesh(const char *name, const char *filePath);
	~Mesh();

	virtual void render() override; 

	void load_obj();

public:
	std::string file_path; 

	struct
	{
		std::vector<glm::vec3> v_p;
		std::vector<glm::vec3> v_n;
		std::vector<glm::vec3> v_c;
		std::vector<glm::vec2> v_t; 
	} obj_data;

};


#endif