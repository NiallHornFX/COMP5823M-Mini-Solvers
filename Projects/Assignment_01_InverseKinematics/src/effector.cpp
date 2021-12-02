// Implements
#include "effector.h"

// Std Headers
#include <string>

// Project Headers
#include "bvhdata.h"

Effector::Effector(const glm::vec3 &Pos, std::size_t Idx)
	: pos(Pos)
{
	// Create Effector Mesh Prim
	std::string name = "Effector_Mesh_" + std::to_string(idx);
	mesh = new Mesh(name.c_str(), "../../assets/mesh/sphere.obj");
	mesh->load_obj(false);
	mesh->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	mesh->set_colour(glm::vec3(1.f, 0.f, 0.f));
	mesh->translate(pos);
}

Effector::~Effector()
{
	if (mesh) delete mesh; 
}

// Re-Set Effector position 
void Effector::set_pos(const glm::vec3 &updt_Pos)
{
	// Reset Mesh Translation
	mesh->translate(-pos);
	mesh->translate(updt_Pos);

	pos = updt_Pos;
}

// Translate Effector from current location
void Effector::translate(const glm::vec3 &transl)
{
	pos += transl;
	// Translate Mesh
	mesh->translate(transl);
}