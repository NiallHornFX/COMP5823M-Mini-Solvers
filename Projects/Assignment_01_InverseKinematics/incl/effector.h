#ifndef EFFECTOR_H
#define EFFECTOR_H

// Project Headers
#include "primitive.h"
#include "bone.h"

// Ext Headers
#include "ext/glm/glm.hpp"

// Std Headers
#include <vector>

struct Joint;

// Basic class to define effector of joints, and ability to draw it. 

class Effector
{
public:
	Effector(const glm::vec3 &Pos, std::size_t Idx);
	~Effector();

	void set_pos(const glm::vec3 &upt_Pos);
	void translate(const glm::vec3 &transl);

public:
	glm::vec3 pos; 
	std::size_t idx; 
	std::vector<Joint*> joints;

	// Viz 
	Mesh *mesh; 
};

#endif