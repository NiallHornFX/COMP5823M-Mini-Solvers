#ifndef BONE_H
#define BONE_H

// Std Headers
#include <vector>

// Project Headers
#include "mesh.h"

// Mesh dervied class for rendering bones ethier as mesh or lines. 

class Bone : public Mesh
{
public:
	Bone(glm::vec3 Start, glm::vec3 End, std::size_t ID); 
	~Bone() = default; 

	virtual void render()   override;
public:
	std::size_t bone_id; 

	Primitive *line; 
	glm::vec3 start, end; 
};

#endif