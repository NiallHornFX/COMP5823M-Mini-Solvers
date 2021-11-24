#ifndef SKELETON_H
#define SKELETON_H

// Std Headers
#include <vector>

// Project Headers
#include "mesh.h"

// Class holds array of bones between joints to render ethier as tris or lines using computed transforms from ethier FK or IK joints. 


class Skeleton
{
public:
	Skeleton(std::size_t n_Joints);
	~Skeleton() = default; 

	void add_bone(const glm::vec3 start, const glm::vec3 end);

	void render();

public:
	std::vector<Mesh*> bones; 
	bool render_mesh;
	std::size_t count; 
};


#endif