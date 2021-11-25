#ifndef BONE_H
#define BONE_H

// Std Headers
#include <vector>
#include <tuple>

// Project Headers
#include "mesh.h"


// Class for rendering bones ethier as mesh or lines (primitives). 

class Bone
{
public:
	Bone(glm::vec3 Start, glm::vec3 End, glm::mat4 Trs, std::size_t ID); 
	~Bone() = default; 

	 void render(bool Render_Line = false);
	 void set_cameraTransform(const glm::mat4x4 &view, const glm::mat4x4 &persp);
	 void set_jointIDs(std::size_t joint_a, std::size_t joint_b);

public:
	std::size_t bone_id; 
	glm::vec3 start, end;
	glm::mat4 transform; 
	std::pair<std::size_t, std::size_t> joint_ids;

private:

	Mesh *mesh;
	Primitive *line; 

};

#endif