// Implements
#include "skeleton.h"

Skeleton::Skeleton(const glm::mat4 &TrsRoot)
	: root_transform(TrsRoot)
{
	bone_count = 0;
	render_mesh = false; 
}

Skeleton::Skeleton()
	: root_transform(glm::mat4(1))
{
	bone_count = 0;
	render_mesh = false;
}


// Add Bone, using Joint Offsets and Transformations

// Trasnformation Precomputed
void Skeleton::add_bone(const glm::vec3 &start, const glm::vec3 &end, const glm::mat4 &trs)
{
	bones.emplace_back(start, end, trs, bone_count + 1);
	bone_count++;
}

// With joint indices
void Skeleton::add_bone(const glm::vec3 &start, const glm::vec3 &end, const glm::mat4 &trs, std::size_t joint_a, std::size_t joint_b)
{
	Bone b(start, end, trs, bone_count + 1);
	b.set_jointIDs(joint_a, joint_b);
	bones.push_back(b);
	bone_count++;
}

// Transformation done within body (need to pass parent matrix + joint angles.
void Skeleton::add_bone(const glm::vec3 &start, const glm::vec3 &end, const glm::mat4 &parent, float rot_z, float rot_y, float rot_x)
{
	// Calculte Center

	// Do rotation in LS 

	// Re apply trans

	// Multiply with Parent

	//bones.emplace_back(start, end, trs, bone_count + 1);
	//bone_count++;
}


void Skeleton::render(const glm::mat4x4 &view, const glm::mat4x4 &persp)
{
	// Render bones as mesh
	if (render_mesh)
	{
		for (Bone &b : bones)
		{
			b.set_cameraTransform(view, persp);
			b.render(false);
		}
		return; 
	}
	// Render bones as lines 
	for (Bone &b : bones)
	{
		b.set_cameraTransform(view, persp);
		b.render(true);
	}
}

void Skeleton::reset()
{
	bones.clear();
	bone_count = 0;
	root_transform = glm::mat4(1);
}