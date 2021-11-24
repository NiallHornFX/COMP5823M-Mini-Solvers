// Implements
#include "skeleton.h"

Skeleton::Skeleton(const glm::mat4 &TrsRoot)
	: root_transform(TrsRoot)
{
	bone_count = 0;
	render_mesh = false; 
	bones.reserve(2);
}

void Skeleton::add_bone(const glm::vec3 &start, const glm::vec3 &end, const glm::mat4 &trs)
{
	bones.emplace_back(start, end, trs, bone_count + 1);
	bone_count++;
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