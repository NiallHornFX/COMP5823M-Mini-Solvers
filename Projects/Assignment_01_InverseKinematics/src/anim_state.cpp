// Implements 
#include "anim_state.h"

// Basic Ctor, state is set on demmand as needed. 

Anim_State::Anim_State()
{
	anim_loop = true;
	anim_frame = 0;
	max_frame = 0;

	bvh = nullptr;
}

void Anim_State::set_bvhFile(const char *BVHPath)
{
	// Clear prev state
	if (bvh) delete bvh;
	skel.reset();

	bvh = new BVH_Data(BVHPath);
	bvh->Load();

	max_frame = bvh->num_frame;
	interval  = bvh->interval;

	fetch_bvhData();
}

void Anim_State::fetch_bvhData()
{
	// Fill out Skeleton from BVH Tree
	// Start from root. 
	Joint *root = bvh->joints[0];

	Joint *prev = nullptr; 
	Joint *cur = root; 
	glm::vec3 offs = root->offset; 
	Joint *next = root->children[0];
	Joint *next_b = root->children[0]->children[0];

	skel.add_bone(glm::vec3(0.f, 0.f, 0.f), root->offset + next->offset, glm::mat4(1));
	skel.add_bone(root->offset + next->offset, next->offset + next_b->offset, glm::mat4(1));
	/*
	while (cur->children.size())
	{

		// Breadth First ...
		for (std::size_t c = 0; c < cur->children.size(); ++c)
		{
			prev = cur;
			offs = prev->offset; 
			cur = cur->children[c];
			//skel.add_bone(prev->offset, cur->offset, glm::mat4(1));
			skel.add_bone((offs + prev->offset), (offs + prev->offset) + cur->offset, glm::mat4(1));
		}
		
	}
	*/


	// Prev
	// Cur
	// Matrix Stack


	//skel.add_bone()

	// Get/Set BVH Data of current frame
	// WIP
	//[..]
}




// Set Animation Frame Member Functions
void Anim_State::inc_frame()
{
	anim_frame = ++anim_frame > max_frame ? 0 : anim_frame;
}

void Anim_State::dec_frame()
{
	anim_frame = --anim_frame < 0 ? 0 : anim_frame;
}

void Anim_State::set_frame(std::size_t Frame)
{
	anim_frame = Frame > max_frame ? max_frame : Frame;
}