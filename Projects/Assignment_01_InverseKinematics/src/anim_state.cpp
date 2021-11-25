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

	std::cout << "TEST ROOT = " << root->offset.x << " " << root->offset.y << " " << root->offset.z << "\n";


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