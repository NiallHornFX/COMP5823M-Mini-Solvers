#ifndef ANIM_STATE_H
#define ANIM_STATE_H

// Project Headers
#include "skeleton.h"
#include "bvhdata.h"

// Info :  Class where animation processing is based (BVH, FK, IK). Exists within app as Tick Stage. 

class Anim_State
{
public:
	Anim_State();

	~Anim_State() = default; 


	void set_bvhFile(const char *BVHPath);

	// Controls
	void set_frame(std::size_t Frame);
	void inc_frame();
	void dec_frame();

	// Fetch 
	void fetch_bvhData();

public:

	// ===== BVH Data =====
	BVH_Data *bvh;

	// ===== IK Data =====
	// [..]

	// ===== Shared Anim State =====
	// Skeleton 
	Skeleton skel;

	std::size_t anim_frame, max_frame; 
	float interval; 
	bool anim_loop;
};


#endif