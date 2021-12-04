#ifndef ANIM_STATE_H
#define ANIM_STATE_H

// Std Headers
#include <map>

// Project Headers
#include "skeleton.h"
#include "bvhdata.h"
#include "effector.h"

class IK_Solver; 

// Info :  Class where animation processing is based (BVH, FK, IK). Exists within app as Tick stage. 

class Anim_State
{
public:
	// ===== Ctor / Dtor =====
	Anim_State();
	~Anim_State() = default; 

	// ===== Core =====
	void tick();
	void render(const glm::mat4x4 &view, const glm::mat4x4 &persp);

	// ===== BVH Data =====
	void set_bvhFile(const char *BVHPath);

	// ===== Anim Controls =====
	void set_frame(std::size_t Frame);
	void inc_frame();
	void dec_frame();

	// ===== Skeleton (Bones) Build and Update =====
	void build_bvhSkeleton(); // Build Skeleton from joints, inital state.
	void update_bvhSkeleton(); // Update Skeleton from joints, per tick.
	void fetch_traverse(Joint *joint, glm::mat4 trans); 

	// ===== IK =====
	void ik_test_setup();
	void ik_test_tick();

	void gather_joints(Joint *start, std::vector<Joint*> &joints, int32_t depth=-1); // Form Chain of joints. 

	// ===== Debug =====
	void debug() const;
	void chan_check(std::size_t f) const; 

public:

	// ===== BVH Data =====
	BVH_Data *bvh;

	// ===== IK Data =====
	std::vector<Effector*> effectors; 
	IK_Solver *test_solve; 

	// ===== Shared Anim State =====
	// Skeleton 
	Skeleton skel;

	std::size_t anim_frame, max_frame; 
	float interval; 
	bool anim_loop;

	float viewer_dt; 
	std::size_t viewer_tick_c; 
};


#endif