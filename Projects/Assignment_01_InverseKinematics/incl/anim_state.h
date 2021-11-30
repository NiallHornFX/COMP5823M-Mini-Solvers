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

	// Build Skeleton Inital State
	void build_bvhSkeleton();

	// Fetch Skeleton Updates (Channel data)
	//void build_traverse(Joint *joint, glm::mat4 trans); // Recurrsive Tree Traversal to build inital tree state 

	// Fetch Transform of joint (from bone)
	//glm::mat4 fetch_joint_transform(Joint *joint); 
	glm::mat4 fetch_traverse(Joint *joint, glm::mat4 trans); //Recurrsive Tree Traversal to fetch per tick transformations

	// Get Joint motion
	void tick();

	void build_per_tick(); // testing only...
	void build_test(Joint *joint, glm::vec3 poffs, glm::mat4 trs);
	//void build_test_b(Joint *joint, glm::mat4 trs);
	void build_test_b(Joint *joint, glm::mat4 trs);
	void test();


	// Debug
	void debug() const;
	void chan_check(std::size_t f) const; 
	void test_local_transform();

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

	// Testing
	//glm::mat4 global; 
	//glm::vec3 global_offs;
};


#endif