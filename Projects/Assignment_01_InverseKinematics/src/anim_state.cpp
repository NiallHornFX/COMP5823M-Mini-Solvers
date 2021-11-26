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
	// Clear prev state (could just bvh::clear())
	if (bvh) delete bvh;
	skel.reset();

	bvh = new BVH_Data(BVHPath);
	bvh->Load();

	max_frame = bvh->num_frame;
	interval  = bvh->interval;

	build_bvhSkeleton();
}

void Anim_State::build_bvhSkeleton()
{
	// Fill out Skeleton from BVH Tree using offsets (only need to be done once per BVH file load, then update channels per anim frame)

	// Hacky Way to get inital pose 
	for (Joint *cur : bvh->joints)
	{
		// Get Parent Offset 
		glm::vec3 par_offs(0.f);
		Joint *p = cur;
		while (p->parent)
		{
			// Accumulate offset
			par_offs += p->parent->offset;

			// Traverse up to parent 
			p = p->parent;
		}
		// Start is then parent offset, end is the current joint offset + parent offset (total parent offset along tree).

		// Add Bone to Skeleton
		//skel.add_bone(par_offs, (cur->offset + par_offs), glm::mat4(1));
		// With Joint IDs
		std::size_t cur_par_idx = cur->parent ? cur->parent->idx : 0;
		skel.add_bone(par_offs, (cur->offset + par_offs), glm::mat4(1), cur_par_idx, cur->idx);
	}
	
}

// Is it easier to rebuild the Skeleton per tick, (with the current set anim frame motion/channel angles retrived)
// Oppose to building skeleton and updating bone transform joints later in seperate per tick step.
// Just for debugging sake. 
void Anim_State::build_per_tick()
{
	skel.reset(); // Testing calling this per tick, so reset...

	for (Joint *cur : bvh->joints)
	{
		// Get Parent Offset
		glm::vec3 par_offs(0.f);
		// Get Parent Transform
		glm::mat4 rot(1.f);

		Joint *p = cur;
		while (p->parent)
		{
			// Accumulate offset
			par_offs += p->parent->offset;

			// Accumulate Channel Transform
			// Non root joints (3DOF, joint angles only).
			if (!p->parent->is_root)
			{
				// Get Angles from motion data of current frame
				DOF3 angs = bvh->get_joint_DOF3(p->parent->idx, anim_frame);
				// Build Local Matrix to multiply accumlated with 
				glm::mat4 tmp(1.f);
				// Z Rotation 
				tmp = glm::rotate(tmp, float(std::get<0>(angs)), glm::vec3(0.f, 0.f, 1.f));
				// Y Rotation 
				tmp = glm::rotate(tmp, float(std::get<1>(angs)), glm::vec3(0.f, 1.f, 0.f));
				// X Rotation 
				tmp = glm::rotate(tmp, float(std::get<2>(angs)), glm::vec3(1.f, 0.f, 0.f));
				// Accumlate Rotation 
				rot *= tmp;
			}

			// Traverse up to parent 
			p = p->parent;
		}
		// Start is then parent offset, end is the current joint offset + parent offset (total parent offset along tree).
		skel.add_bone(par_offs, (cur->offset + par_offs), rot);
	}

}

// Sets Joint Angles for current frame 
void Anim_State::tick()
{
	// Get/Set BVH Data of current frame
	//[..]

	for (Joint *cur : bvh->joints)
	{
		// Get Parent Transform
		glm::mat4 rot(1.f);
		Joint *p = cur;
		while (p->parent)
		{
			// Accumulate Channel Transform
			// Non root joints (3DOF, joint angles only).
			if (!p->parent->is_root)
			{
				// Get Angles from motion data of current frame
				DOF3 angs = bvh->get_joint_DOF3(p->parent->idx, anim_frame);

				// Check Retrived motion channel data
				//std::cout << std::get<0>(angs) << "  " << std::get<1>(angs) << std::get<2>(angs) << "\n";

				// Build Local Matrix to multiply accumlated with 
				glm::mat4 tmp(1.f);
				// Z Rotation 
				tmp = glm::rotate(tmp, float(std::get<0>(angs)), glm::vec3(0.f, 0.f, 1.f));
				// Y Rotation 
				tmp = glm::rotate(tmp, float(std::get<1>(angs)), glm::vec3(0.f, 1.f, 0.f));
				// X Rotation 
				tmp = glm::rotate(tmp, float(std::get<2>(angs)), glm::vec3(1.f, 0.f, 0.f));

				/*
				// Check Matrix
				std::cout << "Rot Matrix Joint : " << cur->idx << " Frame : " << anim_frame << "\n"
					<< rot[0][0] << " " << rot[0][1] << " " << rot[0][2] << " " << rot[0][3] << "\n"
					<< rot[1][0] << " " << rot[1][1] << " " << rot[1][2] << " " << rot[1][3] << "\n"
					<< rot[2][0] << " " << rot[2][1] << " " << rot[2][2] << " " << rot[2][3] << "\n"
					<< rot[3][0] << " " << rot[3][1] << " " << rot[3][2] << " " << rot[3][3] << "\n\n";
				*/

				// Accumlate Rotation 
				rot *= tmp;
			}

			// Traverse up to parent 
			p = p->parent;
		}

		// Testing, terrible approach 
		// Update Bone Transform (this is where mapping joint-bone makes sense).
		// Hacky check all bones for joint_b
		for (Bone &b : skel.bones)
		{
			if (b.joint_ids.second == cur->idx)
			{
				// Invert to LS
				glm::mat4 tmp = b.transform;
				//b.transform *= glm::inverse(b.transform);
				b.transform *= glm::inverse(rot);

				// Apply Transform
				//b.transform *= rot; 
				// Reapply 
				//b.transform *= tmp;
			}
		}


	}

}

// Set Animation Frame Member Functions
// Inc and Dec loop. 

// Need to make sure inc/dec is only done for interval of current glfw dt. 

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

// Will flesh out debug later. 
void Anim_State::debug() const
{
	std::cout << "Anim::" << bvh->filename << " ::Frame = " << anim_frame << "\n";
}