// Implements 
#include "anim_state.h"

// Basic Ctor, state is set on demmand as needed. 

Anim_State::Anim_State()
{
	anim_loop = true;
	anim_frame = 0;
	max_frame = 0;

	bvh = nullptr;

	// Test global rotation matrix
	//global = glm::mat4(1);
	//global_offs = glm::vec3(0.f);
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

//	build_bvhSkeleton();
	test();
	//chan_check();
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
			if (!p->parent->is_root) // Else just dont accumulate rotation for parent = root 
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

void Anim_State::test()
{
	skel.reset();
	build_test(bvh->joints[0], glm::vec3(0.f), glm::mat4(1.f));
}

// Following BVH Sample file style recursion
void Anim_State::build_test(Joint *joint, glm::vec3 poffs, glm::mat4 rot)
{
	static std::size_t call = 0;
	// 
	if (!joint->parent)
	{
		skel.add_bone(glm::vec3(0.f), joint->offset, glm::mat4(1));
		poffs += glm::vec3(0.f);
	}
	
	if (joint->parent)
	{
		// Get Joint Channels, Accumulate to parent matrix
		glm::mat4 xx(1.f), yy(1.f), zz(1.f); 
		for (const Channel *c : joint->channels)
		{
			switch (c->type) 
			{
				case ChannelEnum::Z_ROTATION : 
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::mat4(1.f), glm::radians(z_r), glm::vec3(0.f, 0.f, 1.f));
					//rot = glm::rotate(rot, glm::radians(z_r), glm::vec3(0.f, 0.f, 1.f));
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					yy = rot = glm::rotate(glm::mat4(1.f), glm::radians(y_r), glm::vec3(0.f, 1.f, 0.f));
					//rot = glm::rotate(rot, glm::radians(y_r), glm::vec3(0.f, 1.f, 0.f));
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::mat4(1.f), glm::radians(x_r), glm::vec3(1.f, 0.f, 0.f));
					//rot = glm::rotate(rot, glm::radians(x_r), glm::vec3(1.f, 0.f, 0.f));
				}
			}
		}

		// Concat Channels Rotations (y,x,z order) and right mult parent rotation.
		rot = (yy * xx * zz) * rot;

		// Accumulate Offset to parent matrix
		poffs += joint->parent->offset;

		skel.add_bone(poffs, poffs + joint->offset, rot);

	   /* // Debug Matrix
		std::cout << "Rot Matrix Joint : " << joint->idx << " Frame : " << anim_frame << "\n"
		<< rot[0][0] << " " << rot[0][1] << " " << rot[0][2] << " " << rot[0][3] << "\n"
		<< rot[1][0] << " " << rot[1][1] << " " << rot[1][2] << " " << rot[1][3] << "\n"
		<< rot[2][0] << " " << rot[2][1] << " " << rot[2][2] << " " << rot[2][3] << "\n"
		<< rot[3][0] << " " << rot[3][1] << " " << rot[3][2] << " " << rot[3][3] << "\n\n"; */
	}

	// Pass each recurrsive call its own copy of the current accumulated offset and rot, to then apply to children.
	// don't reference a global one as it will transform by their non parents. 

	// Recurse for all joint children
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		build_test(joint->children[c], poffs, rot);
	}

	//std::cout << "Call Count = " << call << "\n";
	call++;
}

// Sets Joint Angles for current frame 
void Anim_State::tick()
{
	//build_test(bvh->joints[0]);
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

void Anim_State::chan_check(std::size_t f) const
{
	// Get random joint
	Joint *joint = bvh->joints[3];

	// Check anim_data over all frames.
	for (std::size_t f = 0; f < bvh->num_frame; ++f)
	{
		std::cout << "Frame " << f << " Data " << bvh->motion[f * bvh->num_channel + joint->channels[0]->index] << "\n";
	}
	
}