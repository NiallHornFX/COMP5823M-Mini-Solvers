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
	//build_per_tick();
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

// Abanonded this approach for now. Ignore...
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
		if (p->is_root) continue;
		while (p->parent)
		{
			// Get Angles from motion data of current frame
			DOF3 angs = bvh->get_joint_DOF3(p->parent->idx, anim_frame);
			// Build Local Matrix to multiply accumlated with 
			glm::mat4 tmp(1.f);
			// Z Rotation 
			tmp = glm::rotate(tmp, glm::radians(float(std::get<0>(angs))), glm::vec3(0.f, 0.f, 1.f));
			// Y Rotation 
			tmp = glm::rotate(tmp, glm::radians(float(std::get<1>(angs))), glm::vec3(0.f, 1.f, 0.f));
			// X Rotation 
			tmp = glm::rotate(tmp, glm::radians(float(std::get<2>(angs))), glm::vec3(1.f, 0.f, 0.f));
			// Accumlate Rotation 
			rot *= tmp;

			// Accumulate offset
			//par_offs += p->parent->offset;

			// Traverse up to parent 
			p = p->parent;

			// Start is then parent offset, end is the current joint offset + parent offset (total parent offset along tree).
			//skel.add_bone(par_offs, (cur->offset + par_offs), rot);
			rot = glm::translate(rot, par_offs);
			skel.add_bone(glm::vec3(0.f), cur->offset, rot);
		}

	}
}

void Anim_State::test()
{
	skel.reset();
	glm::tmat4x4<double, glm::highp>;

	//build_test(bvh->joints[0], glm::vec3(0.f), glm::mat4(1.f));
	build_test_b(bvh->joints[0], glm::mat4(1.f));
}

// Following BVH Sample file style recursion
void Anim_State::build_test(Joint *joint, glm::vec3 poffs, glm::mat4 trs)
{
	static std::size_t call = 0;
	// 
	if (!joint->parent) // Must be root (6DOF Channels) (Root translation/offset comes from motion data)
	{
		glm::mat4 xx(1.f), yy(1.f), zz(1.f);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
				case ChannelEnum::X_POSITION:
				{
					float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					poffs.x += x_p;
					break;
				}
				case ChannelEnum::Y_POSITION:
				{
					float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					poffs.y += y_p;
					break;
				}
				case ChannelEnum::Z_POSITION:
				{
					float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					poffs.z += z_p;
					break;
				}
				// Rotation
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::mat4(1.f), glm::radians(z_r), glm::vec3(0.f, 0.f, 1.f));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					yy = glm::rotate(glm::mat4(1.f), glm::radians(y_r), glm::vec3(0.f, 1.f, 0.f));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					xx = glm::rotate(glm::mat4(1.f), glm::radians(x_r), glm::vec3(1.f, 0.f, 0.f));
					break;
				}
			}
		}
		
		// =========== Rotation --> trs Matrix ===========
		// Accumulate Rotation in YXZ Order 
		trs = (yy * xx * zz) * trs; 

		// =========== Translation / Offset --> trs Matrix ===========
		// Root has 0 offset has offset is obtained from channels.
		trs = glm::translate(trs, poffs);

		// Add Bone, Use Rel offsets for start end only.
		skel.add_bone(glm::vec3(0.f), poffs, trs);
	}
	else if (joint->parent) // Non root joints, 3DOF
	{
		// Get Joint Channels, Accumulate to parent matrix
		glm::mat4 xx(1.f), yy(1.f), zz(1.f);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::mat4(1.f), glm::radians(z_r), glm::vec3(0.f, 0.f, 1.f));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					yy = glm::rotate(glm::mat4(1.f), glm::radians(y_r), glm::vec3(0.f, 1.f, 0.f));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					xx = glm::rotate(glm::mat4(1.f), glm::radians(x_r), glm::vec3(1.f, 0.f, 0.f));
					break;
				}
			}
		}

		// =========== Rotation --> trs Matrix ===========
		// DEBUG : This breaks, root transform is fine ...
		// Accumulate Rotation in YXZ Order 
		trs = (yy * xx * zz) * trs; 

		// =========== Translation / Offset --> trs Matrix ===========
		// Add Joint parent offset to matrix as translation
		trs = glm::translate(trs, joint->parent->offset);

		// Start+end at Rel Offset
		skel.add_bone(glm::vec3(0.f), joint->offset, trs);
	}

	// Pass each recurrsive call its own copy of the current accumulated offset and rot, to then apply to children.
	// don't reference a global one as it will trsform by their non parents. 
	// Recurse for all joint children
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		build_test(joint->children[c], poffs, trs);
	}

	call++;
	//std::cout << "Call Count = " << call << "\n";
}


void Anim_State::build_test_b(Joint *joint, glm::mat4 trs)
{
	//  =========== Get Translation  ===========
	if (!joint->parent) // Root joint, translation from channels. 
	{
		glm::vec4 root_offs(0., 0., 0., 1.);

		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
			case ChannelEnum::X_POSITION:
			{
				float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.x = x_p;
				break;
			}
			case ChannelEnum::Y_POSITION:
			{
				float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.y = y_p;
				break;
			}
			case ChannelEnum::Z_POSITION:
			{
				float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.z = z_p;
				break;
			}
			}
		}

		trs = glm::translate(trs, glm::vec3(root_offs));

	}
	else if (joint->parent) // Non root joints, Translation is offset. 
	{
		trs = glm::translate(trs, joint->offset);
	}

	// =========== Get Rotation ===========
	glm::mat4 xx(1.), yy(1.), zz(1.);
	for (const Channel *c : joint->channels)
	{
		switch (c->type)
		{
		case ChannelEnum::Z_ROTATION:
		{
			float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
			trs = glm::rotate(trs, glm::radians(z_r), glm::vec3(0., 0., 1.));
			break;
		}
		case ChannelEnum::Y_ROTATION:
		{
			float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
			trs = glm::rotate(trs, glm::radians(y_r), glm::vec3(0., 1., 0.));
			break;
		}
		case ChannelEnum::X_ROTATION:
		{
			float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
			trs = glm::rotate(trs, glm::radians(x_r), glm::vec3(1., 0., 0.));
			break;
		}
		}
	}

	// Transformations Applied to vertices using current concatenated matrix.

	// ==================== Viz Joints as Points ====================
	// Viz Joints as Points 
	glm::vec4 v0 = trs * glm::vec4(0.f, 0.f, 0.f, 1.f);
	// Joint point (as line)
	skel.add_bone(glm::vec3(v0), glm::vec3(v0), glm::mat4(1.f));

	// ==================== End Point ====================
	if (joint->is_end)
	{
		glm::vec4 v1 = trs * glm::vec4(joint->end, 1.f);
		// Joint point (as line)
		//skel.add_bone(glm::vec3(v1), glm::vec3(v1), glm::mat4(1.f));
		// Bone Line 
		skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.f));
	}
	
	// ==================== Children ====================
	// Pass each recurrsive call its own copy of the current (parent) transformations to then apply to children.
	for (std::size_t c = 0; c < joint->children.size(); ++c) 
	{
		// =========== Add Bone for each child offset, rel to parent transform ===========
		Joint *child = joint->children[c];
		glm::vec4 v2 = trs * glm::vec4(child->offset, 1.f);
		// Bone Line
		skel.add_bone(glm::vec3(v0), glm::vec3(v2), glm::mat4(1.f));
		
		// =========== Recurse for all joint children ===========
		build_test_b(joint->children[c], trs);
	}

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