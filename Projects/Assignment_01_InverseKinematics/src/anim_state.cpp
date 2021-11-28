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
	static std::size_t call = 0;
	if (!joint->parent) // Must be root (6DOF Channels) (Root translation/offset comes from motion data)
	{
		glm::mat4 xx(1.f), yy(1.f), zz(1.f);
		glm::vec4 root_offs (0.f, 0.f, 0.f, 1.f);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
				case ChannelEnum::X_POSITION:
				{
					float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.x += x_p;
					break;
				}
				case ChannelEnum::Y_POSITION:
				{
					float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.y += y_p;
					break;
				}
				case ChannelEnum::Z_POSITION:
				{
					float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.z += z_p;
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
		trs = (yy * xx * zz);
		trs[3] = root_offs;

		// Add Bone, Use Rel offsets for start end only.
		glm::vec4 v0 = trs * glm::vec4(0.f, 0.f, 0.f, 1.f);
		glm::vec4 v1 = trs * root_offs;
		skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.f));
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

		// Using parent offset, but joint cur rotations ... 

		glm::mat4 par_trs = trs; 
		glm::mat4 child_trs = (yy * xx * zz);
		trs = (yy * xx * zz) * trs; // Combined. 

		// =========== Translation / Offset --> trs Matrix ===========
		poffs += joint->parent->offset;
		// Add Joint parent offset to matrix as translation
		//trs = glm::translate(trs, joint->parent->offset);

		///////////// DO TRANSFORM HERE

		//glm::vec4 vp(poffs, 1.f);
		//glm::vec4 v0(poffs, 1.f);
		//glm::vec4 v1(poffs + joint->offset, 1.f);
		//glm::vec4 vp1(poffs + joint->offset, 1.f);
		//float len = glm::length(v1 - v0);

		glm::mat4 parent = par_trs;
		parent[3] = glm::vec4(poffs, 1.f);

		glm::mat4 child = child_trs;
		child[3] = glm::vec4(joint->offset, 1.f);

		// Init in rel local space
		glm::vec4 v0(0.f, 0.f, 0.f, 1.f);
		glm::vec4 v1(joint->offset, 1.f);

		// Apply Child Transform
		//v0 = child * v0; 
		//v1 = child * v1;

		// Apply Parent
		//v0 = parent * v0;
		//v1 = parent * v1;

		// -------
		// Only add parent offset translation, joint offset already on second vert..

		//v0 += glm::vec4(poffs, 1.f); 
		//v1 += glm::vec4(poffs, 1.f); 

		// In matrix form 
		glm::mat4 par_2(1.f);
		par_2[3] = glm::vec4(poffs, 1.f);

		v0 = par_2 * v0;
		v1 = par_2 * v1;
		// -------

		//v0 = (child * parent) * v0;
		//v1 = (child * parent) * v1;


		//v0 += glm::vec4(poffs, 1.f) + glm::vec4(joint->offset, 1.f);
		//v1 += glm::vec4(poffs, 1.f) + glm::vec4(joint->offset, 1.f);


		//v0 = (child * parent) * v0;
		//v1 = (child * parent) * v1;



		//glm::vec4 b0(0.f, 0.f, 0.f, 1.f);
		//glm::vec4 b1(joint->offset, 1.f);
		//trs[3] = vp1;
		//b0 = trs * b0;
		//b1 = trs * b1;

		//par_trs[3] = vp;
		//trs[3] = vp1;

		//par_trs[3] = vp1;
		//v0 = par_trs * v0;
		//v1 = par_trs * v1;

		// To Local Space rel to parent
		//v0 -= vp;
		//v1 -= vp;

		//v0 = glm::inverse(par_trs) * v0;
		//v1 = glm::inverse(par_trs) * v1;

		// Rotate 
		//v0 = trs * glm::vec4(v0.x, v0.y, v0.z, 1.f);
		//v1 = trs * glm::vec4(v1.x, v1.y, v1.z, 1.f);

		//v0 = par_trs * v0;
		//v1 = par_trs * v1;

	//	v0 = trs * glm::vec4(v0.x, v0.y, v0.z, 1.f);
	//	v1 = trs * glm::vec4(v1.x, v1.y, v1.z, 1.f);

		// Back to WS ... but also need parents rotation here... cant just add trans back because parents rot is differnet now also ...
		// Parent Rot + my trans ? 

		//glm::mat4 baz = par_trs; 
		//baz = glm::translate(baz, glm::vec3(vp));
		//baz[3] = glm::vec4(0.f, 100.f, 0.f, 1.f);

		//v0 = baz * v0;
		//v1 = baz * v1;

	//	v0 += vp;
	//	v1 += vp;

	//	v0 = par_trs * v0;
	//	v1 = par_trs * v1; */

		// Start+end at Rel Offset
		skel.add_bone(glm::vec3(v0), glm::vec3(v1),  glm::mat4(1.f));
		//skel.add_bone(glm::vec3(b0), glm::vec3(b1),  glm::mat4(1.f));
	}

	// Pass each recurrsive call its own copy of the current accumulated offset and rot, to then apply to children.
	// don't reference a global one as it will trsform by their non parents. 
	// Recurse for all joint children
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		build_test_b(joint->children[c], trs);
	}

	call++;
	//std::cout << "Call Count = " << call << "\n";
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