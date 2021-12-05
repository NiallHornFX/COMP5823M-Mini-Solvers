// Implements 
#include "anim_state.h"

// Project Headers
#include "ik_solver.h"

// Std Headers
#include <cassert>

#define CREATE_ROOT_BONE 0 

// Default Ctor, state is set on demmand as needed. 
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

	// Load BVH File into new BVH_Data
	bvh = new BVH_Data(BVHPath);
	bvh->Load();

	// Update Frame / Time 
	max_frame = bvh->num_frame;
	interval  = bvh->interval;

	// Build Inital Skeleton State
	build_bvhSkeleton();
	update_bvhSkeleton();

	// IK Setup
	ik_test_setup();
}

// Sets Joint Angles for current frame 
void Anim_State::tick()
{
	// Incrment Anim Frame
	if (anim_loop) inc_frame();

	// Update Effector Positions (wrap this into effector::tick() ? )
	for (Effector *e : effectors)
	{
		//e->set_pos(e->target->position + (e->target_offset * glm::sin(viewer_dt * 5.f)));
		e->set_pos(e->target->position + e->target_offset);
	}

	// Update Skeleton per tick, from joint angles. 
	update_bvhSkeleton();
}

// Fill out Skeleton from BVH Tree using offsets. 
// Only needs to be done once per BVH file load, then update joint angles per tick / anim frame.

void Anim_State::build_bvhSkeleton()
{
	// Get root offset  from Channel Data of 0th frame
	// Channel indices should be first 3 (as root is first joint in hierachy)
	glm::vec3 root_offs(bvh->motion[0], bvh->motion[1], bvh->motion[2]);

	for (Joint *joint : bvh->joints)
	{
		if (joint->is_root)
		{
			// Use fetched root offset
			#if CREATE_ROOT_BONE == 1
			skel.add_bone(glm::vec3(0.f), root_offs, glm::mat4(1), -1); // Bone has no starting parent joint (hence -1 index).
			#endif
		}
		else // Regular Joint
		{
			// Get Parent Offset 
			glm::vec3 par_offs(0.f);
			Joint *p = joint;
			while (p->parent)
			{
				// Accumulate offset
				par_offs += p->parent->offset;

				// Traverse up to parent 
				p = p->parent;
			}
			// Add fetched Root offset
			par_offs += root_offs;

			// Add Bone to Skeleton
			skel.add_bone(par_offs, (par_offs + joint->offset), glm::mat4(1), joint->parent->idx); // Bone starts at parent joint. 
		}
	}
}

// Update Bones based on joint data for current set anim_frame. 
void Anim_State::update_bvhSkeleton()
{
	 fetch_traverse(bvh->joints[0], glm::mat4(1.f));
} 

// Recursivly traverse through hierachy, update joints and their resulting bones transforms. 
void Anim_State::fetch_traverse(Joint *joint, glm::mat4 trans)
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

		trans = glm::translate(trans, glm::vec3(root_offs));
	}
	else if (joint->parent) // Non root joints, Translation is offset. 
	{
		trans = glm::translate(trans, joint->offset);
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
				trans = glm::rotate(trans, glm::radians(z_r), glm::vec3(0., 0., 1.));
				break;
			}
			case ChannelEnum::Y_ROTATION:
			{
				float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				trans = glm::rotate(trans, glm::radians(y_r), glm::vec3(0., 1., 0.));
				break;
			}
			case ChannelEnum::X_ROTATION:
			{
				float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				trans = glm::rotate(trans, glm::radians(x_r), glm::vec3(1., 0., 0.));
				break;
			}
		}
	}

	// Store current position. 
	joint->position = glm::vec3(trans * glm::vec4(0.f, 0.f, 0.f, 1.f));

	
	// Search for joint in bones and update transform of each end point.
	for (Bone *bone : skel.bones)
	{
		if (bone->joint_id == joint->idx)
		{
			bone->update(trans); // Pass Joint transform matrix to update bone
		}
	}
	
	/*
	// ==================== End Point ====================
	if (joint->is_end)
	{
		// Search for bone with end point joint and update transform.
		// [..] (not needed for now).
	}
	*/

	// ==================== Children ====================
	// Pass each recurrsive call its own copy of the current (parent) transformations to then apply to children.
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		fetch_traverse(joint->children[c], trans);
	}
}

// ===================== IK Setup =====================
void Anim_State::ik_test_setup()
{
	// Create target effector (based on LThumbs Location + offset) 
	Joint *l_thumb = bvh->find_joint("LThumb");
	assert(l_thumb);
	Effector *eff_arm_r = new Effector(l_thumb->position, 0);
	eff_arm_r->target = l_thumb;
	eff_arm_r->target_offset = glm::vec3(-20.0f, 8.f, 0.f);
	effectors.push_back(eff_arm_r);

	// Create IK Chain based at RThumb (end_site)
	Joint *r_thumb = bvh->find_joint("RThumb");
	assert(r_thumb && r_thumb->is_end);

	// Define IK Chain of joints
	std::vector<Joint*> chain; 
	// Gather Joints back to root up to some depth (or up to root by default)
	gather_joints(r_thumb, chain);

	// Get Perturbed Postions of end effector, with respect to each joint dof to form Jacobian Matrix (P2 - P1 / Dtheta) 
	// End Joint / Effector = r_thumb, Start Joint = root. 



	/*
	// Pass Chain and End Effector to IK Solve 
	test_solve = new IK_Solver(this, chain, eff_arm_r);
	*/

	// IK Solve
	// Update Joint angles Motion Data for affected joints of IK solve. 
}

// Used to create IK chain of joints, gathering joints from starting joint, back to root, up to some depth. 
void Anim_State::gather_joints(Joint *start, std::vector<Joint*> &joints, int32_t depth)
{
	// if depth -1, gather joints all the way back to root. 
	std::size_t c = 0; 
	
	Joint *joint = start;
	while (joint->parent)
	{
		joints.push_back(joint); // Append to chain

		if (depth > 0 && depth == c) break;

		joint = joint->parent;
		c++; 
	}
}

// For drawing Anim_States own primtives (eg effectors)
void Anim_State::render(const glm::mat4x4 &view, const glm::mat4x4 &persp)
{
	// =========== Render Effectors ===========
	for (Effector *effec : effectors)
	{
		// Set Scaled Transltion for rendering (copy model current model transform) use 0.025 also used for bones. 
		effec->mesh->model[3] = glm::vec4((effec->pos * glm::vec3(0.025)), 1.f);

		// Set Camera Transform
		effec->mesh->set_cameraTransform(view, persp);

		// Render
		effec->mesh->render();
	}
}

// ===================== Animation Frame state member functions =====================
// Need to make sure inc/dec is only done for interval of current dt. 

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


// ===================== Joint Perturbation =====================
// For construction of Jacobian Matrix, relating joint angles to end effector. 

// Both start_joint + end_effec should be within the joint chain been evaulated. 
// chain          - joint chain to get perturbed postions for. 
// end_effc       - end_site joint that defined end effector
// start_joint    - start traversal along chain from this joint, (typically root joint or first joint along chain rel to root).
// perturb_factor - factor to perturb each joint's DOF angle by. 
std::vector<std::pair<glm::vec3, glm::vec3>> Anim_State::perturb_joints(std::vector<Joint*> &chain, Joint *end_effec, Joint *start_joint, float perturb_factor)
{
	std::size_t dof_c = chain.size() * 3;    // Number of Columns (theta_0 ... theta_n)
	glm::vec3 end_pos = end_effec->position; // We know number of rows is 3 (3 DOFs in end effector) pos (P_x ... P_z)

	// Vector to store Orginal and preturbed postions of end effector, for each perturbed joint (for all rot DOFs). 
	// (Defines single col of Jacobian in the form of P2 - P1 / Dtheta)
	std::vector<std::pair<glm::vec3, glm::vec3>> pertrub_pos(dof_c, std::pair<glm::vec3, glm::vec3>(glm::vec3(0.f), glm::vec3(0.f)));
	

	// For each joint, traverse the chain hierachy, perturbing only it, and then deriving the end site postion
	// Not end site position == last joint (in chain) postion as there is no offset on the end_site locations. 

	for (Joint *perturb_joint : chain)
	{
		glm::vec3 org_pos = perturb_joint->position; // Orginal Pos before perturbation of joint. 
		glm::vec3 col(0.f); // 
		glm::vec3 P1 = end_effec->position; // Un-perturbed end effector postion (x,y,z) 
		glm::vec3 P2(0.f);
		
		// Solve for each joints, perturbed DOF (x,y,z rotation)

		// Peturb X Rotational DOF 
		perturb_traverse(start_joint, perturb_joint, ChannelEnum::X_ROTATION, 0.001f, glm::mat4(1.f));
		// Query Resulting Modified End Effector Postion X component
		P2.x = end_effec->position.x; 
		// Reset to org pos
		perturb_joint->position = org_pos; 

		// Peturb Y Rotational DOF 
		perturb_traverse(start_joint, perturb_joint, ChannelEnum::Y_ROTATION, 0.001f, glm::mat4(1.f));
		// Query Resulting Modified End Effector Postion X component
		P2.y = end_effec->position.y;
		// Reset to org pos
		perturb_joint->position = org_pos;

		// Peturb X Rotational DOF 
		perturb_traverse(start_joint, perturb_joint, ChannelEnum::Z_ROTATION, 0.001f, glm::mat4(1.f));
		// Query Resulting Modified End Effector Postion X component
		P2.z = end_effec->position.z;
		// Reset to org pos
		perturb_joint->position = org_pos;

		// Now P1 defines orginal effector pos, P2 defines perturbed effector pos, for all of Joints DOFs. 
		pertrub_pos.push_back(std::pair<glm::vec3, glm::vec3>(P1, P2));
	}

}

//
// joint - Joint that we are currently operating (recusivly called) on. 
// perturb_joint - the joint in the chain we want to perturb, (check if current joint, == perturb joint).
// dof - the DOF / axis angle we want to perturb
// perturb_fac - Perturbation amount

// This is not great as it uses the same recursive approach before to accumulate transforms (with the one perturbed joint, dof) to calc the resulting joint postions
// including the end postions. 
// Ideally we iterativly traverse along the Joint chain, so we don't need to eval the whole joint hierachy over and over ....
void Anim_State::perturb_traverse(Joint *joint, Joint *perturb_joint, ChannelEnum dof, float perturb_fac, glm::mat4 trans)
{
	// Do same recursive transformation accumulation approach as before

	// Do we perturb the joint currently been evaluated by this call ? 
	bool perturb_joint = joint->idx == perturb_joint->idx ? true : false;

	//  =========== Translation is the same, as preturbation only occurs on rotation =========== 
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

		trans = glm::translate(trans, glm::vec3(root_offs));
	}
	else if (joint->parent) // Non root joints, Translation is offset. 
	{
		trans = glm::translate(trans, joint->offset);
	}

	// =========== Get Rotation - Constant Or Perturbed ===========
	glm::mat4 xx(1.), yy(1.), zz(1.);

	if (perturb_joint) // Perturbed Joint Rotation 
	{
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
			case ChannelEnum::Z_ROTATION:
			{
				float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				// Preturb for DOF Rot_Z
				if (dof == ChannelEnum::Z_ROTATION) z_r += perturb_fac;
				trans = glm::rotate(trans, glm::radians(z_r), glm::vec3(0., 0., 1.));
				break;
			}
			case ChannelEnum::Y_ROTATION:
			{
				float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				// Preturb for DOF Rot_Y
				if (dof == ChannelEnum::Y_ROTATION) y_r += perturb_fac;
				trans = glm::rotate(trans, glm::radians(y_r), glm::vec3(0., 1., 0.));
				break;
			}
			case ChannelEnum::X_ROTATION:
			{
				float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				// Preturb for DOF Rot_X
				if (dof == ChannelEnum::X_ROTATION) x_r += perturb_fac;
				trans = glm::rotate(trans, glm::radians(x_r), glm::vec3(1., 0., 0.));
				break;
			}
			}
		}
	}
	else // Constant Joint Rotation
	{
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					trans = glm::rotate(trans, glm::radians(z_r), glm::vec3(0., 0., 1.));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					trans = glm::rotate(trans, glm::radians(y_r), glm::vec3(0., 1., 0.));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					trans = glm::rotate(trans, glm::radians(x_r), glm::vec3(1., 0., 0.));
					break;
				}
			}
		}
	}


	// Update current position. (which eventually will be the end_site joint) 
	joint->position = glm::vec3(trans * glm::vec4(0.f, 0.f, 0.f, 1.f));

	// ==================== Children ====================
	// Contiune Traversing Joint Children untill end site of chain is reached. 
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		fetch_traverse(joint->children[c], trans);
	}

	// As state above this recursive approach is not ideal, because its unlikely we want to perturb joint angles / DOFs over 
	// the whole joint hieracy, ideally we should do this iterativly over the joint chain instead ... 
}





// ===================== Debug =====================

void Anim_State::debug() const
{
	std::cout << "Anim_State::" << bvh->filename << " ::Frame = " << anim_frame << "\n";
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