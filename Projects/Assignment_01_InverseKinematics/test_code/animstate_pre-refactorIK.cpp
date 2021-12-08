// Implements 
#include "anim_state.h"

// Project Headers
#include "ik_solver.h"

// Std Headers
#include <cassert>
#include <algorithm>

#define CREATE_ROOT_BONE 0 
#define DEBUG_JACOBIAN 1

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
	//update_bvhSkeleton();
	
	// IK Setup
	ik_test_setup();
	ik_test_tick();

	// Need to now Update Bones from IK set joint positions ...

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

	// Tick IK Solvers for joint chains ...
	ik_test_tick();

	// Update Skeleton per tick, from joint angles. 
	//update_bvhSkeleton();
}

// Fill out Skeleton from BVH Tree using offsets. 
// Only needs to be done once per BVH file load, then update joint angles per tick / anim frame.

// Build BVH Skeleton :
// Builds Inital Skeleton of bones, from accumulated joint offset transforms. Resulting in Rest-Pose bones. 
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

// ===========================================================================================================
//										Skeleton Update (Forward Kinematics)
// ===========================================================================================================

// Update Bones based on joint data for current set anim_frame. 
void Anim_State::update_bvhSkeleton()
{
	 fetch_traverse(bvh->joints[0], glm::mat4(1.f));
} 

// Recursivly traverse through hierachy, update joints and their resulting bones transforms from BVH Motion Data. 
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

// ===========================================================================================================
//										Inverse Kinematics Setup 
// ===========================================================================================================

void Anim_State::ik_setup()
{
	// ========== Single Chain setup ==========
	// Joint chain from RThumb (as end joint / end effector), back to root. 
	std::vector<Joint*> chain = create_joint_chain("RThumb");

	// Create Target End Effector from some otjer joint, not part of the chain. 
	Joint *tgt_joint = bvh->find_joint("LThumb");
	Effector target(tgt_joint, glm::vec3(-20.0f, 8.f, 0.f));

	// Pass Joint Chain and Target End Effector to IK Solver. 
	ik_rightArm = new IK_Solver(this, chain, target);
}

std::vector<Joint*> Anim_State::create_joint_chain(std::string end_joint_name, int32_t depth)
{
	// ========= Find End Joint =========
	Joint *end_joint = bvh->find_joint(end_joint_name);
	if (!end_joint || !end_joint->is_end)
	{
		std::cerr << "ERROR::Cannot create joint chain for::" << end_joint_name << "::Joint is not an end joint." << std::endl;
		std::terminate();
	}
	// ========= Gather Joints in Chain =========
	// Gather Joints from end joint, if depth -1, gather joints all the way back to root. 
	std::vector<Joint*> chain;
	std::size_t d = 0;
	Joint *joint_i = end_joint;
	while (joint_i->parent)
	{
		chain.push_back(joint_i); // Append to chain

		if (depth > 0 && depth == d) break;

		joint_i = joint_i->parent;
		d++;
	}
	if (depth <= 0) chain.push_back(bvh->joints[0]); // Also add root joint. 

	// Reverse joints order in chain, so root (or joint closest to root) 
	// is first, end_site joint is last. 
	std::reverse(chain.begin(), chain.end());
	return chain; 
}




// ===========================================================================================================
//										Anim State Setters
// ===========================================================================================================
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
// ===========================================================================================================
//										TEST INVERSE KINEMTATICS CODE 
// ===========================================================================================================

// ============================== Joint Perturbation ==============================
/* 
   For construction of Jacobian Matrix, relating joint angles to end effector. We need (P2 - P1 / Dtheta) 
   Where P1 is the orginal non-perturbed postion of the end effector, P2 is the perturbed postion, for each joint, DOF
   which defines a single column of the Jacobian Matrix. (Thus the Jacobian is 3 x (j * DOF), which we know is 3 x (j * 3)). 
   as joints have 3 rotational DOFs as each Joint would define 3 Cols (one for each DOF). 
*/

// Both start_joint + end_effec should be within the joint chain been evaulated. 
// chain          - joint chain to get perturbed postions for. 
// end_effc       - end_site joint that defined end effector
// start_joint    - start traversal along chain from this joint, (typically root joint or first joint along chain rel to root).
// perturb_factor - factor to perturb each joint's DOF angle by. 
std::vector<std::pair<glm::vec3, glm::vec3>> Anim_State::perturb_joints(std::vector<Joint*> &chain, Joint *end_effec, Joint *start_joint, float perturb_factor)
{
	Joint *start = start_joint == nullptr ? bvh->joints[0] : start_joint; // Set start joint to root if passed nullptr. 
	
	std::size_t dof_c = chain.size() * 3;    // Number of Columns (theta_0 ... theta_n) (size = j * 3) 
	glm::vec3 end_pos = end_effec->position; // We know number of rows is (size = 3) (3 DOFs in end effector) pos (P_x, P_y, P_z)

	// Vector to store Orginal and preturbed postions of end effector, for each perturbed joint (for all rot DOFs). 
	// (Defines single col of Jacobian in the form of P2 - P1 / Dtheta)
	std::vector<vec3pair> pertrub_pos;
	
	// For each joint, for each DOF, traverse the chain hierachy, perturbing only the cur DOF, and then deriving the resulting end site postion
	// Note that end site position == last joint (in chain) postion as there is no offset on the end_site locations. 

	// We want to return pairs of P1 and P2 (where P1 is non-perturbed, P2 is perturbed), with respect to each joint DOF. 
	// We can then use these to build the Jacobian element wise (P2 - P1 / Dtheta).
	for (Joint *perturb_joint : chain)
	{
		glm::vec3 org_pos = perturb_joint->position; // Orginal Pos before perturbation of joint. 

		for (std::size_t c = 0; c < 3; ++c) // 0-2 (X-Z rotation DOFs)
		{
			ChannelEnum DOF = static_cast<ChannelEnum>(c); // Get current DOF to peturb, for joint. 

			// Reset Joint Pos back to orginal (to remove last perturbation)
			perturb_joint->position = org_pos; 

			glm::vec3 P1 = end_effec->position; // Un-perturbed end effector postion (x,y,z) 
			glm::vec3 P2(0.f); // Init P2

			float delta_theta = perturb_factor; 

			// Function traverses the joint from start of chain (assumed to be root for now), and when we reach the preturb joint, we perturb its DOF. 
			// We contiune accumulating the resulting transform along the chain and store the resulting end effector / end joint position. 
			perturb_traverse(chain, perturb_joint, DOF, delta_theta);

			// Query Resulting Modified End Effector Postion component
			P2 = end_effec->position;
			//assert(P1 != P2); // Check something is actually happeneing. 

			// Now P1 defines orginal effector pos, P2 defines perturbed effector pos, for the current DOF (rotational channel).
			pertrub_pos.push_back(vec3pair(P1, P2));
		}
	}

	return pertrub_pos;
}

// Traversal of Joint Chain Hierachy, applying perturbations to specified DOF on specified joint. 

// chain - chain to traverse and apply perturbations
// perturb_joint - the joint in the chain we want to perturb, (check if current joint, == perturb joint).
// dof - the DOF / axis angle we want to perturb
// perturb_fac - Perturbation amount
void Anim_State::perturb_traverse(std::vector<Joint*> &chain, Joint *perturb_joint, ChannelEnum dof, float perturb_fac)
{
	glm::mat4 trans(1.f); // Accumulated Transform

	for (Joint *joint : chain)
	{
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
		// Perturb, or hold constant ? 
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
		// ONLY transform the end_site, we don't care about actually transforming the other joints aslong as we have their
		// transforms accumulated (to propgate to the end site joint as it is what we are measuring the delta of). 
		if (joint->is_end) // Assumes last chain joint is an end_site / effector. 
		{
			joint->position = glm::vec3(trans * glm::vec4(0.f, 0.f, 0.f, 1.f));
		}
	}
}



// Apply Joint Angle Deltas to Joints in IK Chain. 
// Accumulating transforms from first joint / root to end effector joint. 
void Anim_State::ik_apply_deltas(const Eigen::Matrix<float, Eigen::Dynamic, 1> &deltas)
{
	glm::mat4 trans(1.f); // Accumulated Transform

	for (std::size_t j = 0; j < chain_test.size(); ++j)
	{
		Joint *joint = chain_test[j];

		//  =========== Joint Translation =========== 
		if (!joint->parent)
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

		//  =========== Joint Rotation =========== 
		// Apply Joint Angle Delta, get 3 angles of joints DOF.
		std::size_t dt_i = j * 3;
		float dt_x = deltas[j++];
		float dt_y = deltas[j++];
		float dt_z = deltas[j++];

		// Scale deltas by step_size ...
		// Integrate ...

		// For now just apply rotation directly.

		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
			case ChannelEnum::Z_ROTATION:
			{
				trans = glm::rotate(trans, glm::radians(dt_z), glm::vec3(0., 0., 1.));
				break;
			}
			case ChannelEnum::Y_ROTATION:
			{
				trans = glm::rotate(trans, glm::radians(dt_y), glm::vec3(0., 1., 0.));
				break;
			}
			case ChannelEnum::X_ROTATION:
			{
				trans = glm::rotate(trans, glm::radians(dt_x), glm::vec3(1., 0., 0.));
				break;
			}
			}
		}

		// Update Joint Pos (WS) from accumulated trans. 
		joint->position = glm::vec3(trans * glm::vec4(0.f, 0.f, 0.f, 1.f));

		// Search for joint in bones and update transform of each end point.
		for (Bone *bone : skel.bones)
		{
			if (bone->joint_id == joint->idx)
			{
				bone->update(trans); // Pass Joint transform matrix to update bone
			}
		}
	}
}


// ===================== IK Setup =====================
void Anim_State::ik_test_setup()
{
	// Create Target Effector (based on LThumbs Location + offset) 
	Joint *l_thumb = bvh->find_joint("LThumb");
	assert(l_thumb);
	target_endeffec = new Effector(l_thumb->position, 0);
	target_endeffec->target = l_thumb;
	target_endeffec->target_offset = glm::vec3(-20.0f, 8.f, 0.f);

	// Create IK Chain based at RThumb (end_site)
	joint_endeffec = bvh->find_joint("RThumb");
	assert(joint_endeffec && joint_endeffec->is_end);

	// Gather Joints from end joint, back to root up to some depth (or up to root by default)
	gather_joints(joint_endeffec, chain_test);
}

void Anim_State::ik_test_tick()
{
	// ============================ Get Perturbed Effector Postions foreach Joint DOF ============================
	// Get Perturbed Postions of end effector, with respect to each joint dof to use for 
	// formation of Jacobian Matrix. Each pair contains (P1 = unperturbed, P2 = perturbed).
	float delta_theta = 0.01f;
	std::vector<vec3pair> cols_p1p2 = perturb_joints(chain_test, joint_endeffec, nullptr, delta_theta);

	// ============================ Jacobian Construction from Pertrubation Positions ============================
	// Encap into Some class for Eigen use, for now will do inline.
	// Now Construct Jacobian. (3 x (j * 3)) j = number of joints in chain.
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> J;
	std::size_t r = 3, c = chain_test.size() * 3;
	J.resize(r, c);
	J.setZero();
	// Loop column wise
	std::size_t col_ind = 0;
	std::stringstream r0, r1, r2;
	for (auto col : J.colwise())
	{
		// Each Column get Non-Perturbed (P1) and Perturbed (P2) end effector postion vector3s. 
		std::pair<glm::vec3, glm::vec3> &perturb = cols_p1p2[col_ind];

		// Split into Effector Positional components (x,y,z) form ((P2 - P1) / Dtheta) for each el. 
		col(0) = (perturb.second.x - perturb.first.x) / delta_theta;
		col(1) = (perturb.second.y - perturb.first.y) / delta_theta;
		col(2) = (perturb.second.z - perturb.first.z) / delta_theta;

		// Dbg stream output
		r0 << col(0) << ", "; r1 << col(1) << ", "; r2 << col(2) << ", ";

		col_ind++;
	}
#if DEBUG_JACOBIAN == 1
	std::cout << "\n======== DEBUG::JACOBIAN_MATRIX::BEGIN ========\n"
		<< "Rows = " << J.rows() << " Cols = " << J.cols() << "\n"
		<< "|" << r0.str() << "|\n" << "|" << r1.str() << "|\n" << "|" << r2.str() << "|\n"
		<< "======== DEBUG::JACOBIAN_MATRIX::END ========\n";
#endif 

	// ============================ Jacobian Inverse ============================
	// Inverse via Transpose for now
	auto Jt = J.transpose();
	//std::cout << "Jacobian Transpose = \n" << Jt << "\n"; 

	// ============================ Solve for Joint Angle Velocites ============================
	Eigen::Vector3f end_cur(joint_endeffec->position.x, joint_endeffec->position.y, joint_endeffec->position.z);
	Eigen::Vector3f end_tgt(target_endeffec->pos.x, target_endeffec->pos.y, target_endeffec->pos.z);
	Eigen::Vector3f V = end_tgt - end_cur;
	Eigen::Matrix<float, Eigen::Dynamic, 1> theta_vel = Jt * V;
	std::cout << "Theta_Vel = \n" << theta_vel << "\n";

	// ============================ Apply Joint Angle Velocites via FK to chain ============================
	ik_apply_deltas(theta_vel);
}

// ===========================================================================================================
//										DEBUG / TESTS
// ===========================================================================================================

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

// For drawing Anim_States own primtives 
void Anim_State::render(const glm::mat4x4 &view, const glm::mat4x4 &persp)
{
	// Deprecated
}