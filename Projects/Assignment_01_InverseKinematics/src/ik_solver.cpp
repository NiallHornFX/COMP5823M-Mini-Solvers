// Implements
#include "ik_solver.h"


IK_Solver::IK_Solver(Anim_State *Anim, const std::vector<Joint*> &joint_chain, const Joint *joint_end, const Effector &target)
	: anim(Anim)
{	
	// Copy Joint Chain and End Effector Target Ptrs
	chain = joint_chain;
	effector_target = target;
}


// =============== Perturb Joints ================
/* Info : Perturbs each joints DOF while hold the rest constant, to calculate each P2 column entry for the Jacobian.
		  This is the FDM Numerical solve of partial derivatives of the end effector wrt each joint angle. 
*/
std::vector<glm::vec3> IK_Solver::perturb_joints()
{
	// Start of chain, is assumed to be root. 
	Joint *root = anim->bvh->joints[0];

	// Number of Columns (theta_0...theta_n) (size = joints * 3dof) 
	std::size_t dof_c = chain.size() * 3;

	//  Store Preturbed postions of end effector, for each perturbed joint (for all rot DOFs) aka columns of J. 
	std::vector<glm::vec3> pertrub_positons;
	// Also store the current ticks orginal (non per-turbed) end effector postion. 
	endEffector_current = glmToeig(end_joint->position);

	// Delta Theta to perturb joint DOFs by FDM : (P2(theta+h) - P1(theta) / h). 
	double delta_theta = 0.001f; 

	// For each joint, for each DOF, traverse the chain hierachy, perturbing only the cur DOF, and then deriving
	// the resulting end site postion , Note that end site position == end joint/effector postion as there is no 
	// offset on the end_site locations. 
	for (Joint *perturb_joint : chain)
	{
		glm::vec3 org_pos = perturb_joint->position; // Orginal Pos before perturbation of joint. 

		for (std::size_t c = 0; c < 3; ++c) // 0-2 (X-Z rotation DOFs)
		{
			ChannelEnum DOF = static_cast<ChannelEnum>(c); // Get current DOF to peturb, for joint. 

			// Reset Joint Pos back to orginal (to remove last perturbation)
			perturb_joint->position = org_pos;

			// Traverse start of joint chain (assumed to be root), when we reach the preturb joint, we perturb its cur DOF. 
			// Accumulate the resulting transform along the chain and store the resulting end effector / end joint position. 
			perturb_traverse(perturb_joint, DOF, delta_theta);

			// Query Resulting Perturbed End Effector Postion component
			glm::vec3 P2 = end_joint->position;
			// Append to P2 Column Array
			pertrub_positons.push_back(std::move(P2));
		}
	}
	return pertrub_positons;
}

// =============== Perturb Joints : Traversal ================
/* Info : Traversal of Joint Chain Hierachy, applying perturbations to specified DOF on specified joint. 
		  This is done as an iterative loop along the chain. Non perturbed joints are held constant, the
		  accumulated transform are passed forward to the child, untill we reach (and modifiy) the end effector pos. 
   Note : Possible via Friend Class of BVHData and Anim_State
*/

void IK_Solver::perturb_traverse(Joint *perturb_joint, ChannelEnum dof, double perturb_factor)
{
	glm::dmat4 trans(1.f);                                 // Accumulated Transform
	std::size_t anim_frame = anim->anim_frame;             // Current Animation Frame;
	std::size_t num_channel = anim->bvh->num_channel;      // BVH Number of Motion Channels
	real *bvh_motion = anim->bvh->motion;                  // BVH Motion Data Ptr
	real *ik_motion = anim->ik_rightArm_motion;            // IK  Motion Data

	for (Joint *joint : chain)
	{
		//  =========== Translation Still Fetches from BVH Joint Data =========== 
		if (!joint->parent) // Root joint, translation from channels. 
		{
			glm::vec4 root_offs(0., 0., 0., 1.);
			for (const Channel *c : joint->channels)
			{
				switch (c->type)
				{
					// Joint Translation still fetched from BVH
					case ChannelEnum::X_POSITION:
					{
						float x_p = motion[anim_frame * num_channel + c->index];
						root_offs.x = x_p;
						break;
					}
					case ChannelEnum::Y_POSITION:
					{
						float y_p = motion[anim_frame * num_channel + c->index];
						root_offs.y = y_p;
						break;
					}
					case ChannelEnum::Z_POSITION:
					{
						float z_p = motion[anim_frame * num_channel + c->index];
						root_offs.z = z_p;
						break;
					}
				}
			}

			trans = glm::translate(trans, glm::dvec3(root_offs));
		}
		else if (joint->parent) // Non root joints, Translation is offset. 
		{
			trans = glm::translate(trans, joint->offset);
		}

		// ====== Perturbation of Joints OR Hold Constant 
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



// ============================= Static Member Functions ==========================
// glm to eigen conversions
Eigen::Vector3f IK_Solver::glmToeig(const glm::vec3 &vec)
{
	return Eigen::Vector3f(vec.x, vec.y, vec.z);
}

Eigen::Vector4f IK_Solver::glmToeig(const glm::vec4 &vec)
{
	return Eigen::Vector4f(vec.x, vec.y, vec.z, vec.w);
}