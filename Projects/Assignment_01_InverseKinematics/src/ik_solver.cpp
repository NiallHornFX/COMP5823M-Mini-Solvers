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
std::vector<glm::vec3> IK_Solver::perturb_joints(double perturb_fac)
{
	// Start of chain, is assumed to be root. 
	Joint *root = anim->bvh->joints[0];

	// Number of Columns (theta_0...theta_n) (size = joints * 3dof) 
	std::size_t dof_c = chain.size() * 3;

	//  Store Preturbed postions of end effector, for each perturbed joint (for all rot DOFs) aka columns of J. 
	std::vector<glm::vec3> pertrub_positons;
	// Also store the current ticks orginal (non per-turbed) end effector postion. 
	endEffector_current = glmToeig(end_joint->position);

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
			perturb_traverse(chain, perturb_joint, DOF, perturb_fac);

			// Query Resulting Perturbed End Effector Postion component
			glm::vec3 P2 = end_joint->position;
			// Append to P2 Column Array
			pertrub_positons.push_back(std::move(P2));
		}
	}
	return pertrub_positons;
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