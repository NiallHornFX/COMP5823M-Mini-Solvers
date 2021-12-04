// Implements
#include "ik_solver.h"


IK_Solver::IK_Solver(Anim_State *Anim, std::vector<Joint*> &joints, Effector *target)
	: anim(Anim)
{
	// 0th element of chain, should be end site, Current Effector Position
	Eigen::Vector3f chain_end_pos = glmToeig(joints[0]->position);
	// Target Effector Position 
	Eigen::Vector3f target_pos = glmToeig(target->pos);

	// Target Orientation is discarded, we have N = Joint_count * 3DOFs, and a 3DOF End Effector target. 
	// Jacobian size is thus (N = J*3 x 3)
	for (Joint *joint : joints)
	{
		// Calculate delta vectors (Joint Position Current - Target Effector Position) 
		Eigen::Vector3f j_pos = glmToeig(joint->position);
		Eigen::Vector3f delta = target_pos; - j_pos;

		// Get Joint Axis of rotation
		glm::vec3 joint_dof3 = glm::normalize(anim->bvh->get_joint_DOF3(joint->idx, anim->anim_frame));
		Eigen::Vector3f joint_axis(joint_dof3.x, joint_dof3.y, joint_dof3.z); // Re-order to (x,y,z)
		//std::cout << "Joint_" << joint->idx << " Axis = [" << joint_axis.x() << "," << joint_axis.y() << "," << joint_axis.z() << "]\n";

		// Cross Joint Axis with Delta (G goal) (defines column in matrix, for each component been on rows)
		Eigen::Vector3f G = joint_axis.cross(delta);
	}
}




// ========== Static Member Functions ==========
// glm to eigen conversions
Eigen::Vector3f IK_Solver::glmToeig(const glm::vec3 &vec)
{
	return Eigen::Vector3f(vec.x, vec.y, vec.z);
}

Eigen::Vector4f IK_Solver::glmToeig(const glm::vec4 &vec)
{
	return Eigen::Vector4f(vec.x, vec.y, vec.z, vec.w);
}