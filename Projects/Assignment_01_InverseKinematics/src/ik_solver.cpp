// Implements
#include "ik_solver.h"


IK_Solver::IK_Solver(Anim_State *Anim, std::vector<Joint*> &joints, Effector *target)
	: anim(Anim)
{
	// 0th element of chain, should be end site, Current Effector Position
	Eigen::Vector3f chain_end_pos = glmToeig(joints[0]->position);
	// Target Effector Position 
	Eigen::Vector3f target_pos = glmToeig(target->pos);

	// Calc Delta of each Joint Pos - Target Pos
	//std::vector<Eigen::Vector3f> JPdelta; 

	//std::vector<Eigen::Vector3f> JPdelta; 

	for (Joint *joint : joints)
	{
		// Calculate delta vectors (Joint Position Current - Target Effector Position) 
		Eigen::Vector3f jp = glmToeig(joint->position);
		Eigen::Vector3f delta = jp - target_pos; 

		// Get Joint Axis of rotation
		glm::vec3 joint_dof3 = anim->bvh->get_joint_DOF3(joint->idx, anim->anim_frame);
		Eigen::Vector3f joint_axis = glmToeig(glm::normalize(joint_dof3));

		std::cout << "Joint_" << joint->idx << " Axis = [" << joint_axis.x() << "," << joint_axis.y() << "," << joint_axis.z() << "]\n";
	}
}




// ========== Static Member Functions ==========
Eigen::Vector3f IK_Solver::glmToeig(const glm::vec3 &vec)
{
	return Eigen::Vector3f(vec.x, vec.y, vec.z);
}

Eigen::Vector4f IK_Solver::glmToeig(const glm::vec4 &vec)
{
	return Eigen::Vector4f(vec.x, vec.y, vec.z, vec.w);
}