// ===================== Joint Perturbation =====================
// For construction of Jacobian Matrix, relating joint angles to end effector. We need (P2 - P1 / Dtheta) 
// Where P1 is the orginal non-perturbed postion of the end effector, P2 is the perturbed postion, for each joint, DOF
// which defines a single column of the Jacobian Matrix. (Thus the Jacobian is 3 x (j * DOF), which we know is 3 x (j * 3)). 
// as joints have 3 rotational DOFs as each Joint would define 3 Cols (one for each DOF). 

// Both start_joint + end_effec should be within the joint chain been evaulated. 
// chain          - joint chain to get perturbed postions for. 
// end_effc       - end_site joint that defined end effector
// start_joint    - start traversal along chain from this joint, (typically root joint or first joint along chain rel to root).
// perturb_factor - factor to perturb each joint's DOF angle by. 
std::vector<std::pair<glm::vec3, glm::vec3>> Anim_State::perturb_joints(std::vector<Joint*> &chain, Joint *end_effec, Joint *start_joint, float perturb_factor)
{
	std::size_t dof_c = chain.size() * 3;    // Number of Columns (theta_0 ... theta_n) (size = j * 3) 
	glm::vec3 end_pos = end_effec->position; // We know number of rows is (size = 3) (3 DOFs in end effector) pos (P_x, P_y, P_z)

	// Vector to store Orginal and preturbed postions of end effector, for each perturbed joint (for all rot DOFs). 
	// (Defines single col of Jacobian in the form of P2 - P1 / Dtheta)
	std::vector<std::pair<glm::vec3, glm::vec3>> pertrub_pos(dof_c, std::pair<glm::vec3, glm::vec3>(glm::vec3(0.f), glm::vec3(0.f)));
	

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

			glm::vec3 col(0.f); // Column vector init
			glm::vec3 P1 = end_effec->position; // Un-perturbed end effector postion (x,y,z) 
			glm::vec3 P2(0.f); // Init P2

			float delta_theta = 0.001f; 

			// The tbd function that traverses the joint from start, and when we reach the preturb joint, we perturb its DOF. 
			// We contiune accumulating the resulting transform along the chain and store the resulting end effector / end joint position. 
			// Currently I'm using a recrusive traversal approach, but iterative may be perfered as we are only traversing the chain. 
			perturb_traverse(start_joint, perturb_joint, DOF, delta_theta, glm::mat4(1.f));

			// Query Resulting Modified End Effector Postion component
			P2 = end_effec->position;

			// Now P1 defines orginal effector pos, P2 defines perturbed effector pos, for the current DOF (rotational channel).
			pertrub_pos.push_back(std::pair<glm::vec3, glm::vec3>(P1, P2));
		}
	}
}