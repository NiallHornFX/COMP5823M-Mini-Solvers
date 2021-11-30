// Sets Joint Angles for current frame 
void Anim_State::tick()
{
	// Get/Set BVH Data of current frame
	//[..]

	for (Joint *cur : bvh->joints)
	{
		// Get Parent Transform
		glm::mat4 rot(1.f);
		Joint *p = cur;
		while (p->parent)
		{
			// Accumulate Channel Transform
			// Non root joints (3DOF, joint angles only).
			if (!p->parent->is_root)
			{
				// Get Angles from motion data of current frame
				DOF3 angs = bvh->get_joint_DOF3(p->parent->idx, anim_frame);

				// Check Retrived motion channel data
				//std::cout << std::get<0>(angs) << "  " << std::get<1>(angs) << std::get<2>(angs) << "\n";

				// Build Local Matrix to multiply accumlated with 
				glm::mat4 tmp(1.f);
				// Z Rotation 
				tmp = glm::rotate(tmp, float(std::get<0>(angs)), glm::vec3(0.f, 0.f, 1.f));
				// Y Rotation 
				tmp = glm::rotate(tmp, float(std::get<1>(angs)), glm::vec3(0.f, 1.f, 0.f));
				// X Rotation 
				tmp = glm::rotate(tmp, float(std::get<2>(angs)), glm::vec3(1.f, 0.f, 0.f));

				/*
				// Check Matrix
				std::cout << "Rot Matrix Joint : " << cur->idx << " Frame : " << anim_frame << "\n"
					<< rot[0][0] << " " << rot[0][1] << " " << rot[0][2] << " " << rot[0][3] << "\n"
					<< rot[1][0] << " " << rot[1][1] << " " << rot[1][2] << " " << rot[1][3] << "\n"
					<< rot[2][0] << " " << rot[2][1] << " " << rot[2][2] << " " << rot[2][3] << "\n"
					<< rot[3][0] << " " << rot[3][1] << " " << rot[3][2] << " " << rot[3][3] << "\n\n";
				*/

				// Accumlate Rotation 
				rot *= tmp;
			}

			// Traverse up to parent 
			p = p->parent;
		}

		// Testing, terrible approach 
		// Update Bone Transform (this is where mapping joint-bone makes sense).
		// Hacky check all bones for joint_b
		for (Bone &b : skel.bones)
		{
			if (b.joint_ids.second == cur->idx)
			{
				// Invert to LS
				glm::mat4 tmp = b.transform;
				//b.transform *= glm::inverse(b.transform);
				b.transform *= glm::inverse(rot);

				// Apply Transform
				//b.transform *= rot; 
				// Reapply 
				//b.transform *= tmp;
			}
		}


	}

}
