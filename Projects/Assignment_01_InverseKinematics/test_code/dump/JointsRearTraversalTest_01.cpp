	// Rereverse traversal approach (not optimal as it means traversal back to parent is needed per joint)
	for (Joint *cur : bvh->joints)
	{
		// Get Parent Offset
		glm::vec3 par_offs(0.f);
		// Get Parent Transform
		glm::mat4 rot(1.f);

		Joint *p = cur;
		while (p->parent)
		{
			// Accumulate offset
			par_offs += p->parent->offset;

			// Accumulate Channel Transform
			// Non root joints (3DOF, joint angles only).
			if (!p->parent->is_root)
			{
				// Get Angles from motion data of current frame
				DOF3 angs = bvh->get_joint_DOF3(p->parent->idx, anim_frame);
				// Build Local Matrix to multiply accumlated with 
				glm::mat4 tmp(1.f);
				// Z Rotation 
				glm::rotate(tmp, float(std::get<0>(angs)), glm::vec3(0.f, 0.f, 1.f));
				// Y Rotation 
				glm::rotate(tmp, float(std::get<1>(angs)), glm::vec3(0.f, 1.f, 0.f));
				// X Rotation 
				glm::rotate(tmp, float(std::get<2>(angs)), glm::vec3(1.f, 0.f, 0.f));
				// Accumlate Rotation 
				rot *= tmp;
			}

			// Traverse up to parent 
			p = p->parent;
		}
		// Start is then parent offset, end is the current joint offset + parent offset (total parent offset along tree).
		skel.add_bone(par_offs, (cur->offset + par_offs), rot);
	}
