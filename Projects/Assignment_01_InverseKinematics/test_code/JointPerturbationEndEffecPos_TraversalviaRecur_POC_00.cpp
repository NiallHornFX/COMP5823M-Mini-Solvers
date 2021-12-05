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