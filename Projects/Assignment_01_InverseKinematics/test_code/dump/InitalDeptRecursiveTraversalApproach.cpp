// Following BVH Sample file style recursion
void Anim_State::build_test(Joint *joint)
{
	static std::size_t call = 0;
	// 
	if (!joint->parent)
	{
		skel.add_bone(glm::vec3(0.f), joint->offset, glm::mat4(1));
		//global = glm::mat4(1.f); // Init
		global_offs += glm::vec3(0.f);
	}
	
	if (joint->parent)
	{
		// Get Joint Channels, Accumulate to global matrix
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				case ChannelEnum::Z_ROTATION : 
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					global = glm::rotate(global, z_r, glm::vec3(0.f, 0.f, 1.f));
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					global = glm::rotate(global, y_r, glm::vec3(0.f, 1.f, 0.f));
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					global = glm::rotate(global, x_r, glm::vec3(1.f, 0.f, 0.f));
				}
			}
		}

		// Accumulate Offset to global matrix
		global_offs += joint->parent->offset;
		//global = glm::translate(global, joint->parent->offset);

		//skel.add_bone(global_offs, global_offs + joint->offset, global);
		skel.add_bone(global_offs, global_offs + joint->offset, glm::mat4(1.f));

		// Cant just use single global offset for all bones, as global offset is not same for all joints...
	}

	// Recurse
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		build_test(joint->children[c]);
	}

	std::cout << "Call Count = " << call << "\n";
	call++;
}