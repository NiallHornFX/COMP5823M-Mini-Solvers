
void Anim_State::build_test_b(Joint *joint, glm::mat4 trs)
{
	static std::size_t call = 0;
	if (!joint->parent) // Must be root (6DOF Channels) (Root translation/offset comes from motion data)
	{
		glm::mat4 xx(1.f), yy(1.f), zz(1.f);
		glm::vec4 root_offs(0.f);
		glm::vec4 child_offs(0.f);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
				case ChannelEnum::X_POSITION:
				{
					float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.x += x_p;
					break;
				}
				case ChannelEnum::Y_POSITION:
				{
					float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.y += y_p;
					break;
				}
				case ChannelEnum::Z_POSITION:
				{
					float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.z += z_p;
					break;
				}
				// Rotation
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::mat4(1.f), glm::radians(z_r), glm::vec3(0.f, 0.f, 1.f));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					yy = glm::rotate(glm::mat4(1.f), glm::radians(y_r), glm::vec3(0.f, 1.f, 0.f));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					xx = glm::rotate(glm::mat4(1.f), glm::radians(x_r), glm::vec3(1.f, 0.f, 0.f));
					break;
				}
			}
		}

		// =========== Rotation + Offset (Translation) --> trs Matrix ===========
		trs = (yy * xx * zz) * glm::mat4(1.f); // Accumulate Rotation in YXZ Order (no parent rotation)

		trs[3] = root_offs; // Translation by joint offset. 

		child_offs = glm::vec4(joint->children[0]->offset, 1.f);

		// Add Bone, Use Rel offsets for start end only.
		glm::vec4 v0 = trs * root_offs; 
		glm::vec4 v1 = trs * child_offs; 

		skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.f));
	}
	else if (joint->parent) // Non root joints, 3DOF
	{
		
		// Get Joint Channels, Accumulate to parent matrix
		glm::mat4 xx(1.f), yy(1.f), zz(1.f);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::mat4(1.f), glm::radians(z_r), glm::vec3(0.f, 0.f, 1.f));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					yy = glm::rotate(glm::mat4(1.f), glm::radians(y_r), glm::vec3(0.f, 1.f, 0.f));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					xx = glm::rotate(glm::mat4(1.f), glm::radians(x_r), glm::vec3(1.f, 0.f, 0.f));
					break;
				}
			}
		}

		// =========== Rotation --> trs Matrix ===========
		//glm::mat4 par_trs = trs; // Parent Joint Transform
		//glm::mat4 child_trs = (yy * xx * zz); // Current (Child) Joint Transform
		
		trs = (yy * xx * zz) * trs; // Combined. 

		glm::vec4 v0(0.f, 0.f, 0.f, 1.f);
		glm::vec4 v1(joint->offset, 1.f);

		v0 += trs[3];
		v1 += trs[3];

		v0 = glm::vec4(v0.x, v0.y, v0.z, 1.f) * trs;
		v1 = glm::vec4(v1.x, v1.y, v1.z, 1.f) * trs;

		// Start+end at Rel Offset
		skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.f));

		// Add offset To Transform Matrix 
		trs[3] += glm::vec4(joint->offset, 1.f); */
	}

	// Pass each recurrsive call its own copy of the current accumulated offset and rot, to then apply to children.
	for (std::size_t c = 0; c < joint->children.size(); ++c) // Recurse for all joint children
	{
		build_test_b(joint->children[c], trs);
	}

	call++;
	//std::cout << "Call Count = " << call << "\n";
}
