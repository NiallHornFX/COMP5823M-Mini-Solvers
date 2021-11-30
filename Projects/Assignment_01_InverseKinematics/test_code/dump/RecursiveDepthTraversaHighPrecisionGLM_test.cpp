void Anim_State::build_test_b(Joint *joint, glm::highp_dmat4x4 trs)
{
	static std::size_t call = 0;
	if (!joint->parent) // Must be root (6DOF Channels) (Root translation/offset comes from motion data)
	{
		glm::highp_dmat4x4 xx(1.), yy(1.), zz(1.);
		glm::highp_dvec4 root_offs(0., 0., 0., 1.);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
			case ChannelEnum::X_POSITION:
			{
				double x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.x += x_p;
				break;
			}
			case ChannelEnum::Y_POSITION:
			{
				double y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.y += y_p;
				break;
			}
			case ChannelEnum::Z_POSITION:
			{
				double z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.z += z_p;
				break;
			}
			// Rotation
			case ChannelEnum::Z_ROTATION:
			{
				double z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				zz = glm::rotate(glm::highp_dmat4x4(1.), glm::radians(z_r), glm::highp_dvec3(0., 0., 1.));
				break;
			}
			case ChannelEnum::Y_ROTATION:
			{
				double y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				yy = glm::rotate(glm::highp_dmat4x4(1.), glm::radians(y_r), glm::highp_dvec3(0., 1., 0.));
				break;
			}
			case ChannelEnum::X_ROTATION:
			{
				double x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
				xx = glm::rotate(glm::highp_dmat4x4(1.), glm::radians(x_r), glm::highp_dvec3(1., 0., 0.));
				break;
			}
			}
		}

		// =========== Rotation + Offset (Translation) --> trs Matrix ===========
	//	trs = (yy * xx * zz) *  glm::highp_dmat4x4(1.f); // Accumulate Rotation in YXZ Order (no parent rotation)
		trs[3] = root_offs; // Translation by joint offset. 

		// Add Bone, Use Rel offsets for start end only.
		//glm::vec4 v0 = trs * glm::vec4(0.f, 0.f, 0.f, 1.f);
		//glm::vec4 v1 = trs * root_offs;
		//skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.f));
	}
	else if (joint->parent) // Non root joints, 3DOF
	{
		// Create First, then add offsets ... 

		if (!joint->is_end)
		{
			glm::highp_dvec4 v0(0., 0., 0., 1);
			glm::highp_dvec4 v1(joint->offset, 1.);

			v0 = trs * v0;
			v1 = trs * v1;

			skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.));
		}
		else
		{
			glm::highp_dvec4 v0(0., 0., 0., 1);
			glm::highp_dvec4 v1(joint->end, 1.);

			v0 = trs * v0;
			v1 = trs * v1;
			skel.add_bone(glm::vec3(v0), glm::vec3(v1), glm::mat4(1.));
		}

		// ADD OFFSETS for Next 

		glm::highp_dmat4x4 xx(1.), yy(1.), zz(1.);
		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				case ChannelEnum::Z_ROTATION:
				{
					double z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					zz = glm::rotate(glm::highp_dmat4x4(1.), glm::radians(z_r), glm::highp_dvec3(0., 0., 1.));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					double y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					yy = glm::rotate(glm::highp_dmat4x4(1.), glm::radians(y_r), glm::highp_dvec3(0., 1., 0.));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					double x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					xx = glm::rotate(glm::highp_dmat4x4(1.), glm::radians(x_r), glm::highp_dvec3(1., 0., 0.));
					break;
				}
			}
		}

		// Add offset To Transform Matrix
		//trs = (yy * xx * zz) * trs; 
		trs[3] += glm::highp_dvec4(joint->offset, 0.f);
	}

	// Pass each recurrsive call its own copy of the current accumulated offset and rot, to then apply to children.
	for (std::size_t c = 0; c < joint->children.size(); ++c) // Recurse for all joint children
	{
		build_test_b(joint->children[c], trs);
	}

	call++;
	//std::cout << "Call Count = " << call << "\n";
}