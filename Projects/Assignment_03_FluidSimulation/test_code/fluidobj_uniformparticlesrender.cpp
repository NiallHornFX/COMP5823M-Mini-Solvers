
void Fluid_Object::render(Render_Type mode, const glm::mat4 &ortho)
{
	if (mode == Render_Type::POINT_VERTS)
	{   // =================== Point Vertices Render ===================

		// Set Proj Mat
		ren_points->shader.setMat4("proj", ortho);

		// Particle Attribs to Vert Data update
		std::vector<glm::vec3> pos, col;
		pos.resize(particles.size()), col.resize(particles.size());
		for (std::size_t p = 0; p < particles.size(); ++p)
		{
			// Particle Colour : 
			switch (particle_colour)
			{
			case Colour_Viz::Velocity: // Colour Particles by Velocity - speed
			{
				float speed = fitRange(glm::length(particles[p].V), 0.f, 5.f, 0.f, 1.f);
				col[p] = lerp_vec(glm::vec3(0.02f, 0.02f, 0.9f), glm::vec3(1.f, 1.f, 1.f), speed);
				break;
			}
			case Colour_Viz::Pressure: // Colour Particles by Pressure 
			{
				col[p] = glm::vec3(fitRange(particles[p].pressure, 0.f, max_pres, 0.f, 1.f));
				break;
			}
			case Colour_Viz::Density: // Colour Particles by Density 
			{
				col[p] = glm::vec3(fitRange(particles[p].density, 0.f, max_dens, 0.f, 1.f));
				break;
			}
			case Colour_Viz::Colour: // Colour Particles by Colour Field
			{
				col[p] = glm::vec3(fitRange(particles[p].cf, min_cf, max_cf, 0.f, 1.f));
				break;
			}
			case Colour_Viz::GridCell: // Colour particles by grid cell. 
			{
				col[p] = randRange(particles[p].cell_idx, 0.f, 1.f);
				break;
			}
			}
			// Update Position
			pos[p] = particles[p].P;
		}

		// Pass updated attribute arrays to Primitve::update_data... 
		ren_points->update_data_position_col(pos, col);

		// Call Render Points as Verts
		ren_points->point_size = spc * 75.f;
		ren_points->render();
	}
	else if (mode == Render_Type::METABALL)
	{ // =================== Fragment Shader Metaballs Render ===================
	
		// Set Uniform Particle Array
		Shader &shad = ren_quad->shader;

		for (std::size_t i = 0; i < particles.size(); ++i)
		{
			const Particle &pt = particles[i];
			glm::vec2 pos_2(pt.P.x, pt.P.y);
			glm::vec2 vel_2(pt.V.x, pt.V.y);

			// Loc String
			std::string pt_str = "pts[" + std::to_string(i) + "].";
			shad.use();
			// Pos
			GLuint pos_loc = glGetUniformLocation(shad.ID, (pt_str + "pos").c_str());
			glUniform2fv(pos_loc, 1, glm::value_ptr(pos_2));
			// Vel
			GLuint vel_loc = glGetUniformLocation(shad.ID, (pt_str + "vel").c_str());
			glUniform2fv(vel_loc, 1, glm::value_ptr(vel_2));
			// Dens
			GLuint dens_loc = glGetUniformLocation(shad.ID, (pt_str + "dens").c_str());
			glUniform1f(dens_loc, pt.density); 
		}
		glUseProgram(0);
		// As Uniform array is fixed size, set current particle count uniform : 
		shad.setInt("pt_count", particles.size());

		// Pass min and max density range
		shad.setFloat("min_dens", min_dens);
		shad.setFloat("max_dens", max_dens);

		// Render
		ren_quad->render();
	}
	
}