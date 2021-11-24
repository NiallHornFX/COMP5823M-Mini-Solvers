// Implements
#include "bone.h"

// Std Headers
// Project Headers

// Maybe would of made more sense as seperate class not mesh derived. 

Bone::Bone(glm::vec3 Start, glm::vec3 End, std::size_t ID)
	: Mesh("Bone", "../../assets/mesh/bone.obj"), start(Start), end(End), bone_id(ID)
{
	// Bone Mesh
	load_obj(false);
	set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	set_colour(glm::vec3(0.1f, 0.1f, 0.6f));
	mode = Render_Mode::RENDER_MESH;
	// Translate to offset center
	glm::vec3 cent = (start + end) * 0.5f;
	translate(cent);

	// Name
	name += "_" + std::to_string(ID);

	// Bone also has 2 point based line primitive for rendering. 
	std::vector<vert> line_data(2);
	line_data[0].pos = start; 
	line_data[1].pos = end; 
	line_data[0].col = glm::vec3(0, 0, 1.f);
	line_data[1].col = glm::vec3(0, 0, 1.f);
	line->set_data_mesh(line_data);
	line->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	line->mode = Render_Mode::RENDER_LINES;
}

void Bone::render()
{
	// Check for state to render
	if (!check_state())
	{
		std::cerr << "ERROR::Mesh::" << name << "::Render called, with incorrectly set state." << std::endl;
		std::terminate();
	}

	// Bind Primitive State
	shader.use();

	// Update Modfied Uniforms
	shader.setMat4("model", model);

	// Activate and Bind Texture
	if (use_tex)
	{
		tex->activate();
		tex->bind();
	}

	// Draw 
	glBindVertexArray(VAO);
	switch (mode)
	{
		case (RENDER_POINTS):
		{
			// Uses Primtive Line Rendering instead
			line->mode = Render_Mode::RENDER_POINTS;
			line->render();
		}
		case (RENDER_LINES):
		{
			// Uses Primtive Line Rendering instead
			line->mode = Render_Mode::RENDER_LINES;
			line->render();
		}
		case (RENDER_MESH):
		{
			// Render bone as mesh
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glDrawArrays(GL_TRIANGLES, 0, vert_count);
			break;
		}
	}

	// Clear State
	glUseProgram(0);
	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D, 0);
}