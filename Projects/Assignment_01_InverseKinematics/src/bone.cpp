// Implements
#include "bone.h"

Bone::Bone(glm::vec3 Start, glm::vec3 End, glm::mat4 Trs, size_t ID)
	: start(Start), end(End),  transform(Trs), bone_id(ID)
{
	// Bone Mesh
	mesh = new Mesh("bone_", "../../assets/mesh/bone_test.obj");
	mesh->load_obj(false);
	mesh->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	mesh->set_colour(glm::vec3(0.1f, 0.1f, 0.6f));
	mesh->mode = Render_Mode::RENDER_MESH;

	// Update Name
	std::string tmp_name = mesh->name + std::to_string(ID);
	mesh->name = tmp_name;

	// Bone Line primtivie
	line = new Primitive(tmp_name.c_str());
	std::vector<vert> line_data; line_data.resize(2);
	line_data[0].pos = start; 
	line_data[1].pos = end; 
	line_data[0].col = glm::vec3(0, 0, 1.f);
	line_data[1].col = glm::vec3(0, 0, 1.f);
	line->set_data_mesh(line_data);
	line->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	line->mode = Render_Mode::RENDER_LINES;
}

// Forward to Mesh / Primtivie
void Bone::set_cameraTransform(const glm::mat4x4 &view, const glm::mat4x4 &persp)
{
	mesh->set_cameraTransform(view, persp);
	line->set_cameraTransform(view, persp);
}

void Bone::render(bool Render_Line)
{
	// Set Transform --> model matrix
	mesh->model = transform;
	// For Mesh Set Postion to start.
	//mesh->scale(glm::vec3(1.f, glm::length(end-start), 1.f));
	mesh->translate(start); // Do translation After passing bone mat as model matrix. 

	line->model = transform;

	// Render as Line
	if (Render_Line)
	{
		line->mode = Render_Mode::RENDER_LINES;
		line->render();

		// Also Render Points
		//line->mode = Render_Mode::RENDER_POINTS;
		//line->render();
		return;
	}
	// Render as Mesh
	mesh->render();
}

// Set Joint Indices
void Bone::set_jointIDs(std::size_t joint_a, std::size_t joint_b)
{
	joint_ids.first = joint_a, joint_ids.second = joint_b;
}