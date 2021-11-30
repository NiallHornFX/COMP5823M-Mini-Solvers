// Bone single Joint point test

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
	std::vector<vert> line_data; line_data.resize(1);
	line_data[0].col = glm::vec3(0, 0, 1.f);

	// Test single joint point apply rot matrix.
	glm::vec4 pos(0.f, 0.f, 0.f, 1.f);
	// Add Translation to Transform Matrix
	Trs = glm::translate(Trs, start);
	// Apply Transform Matrix to point
	pos = Trs * pos; 
	line_data[0].pos = glm::vec3(pos);

	/*
	std::vector<vert> line_data; line_data.resize(2);
	// Create Segment from start,end, transform
	float dist = glm::length(end - start);
	glm::vec4 v0(start, 1.f);
	glm::vec4 v1(end,   1.f);
	glm::vec4 tmp = v1;
	v0 -= start;
	v1 -= start;
	// Apply Transform to segment.
	v0 = Trs * v0;
	v1 = Trs * v1;
	v0 += tmp;
	v1 += tmp;
	line_data[0].pos = glm::vec3(v0); 
	line_data[1].pos = glm::vec3(v1); 
	//line_data[0].pos = start; 
	//line_data[1].pos = end; 

	line_data[0].col = glm::vec3(0, 0, 1.f);
	line_data[1].col = glm::vec3(0, 0, 1.f);
	*/

	// Joint only Point Test 

	line->set_data_mesh(line_data);
	line->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	//line->mode = Render_Mode::RENDER_LINES;
}