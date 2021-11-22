// Implements
#include "mesh.h"

// Std Headers
#include <fstream>
#include <sstream>

Mesh::Mesh(const char *name, const char *filePath)
	: Primitive(name), file_path(filePath)
{
	
}

Mesh::~Mesh()
{

}

void Mesh::load_obj()
{
	std::ifstream in(file_path);
	if (!in.is_open)
	{
		std::cerr << "ERROR::Mesh::" << name << ":: Invalid .obj file passed" << std::endl;
		return;
	}

	std::string line;
	while (std::getline(in, line))
	{
		// Extract first block as str. 
		std::string str;
		std::istringstream ss(line);
		ss >> str;

		// Vertex Postion
		if (str == "v")
		{
			vec3<float> vp;
			ss >> vp.x;
			ss >> vp.y;
			ss >> vp.z;
			v_pos.push_back(vp);
		}

		// Vertex Normal 
		if (str == "vn")
		{
			vec3<float> vn;
			ss >> vn.x;
			ss >> vn.y;
			ss >> vn.z;
			v_n.push_back(vn);
		}
		// Faces and Indices 
		if (str == "f")
		{
			// Vert Postion & Normal Indices. 
			int vpos_v0, vpos_v1, vpos_v2, vn_v0, vn_v1, vn_v2;
			char c; // For Scratch "//" Between Faces. 
			ss >> vpos_v0; ss >> c; ss >> c; ss >> vn_v0; v_idx.push_back(vec2<int>(vpos_v0, vn_v0));
			ss >> vpos_v1; ss >> c; ss >> c; ss >> vn_v1; v_idx.push_back(vec2<int>(vpos_v1, vn_v1));
			ss >> vpos_v2; ss >> c; ss >> c; ss >> vn_v2; v_idx.push_back(vec2<int>(vpos_v2, vn_v2));

			// Set Vertices 
			vertex v0, v1, v2;
			// Get Vertex Pos and Normals from Indcies (i-1)
			vec3<float> v0_pos = v_pos[vpos_v0 - 1]; vec3<float> v0_n = v_n[vn_v0 - 1];
			vec3<float> v1_pos = v_pos[vpos_v1 - 1]; vec3<float> v1_n = v_n[vn_v0 - 1];
			vec3<float> v2_pos = v_pos[vpos_v2 - 1]; vec3<float> v2_n = v_n[vn_v0 - 1];
			// Per Vertex Vector (Non Unique)
			v0.pos = v0_pos; v0.normal = v0_n; verts.push_back(std::move(v0));
			v1.pos = v1_pos; v1.normal = v1_n; verts.push_back(std::move(v1));
			v2.pos = v2_pos; v2.normal = v2_n; verts.push_back(std::move(v2));

			// Face Triangle (Mesh Vector) - 
			mesh.push_back(new triangle(v0, v1, v2, mat));
		}

	}
}