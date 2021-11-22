// Implements
#include "mesh.h"

// Std Headers
#include <fstream>
#include <sstream>

#define DEBUG_LOG


Mesh::Mesh(const char *name, const char *filePath)
	: Primitive(name), file_path(filePath)
{
	
}

Mesh::~Mesh()
{

}

// For now mesh is treated as triangle soup. No reuse of shared vertices. 
// Very basic, assumes obj file has tris. 
void Mesh::load_obj(bool has_tex)
{
	// Check file exists
	std::ifstream in(file_path);
	if (!in.is_open())
	{
		std::cerr << "ERROR::Mesh::" << name << ":: Invalid .obj file passed" << std::endl;
		return;
	}

	// Debug Stream
	std::ostringstream dbg; 

	std::string line;
	while (std::getline(in, line))
	{
		// Extract first block as str. 
		std::string str;
		std::istringstream ss(line);
		ss >> str;

		dbg << "DEBUG " << str << "\n";

		if (str == "#" || str == "g" || str == "s") continue;

		// Vertex Postion
		if (str == "v")
		{
			float xx, yy, zz;
			ss >> xx;
			ss >> yy;
			ss >> zz;
			obj_data.v_p.emplace_back(xx, yy, zz);
			dbg << "v_" << obj_data.v_p.size() << " = " << xx << "," << yy << "," << zz << "\n";
		}
		// Vertex Normal 
		if (str == "vn")
		{
			float xx, yy, zz;
			ss >> xx;
			ss >> yy;
			ss >> zz;
			obj_data.v_n.emplace_back(xx, yy, zz);
			dbg << "vn_" << obj_data.v_p.size() << " = " << xx << "," << yy << "," << zz << "\n";
		}
		// Vertex Texture
		if (has_tex)
		{
			// Vertex Texture Coord 
			if (str == "vt")
			{
				float uu, vv; 
				ss >> uu;
				ss >> vv;
				obj_data.v_t.emplace_back(uu, vv);
				dbg << "vt_" << obj_data.v_p.size() << " = " << uu << "," << vv << "\n";
			}
		}
		// Faces / Indices
		if (str == "f")
		{
			if (has_tex) // Get each face vertex (w/ texture coords)
			{
				for (int i = 0; i < 3; ++i)
				{
					// Vert Postion & Normal Indices. 
					int32_t i_vp, i_vn, i_vt;
					char c; // scratch write

					// Face Vertex Data
					ss >> i_vp; // v_p index
					ss >> c; // '/'
					ss >> c; // '/'
					ss >> i_vn; // 'v_n' index
					ss >> c; // '/'
					ss >> c; // '/'
					ss >> i_vt; // 'v_t' index

					// Create Vertex
					vert vertex;
					// neg 1 offset for obj indices '1' based. 
					vertex.pos    = obj_data.v_p[i_vp - 1];
					vertex.normal = obj_data.v_p[i_vn - 1];
					vertex.uv     = obj_data.v_p[i_vt - 1];
					vertex.col    = glm::vec3(1.f, 1.f, 1.f);

					// Append to vert array
					obj_data.verts.push_back(std::move(vertex));
				}
			}
			else // Get each face vertex (wo/ texture coords)
			{
				for (int i = 0; i < 3; ++i)
				{
					// Vert Postion & Normal Indices. 
					int32_t i_vp, i_vn;
					char c; // scratch write

					// Face Vertex Data
					ss >> i_vp; // v_p index
					ss >> c; // '/'
					ss >> c; // '/'
					ss >> i_vn; // 'v_n' index

					// Create Vertex
					vert vertex;
					// neg 1 offset for obj indices '1' based. 
					vertex.pos    = obj_data.v_p[i_vp - 1];
					vertex.normal = obj_data.v_p[i_vn - 1];
					vertex.uv     = glm::vec2(0.f, 0.f);
					vertex.col    = glm::vec3(1.f, 1.f, 1.f);

					// Append to vert array
					obj_data.verts.push_back(std::move(vertex));
				}
			}
		}

	} // End file read loop. 
	// Close 
	in.close();

	dbg << "Vert Count = " << obj_data.verts.size() << "\n";

	// Pass verts to primitive::mesh_data
	//set_data_mesh(obj_data.verts);

	#ifdef DEBUG_LOG
		std::cout << dbg.str();
	#endif
}
