// Implements
#include "cloth_state.h"

// Std Headers
#include <iostream>
#include <sstream>

// =================================== Cloth_State Implementation ===================================

Cloth_State::Cloth_State(const char *path)
	: file_path(path)
{
	// Check OBJ Filepath
	std::ifstream obj_file(path);
	if (!obj_file.is_open)
	{
		std::cerr << "ERROR::File Path Not Valid : " << path << " !\n";
		std::terminate();
	}

	// Load OBJ Mesh
	load_obj(obj_file);
}

Cloth_State::~Cloth_State()
{
	if (particles) delete particles; 
	if (springs) delete springs; 

}

// Info : Parsing of .obj to tri-soup based verts.
void Cloth_State::load_obj(std::ifstream &in)
{
	std::stringstream dbg;
	std::string line;
	while (std::getline(in, line))
	{
		// Extract first block as str.
		std::string str;
		std::istringstream ss(line);
		ss >> str;

		// Skip these lines
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
			dbg << "vn_" << obj_data.v_n.size() << " = " << xx << "," << yy << "," << zz << "\n";
		}
		// Vertex Texture
		if (str == "vt")
		{
			float uu, vv;
			ss >> uu;
			ss >> vv;
			obj_data.v_t.emplace_back(uu, vv);
			dbg << "vt_" << obj_data.v_t.size() << " = " << uu << "," << vv << "," << 0 << "\n";
		}
		// Vertex Colour
		if (str == "vc")
		{
			float r, g, b;
			ss >> r;
			ss >> g;
			ss >> b;
			obj_data.v_c.emplace_back(r, g, b);
			dbg << "vc_" << obj_data.v_c.size() << " = " << r << "," << g << "," << b << "\n";
		}
		// Faces, vert indices
		if (str == "f")
		{
			for (int i = 0; i < 3; ++i) // Per Face Vert
			{
				// Vert Postion & Normal Indices.
				int32_t i_vp, i_vc, i_vn, i_vt;

				// Face Vertex Data
				ss >> i_vp;  // 'v_p' index
				ss.get();
				ss >> i_vc;  // 'v_c' index
				ss.get();
				ss >> i_vt;  // 'v_t' index
				ss.get();
				ss >> i_vn;  // 'v_n' index

				// Create Vertex
				vert vertex;
				vertex.pos = obj_data.v_p[i_vp - 1];
				vertex.normal = obj_data.v_n[i_vn - 1];
				vertex.uv = obj_data.v_t[i_vt - 1];
				vertex.col = obj_data.v_c[i_vc - 1];

				// Append to vert array
				vert_data.push_back(std::move(vertex));

				// dbg
				dbg << i_vp << "/" << i_vc << "/" << i_vt << "/" << i_vn << "  ";
			}
			dbg << "\n";
		}
	} // End file read loop.

	// Close file
	in.close();

	// Debug Output
	dbg << "Tri Soup, Vert Count = " << vert_data.size() << "\n";

#ifdef DEBUG_LOG
	std::cout << dbg.str() << std::endl;
#endif
}