// Implements
#include "cloth_state.h"

// Std Headers
#include <iostream>
#include <sstream>

#define DEBUG_LOG

// =================================== Cloth_State Implementation ===================================

Cloth_State::Cloth_State(const char *path)
	: file_path(path)
{
	// Check OBJ Filepath
	std::ifstream obj_file(path);
	if (!obj_file.is_open())
	{
		std::cerr << "ERROR::File Path Not Valid : " << path << " !\n";
		std::terminate();
	}

	// Load OBJ Mesh
	load_obj(obj_file);
}


// Info : Parsing of .obj to tri-soup based verts.
// We discard vertex attributes so we can define unqiue vertices (and thus particles/point masses) based on postions.
// We will re-calc the normals anyway and UV Coords can be approximated later. 
void Cloth_State::load_obj(std::ifstream &in)
{
	std::stringstream dbg;
	std::string line;
	bool has_n, has_t; 
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
			v_p.emplace_back(xx, yy, zz);
			dbg << "v_" << v_p.size() << " = " << xx << "," << yy << "," << zz << "\n";
		}
		// Info : We will re-calculate other vertex attributes later, so discard these for now.
		//		  But we need to store if they are present for attribute index offset count below. 
		// Vertex Texture
		if (str == "vt")
		{
			has_t |= true; 
			continue;
		}
		// Vertex Normal
		if (str == "vn")
		{
			has_n |= true; 
			continue;
		}
		// Faces, vert indices
		if (str == "f")
		{
			glm::ivec3 tri; // Hold Triangle Indices
			for (int i = 0; i < 3; ++i) // Per Tri Vert-->Pt
			{
				// Vert Postion Index
				int32_t i_vp;

				// Face Vertex Data
				ss >> i_vp;          // 'v_p' index
				ss.get(); ss.get();  // '//'  sep
				ss.get();            // 'v_n' index (discard) 

				// Create Particle (Each Unique Vert Position)
				std::size_t idx = i_vp - 1;
				tri[i] = idx; // Tri Ind

				 // Vert --> Inital Particle Pos 
				glm::vec3 pos = v_p[idx];

				// Append Particle
				particles.emplace_back(pos, particles.size());
				dbg << "Particle : " << particles.size() << "P = " << pos.x << "," << pos.y << "," << pos.z << "\n";
			}
			// Append Triangle
			tri_inds.push_back(std::move(tri));
		}
	} 

	// Close file
	in.close();

	// Debug Output
	dbg << "Indexed Vert / Particle Count = " << particles.size() << "\n";

#ifdef DEBUG_LOG
	std::cout << dbg.str() << std::endl;
#endif
}