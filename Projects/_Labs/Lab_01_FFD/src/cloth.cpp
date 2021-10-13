#include "cloth.h"

#include "vec3.h"
#include "spring.h"

#include <cassert>
#include <algorithm>

cloth::cloth(std::size_t Nx, std::size_t Ny, const real dCoeff, const vec3<real> &sCoeff)
	: nx(Nx), ny(Ny), damp_coeff(dCoeff)
{
	set_particles();
	set_springs(sCoeff);
}

cloth::~cloth()
{
	// Delete Springs
	for (spring *s : springs)
	{
		delete s;
	}
	springs.clear();
}

void cloth::set_particles()
{
	std::size_t pt_N = nx; // Assume N*N Square
	real r = 1.0f / (pt_N-1.0f);

	for (std::size_t j = 0; j < pt_N; ++j)
	{
		for (std::size_t i = 0; i < pt_N; ++i)
		{
			vec3<real> P((float)i * r, (float)j * r, 0.0f);
			std::size_t idx_1d = i + pt_N * j; 
			//if ((i == 0 && j == (pt_N - 1)) || (i == (pt_N - 1) && j == (pt_N - 1))) // Bound to Top Left + Right Corner. 
			if ((i == 0 && j == (pt_N - 1)) || (i == (pt_N - 1) && j == (pt_N - 1)) || (i == 0 && j == 0) || (i == (pt_N-1) && j == 0)) // Bound All Corners. 
			//if (i == 0 || i == (pt_N - 1)) // Bound Left and Right Edge. 
			{
				p_list.emplace_back(P, vec3<std::size_t>(i, j, 0), idx_1d, particle::FIXED);
			}
			else
			{
				p_list.emplace_back(P, vec3<std::size_t>(i, j, 0), idx_1d, particle::FREE);
			}
		}
	}
}

// Set Particle Springs Along Each Axis (0 to N-1).
void cloth::set_springs(const vec3<real> &sCoeff)
{
	std::size_t NN = (pt_N * pt_N) - 1; // Max 1D Pt Index. 

	for (std::size_t j = 0; j < pt_N; ++j)
	{
		for (std::size_t i = 0; i < pt_N; ++i)
		{
			std::size_t idx_c = i + pt_N * j; // Cur Pt 1D IDX. 
			// Oppose to checking out of bounds on 1D Index, use Sepreate i,j 2D Components per dim. To Avoid Edge i+1,j+1 next row cases. 

			// Struct/Edge Springs 
			std::size_t idx_ii = (i + 1) + pt_N * j, idx_jj = (i + pt_N * (j + 1)); // Main 2D Struct Indices. 
			if (i + 1 <= (pt_N - 1)) springs.push_back(new spring(&p_list.at(idx_c), &p_list.at(idx_ii), sCoeff.x, damp_coeff, STRUCT_SPRING)); // (i,j)|(i+1,j)
			if (j + 1 <= (pt_N - 1)) springs.push_back(new spring(&p_list.at(idx_c), &p_list.at(idx_jj), sCoeff.x, damp_coeff, STRUCT_SPRING)); // (i,j)|(i,j+1)

			// Shear/Diagonal Springs
			std::size_t idx_sa = (i + 1) + pt_N * (j + 1); std::size_t idx_sb = (i - 1) + pt_N * (j + 1);
			if (i + 1 <= (pt_N - 1) && j + 1 <= (pt_N - 1))springs.push_back(new spring(&p_list.at(idx_c), &p_list.at(idx_sa), sCoeff.y, damp_coeff, SHEAR_SPRING)); // (i,j)|(i+1,j+1)
			if (i - 1 <= (pt_N - 1) && j + 1 <= (pt_N - 1))springs.push_back(new spring(&p_list.at(idx_c), &p_list.at(idx_sb), sCoeff.y, damp_coeff, SHEAR_SPRING)); // (i,j)|(i-1,j+1)

			// Bend Spring
			std::size_t idx_ba = (i + 2) + pt_N * j; std::size_t idx_bb = i + pt_N * (j + 2);
			if (i+2 <= (pt_N - 1))springs.push_back(new spring(&p_list.at(idx_c), &p_list.at(idx_ba), sCoeff.y, damp_coeff, BEND_SPRING)); // (i,j)|(i+2,j)
			if (j+2 <= (pt_N - 1))springs.push_back(new spring(&p_list.at(idx_c), &p_list.at(idx_bb), sCoeff.y, damp_coeff, BEND_SPRING)); // (i,j)|(i,j+2)	
		}
	}
}

// Get Particle Postions and Normal For GL Vertices - 
real* cloth::get_ptVertexAttribs()
{
	// Calc Normals Of Cur Frame - (Pass Adjacent Pts)
	for (std::size_t j = 0; j < pt_N; ++j)
	{
		for (std::size_t i = 0; i < pt_N; ++i)
		{
			// 1D Indices, Cur(i,j) | (i+1,j) | (i,j+1)
			std::size_t idx_c = i + pt_N * j;
			std::size_t idx_ii, idx_jj;

			// Awful lot of branching, but fine for now. !TODO - Need to Handle Edge Points More Correctly. 
			if (j == (pt_N - 1)) // Top Row
			{
				if (i == (pt_N - 1)) idx_ii = idx_c - 1; else idx_ii = idx_c + 1;  // Last TopRight pt No i+. 
				idx_jj = idx_c - pt_N;
				p_list.at(idx_c).calc_normal(p_list.at(idx_ii), p_list.at(idx_jj));
			}
			else if (idx_c == 0)
			{
				idx_ii = idx_c + 1;
				idx_jj = idx_c + pt_N;
				p_list.at(idx_c).calc_normal(p_list.at(idx_ii), p_list.at(idx_jj));
			}
			else
			{
				idx_ii = idx_c + 1;
				idx_jj = idx_c + pt_N;
				p_list.at(idx_c).calc_normal(p_list.at(idx_ii), p_list.at(idx_jj));
			}
		}
	}

	// Pack into VAttr Array -
	// P.x , P.y, P.z | N.x , N.y , N.z
	real *vatr = new real[p_list.size() * 6]; // Passed to Display Per Frame, whom will be responsible for deletion when copied to GL VBO.
	for (std::size_t i = 0, j = 5; i < p_list.size(); i++, j+=6)
	{
		// Attr 1 (N)
		vatr[j] = p_list[i].n.z;
		vatr[j-1] = p_list[i].n.y;
		vatr[j-2] = p_list[i].n.x;
		// Attr 0 (P)
		vatr[j-3] = p_list[i].p.z;
		vatr[j-4] = p_list[i].p.y;
		vatr[j-5] = p_list[i].p.x;
	}
	return vatr; 
}


// Get PList/Vertex Indices. Vertices ^ Updated Per Frame, Indices Not. 
uint* cloth::get_ptVertexIndices()
{
	std::size_t quad_c = (pt_N - 1) * (pt_N - 1); std::size_t tri_c = quad_c * 2; 
	std::size_t indices_c = tri_c * 3; 
	std::vector<uint> indices;

	auto idx_2Dto1D = [&](std::size_t i, std::size_t j) -> std::size_t 
	{
		return i + pt_N * j; 
	};

	// Loop Through Pts -1 in each dim, (0 - (pt_N-2) range) thus excluding edge points. Each Point Sets 2 TriFaces pt_N-2 * pt_N-2 triangles
	// (2D Indices 2Tris Per Quad Face) - ij, ij+1, i+1j+1 | ij, i+1j+1, i+1j
	for (std::size_t j = 0; j < (pt_N - 1); ++j)
	{
		for (std::size_t i = 0; i < (pt_N - 1); ++i)
		{
			// TriFace 0 
			indices.push_back(idx_2Dto1D(i, j)); 
			indices.push_back(idx_2Dto1D(i, j + 1)); 
			indices.push_back(idx_2Dto1D(i + 1, j+1)); 

			// TriFace 1 
			indices.push_back(idx_2Dto1D(i, j)); 
			indices.push_back(idx_2Dto1D(i + 1, j + 1)); 
			indices.push_back(idx_2Dto1D(i + 1, j));
		}
	}

	// Should always be true.
	assert(indices_c == indices.size()); 
	uint *indices_r = new uint[indices.size()];
	std::memcpy(indices_r, indices.data(), indices.size() * sizeof(uint));

	return indices_r; 
}
