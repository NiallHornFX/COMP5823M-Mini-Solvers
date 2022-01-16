// COMP5823M - A3 : Niall Horn - grid_2d.h
#ifndef Grid_2D_H
#define Grid_2D_H

// Std Headers
#include <vector>

// Ext Headers 
#include "ext/glm/glm.hpp" // GLM

// FD
struct Particle;
class Fluid_Object;

#define INLINE __forceinline 

// Info : 2D Uniform Grid, stored in flat 1D Arrays for rendering. 

class Grid_2D
{
public:
	Grid_2D(const char *Name, Fluid_Object *FluidData, float CellSize, float WsSize);
	~Grid_2D() = default; 

	// Grid Functions
	void gather_particles();

	// Index Functions
	INLINE std::size_t idx_2Dto1D(std::size_t i, std::size_t j) const;
	INLINE glm::ivec2  idx_1Dto2D(std::size_t i)                const;

private:
	// Grid Intrinsics
	float cell_size, ws_size, r_cell_dim;
	float cell_ext, cell_ext_sqr, h_cell_ext, h_cell_ext_sqr; // extent : cell size in WS. 
	std::size_t cell_count, cell_dim;
	std::string name; 

	// Grid Cell Data
	std::vector<float>      cell_dens;
	std::vector<float>      cell_u;
	std::vector<float>      cell_v;

	// Fluid Data Ptr
	Fluid_Object *fluid_data;
};

#endif 