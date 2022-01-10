// COMP5823M - A3 : Niall Horn - grid_2d.h
#ifndef GRID_2D_H
#define GRID_2D_H

// Std Headers
#include <vector>

// Ext Headers 
#include "ext/glm/glm.hpp" // GLM

// FD
struct Particle;
class Fluid_Object;

#define INLINE __forceinline 

// 2D Uniform Grid, stored in flat 1D Arrays. 
// get neighbours (cell idx) std::tuple<i-1, i+1, j-1, j+1> 


class Grid_2D
{
public:
	Grid_2D(Fluid_Object *FluidData, float CellSize, float WsSize);
	~Grid_2D(); 

	// Grid Functions
	void gather_particles(); 

	std::vector<Particle*> get_adjcell_particles(const Particle &pt) const;

	// Index Functions
	INLINE std::size_t idx_2Dto1D(std::size_t i, std::size_t j) const;
	INLINE glm::ivec2  idx_1Dto2D(std::size_t i)                const;

private:
	Fluid_Object *fluid_data; 
	float cell_size, ws_size, r_cell_dim;
	float cell_ext, cell_ext_sqr, h_cell_ext, h_cell_ext_sqr; // extent : cell size in WS. 
	std::size_t cell_count, cell_dim; 
	std::vector<Particle*> *cell_pts; 

};

#endif 