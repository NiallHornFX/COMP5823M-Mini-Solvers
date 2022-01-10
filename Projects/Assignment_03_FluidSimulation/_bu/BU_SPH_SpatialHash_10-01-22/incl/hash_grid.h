// COMP5823M - A3 : Niall Horn - hash_grid.h

#ifndef HASH_GRID_H
#define HASH_GRID_H

// Std Headers
#include <vector>
#include <tuple>

// Ext Headers 
#include "ext/glm/glm.hpp" // GLM

// FD
struct Particle; 
class Fluid_Object; 

#define INLINE __forceinline 

// =================================== Hash Grid Class ===================================
// Info : Simple 2D Spatial Hash Grid class, user specified dimensions and cell size. 
//        Only allocates non empty cells (saves memory, but is fragmented) 

class Hash_Grid
{
public:
	Hash_Grid(Fluid_Object *FluidData, std::size_t DimSqr, float CellSize);
	~Hash_Grid(); 

	void hash();

	std::vector<Particle*> get_adjacent_cells(const Particle &pt) const;

	INLINE std::size_t hash_pos(const glm::vec2 &PtPos) const;

public: 
	Fluid_Object *fluidData; 
	float cell_size, r_cell_size; 
	std::size_t dim_sqr, cell_count; 

	std::vector<Particle*> **grid; 
};

#endif
