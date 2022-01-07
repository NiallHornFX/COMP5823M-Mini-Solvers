// COMP5823M - A3 : Niall Horn - hash_grid.cpp
// Implements
#include "hash_grid.h"

// Std Headers
#include <cmath>

// Project Headers
#include "fluid_object.h"

Hash_Grid::Hash_Grid(Fluid_Object *FluidData, std::size_t DimSqr, float CellSize)
	: fluidData(FluidData), dim_sqr(DimSqr), cell_size(CellSize)
{
	r_cell_size = 1.f / cell_size;
	
	// Total x,y Cell Count, assumed dim_sqr is the dimension squared of the HG. 
	cell_count = std::powf((dim_sqr / cell_size), 2.f); 

	//std::cout << "DEBUG::Hash_Grid::Cell_Count = " << cell_count << " | Cell_Size = " << cell_size << " | Dimensions " << dim_sqr << "^2\n";

	// Allocate Outer Hash Grid Array (Pointer to inner cell list vectors) 
	grid = new std::vector<Particle*>*[cell_count];
	// Initalize Cell Vector Ptrs
	for (std::size_t c = 0; c < cell_count; ++c) grid[c] = nullptr; 

}

Hash_Grid::~Hash_Grid()
{
	// Dealloc Inner Cell Vectors
	for (std::size_t c = 0; c < cell_count; ++c) if (grid[c]) delete grid[c];
	// Dealloc Outer Array 
	delete[] grid; 
}

// Info : 2D Spatial Hash, based on [Optimized Spatial Hashing for Collision Detection of Deformable Objects, Teschner.M et al]
std::size_t Hash_Grid::hash_pos(const glm::vec2 &PtPos) const
{
	std::size_t xx = static_cast<std::size_t>(PtPos.x * r_cell_size);
	std::size_t yy = static_cast<std::size_t>(PtPos.y * r_cell_size);
	std::size_t c = 73856093 * xx ^ 19349663 * yy;
	return c % cell_count;
}

// Info : Hash Particle Positions to grid cells. 
void Hash_Grid::hash()
{
	for (std::size_t p = 0; p < fluidData->particles.size(); ++p)
	{
		Particle &Pt = fluidData->particles[p]; 
		std::size_t h_idx = hash_pos(glm::vec2(Pt.P.x, Pt.P.y));

		// Allocate Particle List (vector<particle*>) if not alloc'd for cell
		if (!grid[h_idx]) grid[h_idx] = new std::vector<Particle*>; 

		// Append Particle Ptr to Cell Index List 
		grid[h_idx]->push_back(&Pt);

		// Store Cell Index on particle
		Pt.cell_idx = h_idx; 
	}
}


