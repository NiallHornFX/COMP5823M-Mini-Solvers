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

	std::cout << "DEBUG::Hash_Grid::Cell_Count = " << cell_count << " | Cell_Size = " << cell_size << " | Dimensions " << dim_sqr << "^2\n";

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

// Info : Return (upto) 8 neighbouring cells, particles of current particle along with self cell particles.
std::vector<Particle*> Hash_Grid::get_adjacent_cells(const Particle &pt) const
{
	// Store adj cell particle lists 
	std::vector<std::vector<Particle*>*> AdjCells(8, nullptr);

	const glm::vec3 &PtPos = pt.P;
	// Pos Offsets from Current Particle
	float cell_eps = cell_size + 1e-02f; 
	// (-x, y) | (+x, y)
	glm::vec2 x_n(PtPos.x - cell_eps, PtPos.y); glm::vec2 x_p(PtPos.x + cell_eps, PtPos.y);
	// (x, -y) | (x, +y) | 
	glm::vec2 y_n(PtPos.x, PtPos.y - cell_eps); glm::vec2 y_p(PtPos.x, PtPos.y + cell_eps);
	// (-x, +y) | (+x, -y)
	glm::vec2 nx_py(PtPos.x - cell_eps, PtPos.y + cell_eps); glm::vec2 px_ny(PtPos.x + cell_eps, PtPos.y - cell_eps);
	// (-x, -y) | (+x, +y)
	glm::vec2 nx_ny(PtPos.x - cell_eps, PtPos.y - cell_eps); glm::vec2 px_py(PtPos.x + cell_eps, PtPos.y + cell_eps);

	// Hash Indices into tmp array; 
	std::size_t idx_arr[8] = { hash_pos(x_n), hash_pos(x_p), hash_pos(y_n), hash_pos(y_p), hash_pos(nx_py), hash_pos(px_ny), hash_pos(nx_ny), hash_pos(px_py) };
	// Check if out of bounds (to prevent idx wrap-around) if not store into concat'd particle array if not null ptr. 
	std::vector<Particle*> concat;
	for (std::size_t c = 0; c < 8; ++c)
	{
		// Hashed Adj Cell Idx larger than cell count ? Skip. 
		if (idx_arr[c] > (cell_count - 1)) continue; 
		std::cout << idx_arr[c] << "\n";
		std::vector<Particle*> *cell_list = grid[idx_arr[c]];
		// Adj Cell List null ? Skip.
		if (cell_list) concat.insert(concat.end(), cell_list->begin(), cell_list->end());
	}
	// Also add pt's own cell particle list to concated array. 	
	std::vector<Particle*> *selfcell_pts = grid[pt.cell_idx];
	concat.insert(concat.end(), selfcell_pts->begin(), selfcell_pts->end());

	return concat;
}
