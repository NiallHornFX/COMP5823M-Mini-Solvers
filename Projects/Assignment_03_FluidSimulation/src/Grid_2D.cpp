// COMP5823M - A3 : Niall Horn - grid_2d.cpp
// Implements
#include "grid_2d.h"

// Project Headers
#include "fluid_object.h"

Grid_2D::Grid_2D(Fluid_Object *FluidData, float CellSize, float WsSize)
	: fluid_data(FluidData), cell_size(CellSize), ws_size(WsSize)
{
	// Cells Per Dimension
	cell_dim = static_cast<std::size_t>(ws_size / cell_size);
	r_cell_dim = 1.f / float(cell_dim - 1);
	// Total Cell Count
	cell_count = cell_dim * cell_dim;
	// Cell Extents
	cell_ext = ws_size / float(cell_dim);
	cell_ext_sqr = cell_ext * cell_ext;
	h_cell_ext = 0.5f * cell_ext;
	h_cell_ext_sqr = h_cell_ext * h_cell_ext; 

	// Debug
	std::cout << "Cell Dim = " << cell_dim << "  Cell Count = " << cell_count << "  Cell Ext = " << cell_ext << "\n";

	// Allocate Grid Fields
	cell_pts = new std::vector<Particle*>[cell_count]; // Per Cell, list of particles within.
}

Grid_2D::~Grid_2D()
{
	delete[] cell_pts;
}

// Info : Transform Grid into WS, gather particles per cell if they lie within cell bounds O(n). 
void Grid_2D::gather_particles()
{
	// (2D) Square Distance Between 2 position vectors (3D). 
	auto square_dist = [](const glm::vec3 &A, const glm::vec3 &B) -> float
	{ 
		float xx = B.x - A.x, yy = B.y - A.y; 
		return (xx*xx) + (yy*yy);
	};

	// Loop over 2D Grid
	for (std::size_t i = 0; i < cell_dim; ++i)
	{
		for (std::size_t j = 0; j < cell_dim; ++j)
		{
			// 1D Index
			const std::size_t idx_1d = idx_2Dto1D(i, j);

			// Index --> Grid Space
			float gs_x = float(i) * r_cell_dim, gs_y = float(j) * r_cell_dim;

			// Grid --> World Space
			float ws_x = gs_x * ws_size, ws_y = gs_y * ws_size; // Node Position
			// Get Cell Min Max Node Positions
			float eps = cell_ext * 0.25f;
			glm::vec3 min(ws_x, ws_y, 0.f); 
			glm::vec3 max(ws_x + (cell_ext + eps), ws_y + (cell_ext + eps), 0.f); 

			// Gather Particles if within cell bounds
			for (std::size_t p = 0; p < fluid_data->particles.size(); ++p)
			{
				Particle &pt = fluid_data->particles[p];
				if (pt.P.x > min.x && pt.P.x < max.x && pt.P.y > min.y && pt.P.y < max.y)
				{
					cell_pts[idx_1d].push_back(&pt); // Append particle to cell list.
					pt.cell_idx = idx_1d; // Store 1D index. 
				}
			}
		}
	}
}

// Info : Return concatenated list of particles within current and adjacent cells of pt
// |------|------|------|
// |   *  |  *   |  *   |
// |   *  |  x   |  *   |
// |   *  |  *   |  *   |
// |------|------|------|
std::vector<Particle*> Grid_2D::get_adjcell_particles(const Particle &pt) const
{
	// Retrive particles cell index
	glm::ivec2 idx_c = idx_1Dto2D(pt.cell_idx);

	// Index Offsets from current cell (could do this on 1D indices instead)
	std::size_t adj_cells [8] = {
	//            (-i, j)             |            (+i, j)
	idx_2Dto1D(idx_c.x-1, idx_c.y)       ,idx_2Dto1D(idx_c.x+1, idx_c.y),
	//            (i, -j)             |            (i, +j) 
	idx_2Dto1D(idx_c.x, idx_c.y-1)       ,idx_2Dto1D(idx_c.x, idx_c.y+1),
	//           (-i, +j)             |            (+i, -j)
	idx_2Dto1D(idx_c.x-1, idx_c.y+1)     ,idx_2Dto1D(idx_c.x+1, idx_c.y-1),
	//           (-i, -j)             |            (+i, +j)
	idx_2Dto1D(idx_c.x-1, idx_c.y-1)     ,idx_2Dto1D(idx_c.x+1, idx_c.y+1)};

	// Check if cell indices out of bounds if not store into concat'd particle array.
	// First also add particles of pt's own cell. 
	std::vector<Particle*> concat(cell_pts[pt.cell_idx]);
	for (std::size_t c = 0; c < 8; ++c)
	{
		std::cout << adj_cells[c] << "\n";
		// If cell is out of bounds, skip. 
		if (adj_cells[c] > (cell_count - 1)) continue;
		// Else concat cell particle list 
		concat.insert(concat.end(), cell_pts[adj_cells[c]].begin(), cell_pts[adj_cells[c]].end());
	}

	return concat; 
}

// Info : Map 2D cell index to 1D flat array index. 
std::size_t Grid_2D::idx_2Dto1D(std::size_t i, std::size_t j) const
{
	return i * cell_dim + j;
}

// Info : Map 1D flat index to 2D cell index. 
glm::ivec2 Grid_2D::idx_1Dto2D(std::size_t i) const
{
	return glm::ivec2(i % cell_dim, i / cell_dim);
}