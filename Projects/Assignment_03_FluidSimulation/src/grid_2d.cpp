// COMP5823M - A3 : Niall Horn - grid_2d.cpp
// Implements
#include "Grid_2D.h"

// Project Headers
#include "fluid_object.h"

Grid_2D::Grid_2D(const char *Name, Fluid_Object *FluidData, float CellSize, float WsSize)
	: name(Name), fluid_data(FluidData), cell_size(CellSize), ws_size(WsSize)
{
	// Cells Per Dimension
	cell_dim = std::size_t(ws_size / cell_size);
	r_cell_dim = 1.f / float(cell_dim - 1);
	// Total Cell Count
	cell_count = cell_dim * cell_dim;
	// Cell Extents
	cell_ext = ws_size / float(cell_dim - 1);
	cell_ext_sqr = cell_ext * cell_ext;
	h_cell_ext = 0.5f * cell_ext;
	h_cell_ext_sqr = h_cell_ext * h_cell_ext;

	// Debug
	std::cout << "DBG::Grid2D::" << name << " Cell Dim = " << cell_dim << "  Cell Count = " << cell_count << "  Cell Ext = " << cell_ext << "\n";

	// Allocate Grid Fields
	cell_dens = std::vector<float>(cell_count, 0.f);
	cell_u    = std::vector<float>(cell_count, 0.f);
	cell_v    = std::vector<float>(cell_count, 0.f);
}

// Info : Transform Grid into WS, gather particles per cell if they lie within cell bounds O(n). 
void Grid_2D::gather_particles()
{
	// Reset Grid first
	for (std::size_t i = 0; i < cell_count; ++i)
	{
		cell_dens[i] = 0.f, cell_u[i] = 0.f, cell_v[i] = 0.f; 
	}
	
	// Loop over 2D Grid, cell wise. 
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
			glm::vec3 min(ws_x, ws_y, 0.f);
			glm::vec3 max(ws_x + cell_ext, ws_y + cell_ext, 0.f);

			std::size_t cell_count = 0; 
			// Gather particle quanities if within cell bounds
			for (std::size_t p = 0; p < fluid_data->particles.size(); ++p)
			{
				Particle &pt = fluid_data->particles[p];
				if (pt.P.x >= min.x && pt.P.x <= max.x && pt.P.y >= min.y && pt.P.y <= max.y)
				{
					cell_dens[idx_1d] += pt.density;
					cell_u[idx_1d] += pt.V.x; 
					cell_v[idx_1d] += pt.V.y;
					cell_count++;
				}
			}
			if (!cell_count) continue;
			// Average Velocity 
			float r_c = 1.f / float(cell_count);
			cell_u[idx_1d] *= r_c, cell_v[idx_1d] *= r_c;
		}
	}
}

// Info : Map 2D cell index to 1D flat array index. 
std::size_t Grid_2D::idx_2Dto1D(std::size_t i, std::size_t j) const
{
	return i * cell_dim + j;
}

// Info : Map 1D flat index to 2D cell index. 
glm::ivec2 Grid_2D::idx_1Dto2D(std::size_t i) const
{
	return glm::ivec2(i / cell_dim, i % cell_dim);
}