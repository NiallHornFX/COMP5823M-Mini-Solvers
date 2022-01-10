
// Info : Return concatenated list of particles within current and adjacent cells of pt
// |------|------|------|
// |   *  |  *   |  *   |
// |   *  |  x   |  *   |
// |   *  |  *   |  *   |
// |------|------|------|
std::vector<Particle*> Spatial_Grid::get_adjcell_particles(const Particle &pt) const
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