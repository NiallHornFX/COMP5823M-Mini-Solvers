// Info : Return (upto) 8 neighbouring cells, particles of current particle along with self cell's
// particles in single vector of pt's neighbours. 
std::vector<Particle*> Hash_Grid::get_adjacent_cells(const Particle &pt) const
{
	const glm::vec3 &PtPos = pt.P;
	// Pos Offsets from Current Particle
	float cell_eps = cell_size; 
	// (-x, y) | (+x, y)
	glm::vec2 x_n(PtPos.x - cell_eps, PtPos.y); glm::vec2 x_p(PtPos.x + cell_eps, PtPos.y);
	// (x, -y) | (x, +y) | 
	glm::vec2 y_n(PtPos.x, PtPos.y - cell_eps); glm::vec2 y_p(PtPos.x, PtPos.y + cell_eps);
	// (-x, +y) | (+x, -y)
	glm::vec2 nx_py(PtPos.x - cell_eps, PtPos.y + cell_eps); glm::vec2 px_ny(PtPos.x + cell_eps, PtPos.y - cell_eps);
	// (-x, -y) | (+x, +y)
	glm::vec2 nx_ny(PtPos.x - cell_eps, PtPos.y - cell_eps); glm::vec2 px_py(PtPos.x + cell_eps, PtPos.y + cell_eps);

	// Hash Pos offsets to cell indices.
	std::vector<std::size_t> idx_arr = { hash_pos(x_n), hash_pos(x_p), hash_pos(y_n), hash_pos(y_p), hash_pos(nx_py), hash_pos(px_ny), hash_pos(nx_ny), hash_pos(px_py) };
	// Remove Duplicate Indices 
	for (std::size_t i = 0; i < idx_arr.size(); ++i)
	{
		for (std::size_t j = i+1; j < idx_arr.size(); ++j) if (idx_arr[j] == idx_arr[i]) idx_arr.erase(idx_arr.begin() + j);
	}

	// Check if cell indices out of bounds (to prevent idx wrap-around), if not store into concat'd particle array if not null. 
	std::vector<Particle*> concat;
	for (std::size_t c = 0; c < 8; ++c)
	{
		// Hashed Adj Cell Idx larger than cell count ? Skip. 
		if (idx_arr[c] > (cell_count - 1)) continue; 
		std::vector<Particle*> *cell_list = grid[idx_arr[c]];
		// Adj Cell List null ? Skip.
		if (cell_list) concat.insert(concat.end(), cell_list->begin(), cell_list->end());
	}
	// Also add pt's own cell particle list to concated array. 	
	std::vector<Particle*> *selfcell_pts = grid[pt.cell_idx];
	concat.insert(concat.end(), selfcell_pts->begin(), selfcell_pts->end());

	return concat;
}
