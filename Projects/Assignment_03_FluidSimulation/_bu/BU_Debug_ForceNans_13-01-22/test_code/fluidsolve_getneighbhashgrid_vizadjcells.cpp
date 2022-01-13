void Fluid_Solver::get_neighbours()
{
	// Delete old Hash Grid
	got_neighbours = false; 
	if (hg) delete hg;

	// Cell Size based on Kernel Radius
	float cs = kernel_radius * 1.5f; 

	// New Hash Grid 
	hg = new Hash_Grid(fluidData, 10, cs);
	hg->hash();
	got_neighbours = true; 

	
	// Test : Adjacent Hash of single particle, viz adj cells. 
	std::size_t testPt = 35; 
	auto pts = hg->get_adjacent_cells(fluidData->particles[testPt]);
	std::size_t idx = fluidData->particles[testPt].cell_idx;
	// Rm all pts cell indices first
	for (Particle &pt : fluidData->particles) pt.cell_idx = 0; 
	// Fill Adj particle list cell_idx with same idx for viz debgging.
	for (Particle *pt : pts)
	{
		pt->cell_idx = idx; 
	}
	fluidData->particles[testPt].cell_idx = 5;
}