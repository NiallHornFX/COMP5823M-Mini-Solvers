// Simple Mass Spring Cloth Solver 

#include "cloth.h"
#include "display.h"
#include "solver.h" 

#include <iostream>
#include <vector>
#include <cstdio>

const std::size_t pt_N = 48; // NxN Size. 
constexpr std::size_t face_c = (pt_N - 1) * (pt_N - 1); 
constexpr std::size_t tri_c = (pt_N - 1) * ((pt_N - 1) * 2);
constexpr std::size_t ind_c = tri_c * 3; 

const real struct_c = 0.5f, shear_c = 0.5f, bend_c = 0.5f, damp_c = 50.0f;
constexpr real dt = 1.0f / 480.0f; 

// Render Window
const int width = 800, height = 600;  

int main(int argc, char *argv[])
{
	std::cout << "Face Count = " << face_c << "  | Tri Count = " << tri_c << "   | Indices Count = " << ind_c << "\n";
	display disp(width, height, face_c, tri_c, ind_c, 4, 3, "Cloth Solver Fun");
	cloth Cloth(pt_N, pt_N, damp_c, vec3<real>(struct_c, shear_c, bend_c));

	// Pass Cloth to solver ...
	solver solve(&Cloth, dt);

	// Pass Indices - 
	disp.set_indices(Cloth.get_ptVertexIndices());
	
	// External Sim and Render Step TODO (Encap into App class)
	std::size_t step = 0;
	while (step < 10000 && !(disp.shouldClose()))
	{
		// SIM STEP 
		solve.step();

		// RENDER STEP
		disp.vertex_update(Cloth.get_ptVertexAttribs()); // Updt Verts
		disp.poll_inputs();
		disp.render_step();
		std::cout << "Step = " << step++ << "\n";
	}
	return 0;
}