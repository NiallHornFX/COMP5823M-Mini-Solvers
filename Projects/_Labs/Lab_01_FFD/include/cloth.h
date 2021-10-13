#ifndef CLOTH_H
#define CLOTH_H

#include "particle.h"

#include <vector>

typedef float real;
typedef unsigned int uint; 

// Face Struct...

class cloth
{
public:
	cloth(std::size_t Nx, std::size_t Ny, const real dCoeff, const vec3<real> &sCoeff);
	~cloth();

	void set_particles();
	void set_springs(const vec3<real> &sCoeff);

	std::vector<spring*> springs;
	std::vector<particle> p_list; 
	std::size_t nx, ny; 
	real damp_coeff; 

	real* get_ptVertexAttribs();
	//real* get_ptNormals(); 
	uint* get_ptVertexIndices(); // Call Once 

	// Get Particle-Vertex Pos, and Indices.. in POD format for GL. 
};

#endif 
