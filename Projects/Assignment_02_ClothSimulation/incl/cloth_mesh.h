// COMP5823M - A2 : Niall Horn - cloth_mesh.h
#ifndef CLOTH_MESH_H
#define CLOTH_MESH_H

// Project Headers
#include "primitive.h"
#include "cloth_common.h"

// Info : Class for drawing cloth mesh from particle data, derived from Primtive class.
// This was gonna be a custom class, but will see how this goes first. 
// Unlike Primitive it uses indexed drawing. 

class Cloth_Mesh : public Primitive
{
public:
	// Setup inital atrrib and buffer state from passed pts. 
	Cloth_Mesh(const std::vector<Particle> &init_particles); 
	~Cloth_Mesh() = default; 

	virtual void render() override; 
	virtual void create_buffers() override; 

	// Only update positions from particles --> vert data
	// Also computes particle-vert normals. 
	void update_fromParticles(const std::vector<Particle> &particles);


public:
	// Data
	std::size_t indices_count; 

	GLuint EBO; 
};




#endif