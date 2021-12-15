// COMP5823M A2 : Niall Horn 201486968 - cloth_state.h
#ifndef CLOTH_STATE_H
#define CLOTH_STATE_H

// Std Headers
#include <vector>
#include <string>
#include <fstream>

// Project Headers
#include "cloth_common.h"

// Ext Headers
#include "ext/glm/glm.hpp"

// Struct FDs
struct Particle;
struct Spring; 

class Cloth_State
{
public:
	Cloth_State(const char *path); 
	~Cloth_State() = default;

	void load_obj(std::ifstream &in);

	void build_cloth(); 

	void reset_cloth();

	void render(const glm::mat4x4 &view, const glm::mat4x4 &persp);

	void get_particle_trilist(); 

private:

	// Obj Mesh
	std::string file_path; 

	// Mesh --> Particles Data
	std::vector<glm::vec3>  v_p;
	std::vector<glm::ivec3> tri_inds; 

	// Cloth Sim Data
	std::vector<Particle> particles; 
	std::vector<Spring>   springs; 

	// Per Particle Tris
	std::vector<std::vector<glm::ivec3*>> pt_tris; 

	// Cloth Render Data
	//cloth_mesh *mesh; 

	// Visualizer Data
	std::vector<Primitive*> viz_springs; 
	std::vector<Primitive*> viz_pts; 

	friend class Cloth_Solver; 
};




#endif