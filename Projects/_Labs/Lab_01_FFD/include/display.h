#ifndef DISPLAY_H
#define DISPLAY_H

#include <vec3.h>

#include "glm.hpp";

typedef float real; 
typedef unsigned char byte; 
typedef unsigned int uint; 

class GLFWwindow; 

class display
{
public:
	display(std::size_t W, std::size_t H, std::size_t fc, std::size_t tc, std::size_t ic, short major, short minor, const char *Title);
	display() = delete;
	~display();

	void vertex_update(real *const vert_a);
	void set_indices(uint *const indices);
	void poll_inputs(); 
	void render_step();

	// Get Window Externally. 
	GLFWwindow* getwin();
	int shouldClose(); 

protected:
	// Setup
	void window_context();
	void extensions_load();
	void shader_loader(const char *vert_path, const char *frag_path);
	void vertex_setup();

	// Cam 
	void cam_update();

	// Util 
	void shader_checkCompile(const char *type);
	void shader_checkLink();

private:
	// Window 
	std::size_t width, height;
	std::size_t face_c, tri_c, ind_c; 
	const char *title;
	GLFWwindow *window;

	// Context 
	const byte *render_device;
	const byte *version;
	short gl_ver_major, gl_ver_minor;

	// Buffers
	uint Cloth_VAO, Cloth_VBO, Cloth_EBO;

	// Matrices
	glm::mat4 model, view, proj; 
	glm::vec3 c_pos, c_tgt; 
	glm::vec3 c_x, c_y, c_z; 

	// Shaders
	uint cloth_vert_shader, cloth_frag_shader;
	uint cloth_shader_prog;

	// Geo Arrays
	real *cloth_vertices; // P,N Attrib Array. 
	uint *cloth_indices;
};

#endif