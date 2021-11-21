#ifndef VIEWER_H
#define VIEWER_H

// Std Includes
#include <string>

// Project Headers
#include "camera.h"
#include "primitive.h"

// Ext Headers
#include "ext/glm/glm.hpp"

// Typedef 
using byte = unsigned char;

// OpenGL Version
#define GL_MAJOR 4
#define GL_MINOR 3

// FD 
struct GLFWwindow;

// Viewer application for rendering, GUI and handling user input.

class Viewer
{
public:
	Viewer() = delete; 
	Viewer(std::size_t W, std::size_t H, const char *Title);
	~Viewer(); 

	// OpenGL Setup
	void window_context();
	void extensions_load();

	// OpenGL 
	void render_prep();
	void render();
	void update_camera();

	// Imgui Setup
	//

	// Application 
	void exec(); // Exec Viewer Application 
	void tick(); // Single Tick

	void update_window();


	//void get_animData(); 

	// Debug
	void test_prim(); 

private:
	void get_GLError();

private:

	// Render State
	GLFWwindow *window; 
	std::size_t width, height;
	std::string title; 

	const byte *render_device;
	const byte *version;

	float dt; 
	float cur_t, prev_t; 

	// Camera 
	Camera camera; 

	// Primtivies
	//std::vector<Primitive> prims; 

	// Debugging
	Primitive prim_t;
	Shader shader;

	// Animation Controls
	std::size_t anim_frame; 
	bool anim_loop; 

	std::size_t tick_c; 
};



#endif