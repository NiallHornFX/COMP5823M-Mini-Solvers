// Implements
#include "viewer.h"

// Project Headers
#include "shader.h"

// Ext Headers
// GLEW
#include "ext/GLEW/glew.h" 
// GLFW
#include "ext/GLFW/glfw3.h" 
// GLM
#include "ext/glm/gtc/matrix_transform.hpp"
#include "ext/glm/gtc/type_ptr.hpp"

// Std Headers
#include <iostream>
#include <vector>


Viewer::Viewer(std::size_t W, std::size_t H, const char *Title)
	: width(W), height(H), title(Title)
{
	// ==== Init ====
	anim_loop = true; 
	anim_frame = 0; 
	tick_c = 0;

	// ==== OpenGL Setup ==== 

	// Setup OpenGL Context and Window :
	window_context(); 
	// Load OpenGL Extensions
	extensions_load();

	// Shader Test
	Shader test_shader("test_shader", "../../shaders/test.vert", "../../shaders/test.frag");
	test_shader.setVec("col", glm::vec3(1.f, 0.f, 0.f));
}

Viewer::~Viewer() 
{
	glfwDestroyWindow(window); window = nullptr;
}

void Viewer::window_context()
{
	// GLFW Setup -
	glfwInit();
	if (!glfwInit())
	{
		std::cerr << "Error::Viewer:: GLFW failed to initalize.\n";
		std::terminate();
	}

	// Window State
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, GL_MAJOR);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, GL_MINOR);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE); // Fixed Window Size. 
	glfwWindowHint(GLFW_SAMPLES, 2); // MSAA.

	// Create Window
	window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
	if (window == NULL)
	{
		std::cerr << "Error::Viewer:: GLFW failed to initalize.\n";
		glfwTerminate();
		std::terminate();
	}

	// Set Context and Viewport 
	glfwMakeContextCurrent(window); 
	glViewport(0, 0, width, height);
}

void Viewer::extensions_load()
{
	// GLEW Setup
	glewExperimental = GL_TRUE;
	glewInit();
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "Error::Viewer:: GLFW failed to initalize.\n";
		std::terminate();
	}

	// Query GL Device and Version Info - 
	render_device = glGetString(GL_RENDERER);
	version = glGetString(GL_VERSION);
	// Cleanup Debug Output
	std::cout << "<OPENGL VERSION INFO BEGIN> \n";
	std::cout << "RENDER DEVICE = " << render_device << "\n";
	std::cout << "VERSION = " << version << "\n";
	std::cout << "<OPENGL VERSION INFO END> \n \n";
}

void Viewer::render()
{
	glEnable(GL_DEPTH_TEST); // Put these in pre_renderstate setup?
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_PROGRAM_POINT_SIZE);

	// Step - 
	glClearColor(0.2f, 0.2f, 0.2f, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// If Enabled (primtive, draw) ...

	// Intresting stuff [..]

	// Swap and Poll
	glfwSwapBuffers(window);
	glfwPollEvents();
}

void Viewer::tick()
{
	// Pefrom tick

	// App Operations
	update_window();

	// Cool operations ...
	// Get BVH Update
	// IK 

	// Render
	render();
	tick_c++;
}

void Viewer::exec()
{
	// execte viewer indefintly
	bool kill = false; 
	while (1 && !kill)
	{
		tick();

		// Poll Inputs
		// Inline
		kill = glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS; 
	}
}

void Viewer::update_window()
{
	// Update Window Title 
	std::string title_u; 
	title_u = title + "  " + std::to_string(tick_c);
	glfwSetWindowTitle(window, title_u.c_str());
}