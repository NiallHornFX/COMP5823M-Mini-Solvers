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

	// Camera Test
	camera = Camera(glm::vec3(0.f, 0.f, 0.f), -1.f, W, H);
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
	std::cout << "======== DEBUG::OPENGL::BEGIN ========\n"
		<< "RENDER DEVICE = " << render_device << "\n"
		<< "VERSION = " << version << "\n";
	std::cout << "======== DEBUG::OPENGL::END ========\n\n";
}

// Init Render State
void Viewer::render_prep()
{
	// Blending and Depth. 
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// State Enable
	glEnable(GL_PROGRAM_POINT_SIZE);
}

void Viewer::render()
{
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
	std::cout << "tick : " << std::endl;
	
	if  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		std::cout << "PRESSED \n";
	}

	// App Operations
	update_window();
	update_camera();

	// Debug Camera State
	std::cout << camera.debug().rdbuf();

	// Cool operations ...
	// Get BVH Update
	// IK 

	// Render
	render();
	tick_c++;
}

void Viewer::exec()
{
	// ----- Init Operations ----
	render_prep();
	
	// ---- App Loop ----
	while (!glfwWindowShouldClose(window))
	{
		// Tick viewer application
		tick();
	}
}

void Viewer::update_window()
{
	// Update Window Title 
	std::string title_u; 
	title_u = title + "  " + std::to_string(tick_c);
	glfwSetWindowTitle(window, title_u.c_str());
}

void Viewer::update_camera()
{
	camera.update_camera(window, 1.f, dt);
	// Need to pass camera matrices to each primitives shader. 
}

void Viewer::get_GLError()
{
	GLenum err = glGetError();
	int i = 0;
	while (err != GL_NO_ERROR)
	{
		if (i <= 10)
		{
			std::cerr << "ERROR::Viewer::GL_ERROR = " << err << "\n";
			i++;
		}
	}
}