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
#include <thread>

Viewer::Viewer(std::size_t W, std::size_t H, const char *Title)
	: width(W), height(H), title(Title)
{
	// ==== Init ====
	anim_loop = true; 
	anim_frame = 0; 
	tick_c = 0;

	// ==== OpenGL Setup ==== 
	// Setup OpenGL Context and Window
	window_context(); 
	// Load OpenGL Extensions
	extensions_load();

	// Create Camera
	camera = Camera(glm::vec3(0.f, 0.5f, 1.f), -100.f, W, H);
}

Viewer::~Viewer() 
{
	glfwDestroyWindow(window); window = nullptr;
	glfwTerminate();
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
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Viewer::render()
{
	// Step - 
	glClearColor(0.2f, 0.2f, 0.2f, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Test Draw Primtivies
	get_GLError();
	for (Primitive *p : prims)
	{
		p->set_cameraTransform(camera.get_ViewMatrix(), camera.get_PerspMatrix());
		p->render();
	}
	get_GLError();

	// Swap and Poll
	glfwSwapBuffers(window);
	glfwPollEvents();
}

void Viewer::tick()
{
	// Pefrom tick
	// App Operations
	get_dt();
	update_window();
	update_camera();

	// Debug Camera State
	//std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//std::cout << camera.debug().str();

	// Cool operations ...
	// Get Skeleton Update
	// IK 

	// Render
	render();

	tick_c++;
}

void Viewer::exec()
{
	// ----- Init Operations ----
	render_prep();

	// Create Test Primtiive
	//test_prim();
	test_mesh();

	// ---- App Loop ----
	while (!glfwWindowShouldClose(window) && !esc_pressed())
	{
		// Tick viewer application
		tick();
	}
}

void Viewer::update_window()
{
	if (tick_c % 5 != 0) return;
	// Update Window Title 
	std::string title_u; 
	title_u = title + "      FPS : " + std::to_string( 1.f / dt);
	glfwSetWindowTitle(window, title_u.c_str());
}

void Viewer::update_camera()
{
	//camera.update_camera(window, 1.f, dt);
	camera.update_camera(window, 1.f, 0.01f);
	// Need to pass camera matrices to each primitives shader. 
}

void Viewer::get_GLError()
{
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) std::cerr << "ERROR::Viewer::GL_ERROR = " << err << std::endl;
}

void Viewer::get_dt()
{
	prev_t = cur_t; 
	cur_t = glfwGetTime();
	dt = cur_t - prev_t; 
}

bool Viewer::esc_pressed()
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) return true;
}

// ------------- DEBUG CODE ---------------
// Temp, testing primtive rendering via Primitive Class
void Viewer::test_prim()
{
	Primitive *prim_t = new Primitive("test");
	float test_verts[11 * 3] =
	{
		// Face 0
		0.0, 0.0, 0.0,  1.0, 1.0, 1.0,	1.0, 0.0, 0.0,	0.1, 0.2,
		1.0, 0.0, 0.0,  1.0, 1.0, 1.0,	0.0, 1.0, 0.0,	0.1, 0.2,
		0.5, 1.0, 0.0,  1.0, 1.0, 1.0,	0.0, 0.0, 1.0,	0.1, 0.2
	};
	prim_t->set_data_mesh(test_verts, 3);
	prim_t->set_shader("test.vert", "test.frag");
	prim_t->mode = Render_Mode::RENDER_MESH;
	prims.push_back(prim_t);
}

// Obj Loading Test
void Viewer::test_mesh()
{
	Mesh *mesh_t = new Mesh("Test", "triplane.obj");
	mesh_t->load_obj(true);
	mesh_t->set_shader("test_tex.vert", "test_tex.frag");
	//mesh_t->load_texture("check.jpg", 0);
	//mesh_t->set_colour(glm::vec3(1.f, 0.f, 0.f));
	mesh_t->mode = Render_Mode::RENDER_MESH;

	prims.push_back(mesh_t);
}