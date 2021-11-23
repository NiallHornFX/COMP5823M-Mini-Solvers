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
	camera = Camera(glm::vec3(0.f, 0.2f, 1.f), -100.f, W, H);
}

Viewer::~Viewer() 
{
	glfwDestroyWindow(window); window = nullptr;
	glfwTerminate();
}

// Initalizes viewer state and calls indefinite application execution loop.
void Viewer::exec()
{
	// ==== Init Operations ====
	render_prep();

	// Create Test Primtiive
	//test_prim();
	test_mesh();

	// ==== Application Loop ====
	while (!glfwWindowShouldClose(window) && !esc_pressed())
	{
		// Tick viewer application
		tick();
	}
}


// Single tick of the viewer application, all runtime operations are called from here. 
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

// Create Window via GLFW and Initalize OpenGL Context on current thread. 
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
	glfwWindowHint(GLFW_SAMPLES, 16); // MSAA.

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

// Load OpenGL Functions via GLEW and output device info.
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

// Initalize Render State
void Viewer::render_prep()
{
	// ==== OpenGL Pre Render State ====
	// Multisampling 
	glEnable(GL_MULTISAMPLE);
	// Blending and Depth. 
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

// Render Operations
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
	camera.update_camera(window, 1.f, dt);
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

// =========================================== DEBUG CODE ===========================================

// Function to test OpenGL within Viewer App via Mesh Class. 
void Viewer::test_prim()
{
	Primitive *prim_t = new Primitive("test");
	float test_verts[11 * 3] =
	{
		// Face 0
		0.0, 0.0, 0.0,  0.0, 1.0, 1.0,	1.0, 0.0, 0.0,	0.1, 0.2,
		1.0, 0.0, 0.0,  1.0, 0.0, 1.0,	0.0, 1.0, 0.0,	0.1, 0.2,
		0.5, 1.0, 0.0,  1.0, 1.0, 0.0,	0.0, 0.0, 1.0,	0.1, 0.2
	};
	prim_t->set_data_mesh(test_verts, 3);
	prim_t->set_shader("test.vert", "test.frag");
	prim_t->scale(glm::vec3(0.2f));
	prim_t->mode = Render_Mode::RENDER_MESH;
	prims.push_back(prim_t);
}

// Obj Loading Test
void Viewer::test_mesh()
{
	// Textured Mesh Test 
	Mesh *mesh_t = new Mesh("Test", "grid.obj");
	mesh_t->load_obj(true);
	mesh_t->set_shader("test_tex.vert", "test_tex.frag");
	mesh_t->load_texture("grid2.png", 0);
	mesh_t->tex->set_params(Texture::filter_type::LINEAR);
	mesh_t->set_colour(glm::vec3(1.f, 0.f, 0.f));
	mesh_t->mode = Render_Mode::RENDER_MESH;
	prims.push_back(mesh_t);

	//mesh_t->load_obj(false);
	//mesh_t->set_shader("test.vert", "test.frag");
	//mesh_t->set_colour(glm::vec3(1.f, 0.f, 0.f));
	//mesh_t->scale(glm::vec3(0.1f));
	//mesh_t->mode = Render_Mode::RENDER_MESH;

	// Pig
	
	Mesh *pig = new Mesh("pig", "pighead.obj");
	pig->load_obj(false);
	pig->set_shader("test.vert", "test.frag");
	pig->set_colour(glm::vec3(1.f, 0.f, 0.f));
	pig->translate(glm::vec3(0.f, 0.f, 0.5f));
	pig->scale(glm::vec3(1.f));
	pig->mode = Render_Mode::RENDER_MESH;
	prims.push_back(pig);


	// Bones
	/*
	for (std::size_t i = 0; i < 10; ++i)
	{
		float norm = float(i) / 9.f; 
		Mesh *mesh_t = new Mesh("Test", "bone.obj");
		mesh_t->load_obj(false);
		mesh_t->set_shader("test.vert", "test.frag");
		mesh_t->set_colour(glm::vec3(1.f, 0.f, 0.f));
		mesh_t->scale(glm::vec3(0.05f));
		mesh_t->translate(glm::vec3(0.f, norm * 33.f, 0.f));
		prims.push_back(mesh_t);
	}
	*/
	//prims.push_back(mesh_t);
}