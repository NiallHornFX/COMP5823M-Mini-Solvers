// COMP5823M - A1 : Niall Horn - viewer.cpp

// Implements
#include "viewer.h"

// Std Headers
#include <iostream>
#include <vector>
#include <thread>

// Ext Headers
// GLEW
#include "ext/GLEW/glew.h" 
// GLFW
#include "ext/GLFW/glfw3.h" 
// GLM
#include "ext/glm/gtc/matrix_transform.hpp"
#include "ext/glm/gtc/type_ptr.hpp"
// dearimgui
#include "ext/dearimgui/imgui.h"
#include "ext/dearimgui/imgui_impl_glfw.h"
#include "ext/dearimgui/imgui_impl_opengl3.h"

#define USE_FREE_CAMERA 1

// Global GLFW State
struct
{
	int    width, height;
	double mouse_offset_x  = 0.f, mouse_offset_y  = 0.f;
	double mousepos_prev_x = 0.f, mousepos_prev_y = 0.f;
	double scroll_y = 0.f;
	bool is_init = false; 

}GLFWState;

// =========================================== Viewer Class Implementation ===========================================

Viewer::Viewer(std::size_t W, std::size_t H, const char *Title)
	: width(W), height(H), title(Title)
{
	// ============= Init =============
	tick_c = 0;
	last_yawoffs = 0.f;
	last_pitchoffs = 0.f;
	last_zoom = 0.f;
	draw_grid = true;
	draw_axis = true;

	// ============= OpenGL Setup ============= 
	// Setup OpenGL Context and Window
	window_context(); 
	// Load OpenGL Extensions
	extensions_load();

	// ============= Cloth Setup =============
	cloth = new Cloth_State("clothgrid_a.obj");
	cloth_solver = new Cloth_Solver(*cloth, (1.f / 90.f));

	// ============= Camera =============
	#if USE_FREE_CAMERA == 0
	//camera = Camera(glm::vec3(0.f, 0.25f, 1.f), 1.f, 80.f, width / height, false); // Fixed
	#else
	camera = Camera(glm::vec3(3.f, 1.f, -1.f), 1.f, 80.f, width / height, true);    // Free
	#endif
}

Viewer::~Viewer() 
{
	glfwDestroyWindow(window); window = nullptr;
	glfwTerminate();
}

// Initalizes viewer state and calls indefinite application execution loop.
void Viewer::exec()
{
	// ============= Init Operations =============
	render_prep();

	// ============= Test Operations =============
	// Create Test Mesh
	test_mesh();

	// ============= Application Loop =============
	bool esc = false; 
	while (!glfwWindowShouldClose(window) && !esc)
	{
		// Tick viewer application
		tick();

		// Query Esc key
		esc = esc_pressed();
	} 

	// ============= Shutdown GUI =============
	gui_shutdown();
}


// Single tick of the viewer application, all runtime operations are called from here. 
void Viewer::tick()
{
	// ============= App Operations =============
	get_dt();
	update_window();
	update_camera();

	// ============= Input Query =============

	// ============= Simulation =============
	cloth_solver->tick(dt);

	// ============= Render =============
	render();

	// ============= Post Tick Operations =============
	tick_c++;
}

// Create Window via GLFW and Initalize OpenGL Context on current thread. 
void Viewer::window_context()
{
	// ============= GLFW Setup =============
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
	//glfwWindowHint(GLFW_RESIZABLE, GL_FALSE); // Fixed Window Size. 
	glfwWindowHint(GLFW_SAMPLES, 16); // MSAA.
	// Create Window
	window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
	if (window == NULL)
	{
		std::cerr << "Error::Viewer:: GLFW failed to initalize.\n";
		glfwTerminate();
		std::terminate();
	}

	// ============= Set GLFW Callbacks =============
	// Window Callack
	glfwSetFramebufferSizeCallback(window, &framebuffer_size_callback);
	// Mouse Callbacks
	glfwSetCursorPosCallback(window, &mouse_callback);
	glfwSetScrollCallback(window, &scroll_callback);

	// ============= Set Context and Viewport =============
	glfwMakeContextCurrent(window);
	glViewport(0, 0, width, height);

	// ============= Setup GUI =============
	gui_setup();
}

// Load OpenGL Functions via GLEW and output device info.
void Viewer::extensions_load()
{
	// GLEW Setup
	glewExperimental = GL_TRUE;
	glewInit();
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "ERROR::Viewer:: GLFW failed to initalize.\n";
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
	// ============= OpenGL Pre Render State =============
	// Multisampling 
	glEnable(GL_MULTISAMPLE);

	// Sizes
	glPointSize(5.f);
	glLineWidth(2.5f);

	// Blending and Depth. 
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// ============= Create Viewer Primtivies =============
	// Ground Plane
	ground = new Ground;
	ground->set_size(4.f);
	ground->set_tile(2.f);

	// Axis
	axis = new Primitive("axis");
	float data[66] =
	{
		0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f,
		1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f,
		0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f,
		0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f,
		0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
	};
	axis->set_data_mesh(data, 6);
	axis->scale(glm::vec3(0.5f));
	axis->translate(glm::vec3(0.f, 0.01f, 0.f));
	axis->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
	axis->mode = Render_Mode::RENDER_LINES;
}

// Render Operations
void Viewer::render()
{
	// ==================== Render State ====================
	glClearColor(0.15f, 0.15f, 0.15f, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// ==================== Render Viewer Primtivies ====================
	get_GLError();
	// Draw Grid 
	if (draw_grid)
	{
		ground->set_cameraTransform(camera.get_ViewMatrix(), camera.get_PerspMatrix());
		ground->render();
	}

	// Draw Axis
	if (draw_axis)
	{
		axis->set_cameraTransform(camera.get_ViewMatrix(), camera.get_PerspMatrix());
		glLineWidth(2.5f); // Reset for axis width.
		axis->render();
	}

	// Draw Primtivies
	get_GLError();
	if (prims.size())
	{
		for (Primitive *p : prims)
		{
			p->set_cameraTransform(camera.get_ViewMatrix(), camera.get_PerspMatrix());
			p->render();
		}
	}

	// ==================== Render Cloth ====================
	cloth->render(camera.get_ViewMatrix(), camera.get_PerspMatrix());

	// ==================== Render GUI ====================
	gui_render();

	// ====================  Swap and Poll ====================
	get_GLError();
	glfwSwapBuffers(window);
	glfwPollEvents();
}

void Viewer::query_drawState()
{
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
	{
		draw_grid = !draw_grid;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
	{
		draw_axis = !draw_axis;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void Viewer::update_window()
{
	// Nth frame update
	if (tick_c % 5 != 0) return;

	// Update Window Title 
	std::string title_u;
	title_u = title + "      OpenGL " + std::to_string(GL_MAJOR) + "." + std::to_string(GL_MINOR)
		+ "       FPS : " + std::to_string(1.f / dt);
	
	glfwSetWindowTitle(window, title_u.c_str());
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

// Obj Loading Test
void Viewer::test_mesh()
{
	// Pig
	Mesh *pig = new Mesh("test_pig", "../../assets/mesh/pighead.obj");
	pig->load_obj(false);
	pig->set_shader("../../shaders/normal.vert", "../../shaders/colour.frag");
	pig->set_colour(glm::vec3(1.f, 0.f, 0.f));
	pig->translate(glm::vec3(0.f, 0.f, 0.5f));
	pig->scale(glm::vec3(1.f));
	pig->mode = Render_Mode::RENDER_MESH;
	prims.push_back(pig);
}

// =========================================== DearImGUI Implementation ===========================================

void Viewer::gui_setup()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 400");
}


void Viewer::gui_render()
{
	get_GLError();
	bool window = true; 

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	//ImGui::SetWindowSize(ImVec2(100, 200));

	// Solver State Text
	std::string state; 
	if (cloth_solver->simulate) state = "Solve Running"; else state = "Solve Stopped";

	// Should be member vars.
	float tmp = 0.f; 

	// ============= Imgui layout =============
	{
		ImGui::Begin("Simulation Controls");

		// Text
		if (cloth_solver->simulate) ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255)); else ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
		ImGui::Text(state.c_str());
		ImGui::Text("Simulation Frame = %d", cloth_solver->frame);
		ImGui::PopStyleColor();

		// ==== Solver State ====

		// Anim Loop Play Pause
		if (ImGui::Button("Start/Stop"))
		{
			cloth_solver->simulate = !cloth_solver->simulate;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		// Reset
		if (ImGui::Button("Reset"))
		{
			cloth_solver->reset();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// ==== Solver Controls ====
		ImGui::SliderFloat("K_stiff", &cloth_solver->K_s, -10.f, 10.f);
		ImGui::SliderFloat("K_damp",  &cloth_solver->K_c, -10.f, 10.f);
		ImGui::SliderFloat("Gravity", &cloth_solver->gravity, -50.f, 50.f);

		// ==== Viewer State ====

		// Draw Axis
		if (ImGui::Button("Draw Origin Axis"))
		{
			draw_axis = !draw_axis; 
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		// Draw Grid
		if (ImGui::Button("Draw Grid"))
		{
			draw_grid = !draw_grid;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		

		ImGui::End();
	}

	// ============= Imgui Render =============
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	get_GLError();
}

void Viewer::gui_shutdown()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

// =========================================== GLFW State + Callbacks ===========================================

void Viewer::update_camera()
{
	// Fetch global GLFW State
	width = GLFWState.width, height = GLFWState.height;

	// Only set yaw,pitch if delta from last tick (update_camera() call). 
	float delta_yaw   = (GLFWState.mouse_offset_x   != last_yawoffs)   ? GLFWState.mouse_offset_x : 0.f; 
	float delta_pitch = (GLFWState.mouse_offset_y   != last_pitchoffs) ? GLFWState.mouse_offset_y : 0.f;
	float delta_zoom =  (GLFWState.scroll_y         != last_zoom)      ? GLFWState.scroll_y : 0.f;
	// Set last offsets
	last_yawoffs   = GLFWState.mouse_offset_x;
	last_pitchoffs = GLFWState.mouse_offset_y;
	last_zoom = GLFWState.scroll_y;

	// Update Camera State
	camera.update_camera(window, 1.f, dt, delta_yaw, delta_pitch);

	// Update Aspect Ratio if changed
	if (width != 0 && height != 0)
	{
		float ar = width / height;
		camera.Aspect_Ratio = ar;
	}
}

// ======= Callback Functions =======
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	GLFWState.width = width, GLFWState.height = height; 
	glViewport(0, 0, GLFWState.width, GLFWState.height);
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
	if (!GLFWState.is_init)
	{
		GLFWState.mousepos_prev_x = xpos;
		GLFWState.mousepos_prev_y = ypos;
		GLFWState.is_init = true;
	}
	// Mouse Offset
	GLFWState.mouse_offset_x =  (xpos - GLFWState.mousepos_prev_x);
	GLFWState.mouse_offset_y =  (ypos - GLFWState.mousepos_prev_y);

	// Prev Pos
	GLFWState.mousepos_prev_x = xpos;
	GLFWState.mousepos_prev_y = ypos;
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
	GLFWState.scroll_y = yoffset;
}


