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
	// Cloth State 
	cloth = new Cloth_State("clothgrid_a.obj");
	// Cloth Solver 
	cloth_solver = new Cloth_Solver(*cloth, (1.f / 90.f));
	// Collider Primtives
	collision_plane = new Cloth_Collider_Plane(glm::vec3(0.f, 1.f, 0.f));
	collision_sphere = new Cloth_Collider_Sphere(glm::vec3(0.f, 0.f, 0.f), 1.f);
	// Pass Colliders to Solver
	cloth_solver->colliders.push_back(collision_plane);
	cloth_solver->colliders.push_back(collision_sphere);

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
	//test_mesh();

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

	//  ==================== Render Cloth Colldiers ====================
	if (collision_sphere)
	{
		collision_sphere->render_mesh->set_cameraTransform(camera.get_ViewMatrix(), camera.get_PerspMatrix());
		collision_sphere->render_mesh->render();
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

// Info : Test Mesh to verify viewer and OpenGL is setup correctly. 
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

// Info : imgui Startup 
void Viewer::gui_setup()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 400");
}

// Info : GUI Render with forwarding the relveant data based on input.
void Viewer::gui_render()
{
	get_GLError();
	bool window = true; 

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// ============= GUI Static Locals / State =============
	// Solver State Text
	std::string state; 
	if (cloth_solver->simulate) state = "Solve Running"; else state = "Solve Stopped";
	// Cloth Mesh Path
	static char obj_mesh_input[256]{ "../../assets/mesh/clothgrid_a.obj" };
	static char obj_mesh_output[256]{ "export.obj" };
	// Wind Force
	static float wind[3] = { 0.f, 0.f, 0.f };
	cloth_solver->wind.x = wind[0], cloth_solver->wind.y = wind[1], cloth_solver->wind.z = wind[2];
	// Collider State
	static bool p_use = true;
	static bool s_use = true;
	static char plane_onoff [32] {"Disable Plane Collider"};
	static char sphere_onoff[32] {"Disable Sphere Collider"};
	static float s_rad = 1.f; 
	static float s_cent [3] = { 0.f, 0.f, 0.f };
	// Get Dt 1/n. 
	float n = 1.f / cloth_solver->dt;
	static int tmp_count = 90;
	// Cloth State Locals
	bool static fix_corners = true;
	static char corners_onoff[32]{ "Cut Cloth Corners" };
	static Cloth_Collider *tmp_plane = collision_plane;
	static Cloth_Collider *tmp_sphere = collision_sphere;


	// ============= Imgui layout =============
	{
		ImGui::Begin("Simulation Controls");

		// ========== Solver State ==========
		// Labels
		if (cloth_solver->simulate) ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 255, 0, 255)); else ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
		ImGui::Text(state.c_str());
		ImGui::Text("Simulation Frame = %d, Substep = %d", cloth_solver->frame, cloth_solver->timestep);
		ImGui::Text("Dt = 1/%d", std::size_t(n));
		ImGui::PopStyleColor();

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

		// ========== Cloth State Controls ==========
		ImGui::Dummy(ImVec2(0.0f, 10.0f));
		ImGui::Text("Cloth State Controls");
		// Mesh Import
		ImGui::InputText("Mesh Import", obj_mesh_input, 256);
		if (ImGui::Button("Load Cloth Mesh"))
		{
			delete cloth;
			cloth = new Cloth_State(obj_mesh_input);
			delete cloth_solver; 
			cloth_solver = new Cloth_Solver(*cloth, 1.f / 90.f);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// Mesh Export
		ImGui::InputText("Mesh Export", obj_mesh_output, 256);
		if (ImGui::Button("Export Cloth Mesh"))
		{
			cloth_solver->simulate = false; 
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			cloth->export_mesh(obj_mesh_output);
		}

		// Cloth Corners
		ImGui::Dummy(ImVec2(0.0f, 10.0f));
		ImGui::Text("Constrain Cloth Corners");
		if (ImGui::Button(corners_onoff))
		{
			fix_corners = !fix_corners;
			if (!fix_corners)
			{
				cloth->set_fixed_corners(false);
				strcpy_s(corners_onoff, 32, "Fix Cloth Corners");
			}
			else
			{
				cloth->set_fixed_corners(true);
				strcpy_s(corners_onoff, 32, "Cut Cloth Corners");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}


		ImGui::Dummy(ImVec2(0.0f, 10.0f));
		ImGui::Text("Cloth Colliders");

		// Slightly Hacky using Solver Collider Indices. 
		if (ImGui::Button(sphere_onoff))
		{
			s_use = !s_use; 
			if (!s_use)
			{
				collision_sphere           = nullptr; 
				cloth_solver->colliders[1] = nullptr; 
				strcpy_s(sphere_onoff, 32, "Enable Sphere Collider");
			}
			else
			{
				collision_sphere           = tmp_sphere;
				cloth_solver->colliders[1] = tmp_sphere;
				strcpy_s(sphere_onoff, 32, "Disable Sphere Collider");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		if (ImGui::SliderFloat("Sphere Radius", &s_rad, 0.1f, 3.f))
		{
			static_cast<Cloth_Collider_Sphere*>(collision_sphere)->set_radius(s_rad);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (ImGui::SliderFloat3("Sphere Centre", s_cent, 0.f, 10.f))
		{
			static_cast<Cloth_Collider_Sphere*>(collision_sphere)->set_centre(glm::vec3(s_cent[0], s_cent[1], s_cent[2]));
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (ImGui::Button(plane_onoff))
		{
			p_use = !p_use;
			if (!p_use)
			{
				collision_plane            = nullptr;
				cloth_solver->colliders[0] = nullptr;
				strcpy_s(plane_onoff, 32, "Enable Plane Collider");
			}
			else
			{
				collision_plane            = tmp_plane;
				cloth_solver->colliders[0] = tmp_plane;
				strcpy_s(plane_onoff, 32, "Disable Plane Collider");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// ========== Solver Controls ==========
		ImGui::Dummy(ImVec2(0.0f, 10.0f));
		ImGui::Text("Solver Controls");
		// Physics Timestep
		if (ImGui::InputInt("Timestep 1/x", &tmp_count))
		{
			cloth_solver->set_timestep(tmp_count);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		ImGui::SliderFloat("K_stiff", &cloth_solver->K_s, 0.f, 1000.f);
		ImGui::SliderFloat("K_damp",  &cloth_solver->K_c, 0.f, 10.f);
		ImGui::SliderFloat("K_visc",  &cloth_solver->K_v, 0.f, 2.f);
		ImGui::SliderFloat("Gravity", &cloth_solver->gravity, -10.f, 10.f);
		ImGui::SliderFloat3("Wind", wind, 0.f, 5.f);

		// ========== Viewer State ==========
		ImGui::Dummy(ImVec2(0.0f, 10.0f));
		ImGui::Text("Viewer Controls");
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

// Info : imgui shut down. 
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
	float delta_zoom  =  (GLFWState.scroll_y        != last_zoom)      ? GLFWState.scroll_y       : 0.f;
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


