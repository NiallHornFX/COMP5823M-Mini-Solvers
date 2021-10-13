#include "display.h"

#define GLEW_STATIC
#include "extern/GLEW/glew.h"
#include "extern/GLFW/glfw3.h"
#include "glm.hpp";
#include "gtc/matrix_transform.hpp"
#include "gtc/type_ptr.hpp"

#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <sstream>
#include <cassert>

#define DRAW_TRIS 1

// ! Keep all GL Includes And Logic Here only (Apart from glm.h also in header for mats), expose via render step or getters to external app logic.

extern const std::size_t pt_N;

extern const int width, height;

display::display(std::size_t W, std::size_t H, std::size_t fc, std::size_t tc, std::size_t ic, short major, short minor, const char *Title)
	: width(W), height(H), gl_ver_major(major), gl_ver_minor(minor), title(Title), face_c(fc), tri_c(tc), ind_c(ic), model(1.0f), view(1.0f), proj(1.0f)
{
	window_context();
	extensions_load();
	shader_loader("shaders/Cloth_Vertex.glsl", "shaders/Cloth_Fragment.glsl");
	vertex_setup();
}

display::~display()
{
	delete cloth_indices; cloth_indices = nullptr; // Display Responsible for Indices Deletion.

	glDeleteShader(cloth_vert_shader); cloth_vert_shader = NULL;
	glDeleteShader(cloth_frag_shader); cloth_frag_shader = NULL;
	glDeleteProgram(cloth_shader_prog); cloth_shader_prog = NULL;

	glfwDestroyWindow(window); window = nullptr;
}

void display::window_context()
{
	// GLFW Setup -
	glfwInit();
	if (!glfwInit())
	{
		std::cerr << "ERR::GLFW FAILED TO INITALIZE \n";
		std::terminate();
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, gl_ver_major);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, gl_ver_minor);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE); // Fixed Window Size. 
	glfwWindowHint(GLFW_SAMPLES, 2); // MSAA.
	window = glfwCreateWindow(width, height, title, NULL, NULL);
	if (window == NULL)
	{
		std::cerr << "ERR::GLFW FAILED TO CREATE WINDOW \n";
		glfwTerminate();
		std::terminate();
	}
	glfwMakeContextCurrent(window); // Main Thread
	glViewport(0, 0, width, height);
	std::cout << "DBG::GLFW Window and Initalzation Scuessful \n \n";
}

void display::extensions_load()
{
	// GLEW Setup
	glewExperimental = GL_TRUE;
	glewInit();
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "ERR::GLEW FAILED TO INITALIZE \n";
		std::terminate();
	}

	// Query GL Device and Version Info - 
	render_device = glGetString(GL_RENDERER);
	version = glGetString(GL_VERSION);

	std::cout << "<OPENGL VERSION INFO BEGIN> \n";
	std::cout << "RENDER DEVICE = " << render_device << "\n";
	std::cout << "VERSION = " << version << "\n";
	std::cout << "<OPENGL VERSION INFO END> \n \n";
}

void display::shader_checkCompile(const char *type)
{
	assert(type == (const char*) "vertex" || type == (const char*) "fragment");
	int sucess_v, sucess_f;
	const int len = 512;
	char err_log_v[len], err_log_f[len];

	// Vertex Shader Check - 
	if (strcmp(type, "vertex") == 0)
	{
		glGetShaderiv(cloth_vert_shader, GL_COMPILE_STATUS, &sucess_v);
		if (!sucess_v)
		{
			glGetShaderInfoLog(cloth_vert_shader, len, NULL, err_log_v);
			std::cerr << "ERR::VERTEX SHADER " << " - " << cloth_vert_shader << " COMPILE FAILED \n " << err_log_v << std::endl;
			std::terminate();
		}
	}

	// Fragment Shader Check - 
	if (strcmp(type, "fragment") == 0)
	{
		glGetShaderiv(cloth_frag_shader, GL_COMPILE_STATUS, &sucess_f);
		if (!sucess_f)
		{
			glGetShaderInfoLog(cloth_frag_shader, len, NULL, err_log_f);
			std::cerr << "ERR::FRAGMENT SHADER " << " - " << cloth_frag_shader << " COMPILE FAILED \n " << err_log_f << std::endl;
			std::terminate();
		}
	}
}

// RenderObject_3D_OGL ShaderProgram Linker Checker MFunc Implementation - 
void display::shader_checkLink()
{
	int sucess; const int len = 512; char err_log[len];

	glGetProgramiv(cloth_shader_prog, GL_LINK_STATUS, &sucess);
	if (!sucess)
	{
		glGetProgramInfoLog(cloth_shader_prog, len, NULL, err_log);
		std::cerr << "ERR:SHADER-PROGRAM: " << " - " << cloth_shader_prog << " LINKAGE_FAILED" << std::endl;
		std::cerr << err_log << std::endl;
		std::terminate();
	}
}

// RenderObject_3D_OGL Shader Loader Implementation - 
void display::shader_loader(const char *vert_path, const char *frag_path)
{
	std::ifstream vert_shader_load, frag_shader_load;
	std::stringstream v_shad_buf, f_shad_buf;
	std::string temp_v, temp_f;

	vert_shader_load.exceptions(std::ios::badbit | std::ios::failbit);
	frag_shader_load.exceptions(std::ios::badbit | std::ios::failbit);
	try
	{
		vert_shader_load.open(vert_path);
		frag_shader_load.open(frag_path);

		v_shad_buf << vert_shader_load.rdbuf(); temp_v = v_shad_buf.str(); 
		f_shad_buf << frag_shader_load.rdbuf(); temp_f = f_shad_buf.str();

		vert_shader_load.close();
		frag_shader_load.close();

		if (vert_shader_load.is_open() || frag_shader_load.is_open()) std::cerr << "ERR::Shader Closed Incorrectly \n";
	}
	catch (std::ifstream::failure err)
	{
		std::cerr << "ERR::Shader Load Err: " << err.what() << "\n";
		std::terminate();
	}
	const char *cloth_vert_shader_code = temp_v.c_str();
	const char *cloth_frag_shader_code = temp_f.c_str();

	// Create, Compile, Link Cloth Shader Program. 
	cloth_vert_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(cloth_vert_shader, 1, &cloth_vert_shader_code, NULL);
	glCompileShader(cloth_vert_shader);
	cloth_frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(cloth_frag_shader, 1, &cloth_frag_shader_code, NULL);
	glCompileShader(cloth_frag_shader);
	shader_checkCompile("vertex"); shader_checkCompile("fragment");

	// Cloth Shader Program - 
	cloth_shader_prog = glCreateProgram();
	glAttachShader(cloth_shader_prog, cloth_vert_shader); glAttachShader(cloth_shader_prog, cloth_frag_shader);
	glLinkProgram(cloth_shader_prog);
	shader_checkLink();
}

void display::vertex_setup()
{
	glGenVertexArrays(1, &Cloth_VAO);
	glGenBuffers(1, &Cloth_VBO);
	glBindVertexArray(Cloth_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, Cloth_VBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(real), 0); // VertPos
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(real), (GLvoid*) (sizeof(real) * 3)); // VertNormal,
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0); // Will Fill Buffer Per Frame in vertex_update(). 

	// Set Inital Transforms - 

	// Model
	model = glm::rotate(model, glm::radians(5.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat3 normal_mat = glm::mat3(model); normal_mat = glm::transpose(glm::inverse(normal_mat)); // Normal World Matrix
	// View
	//view = glm::translate(view, glm::vec3(0.0f, 0.0f, -4.0f));
	//c_pos = glm::vec3(0.5f, 0.5f, -1.0f); c_tgt = c_pos + glm::vec3(0.0f, 0.0f, 1.0f);
	c_pos = glm::vec3(0.5f, 0.5f, 1.0f); c_tgt = c_pos + glm::vec3(0.0f, 0.0f, -1.0f);
	cam_update(); // Init Cam Basis
	view = glm::lookAt(c_pos, c_tgt, glm::vec3(0.0, 1.0, 0.0));
	// Proj
	proj = glm::perspective(glm::radians(45.0f), ((float)width / (float)height), 0.01f, 1000.0f); 

	glUseProgram(cloth_shader_prog);
	glUniformMatrix4fv(glGetUniformLocation(cloth_shader_prog, "model"), 1, GL_FALSE, glm::value_ptr(model));
	glUniformMatrix3fv(glGetUniformLocation(cloth_shader_prog, "normal"), 1, GL_FALSE, glm::value_ptr(normal_mat));
	glUniformMatrix4fv(glGetUniformLocation(cloth_shader_prog, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(cloth_shader_prog, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
	glUseProgram(0);
}

void display::vertex_update(real *const vert_a)
{
	cloth_vertices = vert_a;
	//std::cout << "DEBUG ! [" << vert_a[0] << "," << vert_a[1] << "," << vert_a[2] << "," << vert_a[3] << "," << vert_a[4] << "," << vert_a[5] << "]\n";
	glBindBuffer(GL_ARRAY_BUFFER, Cloth_VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(real) * ((pt_N * pt_N) * 6), cloth_vertices, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	delete cloth_vertices; // Delete Per Frame Verts Array. 
}

void display::set_indices(uint *const indices)
{
	cloth_indices = indices;
	uint indices_size = ind_c; 

	glGenBuffers(1, &Cloth_EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Cloth_EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint) * indices_size, cloth_indices, GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

// Render Step - (Called Externally, Within Application Loop) - 
void display::render_step()
{
	glEnable(GL_DEPTH_TEST); // Put these in pre_renderstate setup?
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_PROGRAM_POINT_SIZE);
	
	// Step - 
	glClearColor(0.0f, 0.0f, 0.0f, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(cloth_shader_prog);
	glBindVertexArray(Cloth_VAO);

#if DRAW_TRIS == 0
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDrawArrays(GL_POINTS, 0, pt_N * pt_N);
#else
	// Tri Fill
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Cloth_EBO);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDrawElements(GL_TRIANGLES, ind_c, GL_UNSIGNED_INT, 0);
	// Wireframe
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_TRIANGLES, ind_c, GL_UNSIGNED_INT, 0);
#endif

	// Clear Draw State. 
	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//glfwSwapInterval(0);
	glfwSwapBuffers(window);
	glfwPollEvents();
}

GLFWwindow* display::getwin()
{
	return window;
}

int display::shouldClose()
{
	return glfwWindowShouldClose(window);
}

void display::poll_inputs()
{
	// CAMERA
	glfwPollEvents();

	float delta = 0.05f; 
	bool input_changed = false; 
	// Translation inversed via lookat(). 
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		c_pos -= c_z * delta; 
		input_changed = true;
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		c_pos += c_z * delta;
		input_changed = true;
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		c_pos -= c_x * delta;
		input_changed = true;
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		c_pos += c_x * delta;
		input_changed = true;
	}

	// Pitch,Yaw Update (or via GLFW CBs)... 

	if (input_changed == true) cam_update();
}


void display::cam_update()
{
	c_z = glm::normalize(c_pos - c_tgt); 
	c_x = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), c_z);
	c_y = glm::normalize(glm::cross(c_z, c_x));
	// Make LookAt , Own Using Cam Basis's (seen as we have em) + (4th col) CamPos , VS. just using glm Lookat
	//view = glm::mat4(glm::vec4(c_x.x, c_x.y, c_x.z, 0.0), glm::vec4(c_y.x, c_y.y, c_y.z, 0.0), glm::vec4(c_z.x, c_z.y, c_z.z, 0.0), glm::vec4(c_pos.x, c_pos.y, c_pos.z, 1.0f));
	view = glm::lookAt(c_pos, c_tgt, glm::vec3(0.0f, 1.0f, 0.0f));
	

	glUseProgram(cloth_shader_prog);
	glUniformMatrix4fv(glGetUniformLocation(cloth_shader_prog, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUseProgram(0);
}