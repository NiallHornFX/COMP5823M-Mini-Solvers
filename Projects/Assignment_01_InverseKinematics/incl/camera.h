#ifndef CAMERA_H
#define CAMERA_H

// Ext Headers
// GLM
#include "ext/glm/gtc/matrix_transform.hpp"
#include "ext/glm/gtc/type_ptr.hpp"
// GLFW
#include "ext/GLFW/glfw3.h" 

const glm::vec3 WORLD_UP = glm::vec3(0.0f, 1.0f, 0.0f);

// Info : Basic Camera class using GLFW for input polling. 

class Camera
{
public:
	Camera(glm::vec3 pos, float target_offset, float width, float height);
	Camera() = delete;
	~Camera() = default; 

	// View Members
	glm::vec3 Cam_Pos;
	glm::vec3 Cam_Target_Pos; 
	glm::vec3 Cam_Dir;

	// Camera Basis Members
	glm::vec3 Cam_Basis_X;
	glm::vec3 Cam_Basis_Y;
	glm::vec3 Cam_Basis_Z;

	// View/Proj Matrices
	glm::mat4 lookAt_mat; 
	glm::mat4 Persp_mat;

	// Mouse Rotation Members - 
	float Yaw, Pitch, Zoom;
	float Yaw_Min, Pitch_Min;
	float Yaw_Max, Pitch_Max;

	float Sensitvity;
	float FOV, Aspect_Ratio, Near_Plane, Far_Plane;

public:
	glm::mat4 get_ViewMatrix();
	glm::mat4 get_PerspMatrix();
	void keyboard_Camera(GLFWwindow *window, float Camera_Speed, float dt);
	void Update_Camera_Vectors();
};

#endif