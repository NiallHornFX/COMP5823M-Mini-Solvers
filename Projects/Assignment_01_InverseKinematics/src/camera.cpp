// Implments
#include "camera.h"

// Ext Headers 
#include "ext/GLEW/glew.h" // GLEW

// Std Headers
#include <iostream>
#include <algorithm>


Camera::Camera(glm::vec3 pos, float target_offset, float width, float height) : Cam_Pos(pos)
{
	// Set Cam Target and Direction
	Cam_Target_Pos = Cam_Pos + glm::vec3(0.0f, 0.0f, target_offset);
	Cam_Dir = glm::normalize(Cam_Pos - Cam_Target_Pos);

	// Calc Local Camera Basis/Axis
	Cam_Basis_X = glm::normalize(glm::cross(WORLD_UP, Cam_Dir));
	Cam_Basis_Y = glm::normalize(glm::cross(Cam_Dir, Cam_Basis_X));
	Cam_Basis_Z = glm::normalize(Cam_Dir); 

	Sensitvity = 0.75f;

	// Eventually add Constructor Variants, to Support User Defined Values For these - 
	// Set Default Yaw/Pitch Angles - 
	Yaw = -90.0f;
	Pitch = 0.0f;

	// Set Max/Clamps for Yaw,Pitch (Deg) - 
	Yaw_Min = -180.0f, Yaw_Max = 120.0f;
	Pitch_Min = -89.0f, Pitch_Max = 89.0f;

	// Set Defualt Perspective Members - 
	FOV = 90.0f;
	Aspect_Ratio = width / height;
	Near_Plane = 0.01f;
	Far_Plane = 100.0f;
}

glm::mat4 Camera::get_ViewMatrix()
{
	return glm::lookAt(Cam_Pos, Cam_Pos + Cam_Target_Pos, WORLD_UP);
}

glm::mat4 Camera::get_PerspMatrix()
{
	return glm::perspective(glm::radians(FOV + Zoom), Aspect_Ratio, Near_Plane, Far_Plane);
}

void Camera::keyboard_Camera(GLFWwindow *window, float Camera_Speed, float dt)
{
	int W_state = glfwGetKey(window, GLFW_KEY_W);
	int A_state = glfwGetKey(window, GLFW_KEY_A);
	int S_state = glfwGetKey(window, GLFW_KEY_S);
	int D_state = glfwGetKey(window, GLFW_KEY_D);

	if (GLFW_KEY_W == GLFW_PRESS || W_state == GLFW_PRESS) // Forwards along cam Z
	{
		Cam_Pos -= Cam_Basis_Z * (Camera_Speed * dt);
	}

	if (GLFW_KEY_S == GLFW_PRESS || S_state == GLFW_PRESS) // Backwards along cam Z
	{
		Cam_Pos += Cam_Basis_Z * (Camera_Speed * dt);
	}

	if (GLFW_KEY_A == GLFW_PRESS || A_state == GLFW_PRESS) // Left along cam X
	{
		Cam_Pos -= Cam_Basis_X * (Camera_Speed * dt);
	}

	if (GLFW_KEY_D == GLFW_PRESS || D_state == GLFW_PRESS) // Right along cam X
	{
		Cam_Pos += Cam_Basis_X * (Camera_Speed * dt);
	}

	// Update Camera Basis Vectors
	Update_Camera_Vectors();
}


void Camera::Update_Camera_Vectors()
{
	// Clamp Pitch
	std::min(std::max(Pitch, Pitch_Min), Pitch_Max);
	// Clamp Yaw
	std::min(std::max(Yaw, Yaw_Min), Yaw_Max);

	// Calc Target Position
	Cam_Target_Pos.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	Cam_Target_Pos.y = sin(glm::radians(Pitch));
	Cam_Target_Pos.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	Cam_Target_Pos = glm::normalize(Cam_Target_Pos);

	// Update Basis Vectors 
	Cam_Dir = glm::normalize(Cam_Pos - Cam_Target_Pos);
	Cam_Basis_Z = glm::normalize(Cam_Dir); 
	Cam_Basis_X = glm::normalize(glm::cross(WORLD_UP, Cam_Dir));
	Cam_Basis_Y = glm::normalize(glm::cross(Cam_Dir, Cam_Basis_X));
}
