// Implments
#include "camera.h"

// Ext Headers 
#include "ext/GLFW/glfw3.h" // GLFW

// Std Headers
#include <iostream>
#include <algorithm>

Camera::Camera(glm::vec3 pos, float target_offset, float width, float height) : Cam_Pos(pos)
{
	// Set Cam Target and Direction
	Cam_Target_Pos = Cam_Pos + glm::vec3(0.0f, 0.0f, target_offset);
	Cam_Dir = glm::normalize(Cam_Pos - Cam_Target_Pos);

	// Global World Up vector. 
	Cam_Up = glm::vec3(0.0f, 1.0f, 0.0f);

	// Calc Local Camera Basis/Axis
	Cam_Basis_X = glm::normalize(glm::cross(Cam_Up, Cam_Dir));
	Cam_Basis_Y = glm::normalize(glm::cross(Cam_Dir, Cam_Basis_X));
	Cam_Basis_Z = glm::normalize(Cam_Dir);

	Sensitvity = 0.75f;

	// Set Default Yaw/Pitch Angles - 
	Yaw =   0.0f, Pitch = 0.0f;

	// Set Max/Clamps for Yaw,Pitch (Deg) - 
	Yaw_Min = -180.0f, Yaw_Max = 120.0f;
	Pitch_Min = -89.0f, Pitch_Max = 89.0f;

	// Set Defualt Perspective Members - 
	FOV = 80.0f;
	Aspect_Ratio = width / height;
	Near_Plane = 0.01f;
	Far_Plane = 100.0f;
}

glm::mat4 Camera::get_ViewMatrix()
{
	return glm::lookAt(Cam_Pos, Cam_Pos + Cam_Target_Pos, Cam_Up);
}

glm::mat4 Camera::get_PerspMatrix()
{
	return glm::perspective(glm::radians(FOV), Aspect_Ratio, Near_Plane, Far_Plane);
}

void Camera::update_camera(GLFWwindow *window, float Camera_Speed, float dt)
{
	glfwPollEvents();

	bool update = false; 
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) // Forwards along cam Z
	{
		Cam_Pos -= Cam_Basis_Z * (Camera_Speed * dt);
		update |= true; 
	}

	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) // Backwards along cam Z
	{
		Cam_Pos += Cam_Basis_Z * (Camera_Speed * dt);
		update |= true; 
	}

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) // Left along cam X
	{
		Cam_Pos -= Cam_Basis_X * (Camera_Speed * dt);
		update |= true; 
	}

	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) // Right along cam X
	{
		Cam_Pos += Cam_Basis_X * (Camera_Speed * dt);
		update |= true; 
	}

	// Update Camera Basis Vectors
	if (update) update_basis();
}


void Camera::update_basis()
{
	// Clamp Pitch
	std::max(Pitch_Min, std::min(Pitch, Pitch_Max));
	// Clamp Yaw
	std::max(Yaw_Min, std::min(Yaw, Yaw_Max));

	// Calc Target Position
	//float d_x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	//float d_y = sin(glm::radians(Pitch));
	//float d_z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	//Cam_Target_Pos = glm::normalize(glm::vec3(d_x, d_y, d_z) + glm::vec3(0.f, 0.f, -1.f));

	// Update Basis Vectors 
	Cam_Dir = glm::normalize(Cam_Pos - Cam_Target_Pos);
	Cam_Basis_Z = glm::normalize(Cam_Dir); 
	Cam_Basis_X = glm::normalize(glm::cross(Cam_Up, Cam_Dir));
	Cam_Basis_Y = glm::normalize(glm::cross(Cam_Dir, Cam_Basis_X));
}

std::ostringstream Camera::debug()
{
	std::ostringstream out;
	out << "======== DEBUG::Camera::BEGIN ========\n"
		<< "X = " << "[" << Cam_Basis_X.x << "," << Cam_Basis_X.y << "," << Cam_Basis_X.z << "]\n"
		<< "Y = " << "[" << Cam_Basis_Y.x << "," << Cam_Basis_Y.y << "," << Cam_Basis_Y.z << "]\n"
		<< "Z = " << "[" << Cam_Basis_Z.x << "," << Cam_Basis_Z.y << "," << Cam_Basis_Z.z << "]\n"
		<< "Yaw = " << Yaw << "  Pitch = " << Pitch << "\n"
		<< "Pos = " << "[" << Cam_Pos.x << "," << Cam_Pos.y << "," << Cam_Pos.z << "]\n"
		<< "Target = " << "[" << Cam_Target_Pos.x << "," << Cam_Target_Pos.y << "," << Cam_Target_Pos.z << "]\n";
	out << "======== DEBUG::Camera::END ========\n";

	return out; 
}