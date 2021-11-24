// Implments
#include "camera.h"

// Ext Headers 
#include "ext/GLFW/glfw3.h" // GLFW

// Std Headers
#include <iostream>
#include <algorithm>

// Info : Camera Basis follows Right Hand Coord Sys, Camera Z (Forward Vector) faces towards screen. 

Camera::Camera(glm::vec3 pos, float target_offset, float width, float height) : Cam_Pos(pos), Cam_Up(glm::vec3(0.f, 1.f, 0.f))
{
	// Set Cam Target and Direction
	Cam_Target_Pos = Cam_Pos + glm::vec3(0.0f, 0.0f, -target_offset); // RHS CoordSys, Target offser along depth -z. 
	Cam_Direction = glm::normalize(Cam_Pos - Cam_Target_Pos);
	std::cout << Cam_Direction.z << "\n";

	// Init Camera Bases
	Cam_Basis_X = glm::normalize(glm::cross(Cam_Up, Cam_Direction));
	Cam_Basis_Y = glm::normalize(glm::cross(Cam_Direction, Cam_Basis_X));
	Cam_Basis_Z = glm::normalize(Cam_Pos - Cam_Target_Pos);

	// Set Default Yaw/Pitch Angles - 
	Yaw = 0.0f, Pitch = 0.0f;
	Yaw_Min = -120.0f, Yaw_Max = 120.0f;
	Pitch_Min = -89.0f, Pitch_Max = 89.0f;
	Sensitvity = 0.5f;
	free_look = true;

	// Set Defualt Perspective Members - 
	FOV = 80.0f;
	Zoom = FOV;
	Aspect_Ratio = width / height;
	Near_Plane = 0.01f;
	Far_Plane = 100.0f;
}

glm::mat4 Camera::get_ViewMatrix()
{
	// Flip Basis for target direction
	return glm::lookAt(Cam_Pos, Cam_Pos - Cam_Basis_Z, Cam_Up);
}

glm::mat4 Camera::get_PerspMatrix()
{
	return glm::perspective(glm::radians(FOV), Aspect_Ratio, Near_Plane, Far_Plane);
}

void Camera::update_camera(GLFWwindow *window, float Camera_Speed, float dt, float yaw, float pitch, float zoom)
{
	
	Zoom += zoom;
	// ========== Camera Direction Update ==========
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && free_look)
	{
		Yaw   += yaw   * Sensitvity;
		Pitch += pitch * Sensitvity;
		update_basis();
	} 
	

	// Poll 
	glfwPollEvents();

	// ========== Camera Position Update ==========
	
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) // Forwards along cam Z
	{
		Cam_Pos -= Cam_Basis_Z * (Camera_Speed * dt);
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) // Backwards along cam Z
	{
		Cam_Pos += Cam_Basis_Z * (Camera_Speed * dt);
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) // Left along cam X
	{
		Cam_Pos -= Cam_Basis_X * (Camera_Speed * dt);
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) // Right along cam X
	{
		Cam_Pos += Cam_Basis_X * (Camera_Speed * dt);
	}
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) // Up along Y
	{
		Cam_Pos += Cam_Basis_Y * (Camera_Speed * dt);
	}
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) // Down along Y
	{
		Cam_Pos -= Cam_Basis_Y * (Camera_Speed * dt);
	}

	/*
	if (!glfwGetWindowAttrib(window, GLFW_HOVERED))
	{
		// Reset Basis 
		Cam_Basis_X = glm::vec3(1.f, 0.f, 0.f);
		Cam_Basis_Y = glm::vec3(0.f, 1.f, 0.f);
		Cam_Basis_Z = glm::vec3(0.f, 0.f, 1.f);
		//Cam_Pos = 
	} */
}


void Camera::update_basis()
{
	// Clamp Pitch
	//Pitch = std::max(Pitch_Min, std::min(Pitch, Pitch_Max));
	// Clamp Yaw
	//Yaw = std::max(Yaw_Min, std::min(Yaw, Yaw_Max));

	// Calc Target Direction
	float d_x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	float d_y = sin(glm::radians(Pitch));
	float d_z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	Cam_Direction = glm::vec3(d_x, d_y, d_z);

	// Update Basis Vectors 
	Cam_Basis_Z = glm::normalize(Cam_Direction);
	Cam_Basis_X = glm::normalize(glm::cross(Cam_Up, Cam_Direction));
	Cam_Basis_Y = glm::normalize(glm::cross(Cam_Direction, Cam_Basis_X));
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