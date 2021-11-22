#version 430 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;
layout (location = 2) in vec3 v_C;
layout (location = 3) in vec2 v_UV;

uniform mat4 model; 
uniform mat4 view;
uniform mat4 proj;

out vec3 colour; 
out vec3 normal; 

void main()
{
	gl_Position = proj * view * model * vec4(v_P, 1.0);
	colour = v_C;
	normal = v_N;
}

