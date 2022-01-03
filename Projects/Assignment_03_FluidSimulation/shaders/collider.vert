// COMP5823M - A3 : Niall Horn - basic.vert
#version 400 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;
layout (location = 2) in vec3 v_C;
layout (location = 3) in vec2 v_UV;

// Output
out vec3 colour; 

uniform mat4 model;

void main()
{
	colour = v_C; 
	vec3 tPos = v_P / v_P;
	gl_Position = model * vec4(tPos - 1.0, 1.0);
}

