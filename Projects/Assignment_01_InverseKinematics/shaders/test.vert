#version 430 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;

// Output
out vec4 pos_view; 

// Uniforms
uniform mat4 model; 
uniform mat4 view; 
uniform mat4 projection; 
uniform mat3 normal;

void main()
{
	vec4 proj_p = projection * view * model * vec4(v_P, 1.0); 
	vec4 view_p = view * model * vec4(v_P, 1.0); 
	
	// Out 
	pos_view = view_p; 
	gl_Position = proj_p;
}

