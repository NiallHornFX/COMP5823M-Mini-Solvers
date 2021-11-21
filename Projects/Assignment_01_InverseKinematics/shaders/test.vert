// test.frag : Test vertex shader. 
#version 430 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;
layout (location = 2) in vec3 v_C;
layout (location = 3) in vec2 v_UV;

// Uniforms
uniform mat4 model; 
uniform mat4 view; 
uniform mat4 proj; 

void main()
{
	//vec4 pos_proj = proj * view * model * vec4(v_P, 1.0); 
	//gl_Position = view * vec4(v_P, 1.0); 
	gl_Position = vec4(v_P, 1.0); 
}

