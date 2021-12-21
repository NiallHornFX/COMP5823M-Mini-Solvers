#version 400 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;
layout (location = 2) in vec3 v_C;
layout (location = 3) in vec2 v_UV;

// Output
out vec3 pos_view;
out vec3 normal_view; 
out vec3 colour; 

// Uniforms
uniform mat4 model; 
uniform mat4 view; 
uniform mat4 proj; 

void main()
{
	colour = v_C; 
	normal_view = transpose(inverse(mat3(view * model))) * v_N; // Should precompute on host. 
	pos_view    = vec3(view * model * vec4(v_P, 1.0));
	gl_Position = proj * view * model * vec4(v_P, 1.0); 
}

