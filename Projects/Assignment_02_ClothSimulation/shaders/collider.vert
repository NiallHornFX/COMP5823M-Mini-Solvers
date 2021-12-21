#version 400 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;
layout (location = 2) in vec3 v_C;
layout (location = 3) in vec2 v_UV;

// Output
out vec3 colour; 
out vec3 normal; 

// Uniforms
uniform mat4 model; 
uniform mat4 view; 
uniform mat4 proj; 

void main()
{
	colour = v_C; 
	normal = transpose(inverse(mat3(model))) * v_N; // Should precompute on host. 
	gl_Position = proj * view * model * vec4(v_P, 1.0); 
}

