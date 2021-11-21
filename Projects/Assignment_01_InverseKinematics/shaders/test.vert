// test.frag : Test vertex shader. 
#version 430 core 

// Input 
layout (location = 0) in vec3 v_P;
layout (location = 1) in vec3 v_N;
layout (location = 2) in vec3 v_C;
layout (location = 3) in vec2 v_UV;

// Output
out vec4 pos_view; 
out vec4 norm_view; 

out vec3 frag_col;
out vec2 frag_uv; 

// Uniforms
uniform mat4 model; 
uniform mat4 view; 
uniform mat4 proj; 
uniform mat3 normal;

void main()
{
	// ====== Transformations ====
	// Position
	vec4 pos_proj = proj * view * model * vec4(v_P, 1.0); 
	pos_view = view * model * vec4(v_P, 1.0); 
	gl_Position = pos_proj;
	// Normal
	norm_view =  view * model * vec4(v_N, 1.0); 

	// ====== Direct Output ======
	frag_col = v_C; 
	frag_uv  = v_UV;
}

