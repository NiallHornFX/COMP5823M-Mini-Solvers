#version 430 core 

// Input
in vec4 pos_view; 

// Output
out vec4 frag_color; 

// Uniforms
uniform vec3 col;

void main()
{
	frag_color = vec4(col, 1.0); 
}
