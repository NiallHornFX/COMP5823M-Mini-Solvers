// test.frag : Test fragment shader 
#version 430 core 

// Input
in vec4 pos_view; 
in vec4 norm_view; 

in vec3 frag_col;
in vec2 frag_uv; 

// Output
out vec4 frag_color; 

// Uniforms
uniform vec3 col;

void main()
{
	frag_color = vec4(col, 1.0); 
}
