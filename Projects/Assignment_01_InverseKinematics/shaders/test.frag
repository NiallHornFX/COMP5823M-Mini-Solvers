#version 430 core 

in vec4 vpos; 
in vec4 gl_FragCoord; 
out vec4 frag_color; 

uniform vec3 col;

void main()
{
	frag_color = vec4(col, 1.0); 
}
