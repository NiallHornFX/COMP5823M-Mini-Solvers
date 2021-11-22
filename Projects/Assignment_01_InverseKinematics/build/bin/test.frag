#version 430 core 

in vec3 colour; 
out vec4 frag_color; 

void main()
{
	//frag_color = vec4(1.0, 1.0, 1.0, 1.0);
	frag_color = vec4(colour, 1.0);
}
