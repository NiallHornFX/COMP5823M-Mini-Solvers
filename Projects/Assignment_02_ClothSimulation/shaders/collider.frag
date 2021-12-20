#version 400 core 

// Input
in vec3 colour; 
in vec3 normal; 

// Output
out vec4 frag_colour; 

void main()
{
	frag_colour = vec4(colour, 1.0);
}
