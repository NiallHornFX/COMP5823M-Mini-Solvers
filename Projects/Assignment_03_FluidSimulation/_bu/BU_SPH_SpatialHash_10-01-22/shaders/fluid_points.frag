// COMP5823M - A3 : Niall Horn - fluid_points.frag
#version 400 core 

in vec3 colour;
in vec3 velocity; 

out vec4 frag_colour; 

void main()
{
	frag_colour = vec4(colour, 1.0);
}
