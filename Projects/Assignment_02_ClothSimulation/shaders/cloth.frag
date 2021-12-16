// test.frag : Test fragment shader 
#version 400 core 

in vec3 colour;
in vec3 normal;
in vec3 uvw; 

out vec4 frag_colour; 

void main()
{
	//frag_colour = vec4(colour, 1.0);
	frag_colour = vec4(normal, 1.0);
	frag_colour = vec4(uvw, 1.0);
}
