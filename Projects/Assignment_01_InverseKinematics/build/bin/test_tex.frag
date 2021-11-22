#version 430 core 

in vec3 colour; 
in vec2 uv; 

out vec4 frag_colour; 

uniform sampler2D tex; 

void main()
{
	//vec4 tex_sample = texture(tex, uv);
	
	frag_colour = vec4(uv.x, uv.y, 0.0, 1.0);
	//frag_colour = vec4(tex_sample.x, tex_sample.y, tex_sample.z, 1.0);
	//frag_colour = vec4(colour, 1.0);
}
