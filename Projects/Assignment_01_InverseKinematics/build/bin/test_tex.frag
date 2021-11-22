#version 430 core 

in vec3 colour; 
in vec2 uv; 

out vec4 frag_color; 

uniform sampler2D tex; 

void main()
{
	vec4 tex_sample = texture(tex, uv);
	//frag_color = vec4(1.0, 1.0, 1.0, 1.0);
	frag_color = vec4(tex_sample.x, tex_sample.y, tex_sample.z, 1.0);
}
