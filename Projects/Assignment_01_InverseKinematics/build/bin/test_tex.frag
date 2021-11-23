#version 430 core 

in vec3 colour; 
in vec3 poscol; 
in vec2 uv; 

out vec4 frag_colour; 

uniform sampler2D tex; 

void main()
{
	vec4 tex_sample = texture(tex, uv);
	float c = tex_sample.x; 
	if (c < 0.1) discard;
	float a = c;
	a *= (3.0 - poscol.z);
	
	frag_colour = vec4(vec3(c * 0.75), a);
	
	// Render UV Coords 
	//frag_colour = vec4(uv.x, uv.y, 0.0, 1.0);
	
	// Render Texture Sample 
	//frag_colour = vec4(tex_sample.x, tex_sample.y, tex_sample.z, 1.0);
	
}
