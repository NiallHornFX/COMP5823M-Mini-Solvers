#version 430 core 

in vec3 colour; 
in vec3 poscol; 
in vec2 uv; 

out vec4 frag_colour; 

uniform sampler2D tex; 

void main()
{
	vec4 tex_sample = texture(tex, uv);
	if (tex_sample.x < 0.1) discard;
	vec4 tex_sample_1 = texture(tex, uv + vec2(0.001, 0.0));
	vec4 tex_sample_2 = texture(tex, uv + vec2(-0.001, 0.0));
	vec4 tex_sample_3 = texture(tex, uv + vec2(0.0, 0.001));
	vec4 tex_sample_4 = texture(tex, uv + vec2(0.0, -0.001));
	
	float c = tex_sample_1.x + tex_sample_2.x + tex_sample_3.x + tex_sample_4.x; 
	//float c = tex_sample.x + tex_sample_1.x + tex_sample_2.x; 
	c *= 0.25;

	float a = c;
	a *= (1.75 - poscol.z);
	

	
	frag_colour = vec4(vec3(c * 1.0), a);
	
	// Render UV Coords 
	//frag_colour = vec4(uv.x, uv.y, 0.0, 1.0);
	
	// Render Texture Sample 
	//frag_colour = vec4(tex_sample.x, tex_sample.y, tex_sample.z, 1.0);
	
}
