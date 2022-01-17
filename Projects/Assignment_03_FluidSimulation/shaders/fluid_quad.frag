// COMP5823M - A3 : Niall Horn - fluid_quad.frag
#version 430 core 

// Input
in vec4 vpos; 
in vec4 gl_FragCoord; 

// Output
out vec4 frag_color; 

// Uniforms
uniform int pt_count;
uniform float min_dens; 
uniform float max_dens;

// GPU Particle Struct
/*
struct particle
{
	vec2 pos;
	vec2 vel; 
	float dens; 
};*/

struct particle
{
	float p_x, p_y; 
	float v_x, v_y;  
	float dens; 
};

struct foo
{
	float r, g; 
};

// SSBO 
layout(std430, binding = 0) buffer data
{
	//particle pts[];  
	foo baz[];
};

// ===== Util Functions =====
float fit (float value, float min_a, float max_a, float min_b, float max_b)
{
	return min_b + (value - min_a)*(max_b - min_b) / (max_a - min_a);
}

float meta(vec2 r, float h)
{
	float rl = length(r); 
	if (rl > h) return 0.0;
	float rlh = rl / h;
	return 1.0 - 3.0 * pow(rlh, 2.0) + 3.0 * pow(rlh, 4.0) - pow(rlh, 6.0);
}


void main()
{
	// Map from 0-Window FragCoord_Space to 0-1 UV Space. 
	vec2 uv = (gl_FragCoord.xy - 0) / 1024;
	// Scale to (0-10, XY to match simulation domain space).
	uv *= 10.0; 
	
	vec2 val = vec2(0, 0); 
	for (int p = 0; p < pt_count; ++p)
	{
		val += vec2(baz[p].r, baz[p].g);
	}
	frag_color = vec4(val, 0.0, 1.0); 
	/*
	// Loop through particles, eval implicit function
	float val = 0.0;
	for (int p = 0; p < pt_count; ++p)
	{
		particle pt = pts[p];
		vec2 pos = vec2(pt.p_x, pt.p_y); 
		float rad = fit(pt.dens, min_dens, max_dens, 0.25, 0.35);
		vec2 r_pt = uv - pos;
		val += meta(r_pt, rad);  
	}
	if (val >= 0.5) // Iso threshold
	{
		frag_color = vec4(val, val, val, 1.0); 
	} 
	*/
}
