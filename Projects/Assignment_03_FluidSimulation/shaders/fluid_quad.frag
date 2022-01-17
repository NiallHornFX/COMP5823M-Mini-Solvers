// COMP5823M - A3 : Niall Horn - fluid_quad.frag
#version 400 core 

// Input
in vec4 vpos; 
in vec4 gl_FragCoord; 

// Output
out vec4 frag_color; 

// Uniform Particle Array
struct particle
{
	vec2 pos;
	vec2 vel; 
	float dens; 
};

// Fixed size xxx, pass in current pt count as uniform; 
uniform int pt_count;
uniform particle pts[100];
uniform float min_dens; 
uniform float max_dens;

// Util Functions
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
	
	// Loop through particles, eval implicit function
	float dens = 0.0;
	for (int p = 0; p < 100; ++p)
	{
		particle pt = pts[p];
		float r_sqr = pow(fit(pt.dens, min_dens, max_dens, 0.05, 0.2) + 0.025, 2.0); 
		vec2 c = uv - pt.pos;
		dens += meta(c, 0.5);  
		/*
		// Implicit Circle
		if (dot(c,c) - r_sqr <= 0.001)
		{
			dens = 1.0;
		}*/
	}
	frag_color = vec4(dens, dens, dens, 1.0); 

}
