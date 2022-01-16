#version 400 core 

in vec4 vpos; 
in vec4 gl_FragCoord; 

out vec4 frag_color; 

// Grid Texture Samplers - 
uniform sampler2D d_tex;       // Texture Unit 0 : Density (Scalar R) 
uniform sampler2D v_u_tex;     // Texture Unit 1 : Velocity_u (Scalar R)
uniform sampler2D v_v_tex;     // Texture Unit 2 : Veloicty_v (Scalar R) 
uniform sampler2D c_tex;       // Texture Unit 3 : Collision (Scalar R)
uniform sampler2D img_rgb_tex; // Texture Unit 4 : RGB_Texture (Packed RGB) 

// Constant Uniforms - 
uniform int N_Size; // Grid Size + Edge Cells. Per Dimension (N).
uniform int Mode; // 0 = Render Density, 1 = Render Velocity. 
uniform int Step; // Current Solve Step. 

// Util Functions - 
float fit (float value, float min_a, float max_a, float min_b, float max_b)
{
	return min_b + (value - min_a)*(max_b - min_b) / (max_a - min_a);
}

float deg2rad (int deg)
{
	float PI = 3.14159265;
	return deg * PI / 180.0; 
}

float rand(float n)
{
	return fract(sin(n) * 43758.5453123);
}

void main()
{
	// Map from 0-N FragCoord_Space to 0-1 UV Space. 
	vec2 uv = (gl_FragCoord.xy - 0.0) / N_Size; 

	float dens =  clamp(texture(d_tex, uv), 0.0, 1.0).r; 
	float cold =  clamp(texture(c_tex, uv), 0.0, 1.0).r; 
	float vel_x = clamp(texture(v_u_tex, uv), 0.0, 1.0).r; 
	float vel_y = clamp(texture(v_v_tex, uv), 0.0, 1.0).r; 
	
	// Packed RGB Channels. 
	vec4 rgb = clamp(texture(img_rgb_tex, uv), 0.0, 1.0); 
	vec3 vel_f = vec3(vel_x, vel_y, 0.0);

	if (Mode == 0)
	{
		// Density + Collider Viz
		frag_color = vec4(clamp(dens+ cold, 0.0, 1.0), dens, dens, 1.0); 

		vec2 vel = vec2(vel_x, vel_y); 
		float vel_rm = length(vel); 
		
		
		// Colour Using Mix/LERP of 2 Colours of vel_rm. 
		vec3 vel_col = mix(vec3(0.8, 0.01, 0.0), vec3(1.0, 0.3, 0.05), vel_rm); 
		
		// Colour Using BlackBody Lookup of vel_rm - 
		vel_col = (vec4(1,1./4.,1./16.,1) * exp(4.*vel_rm - 1.)).xyz;
		vel_col *= min(dens, 1.0); // Only Where Density Is. 
		
		// Add Collider Viz Set FragCol. 
		vel_col.x += cold; // Add Collider Colour to Red.
		frag_color = vec4(clamp(vel_col, 0.0, 1.0), 1.0); 
		
	}
	else if (Mode == 1)
	{
		// Velocity + Collider Viz
		frag_color = vec4(clamp(vel_x + cold, 0.0, 1.0), vel_y, 0.0, 1.0); 
	}
	
}
