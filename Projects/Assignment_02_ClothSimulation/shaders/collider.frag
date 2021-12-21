#version 400 core 

// Input
in vec3 colour; 
// View Space Lighting
in vec3 pos_view;
in vec3 normal_view; 

// Output
out vec4 frag_colour; 

void main()
{
	// Basic 
	//frag_colour = vec4(normal, 1.0);
	//frag_colour = vec4(colour * 0.6, 1.0);

	// Blinn-Phong
	vec3 light_pos    = vec3(0.0, 2.0, 0.0);
	vec3 light_dir    = normalize(light_pos - pos_view); 
	vec3 view_dir     = normalize(pos_view);
	vec3 half         = normalize(light_pos) + view_dir;
	float d = max(dot(normal_view, light_dir), 0.0); 
	float s = pow(max(dot(normal_view, half), 0.0), 4.0); 
	
	frag_colour = vec4((d + s + 0.1) * vec3(colour * 0.8), 1.0);
	
}
