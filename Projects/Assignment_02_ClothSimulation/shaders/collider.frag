#version 400 core 

// Input
in vec3 colour; 
in vec3 pos_world;
in vec3 normal_world; 

// Output
out vec4 frag_colour; 

// Uniform 
uniform vec3 camPos_world; 
uniform vec3 lightPos_world;
uniform float lightStr; 

void main()
{
	//frag_colour = vec4(normal, 1.0);
	//frag_colour = vec4(colour * 0.6, 1.0);

	// Blinn-Phong (WS)
	vec3 light_dir    = normalize(lightPos_world - pos_world); 
	vec3 view_dir     = normalize(camPos_world - pos_world);
	vec3 half         = normalize(light_dir + view_dir);
	vec3 a = 0.125 * colour; 
	vec3 d = (max(dot(normal_world, light_dir), 0.0) * colour) * lightStr; 
	vec3 s = (pow(max(dot(normal_world, half), 0.0), 4.0) * vec3(1,1,1)) * lightStr; 
	
	frag_colour = vec4(a+d+s, 1.0); 
	
	//frag_colour = vec4((d + s + 0.1) * vec3(colour * 0.8), 1.0);
	//frag_colour = vec4( min((a + d + s), 1.0) * vec3(0.5, 0.05, 0.05), 1.0);
	
	//frag_colour = vec4(normalize(view_dir), 1.0); 
	
}
