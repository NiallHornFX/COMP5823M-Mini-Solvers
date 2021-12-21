// test.frag : Test fragment shader 
#version 400 core 

in vec3 colour;
in vec3 normal;
in vec3 uvw; 

// View Space Lighting
in vec3 pos_view;
in vec3 normal_view; 

out vec4 frag_colour; 

vec3 checker(in float u, in float v, in float size)
{
  float fmodResult = mod(floor(size * u) + floor(size * v), 2.0);
  float res = max(sign(fmodResult), 0.0);
  return vec3(res, res, res);
}

void main()
{
	//frag_colour = vec4(colour, 1.0);
	//frag_colour = vec4(normal, 1.0);
	//frag_colour = vec4(uvw,    1.0);
	
	vec3 check = (checker(uvw.x, uvw.y, 10) * 0.25) + 0.2; 
	
	// Blinn-Phong
	vec3 light_pos    = vec3(0, 5.0, 0);
	vec3 light_dir    = normalize(light_pos - pos_view); 
	vec3 view_dir     = normalize(pos_view);
	vec3 half         = normalize(light_pos) + view_dir;
	float d = max(dot(normal_view, light_dir), 0.0); 
	float s = pow(max(dot(normal_view, half), 0.0), 10.0); 
	
	//frag_colour = vec4((d + s + 0.1) * check, 1.0);
	
	//frag_colour = vec4((d + s + 0.1) * vec3(0.05, 0.35, 0.05), 1.0);
	
	frag_colour = vec4(s * vec3(1,1,1), 1.0);
	
	//frag_colour = vec4(mix((checker(uvw.x, uvw.y, 10) * 0.5) + 0.1, normal, 0.5), 1.0);
}
