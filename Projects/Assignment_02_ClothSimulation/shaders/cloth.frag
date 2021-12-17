// test.frag : Test fragment shader 
#version 400 core 

in vec3 colour;
in vec3 normal;
in vec3 uvw; 

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
	
	// Checker 
	frag_colour = vec4(mix((checker(uvw.x, uvw.y, 10) * 0.5) + 0.1, normal, 0.5), 1.0);
}
