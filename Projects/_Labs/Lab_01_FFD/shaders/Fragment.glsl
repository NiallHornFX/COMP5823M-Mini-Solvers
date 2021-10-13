#version 430 core 

out vec4 frag_colour; 

// Marhsalled Rasterized Vertex Attribs
in vec3 vN; // WorldSpace Normals

// Marshalled Rasterized Vertex Vars
in vec2 uv; // LocalSpace UV Coords
in vec4 wpos; // WorldSpacePos
in vec4 vpos; // ViewSpacePos
in vec4 ppos; // ProjectionSpacePos


uniform mat4 model; 
uniform mat3 normal;
uniform mat4 view; 
uniform mat4 projection; 


void main()
{
	// World vs View Space Lighting, Make up your mind...
	vec3 light = vec3(2.5, 2.0, 0.5); 
	vec3 LN = normalize(light - wpos.xyz); 
	
	vec3 vd = -vpos.xyz; 
	vec3 rd = reflect(-LN, vN); 
	
	float d = max(dot(vN, LN), 0.0); 
	float s = pow(max(dot(vd, rd), 0.0), 2); 

	//frag_colour =  vec4(d * vec3(0.1, 0.2, 1.0), 0.75); // Diff 
	//frag_colour =  vec4((d * s) * vec3(0.1, 0.2, 1.0), 1.0); // Diff + Spec. 
	 
	// Tris 
	//frag_colour =  vec4(0.1, 0.2, 1.0, 0.75); // Diff 
	//frag_colour = vec4(wpos.xyz, 0.5); 
	//frag_colour = vec4(rd.xyz, 0.5); 
	
	// Normal Viz - 
	frag_colour = vec4(abs(vN.x), abs(vN.y), abs(vN.z), 1.0); 
	
	// Checker - 
	float freq = 25.0; float amp = 100.0; 
    float f = clamp((cos(uv.x * freq) * sin(uv.y * freq)) * amp, 0.075, 1.0);
     vec3 col = vec3(f,f,f);
	//frag_colour = vec4(uv, 0.0, 0.75); // UV Viz. 
	//frag_colour = vec4(col, 1.0); // Checker
	//frag_colour = vec4(d * (vec3(0.1, 0.2, 1.0) + col), 0.85); // Checker Diff
	//frag_colour =  vec4( ((d * (s * 0.75)) * (vec3(0.1, 0.2, 1.0) + col)), 0.8); // Checker Spec + Diff
	
	
	// Points
	//frag_colour = vec4(gl_PointCoord.xy, 0.0, 1.0); 
}
