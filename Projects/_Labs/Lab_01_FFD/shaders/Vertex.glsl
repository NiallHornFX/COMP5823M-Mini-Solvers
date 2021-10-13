#version 430 core 

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aN;

out vec2 uv; // LocalSpace UV Coords
out vec3 vN; // WorldSpace Normals

out vec4 wpos; // WorldSpacePos
out vec4 vpos; // ViewSpacePos
out vec4 ppos; // ProjectionSpacePos

uniform mat4 model; 
uniform mat3 normal;
uniform mat4 view; 
uniform mat4 projection; 

void main()
{
	gl_PointSize = 5.0;

	
	uv = aPos.xy; // Assume LocalSpace Verts 0-1 (xy). 
	//vN =  model * vec4(aN, 1.0); // World Space Normals. 
	vN =  normal * aN; // World Space Normals.
	//vN = mat3(transpose(inverse(model))) * aN;
	
	wpos = model * vec4(aPos, 1.0);
	vpos = view * model * vec4(aPos, 1.0);
	ppos = projection * view * model * vec4(aPos, 1.0); 
	
	gl_Position = projection * view * model * vec4(aPos, 1.0);
}