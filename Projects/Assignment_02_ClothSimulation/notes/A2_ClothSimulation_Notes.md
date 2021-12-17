#### COMP5823M - Assignment 2 - Cloth Simulation - Notes

##### Niall Horn - University of Leeds - 2021

___

#### Initial Notes

I'm not going to have time to write many notes for this one, its mass spring based, i've done this before. However even though this is based on a 2D plane / flat piece of cloth its required to implement this via a loaded obj mesh, using edges as springs (ie only distance springs) however I may add shear and dihedral (although this is a bit ambiguous with an instructed obj mesh). 

The base of the app is my "Viewer Application" I wrote for A1 but without any of  the animation code ofcourse, this gives me a nice basis, with a GUI to add the solver into. 

Basic Collision detection with a sphere primitive (defined parametrically, rendered using a sphere mesh).

Integration will be based on Semi Implicit Euler and possibly if I have time I will add another scheme like RK2 or RK4. 

Some parts of the assignment like been able to render out a play blast to a video file is not really a big prio. 

Shading wise I will use Blinn-Phong on the cloth so we can see its shape more, along with toggle modes for the springs / edges and vertices (masses/particles). 

I wanted to be hip and add self collision based on my Verlet Cloth / PBD project and prior knowledge of writing cloth solvers, but this is only if I have time. 

Ideally we can switch between the desired simulation scenarios purposed (hanging cloth, falling cloth onto sphere, rotating sphere), surprised they asked for rotating sphere when there's no self collisions required, this will look ugly. Want to implement correct normal and tangential decomp friction...

To get the shear spring on the diagonal (tri hyp) we could load in obj meshes as quads, and then triangulate them internally so we know which edge is the shear spring ? For now I'm just using my VerletCloth Workflow where I loop over particles, tris (based on indices) and form edges from these, but their is little control / way of defining where each type of spring should be, they are assume to all just be distance / struct springs. 

As we see below I'm using the same approach as my VerletClothSolver project where each unique vert positions becomes a particle hence each vertex maps directly a a particle so particle and vertex is used synonymously but typically particle refers to the point for simulation and vertex for rendering but can be interchanged so note this. For rendering ofcourse each particle/vert is indexed via an EBO based on the per tri indices as each point is shared across multiple tris, hence my cloth_mesh class was written specifically for indexed drawing of the cloth particles with per-tick computed attributes. 

Classes for this project : 

* ***Cloth_State*** : Holds state of cloth, Handles ObjMesh-Cloth Conversion, Spring Creation etc.
* ***Cloth_Solver*** : Performs Cloth Solve : Calculation of spring forces, integration of particles + collisions. 
* ***Cloth_Mesh*** : Takes in Cloth_State reference and renders cloth per frame
* ***Cloth_Collider*** : Collider to test Cloth_State against within Cloth_Solver tick / time-step. 

____

##### Cloth_State Class

This will include obj loading of the passed input mesh (via the GUI). All state update on the cloth simulation state (particles, springs) and render state (vertices & tris) will be done within here along with the final render() called from viewer app render loop. The Simulation will be done within the Cloth_Solver class which references a cloth object and is modifies its state based on the solve. The Solver will be ticked also from the viewer application loop. 

This creates a nice decoupling of cloth state + data and the solver itself which modifies this. 

Spring + Particle Struct declarations

Plan to try and optimize the mesh / primitive classes to make the cloth -> mesh data update per frame more efficient, but it seems to be ok for now. 

We use the mesh class for rendering which I wrote initially to be a primitive that loads an obj file, with texture support, however in this case we are passing it a pre-parsed set of vertices, and we need index drawing, so I may need to modify mesh or create a new class to account for this, could call it "cloth_mesh" and then implement indexed drawing (also this will not inherit from Primitive as I need to add EBO setup etc). Or we could use the mesh class and map each indexed vertex ( and thus particle) to each duplicate vertex using the mesh classes triangle soup approach and then the resulting vertices update their positions from the particle per frame while keeping their original attributes and thus we draw using non indexed method, but this is not very clean. 

obj_loading both loads the vert position data and indices (to form unique particles per vertex position, see below) but also creates the particle data. So anytime a new obj is loaded this will be re-created, but most likely for this project the obj file will be the same, although would be cool to test with some other meshes, been careful of the hardcoded limitations / assumptions of it been a square piece of cloth tri mesh for this project. 

Cloth_Mesh will be re-created for each new obj file (most likely that won't occur as the assignment only deals with a single 2D plane cloth mesh) could also define seperate cloth_meshes for difference simulation scenarios (pinned corners, freefalling etc and switch between them based on GUI, oppose to modifying state of a single instance each time).

##### render() :

Calls render on cloth_mesh but also renders the visualizer primitives for springs and particles based on if enabled in gui. 

##### buildcloth() :

* Creates the springs based on particle pair edges of particles defined indices from resulting particle data from obj loading. 
* Calculates spring  rest lengths. 

As we ideally want all 3 types of springs you typically see in a mass-spring solver we need to make sure the indices are correct so have distance springs along the quad edges, shear springs along the tri hyp edges and then bend springs skipping over each adjacent particle (typically would be in face/tri centres as dihedral springs) bend/dihedral springs are not a prio for now. 

The way I did this in VerletClothMesh was to pre-compute and store per particle (vert) tris they are a part of (remember tris are stored as vec3<int>s for each index), then from this per particle we loop over each tri and create a constraint along each edge, I also used a hash function and check to determine if two particles already had a constraint (eg for the same particle pair in the inverse direction). I could use this same approach but it makes determining where to add shear and bend springs hard, as we'd just typically treat each spring as distance, we could dot each edge against themselves to find the hyp but this is slow. 

For now we will just assume all springs are struct / distance as shear springs use the same forces anyway the spring types were mainly for identification purposes, as long as their is springs along each edge (including all tri edges, ideally without reciprocal duplicates) then that itself implicitly defines struct and shear springs (all as the same spring type). Of course the quality of these springs depends on how clean the topology is of the mesh, but for the test case purposed in the assignment this won't be an issue. 

###### Per Particle Triangle Array

To pre-compute the per particle triangles (which tri is each particle (aka vertex)) a part of I use a basic scatter approach where we loop through each tri (which is just a glm::ivec3 with 3 vert/particle indices) for each index pass a ptr to this current tri to an 2D array which is a per particle array, array of triangle ptrs whom contain its index. Of course this only needs to be computed once per new cloth mesh : 

```C++
void Cloth_State::get_particle_trilist()
{
	// Reset per particle inner array tri list. 
	pt_tris.resize(particles.size());
	for (std::vector<glm::ivec3*> trilist : pt_tris) trilist.clear();
    
    // Loop over tris 
    for (glm::ivec3 &tri : tri_inds)
    {
        // Add tri ptr to each particle defined by tri indices tri list array. 
        for (std::size_t i = 0; i < 3; ++i)
        {
            int p_ind = tri[i];
            pt_tris[p_ind].push_back(&tri);
        }
    }
}    
```

This seems to work as for a 2D plane we expect most particles will be part of 6 tris, other than particles on the edges or at the corners whom will only be part of 1-4. 

###### Building Springs

Essentially we use the indices of the input mesh ie tris to define the springs along the edges of each unique vertex / particle this is a bonus of using tri meshes that we get a diagonal edge to use as an implicit shear spring. Indices ofcourse are just other particles so we form particle pairs, however we ideally should check if a spring already exists with the particle pair else we get duplicate and a possibly over constrained and thus unstable system with counteracting spring forces (that theoretically should cancel each other out, but most likely wouldn't).

We will worry about duplicates / con hash checking in a bit, for now lets just get the spring creation working. Using similar approach to VerletClothMesh we loop over per particle tris (its part of), get each other particle at each tri index, check its not self and create spring between particle self and the other particle defined by the tri indices. 

```c++
// Cloth_State::build_cloth()
// ============= Build Cloth Springs ==============
	// For each particle
	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		Particle &curPt = particles[p];

		// For each tri, particle is part of
		std::vector<glm::ivec3*> &triPts = pt_tris[p];
		for (std::size_t t = 0; t < triPts.size(); ++t)
		{
			// For each tri index (Adjacent Particles)
			for (std::size_t i = 0; i < 3; ++i)
			{
				std::size_t ind = (*triPts[t])[i];
				assert(curPt.id == p); // Make sure iter index matches Pt_id. 
				// Build Spring for particle pair
				if (curPt.id != ind) // Make sure we dont make spring with self pt
				{
					// Get other particle defined by index
					Particle &othPt = particles[ind];

					// Compute Rest Length 
					float rl = glm::length(curPt.P - othPt.P);

					// Create Spring
					springs.emplace_back(&curPt, &othPt, rl);

					// Increment Cur and Oth Particle Spring Count
					curPt.spring_count++, othPt.spring_count++;
				}
			}
		}
	}
```

Update I did add the test condition for duplicate springs (checking for both commutative pairs of curPt and othPt) : 

```C++
// Cloth_State::build_cloth()
// [..]
bool is_dupe = false;
for (std::size_t s = 0; s < springs.size(); ++s)
{
std::size_t s_p0 = springs[s].pt_0->id, s_p1 = springs[s].pt_1->id;
if ((s_p0 == curPt.id && s_p1 == othPt.id) || (s_p0 == othPt.id && s_p1 == curPt.id)) is_dupe |= true; 
}
// Build Spring for particle pair (if othPt not curPt and Spring is not duplicate)
if (curPt.id != othPt.id && !is_dupe) 
// [..]				
```

For my test mesh (16^2 tri 2D Grid, this yields 800 springs, which correctly matches the number of edges on the mesh). Of course this means we have 4 nested loops not, but its only ran once (on cloth_state construction).

###### reset_cloth() :

Will reset the cloth particle positions to their initial state and removes any set forces, velocities on them. 

____

##### Cloth_State : OBJ Loading

We need to treat the mesh as a index face data structure as we need a single mass point / particle for each unique vertex only. 

For rendering sake we will keep all attribute of the input mesh (including the initial normals, which will need to be re-calculated per frame this can be done on the particles or the resulting render verts internally using adjacent verts).

As we need adjacent vertices to form edges this is where a smarter data structure like half edge or directed edge might benefit, however as my VerletCloth Solver used pure index_face vertices --> particles and constraints (springs in this case) I'm going to try and replicate that approach to get adjacent verts and thus edges to define springs along. 

Could just do normal obj loading (Creating duplicate verts) and then do a triangle wise loop (for very 3 verts) and from this, form the indices and thus particles. Problem is as verts have other attributes it makes indexing them harder as we can only use a single EBO.

I think the easiest thing to do for now is just discard all input attributes other than vert position (and thus particles will be based on vert pos indices), we will re-calc normals per tick and in this case, because its a flat 2D plane (as an obj file) we can re-calc the texture coords ourselves, therefore these are only done on the particle / unique positions vertices and can use indexed drawing, we also need to store the input obj vertex pos indices as these are what define these verts and thus the particles and thus we need to store their per tri indices. This makes more sense to why we need a custom class for rendering them. 

Two Workflow Ideas : 

* Map each unique vertex position (as particle) too all duplicate vert positions (with unique attribs) per triangle, and update them using the particle positions per tick, while keeping their original other attributes. The drawing would use a standard VAO approach. 
* But as we are going to update Normals anyway, we can just discard all attributes on the input cloth mesh obj, treat each unique vertex position as the particles, and use these to form the indices per tri for the render mesh using an EBO approach. Each of these Particles->Vertices gets its Normals and UVs calculated internally so each particle --> vert is shared, without needing copies of vertices for vertices with different Normals and UVs that then need to be referenced to particles that lie in their same position (at rest) as the above case proposes.

The latter makes sense as unique vertex positions are what should be treated as unique vertices and thus form particles / point masses each, and we don't plan on having fancy normal creasing etc, we re-calc per particle (which then forms each unique / indexed vertices) normals per tick anyway and the UV's can be re-computed based on the rest positions of the particles in local space. Without needing Triangle Soup based copies of vertices (with same positions) just because their other attributes differ. 

So Obj loading we do the normal parsing, but we only store vert positions we can just discard all other attributes (which we need to check for), then when we get to per tri indices we just copy the vert position indices and these indices will map 1:1 with particles, therefore we don't even need to store the vertex positions we can just use them to insatiate particles array whom will be passed to cloth_mesh where the normal and uv coord attributes will be re-added like in VerletClothMesh. So the output will be the particle array and the triangle indices array (which now map to particle indices). When iterating through particles in order we can just use the iter index as its assumed their ID should match the order they were stored which itself matches the indices order of the mesh. When comparing other particles we need that particles ID so we need this index stored per particle also.

To make things easier as I don't think their gonna try loading arbitrary Obj files when marking I will just assume its using v//vn format. 

Make sure we store a per particle array of the initial state positions so we can "reset to initial state". Also make sure to offset all indices by -1 to account for obj's 1 based indexing. Particles don't need to store an idx as their ID is their position within the particle array (thus based on the original obj unique vert pos indices). We still store IDs because we need them when comparing against other particles, or accessing in alternate iteration orders. 

To save another class we could just make cloth_mesh based within the cloth_state class although I'd be cleaner to separate for the OpenGL state sake, class will be based on primitive class, apart from optimized for updating cloth positions and normal attributes per frame and using indexed drawing based on the passed particles. (Which we then map to a internal indexed vertex array with the correct attribute layout using the calculated attributes per tick for the particles--> now indexed verts to render, using the per tri index list). We can also use the index list back in cloth_state to get edges. 

____

##### Cloth_Mesh Class

As per the internal discussion above, we pass the cloth particles (which like my VerletClothSolver project are just defined as unique vertex positions) to this class, which then calculates the current normals (using neighbouring particles based on index information) this is then serialized into the Vertex Data array (VBO) the Indices only need to be set once to the EBO (when the cloth state is first built as we don't have changing cloth topology here).

Cloth Particles and Cloth tri indices are passed on construction to the cloth_mesh instance, so their initial buffers can be allocated, initial attributes ie normals and  uv's (uvs wont change and thus need to be stored) calculated. Then per tick we call `update_fromParticles()` to update position and normals of particles-verts. Technically we don't need to store the indices copy locally, we could store a reference to it, or we could just pass it directly to the EBO on construction and then not store / reference it as a member further, as indices don't need to be modified beyond, however I think I will store serialized std::size_t indices array (from the passed per triangle glm::ivec3 indices) for debugging sake. Also having a reference to the index buffer in its ivec3 form (per tri) will be useful and we can also reference the cloth_state pt_tris array which will be useful for getting each particles tri indices and thus its neighbours for computing normals. 

Hmm Oppose to passing all these arrays, we could just pass a ptr to the cloth_state class and give it friend access, will make more sense in this case. Then we can directly fetch Particles, Tri Indices, Particle-TriIndices so then update our internal serialized data for GLResources. However we then get a circular dependency of cloth_mesh->cloth_state->cloth_mesh includes so maybe not. We could just pass references of the Particles Array, Triangles (Indices) Array and Per Particle Tris Array on construction and then do the updates through these references, oppose to manually passing the same particle array to the update functions from cloth_state each tick, that makes more sense and it avoids the need to pass the whole cloth_state class reference and thus eliminates the circular dependency. 

 Could compute particle normals within cloth_state and then we don't have to pass the Particle-TriIndices array also to cloth_mesh, this makes sense as normals may be needed in simulation also (eg force along particle normals, most likely not but possibly if I try to implement extra features) Or we can just pass a reference of the Indices and PtTriIndices arrays on cloth_mesh construction and use these per tick to calc normals internally of cloth_mesh, yep lets do that as we probs dont need normals within particles for simulation sake. I've typedef the std::vector particles, ivec3 and vector of vector ivec3 (per particle tri list) to avoid typing out std::vector... each time only for the tri_index array and PtTriIndices array as they are slightly more ambiguous with using ivec3s to implicitly store triangles via indices. I'm already getting bogged down with design decisions which I should not be caring about. 

Don't want to encap to much into the ctor as Primitive relies on external calls after construction to do its setup, so I still want that logic, but some of these calls will be needed in the ctor to pass the initial particle->vert data to primitive::vert_data and the VAO/VBO setup. I was going to be verbose and prefix all base class Primitive Calls with Primitive:: but that just looks amateur and only should use Primitive:: scope when calling a Primitive implementation of a virtual function. 

Primitive Derivate of new class (based on Primitive ?) : This class can be based on the current Primitive class (its code, not derived), but with optimized updated the positions and normal data and using indexed drawing per particle via tris indices. Or we could try and extend the primitive class via inheritance to define the cloth_mesh class, as its setup quite well to be extended, the render method is virtual so we can implemented indexed drawing, we can make the create_buffers() function virtual and both call the primitive version and then also create the EBOs here internally. 

We still need the VAO and VBO defined in Primitive anyway, then  can add our cloth specific attribute calculation member functions within this, worth a shot before creating a brand new class based off Primitive's code anyway for code reuse sake. I mean for perf sake we could just write a custom class with no runtime polymorphism / virtual at all, but will try this primitive derived class first. Primitive class has methods that take in vert array to it, but we can just use our own overloads based on passing particle array (current particle state from cloth_state per tick) its all serialized into the same internal float array which is then what the VAO is derived from. We will still use this workflow as oppose to not storing a member vert float array as if we don't we'd have to pass all particle attributes per tick updating attributes that may not of changed, but then again we do a glBufferData update per tick anyway (we should ideally use  glbuffersubdata to only update the particle->vert data that has changed), also we need to put it into serialized float form so might as well use the same member array primitive uses for this rather than keeping it local within the create_buffers() call. 

 For `update_fromParticles()` we could use glbuffersubdata() to only update the modified position and normals from particles -> vert data, However i'd have to override Primitive's `create_buffers()` implementation to structure the attributes into separate sections of the buffer, instead of been stored per attribute with strides so for now, forget about this. I don't think glbuffersubsata() can update attributes along a stride, it only can update within a specific start-end memory region.  We could use glmapbuffer() to get a ptr to the buffer and manually traverse through by the specific attribute stride updating it (without re-allocating) but we will worry about these optimizations later. 

Because I need particles as a non ptr/reference in cloth_mesh class i've moved the cloth structs into a separate header "cloth_common.h" to eliminate a circular dependency that would occur when cloth_mesh is then included in cloth_state. 

For normals calculation we could do it per tri normal calculations (face normals) and then each particle averages its tris (its a part of) normals to define itself per particle-vert normal (used this approach in VerletClothMesh Project). Or for cheaper we could just get the per particle triangle indices of a single tri its part of, and use the two other particles to build the basis without looking at any other / averaging its other tris. 

I'm actually going to store separate attribute arrays here for the cloth_mesh that will be static (uv's) or calculated internally (normals) so when position is passed we can serialize these together, oppose to doing insertions of each attribute one by one. This makes more sense to why separating attributes into different parts of a single buffer makes more sense and could use buffersubdata() but will worry about that later, for now i'm sticking with the (P,N,C,UV) per vertex layout and the issues that come with it. 

I made the silly mistake again of trying to test my cloth_mesh code which contains OpenGL calls ofcourse when allocated within main() (without creating viewer application and doing OpenGL Context setup etc) hence non of the GL Calls will work ! To test any class that calls OpenGL Functions remember to actually create the instance within the scope where OpenGL context is initialized ! Otherwise you will get "Access violation executing location 0x00..." I.e. the function is not loaded as we're not within an OpenGL Context so there's no instruction to execute.  This is why it might make sense to make the OpenGLContext creation a separate call that's done within main() and then passed (glfw window) to the viewer state class (and thus we can use main to create classes whom contain gl calls themselves without using viewer) however for now as all OpenGL setup is done within Viewer, all tests of OpenGL based classes will have to be done in its Ctor after the OpenGL setup is complete, eg to test if cloth_state which allocates its cloth_state member construction is occurring correctly with its GL Calls.  For now this is fine just a test instance of cloth_state in viewer's ctor and don't call exec() just use the ctor (post OpenGL Setup) as the scope to test the cloth_state and thus cloth_mesh class construction (and thus its ensure its OpenGL operations are working correctly)

In practice Viewer will contain a single instance members of Cloth_State and Cloth_Solver. Cloth_State then ofcourse contains its Cloth_Mesh instance and Cloth_Solver contains a Cloth_Collider instance (or array of them).

##### Cloth_Mesh Calculating Attributes from Particles --> Verts

###### Building Normals

I thought this approach would be cheaper, but it fails on the edge particles (despite the fact they should still have neighbours via their tri indices). Ideas was to get the first tri the particle is part of (ie the first tri within its Particle_TriInd Array) and then use the other 2 verts that are not self to build basis to cross for normal : 

```C++
std::vector<glm::vec3> Cloth_Mesh::calc_normals()
{
	std::vector<glm::vec3> pt_normal(particles.size());

	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		const Particle &curPt = particles[p];

 // Get Particle neighbours via indices of first tri of its particle_tri list,
 // who are not itsself. 
		std::vector<Particle> neighbours; neighbours.reserve(2);
		glm::ivec3 *first_tri = particle_tris[p][0];
		for (std::size_t i = 0; i < 3; ++i) if ((*first_tri)[i] != p) neighbours.push_back(particles[(*first_tri)[i]]);

		// From these form basis for normal. 
		glm::vec3 tang   = neighbours[1].P - curPt.P;
		glm::vec3 bitang = neighbours[0].P - curPt.P;
		glm::vec3 normal = glm::cross(tang, bitang);

		// Set Normal
		pt_normal[p] = glm::normalize(normal); 

	}
	return pt_normal;
}
```

I didn't want to need to do per face normals and then average each face the particle is part of to define particle normals, but that may be a better approach and I used it in VerletClothMesh. 

###### Building UVs :

If  we assume the input obj mesh is a uniform 2D grid, we can calculate the UVs by iterating over the particles / verts converting their indices (which means both the indices in terms of the array but also this equals the actual indices of the mesh, slightly confusing!) into 2D Index via a 1D->2D index lambda where we define m as the square root of the total particle/vert count. The 2D Indices are gotten via division and modulo against this, then we divide the resulting 2D indices that range from [0,M] (i,j) by float(M-1) to get [0.f, 1.f] (u,v) coordinates over the mesh. Code :

```C++
// Assumes mesh is a uniform 2D Grid, uses 2D indexing to calculate UVs as such. 
std::vector<glm::vec2> Cloth_Mesh::calc_uvs()
{
	// Defines single dimension size of grid. 
	std::size_t m = std::sqrt(particles.size());
	float m_r = 1.f / float(m-1);

	auto idx_1dto2D = [](std::size_t i, std::size_t m) -> glm::ivec2
	{
		return glm::ivec2((i / m), (i % m));
	};

	// Approximate UV Coords over 2D grid.  
	std::vector<glm::vec2> pt_uv(particles.size());
	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		const Particle &curPt = particles[p];
		glm::ivec2 ij = idx_1dto2D(p, m);

		float u = float(ij[0]) * m_r; 
		float v = float(ij[1]) * m_r; 
		pt_uv[p] = std::move(glm::vec2(u, v));

		// Debug
		//std::cout << "particle_" << p << " UV = " << u << "," << v << "\n";
	}
	return pt_uv;
}
```

This works great (assuming the input mesh is a 2D Grid, which for this assignment it will be). Technically the V (i) should be flipped by 1s complement so iteration is bottom up (in cartesian uv space) the V axis but it doesn't matter for now especially as cloth is a plane on XZ anyway so UV orientation is not a big deal. 

Side Note Make sure M is the particle/vert count not indices! Note I don't store Particles, Indices size into vars, I just query their vector.size() member function each time, not great for perf but in this case its negligible.  

We need to add textures to the cloth_mesh class, because its not based on mesh class which added texture support to Primitive class. Not a prio for now, but will be useful for having checker texture on cloth.

###### Update Vert Data from Particles

Oppose to calling the Primitive::update_data_position() function which i'd need to implement an equivalent for normals, i'm going to do a custom function within cloth_mesh to directly update the positions and normals for the serialized vert_data array without any Primitive:: base class calls. Will still use the same method and will still need to re-allocate the glBufferData sadly (for now). Because we are just fetching the particle state from the particle array reference passed on cloth_mesh construction from the cloth_state instance, we could multithread the normal update call (to produce the new per particle normal array) along with the extraction of the particle positions. This is the slow-ish part that we have to loop through the particles each time, extract each position member, put it into an array, get the updated normals from these new positions (via the particle array reference cloth_state also) then combine them back into a serialized vert_data float array (along with the unchanged colour and uv attributes) and then update / realloc the data onto the GPU via the VBO update. 

Ideally we have some check to see if we need to update the normals based on if the cloth particles have moved (not checked per particle but as a whole for the cloth) for now its just updating normals every time `Cloth_Mesh::update_fromParticles()` is called. 

Implementation is pretty compact for now, we update both positions and normals within the serialized base Primitive::vert_data float array in the same loop by getting the attrib index locations taking into account the known attrib stride based on attrib layout: 

```C++
void Cloth_Mesh::update_fromParticles()
{
	if (!vert_data.size()) return; // Inital Cloth -->Prim Vert Data must be set first. 

	// Get Updated Normals
	std::vector<glm::vec3> normals = calc_normals();

	// Update Particle-Vert Positions and Normals within Primitive::vert_data array. 
	// Attrib Layout (P,N,C,UV) C and UV are left unchanged. 
	for (std::size_t p = 0; p < particles.size(); ++p)
	{
		const Particle &curPt = particles[p];
		// Get Attrib start indices 
		// Vert Index, Position. (|a_P, a_P+1, a_P+2| (Nxyz)...)
		std::size_t a_P = p * 11;    
		// Vert Index, Normal. ((Pxyz) |a_N, a_N+1, a_N+2| (Crgb)...)
		std::size_t a_N = 3 + p * 11; 

		// Update Position Data
		vert_data[a_P++] = curPt.P.x, vert_data[a_P++] = curPt.P.y, vert_data[a_P] = curPt.P.z;
		// Update Normal Data 
		const glm::vec3 &N = normals[p];
		vert_data[a_N++] = N.x, vert_data[a_N++] = N.y, vert_data[a_N] = N.z;
	}

	// Refill Buffer (not ideal)
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, (vert_count * 11 * sizeof(float)), vert_data.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
```

This function for now is called within `Cloth_State::render()` so ideally we will only need to call `Cloth_Solver::step()` and then `Cloth_State::Render()` from the Viewer Render Loop, to update the solve and then update the cloth mesh and render it. 

____

##### Cloth_Solver Class

We take in a reference to the Cloth_State class whom we have friend access to, so we can ofcourse modify the particles and access the springs data etc. The Solver class will also have its own time state for simulation frames (which ofcourse can be reset by user interaction) so that this is decoupled from the viewer time state (total time of application).

Springs-Eval will be similar to my original code, but oppose to it been within a Member function of the spring class, it will be within cloth_solver, the spring then directly modifies / adds the resulting force to its two particles which it references. 





____

##### Cloth_Collider Class

Has two components the simulation definition / calculation of the collisions based on some parametric shape, sphere in this primary case, and then the render mesh which needs to match the size based on the parameters passed and used for the collision detection. 

Collision function will eval if some passed particle is within bounds of collider and if so return the signed distance and displacement vector to project out of or the force or impulse to apply. This would be a good class to use polymorphism with this as a virtual function we could support boxes and planes using the same parametric approach. Triangle Mesh based cloth collisions are not really a prio / required at all so probs will leave out for now as this would need acceleration and need parity between render and simulation representations (cannot use implicit or parametric functions to approximate collision bounds). Or could implement SDF Collisions if I had time, that would be fun ! But not gonna happen. 

____

##### Bugs List 

Primitive Flags struct was using int8_t which meant the bit field of 1 bit was using the sign bit which did work, but this is not good, should be uint8_t so that we get a clean 0|1 value to use as a flag. Need to fix this in the IK project also. 

Using std::size_t for indices instead of uint, this led to the mesh now drawing correctly (all triangles originated at vertex 0) this is a stupid mistake as std::size_t has to hold the maximum addressable memory address on the platform so is most likely a 64bit uint, whereas indices are typically 32 bit uints, so just use standard C++ uints which for most cases will always be 32bit uints. 
