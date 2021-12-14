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

____

##### Cloth_State Class

This will include obj loading of the passed input mesh (via the GUI). All state update on the cloth simulation state (particles, springs) and render state (vertices & tris) will be done within here along with the final render() called from viewer app render loop. The Simulation will be done within the Cloth_Solver class which references a cloth object and is modifies its state based on the solve. The Solver will be ticked also from the viewer application loop. 

This creates a nice decoupling of cloth state + data and the solver itself which modifies this. 

Spring + Particle Struct declarations

Plan to try and optimize the mesh / primitive classes to make the cloth -> mesh data update per frame more efficient, but it seems to be ok for now. 

We use the mesh class for rendering which I wrote initially to be a primitive that loads an obj file, with texture support, however in this case we are passing it a pre-parsed set of vertices, and we need index drawing, so I may need to modify mesh or create a new class to account for this, could call it "cloth_mesh" and then implement indexed drawing (also this will not inherit from Primitive as I need to add EBO setup etc). Or we could use the mesh class and map each indexed vertex ( and thus particle) to each duplicate vertex using the mesh classes triangle soup approach and then the resulting vertices update their positions from the particle per frame while keeping their original attributes and thus we draw using non indexed method, but this is not very clean. 

obj_loading both loads the vert position data and indices (to form unique particles per vertex position, see below) but also creates the particle data. So anytime a new obj is loaded this will be re-created, but most likely for this project the obj file will be the same, although would be cool to test with some other meshes, been careful of the hardcoded limitations / assumptions of it been a square piece of cloth tri mesh for this project. 

###### render() :

Calls render on cloth_mesh but also renders the visualizer pirmtives for springs and particles based on if enabled in gui. 

###### buildcloth() :

* Creates the springs based on particle pair edges of particles defined indices from resulting particle data from obj loading. 
* Calculates spring  rest lengths. 

As we ideally want all 3 types of springs you typically see in a mass-spring solver we need to make sure the indices are correct so have distance springs along the quad edges, shear springs along the tri hyp edges and then bend springs skipping over each adjacent particle (typically would be in face/tri centres as dihedral springs) bend/dihedral springs are not a prio for now. 

###### reset_cloth() :

will reset the cloth particle positions to their initial state and removes any set forces, velocities on them. 

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

So Obj loading we do the normal parsing, but we only store vert positions we can just discard all other attributes (which we need to check for), then when we get to per tri indices we just copy the vert position indices and these indices will map 1:1 with particles, therefore we don't even need to store the vertex positions we can just use them to insatiate particles array whom will be passed to cloth_mesh where the normal and uv coord attributes will be re-added like in VerletClothMesh. So the output will be the particle array and the triangle indices array (which now map to particle indices).

To make things easier as I don't think their gonna try loading arbitrary Obj files when marking I will just assume its using v//vn format. 

Make sure we store a per particle array of the initial state positions so we can "reset to initial state". Also make sure to offset all indices by -1 to account for obj's 1 based indexing. Particles don't need to store an idx as their ID is their position within the particle array (thus based on the original obj unique vert pos indices).

To save another class we could just make cloth_mesh based within the cloth_state class although I'd be cleaner to separate for the OpenGL state sake, class will be based on primitive class, apart from optimized for updating cloth positions and normal attributes per frame and using indexed drawing based on the passed particles. (Which we then map to a internal indexed vertex array with the correct attribute layout using the calculated attributes per tick for the particles--> now indexed verts to render, using the per tri index list). We can also use the index list back in cloth_state to get edges. 

____

As per the internal discussion above, we pass the cloth particles (which like my VerletClothSolver project are just defined as unique vertex positions) to this class, which then calculates the current normals (using neighbouring particles based on index information) this is then serialized into the Vertex Data array (VBO) the Indices only need to be set once to the EBO (when the cloth state is first built as we don't have changing cloth topology here).

____

##### Cloth_Solver Class

Springs-Eval will be similar to my original code, but oppose to it been within a Member function of the spring class, it will be within cloth_solver, the spring then directly modifies / adds the resulting force to its two particles which it references. 

____

##### Collider Class

Has two components the simulation definition / calculation of the collisions based on some parametric shape, sphere in this primary case, and then the render mesh which needs to match the size based on the parameters passed and used for the collision detection. 

Collision function will eval if some passed particle is within bounds of collider and if so return the signed distance and displacement vector to project out of. This would be a good class to use polymorphism with this as a virtual function we could support boxes and planes using the same parametric approach. Triangle Mesh based cloth collisions are not really a prio / required at all so probs will leave out for now as this would need acceleration and need parity between render and simulation representations (cannot use implicit or parametric functions to approximate collision bounds). Or could implement SDF Collisions if I had time, that would be fun ! But not gonna happen. 
