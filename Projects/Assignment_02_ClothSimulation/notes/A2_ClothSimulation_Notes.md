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

We use the mesh class for rendering which I wrote initially to be a primitive that loads an obj file, with texture support, however in this case we are passing it a pre-parsed set of vertices, and we need index drawing, so I may need to modify mesh or create a new class to account for this, could call it "cloth_mesh" and then implement indexed drawing (also this will not inherit from Primitive as I need to add EBO setup etc). Or we could use the mesh class and map each indexed vertex to each duplicate vertex using the mesh classes triangle soup approach, but this is not very clean. 

____

##### Cloth_State : OBJ Loading

We need to treat the mesh as a index face data structure as we need a single mass point / particle for each unique vertex only. 

For rendering sake we will keep all attribute of the input mesh (including the initial normals, which will need to be re-calculated per frame this can be done on the particles or the resulting render verts internally using adjacent verts).

As we need adjacent vertices to form edges this is where a smarter data structure like half edge or directed edge might benefit, however as my VerletCloth Solver used pure index_face vertices --> particles and constraints (springs in this case) I'm going to try and replicate that approach to get adjacent verts and thus edges to define springs along. 

Not sure if I will explicitly store tris, not sure if need. 



____

##### Cloth_Solver Class

Springs-Eval will be similar to my original code, but oppose to it been within a Member function of the spring class, it will be within cloth_solver, the spring then directly modifies / adds the resulting force to its two particles which it references. 
