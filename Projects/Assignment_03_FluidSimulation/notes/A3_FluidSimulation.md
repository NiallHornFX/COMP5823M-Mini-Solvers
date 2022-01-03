#### COMP5823M - Assignment 3 - Fluid Simulation - Notes

##### Niall Horn - University of Leeds - 2022

___

#### Initial Notes

SPH Based 2D Fluid solver for dam break like scenario with viscosity and surface tension.  

##### Initial Smoothing Function Notes : 

For the pressure and viscosity smoothing kernels we use their derivatives (gradient for pressure and Laplacian for viscosity) which is why only some kernels are suitable for certain forces as their first/second derivatives have saddle points etc. 

Poly6 is used for particle attributes apart from viscosity and pressure force computation. 

Poly6 Can't use for pressure as when function is near 0, derivative is 0, thus if particle $x_i - x_j$ overlap (high density) gradient is zero when function evals to 0, which is incorrect and won't be corrected (will cluster), pressure should be highest here as density is atop. So we need a Kernel function whose derivative is smooth and not zero at any point to ensure correct pressure gradient force is calculated.  Thus spiky kernel is used. 

Spiky has a good first derivative so should be used for pressure (gradient) but not for viscosity as its second derivative that has saddle points. 

However the assignment asks us to use specific kernels for specific force evaluations, so need a way to specify the kernel function used. 

##### Initial Ideas : 

***Scene Definition*** : We are asked for specific boundary conditions with measurements, however the units themselves don't really matter, just need to be relatively scaled over the Frame Buffer / 2D Domain eg 0-10 XY. However I don't want the Scene to depend on the framebuffer as that means having a locked window size etc, so will just scale down into 0-1 scale within vertex shader with centre offset or for things rendered via fragment shaders within fragment shaders (rasterized quad) or before the particle data is passed to  the GPU (cheaper to do the post transform on host ofc). Scene units will be generic so 1.f = 1m.  

We have an open top cube as the fluid container and then square based inflow emission. Boundary Collisions can be specified implicitly as oppose to using explicit particle discretized colliders.  

***Neighbourhood Search Acceleration*** : Vital for accessing neighbouring particles for use with smoothing kernels for attribute calculations, could either use a uniform 2D Grid or spatial hash grid based approach, as I have used the hashing approach recently on my UE4 Cloth solver, I will bring this over as a starting point. 

***Evaluation of Particle Attributes*** : Needs to be a clean and easy way to eval particle attributes using some user defined Kernel as we are asked to use different Kernels. Of course Poly6 won't be used for pressure. Pass in Lambda that defines each type of Kernel function ? Particle Forces (Viscosity + Pressure + external) will need to be evaluated twice per timestep for Leapfrog Integration. 

***Integration*** : As with typical SPH fluid implementation, a Symplectic Integrator is needed, the Leapfrog scheme is specified in the assignment. 

***Tank Boundaries*** : As the fluid is dropped into a open tank boundary, I can just use the plane equation with 3 Planes and specified q and normal vectors, and use only eval at certain positions defined by tank bounds. Can use collider from cloth class for this and loop over planes to eval for each tank side. Using planes makes adjusting the tank bounds at runtime easier. 

***Inflow***: Square of particles in grid pattern to define initial cube of fluid can jitter them if needed to stratify initial positions. 

***Surfacing*** : 2D Metaball / Implicit Circle/Sphere approach in Fragment shader (pass particle positions via UBO and eval surface within fragment shader via unioned Implicit Circle functions). Or possibly a point splatting based approach (Rasterize Points + Convolution Blur/Blending). Could use marching squares would need to rasterize particles to 2D grid first. Will also have option to render as points. Will be nice to have velocity based ramped shading and maybe scale radii based on velocity to get more variation in resulting surface and eliminate high velocity blobby particles that may have become spray like we do in VFX production, in this case it will help the look less generic. 

***Starting Code***:  I won't be using my viewer app as is for the starting code for this project, with it been a 2D Fluid solver there's no need for all the 3D viewport code etc. However I will use the core OpenGL + Shader Setup + GUI code. 

___

#### Smoothing Kernels and Computing Fluid Quantities

A Smoothing Kernel $\omega(||\vec{r}-\vec{r}_j||, h)$ takes in length/distance of vector $\vec{r} - \vec{r}_j$ for some position $\vec{r}$ (typically the ith particle position been iterated to compute quantity at) and the current iterated neighbour particle position $\vec{r}_j$ and computes the smoothed fluid quantity value at that location, within some radius $h$. Note that I'm following standard notation convention, while it may be confusing as Kernel functions may also have a radius however this is denoted h. The second parameter $h$ may be omitted in some cases. 

Kernels need to have the property of their integral been equal to 1: 
$$
\int \omega(r) = 1
$$


##### Poly6 :

Useful for computing all fluid quantities apart from pressure. For viscosity we use the Laplacian of this. 

##### Spiky :

...

##### Computing Fluid Quantities :

Weighted Average interpolation of neighbouring fluid particles, where weighting is calculated via Smoothing Kernel with the quantity divided by the mass, using the following approach : 
$$
Q(x) = \sum_j m_j {Q_j \over \rho_j} \omega(\vec{r} - \vec{r}_j, h)
$$
*Note I use different variable notation to source paper for more clarity, where $Q$ is fluid quantity, $\vec{x}$ is some location / position vector.* 

____

#### Computing Particle Forces

Using the above Smoothing Kernels and Weighted Average function to computes quantities at any given location within the fluid, we can use these to not only evaluate core fluid quantities like density at each particle but the evaluation of the forces of the fluid itself (for each particle). Pressure, Viscosity and Surface Tension been the main forces to evaluate for. 

This will be done within `Fluid_Solver::eval_forces()` which itself will invoke separate force eval functions. 

##### Pressure

As we don't use the Projection method for SPH, we use pressure computed from density, to define the negative pressure gradient force to apply to the particles to approximate incompressibility. To calculate this we need to get the density at each particle via the smoothing kernel. 





____

#### Classes

I will follow the same idea as the Viewer Application based I used for the other 2 assignments, The Viewer App class here will be adopted for just 2D display. We keep the primitive class as we still may need to render mesh data the traditional way. Eg I'm still unsure on if boundaries will be mesh based for rendering or use implicit functions. For time sake we keep all vectors 3D and just omit the Z component. 

Oppose to using an approach with separate SPH Fluid Object and SPH Solver classes, I think for this project I will just keep it as a single class. However it might be better to keep them separate so all the render state  can be put inside the fluid object class while keeping the solver decoupled from the fluid state. The solver class will also be used for setting the boundaries. 

Spatial Hash Grid will also be its own class based on my UE4 Cloth solver hash grid implemetation (but in 2D).

##### Key Class List -

**Fluid_Object** : Contains Particle Array, Fluid State setting Shared Fluid State, Fluid Render Setup+Dispatch functions. Creates Fluid Particle Square at mouse / start location. 

**Fluid_Solver** : Contain solving operations within simulation timestep, Kernel functions etc. Contains ptr to Fluid_Object instance of whom's state it modifies. 

**Fluid_Collider** : Defines a basic polymorphic class who evaluates collisions of some inequality condition against passed particle array (from Fluid_Object, invoked within Fluid_Solver()). Use Primitive Class to render collider via GL_Lines with Line Width. 

**Hash_Grid** : Discretizes Fluids into Spatially coherent cells based on a 2D Spatial Hash Function for accelerating neighbour lookups. 

**Viewer** : Contains the OpenGL,Input and GUI code as well as housing the solver and fluid state instances. 



