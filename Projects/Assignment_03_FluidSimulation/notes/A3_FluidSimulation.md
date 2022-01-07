#### COMP5823M - Assignment 3 - Fluid Simulation - Notes

##### Niall Horn - University of Leeds - 2022

___

#### Initial Notes

SPH Based 2D Fluid solver for dam break like scenario with viscosity and surface tension forces in a open top tank boundary. We need both the solver side, the rendering side (as particles and ideally as a surface)  

##### Initial Smoothing Function Notes : 

For the pressure and viscosity smoothing kernels we use their derivatives (gradient for pressure and Laplacian for viscosity) which is why only some kernels are suitable for certain forces as their first/second derivatives evaluate to 

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

***Tank Boundaries*** : As the fluid is dropped into a open tank boundary, I can just use the plane equation with 3 Planes and specified q and normal vectors, and use only eval at certain positions defined by tank bounds. Can use collider from cloth class for this and loop over planes to eval for each tank side. Using planes makes adjusting the tank bounds at runtime easier. However we need to only eval for particles whom like along the planes length/height and for some depth (so fluid can splash out of the tank and down the sides) else the planes will bound off to infinite depth either side of the tank. 

***Inflow***: Square of particles in grid pattern to define initial cube of fluid can jitter them if needed to stratify initial positions. 

***Surfacing*** : 2D Metaball / Implicit Circle/Sphere approach in Fragment shader (pass particle positions via UBO and eval surface within fragment shader via unioned Implicit Circle functions). Or possibly a point splatting based approach (Rasterize Points + Convolution Blur/Blending). Could use marching squares would need to rasterize particles to 2D grid first. Will also have option to render as points. Will be nice to have velocity based ramped shading and maybe scale radii based on velocity to get more variation in resulting surface and eliminate high velocity blobby particles that may have become spray like we do in VFX production, in this case it will help the look less generic. 

***Starting Code***:  I won't be using my viewer app as is for the starting code for this project, with it been a 2D Fluid solver there's no need for all the 3D viewport code etc. However I will use the core OpenGL + Shader Setup + GUI code. 

___

#### Smoothing Kernels and Computing Fluid Quantities

To be able to treat the discrete particles that make up the fluid as a smooth approximately continuous field we can evaluate fluid quantities and their derivatives at, we use Smoothing Kernels to compute interpolated values as a weighted average of the fluid wrt to each particle position $(i)$ by looping through all of its neighbour particles $j$ and computing a smooth approximation of the fluid quanitiy.

With some Smoothing Kernel $\omega(r=||\vec{r}-\vec{r}_j||, h)$ function that takes in length/distance of vector $\vec{r} - \vec{r}_j$ for some position $\vec{r}$ (typically the ith particle position been iterated to compute quantity at) and the current iterated neighbour particle position $\vec{r}_j$ and computes the smoothed fluid quantity value at that location, within some radius $h$ also denoted as smoothing length. Note that I'm following standard notation convention, while it may be confusing as Kernel functions may also have a radius however this is denoted h. The second parameter $h$ may be omitted in some cases. 

Kernels need to be normalized and thus have the property of their integral been equal to 1 over their radius $h$: 
$$
\int \omega(r) = 1
$$
One thing I realised is that the Smoothing Kernels shown in the paper (and most other SPH text's) assume 3D Space, but the smoothing Kernel function and its derivatives would be different in 2D Space, when I get time I want to derive these by hand. I believe the 2D Kernels were officially derived first in the paper [SPH Based Shallow Water Simulation. Solenthaler, et al. 2011]. 

However I have seen some code online where 3D Kernels are used in a 2D Setting and it seems to work, so maybe I will try and use them at some point for comparison. 

Ok Curve-ball alert : He wants us to use the 3D Kernels in a 2D solver "because the mark scheme says so and I don't have any control over it blah blah", despite the fact the original author of the SPH paper he gave us was an author on the later paper that derives the kernels specifically for a 2D solver. I think they will work, but honestly so stupid. So I will now show the 3D versions and their respective gradient and Laplacians. 

##### Poly6 Kernel : 

###### Poly6 Function : 

3D
$$
\omega_{poly6} (r, h) = {315\over64\pi h^9} \left\{\begin{array}{} (h^2-r^2)^3 \:\:\:\: 0 \leq r \leq h\\ \:\:\:\:\:\:\:\: 0 \:\:\:\:\:\:\:\:\:\:\:\: otherwise \end{array}\right.
$$
2D
$$
\omega_{poly6} (r, h) = {4\over\pi h^8} \left\{\begin{array}{} (h^2-r^2)^3 \:\:\:\: 0 \leq r \leq h\\ \:\:\:\:\:\:\:\: 0 \:\:\:\:\:\:\:\:\:\:\:\: otherwise \end{array}\right.
$$
We see that the Kernel clamps to zero at the boundary thus only returns a non zero result when $r$ is within the range. This is the same for all Smoothing Kernels otherwise they wouldn't define a radius. 

Note $r = |\vec{r}-\vec{r}_j|^2$ within Poly6 is only used squared, so rather than passing the in the length of $\vec{r}$ we can use a square distance function to avoid the square root. Note this is only true for the Poly6 Kernel, Debrun's Spiky uses non square $\vec{r}$ length. Using this we can check if the $r^2 >= h^2$ and return $0$ if so else we return the computed kernel value.

Poly6 is the general purposed SPH Smoothing Kernel and is useful for computing all fluid quantities apart from pressure, as noted above because its gradient evals to zero when the function is near the boundary $[0,h]$ it becomes unstable as the resulting pressure gradient can become 0. However it does have a stable second derivative that we use as the Laplacian for the viscosity force, however we use a separate kernel for viscosity as shown below. 

We can precompute the coefficent part of the kernel asusming that $h$ is fixed for all kernel functions used in the solver. 

###### Poly6 Gradient Derivation : 

The gradient can be calculated by calculating the first derivative of the Kernel function with respect to $r$ and scaling this against the unit direction vector $r$ which defines the gradient direction (ie towards the centre of the Kernel). 

So for the 2D Poly6 Kernel gradient we calculate : 
$$
x = |r|^2 \\
{d \over dx} \begin{bmatrix}{4\over \pi h^8 }{(h^2-x^2)^3}\end{bmatrix} = - {24x(h^2-x^2)^2\over \pi h^8}
$$
Note that $x / r$ here is the length of $\vec{r}$ but as per below we need to pass the vector in to the gradient calculation itself because we need the original vector to define the kernel direction. 

We know the direction vector is simply the input $\vec{r}$ vector (make sure we pass the input vector and not the distance or square distance precomputed), this is then normalized.  
$$
\vec{r} =  \vec{r} - \vec{r}_j\\
\hat{r} =  {\vec{r} \over ||\vec{r}||}
$$
Combine to get : 

###### Gradient of the 2D Poly6 Kernel : 

$$
\nabla\omega_{poly6} (r, h) = -{24\over\pi h^8} {\vec{r}\over ||\vec{r}||} (h-|r|^2)^2
$$

###### Gradient of 3D Poly6 Kernel :

(Derivation later...)
$$
\nabla\omega_{poly6} (r, h) = -{945\over 32\pi h^9} {\vec{r}\over ||\vec{r}||} (h-|r|^2)^2
$$


##### Spiky Kernel :

As noted above Poly6 suffers from the gradient of it approaching zero at the centre and edge (boundaries) which when used for the pressure gradient calculation can cause instability and particle clustering to occur, this [Desbrun] proposes the spiky Kernel which has a gradient which does not approach zero at the centre of the kernel. 

This is important because if we think of two particles whom are ontop of each other in space, we'd expect the pressure to be at its highest to force them apart back to a low pressure zone where they are thus separated, however as poly6 gradient evaluates to zero at the centre of the kernel and the origin/boundary where $r$ will be zero it the resulting gradient itself been zero results in no pressure and thus no projective force been applied. 

Thus we define the Spiky kernel as : 

###### Spiky Function

3D 
$$
\omega_{spiky} (r, h) = {15\over\pi h^6} \left\{\begin{array}{} (h-r)^3 \:\:\:\: 0 \leq r \leq h\\ \:\:\:\:\:\:\:\: 0 \:\:\:\:\:\:\:\:\:\:\:\: otherwise \end{array}\right.
$$
2D 
$$
\omega_{spiky} (r, h) = {10\over\pi h^5} \left\{\begin{array}{} (h-r)^3 \:\:\:\: 0 \leq r \leq h\\ \:\:\:\:\:\:\:\: 0 \:\:\:\:\:\:\:\:\:\:\:\: otherwise \end{array}\right.
$$

###### Spiky Gradient

The gradient can be calculated by calculating the first derivative of the Kernel function with respect to $r$ and scaling this against the unit direction vector $r$ which defines the gradient direction (ie towards the centre of the Kernel). 

So for the 2D Spiky Kernel we calculate : 
$$
x = ||\vec{r}|| \\
{d \over dx} \begin{bmatrix}{10\over \pi h^5 }{(h-x)^3}\end{bmatrix} = -{30(h-x)^2 \over \pi h^5}
$$
Note that $x / r$ here is the length of $\vec{r}$ but as per below we need to pass the vector in to the gradient calculation itself because we need the original vector to define the kernel direction. 

We know the direction vector is simply the input $\vec{r}$ vector (make sure we pass the input vector and not the distance or square distance precomputed), this is then normalized.  
$$
\vec{r} =  \vec{r} - \vec{r}_j\\
\hat{r} =  {\vec{r} \over ||\vec{r}||}
$$
Combine to get the :

###### 2D Gradient of the Spiky Kernel : 

$$
\nabla\omega_{spiky} (r, h) = -{30\over\pi h^5} {\vec{r}\over ||\vec{r}||} (h-||r||)^2
$$

Which can be re-written as scalar vector multipcation : 
$$
\nabla\omega_{spiky} (r, h) = -({30\over\pi h^5} \cdot (h-||r||)^2) \: {\vec{r}\over ||\vec{r}||}
$$

###### 3D Gradient of the Spiky Kernel : 

(Derivation Later ...)
$$
\nabla\omega_{spiky} (r, h) = -({45\over\pi h^6} (h-||r||)^2) {\vec{r}\over ||r||}  
$$




The Spiky Kernel however suffers from the same problem as Poly6 for its Laplacian which is used to define the viscosity force (as per the Navier Stokes momentum equation) so its not usable for Viscosity. So we use a different Kernel for Viscosity. 

##### Viscosity Kernel

This Kernel function was purposed by Muller in the SPH paper :

###### Viscosity Kernel Function

3D
$$
\omega_{visc} (r, h) = {15\over2\pi h^3} \left\{\begin{array}{} -{||r||^3 \over 2h^3} + {||r||^2 \over h^2} + {h \over2||r||} - 1\:\:\:\: 0 \leq r \leq h\\ \:\:\:\:\:\:\:\: 0 \:\:\:\:\:\:\:\:\:\:\:\: otherwise \end{array}\right.
$$
2D
$$
\omega_{visc} (r, h) = {10\over3\pi h^2} \left\{\begin{array}{} -{||r||^3 \over 2h^3} + {||r||^2 \over h^2} + {h \over2||r||} - 1\:\:\:\: 0 \leq r \leq h\\ \:\:\:\:\:\:\:\: 0 \:\:\:\:\:\:\:\:\:\:\:\: otherwise \end{array}\right.
$$

###### Viscosity Kernel 2D Laplacian: 

(Will derive Later...)

*May be incorrect*
$$
\nabla^2\omega_{visc}(r,h) = {20\over \pi h^5}\cdot (h-||r||)
$$

###### Viscosity Kernel 3D Laplacian: 

(Will derive Later...)
$$
\nabla^2\omega_{visc}(r,h) = {45\over \pi h^5}\cdot (1 - {||r||\over h})
$$


##### Modular Kernel Functions

Because I want to be able to easily swap out the Kernels used as the assignment requires us to use Poly6 and Poly6 gradient first and then replace the pressure gradient calc with Spiky gradient, and the other attribs with Spiky etc. So I decided to use a Function pointer approach, i typedef (using) function pointers to use Member Functions of `Fluid_Solver` class via : 

```C++
// Fluid_Solver.h
using kernel_func = float(Fluid_Solver::*) (const glm::vec3 &r);
using kernel_grad_func = glm::vec2(Fluid_Solver::*)(const glm::vec3 &r);
```

And any function that computes fluid attributes (ie using Kernels) takes these func ptrs as input, eg : 

```C++
// Fluid_Solver.h
// [..]
void eval_forces(kernel_func w, kernel_grad_func w_g);
void compute_dens_pres(kernel_func w);
```

To invoke the Kernel Functions, as they are member function pointers, invoked within the class itself, we need to explicitly use `this->*` to dereference them, eg eval Forces funct : 

```C++
// Fluid_Solver.cpp
void Fluid_Solver::eval_forces(kernel_func w, kernel_grad_func w_g)
// Invoke Kernel Gradient FuncPtr 
(this->*w_g)(Pt_i.P - Pt_j.P);
```

##### Implementation of Kernel Functions : 

The Kernels themselves are implemented to use a pre-defined Kernel Radius $h$ which is set on the `Fluid_Solver` construction, while this doesn't allow us to change the Kernel Radius / Smoothing Length in the GUI (Unless we re-build the Fluid_Solver instance) it allows us to precompute the coefficient part of the Kernel in the constructor, to avoid computing them every time the Kernel function is evaluated. 

Precompute Kernel Coefficients in Ctor, using known $h$ / kernel radius. 

 ```C++
 // Fluid_Solver.cpp : Fluid_Solver::Fluid_Solver(..)
 // ===== Pre Compute Kernel + Derivative Scalar Coeffecints =====
 // Poly 6
 poly6_s = 4.f / M_PI  * std::powf(kernel_radius, 8.f);
 poly6_grad_s = -(24.f / M_PI * std::powf(kernel_radius, 8.f));
 // Spiky
 spiky_s = 10.f / M_PI * std::powf(kernel_radius, 5.f);
 spiky_grad_s = -(30.f / M_PI * std::powf(kernel_radius, 5.f));
 // Viscosity
 visc_lapl_s = -(20.f / M_PI * std::powf(kernel_radius, 5.f));
 ```

Then we have the actual member function implementations (which are force inlined) of the Kernels themselves using the pre-computed coefficients along with the $\vec{r}$ vector input :

```C++
// Fluid_Solver.cpp 
float Fluid_Solver::kernel_poly6(const glm::vec3 &r)
{
	float r_sqr = glm::dot(r, r);
	if (r_sqr > kernel_radius_sqr) return 0.f;
	
	return poly6_s * std::powf((kernel_radius_sqr - r_sqr), 3.f);
}

glm::vec2 Fluid_Solver::kernel_poly6_gradient(const glm::vec3 &r)
{
	glm::vec2 r_n2 = glm::normalize(glm::vec2(r.x, r.y));
	float r_sqr = glm::dot(r, r);
	return poly6_grad_s * std::powf((kernel_radius - r_sqr), 2.f) * r_n2;
}

float Fluid_Solver::kernel_spiky(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	if (r_l > kernel_radius) return 0.f; 

	return spiky_s * std::powf((kernel_radius - r_l), 3.f);
}

glm::vec2 Fluid_Solver::kernel_spiky_gradient(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	glm::vec2 r_n2 = glm::normalize(glm::vec2(r.x, r.y));
	return spiky_grad_s * std::powf((kernel_radius - r_l), 2.f) * r_n2; 
}

float Fluid_Solver::kernel_visc_laplacian(const glm::vec3 &r)
{
	float r_l = glm::length(r);
	return visc_lapl_s * (kernel_radius - r_l);
}
```



____

#### Computing Fluid Quantities :

Weighted Average interpolation of neighbouring fluid particles, where weighting is calculated via Smoothing Kernel with the quantity divided by the mass, using the following approach : 
$$
Q(x) = \sum_j m_j {Q_j \over \rho_j} \omega(\vec{r} - \vec{r}_j, h)
$$
For the Gradient and Laplacian derivatives needed over the fluid, we can simply just replace the RHS smoothing kernel, with its respective Gradient or Laplacian, while Muller's paper does use this approach there is alternative approaches to getting more correct derivative approximations over the fluid.  For pressure specifically this can be a problem as it is not symmetric there is ways to approximate the symmetry which I will cover later when computing pressure force. 

###### Computing Density

As density is based on the number of neighbouring particles, its used as both a weight for other attribute / quantities but also to define the pressure of the fluid as shown below. As we don't know density of each particle within the fluid to begin with (unless we use the prev frame/step density) we cannot use it to weight with the smoothing kernel, so when $Q = \rho$ we can compute it using the following : 
$$
\rho(x) = \sum_j m_j\: \omega(\vec{r} - \vec{r}_j, h)
$$
Furthermore we can assume that mass is constant and $m_j = 1$ for all particles and simplify this density calc to just :
$$
\rho(x) = \sum_j \omega(\vec{r} - \vec{r}_j, h)
$$
The smoothing Kernel used in this case will be the Poly6. 

 ###### Computing Pressure : shown below

____

#### Computing Particle Forces

Using the above Smoothing Kernels and Weighted Average function to computes quantities at any given location within the fluid, we can use these to not only evaluate core fluid quantities like density at each particle but the evaluation of the forces of the fluid itself (for each particle). Pressure, Viscosity and Surface Tension been the main forces to evaluate for. 

This will be done within `Fluid_Solver::eval_forces()` which itself will invoke separate force eval functions. 

##### Pressure

As we don't use the Projection method for SPH, we use pressure computed from density, to define the negative pressure gradient force to apply to the particles to approximate incompressibility. To calculate this we need to get the density at each particle via the smoothing kernel from this we can derive areas of high pressure from high density and use a negative pressure gradient based force to move particles to areas of lower pressure. 

For each particle $i$ we first need to compute pressure itself $p_i$, which assumes we have already computed the density $\rho_i$ : 
$$
p_i = k({\rho_i - \rho_{rest}})
$$
Using the particles current density $\rho_i$ and some rest density $\rho_{rest}$ scaled by some stiffness constant $k$. Both $k$ and $\rho_{rest}$ are user defined and need to be picked carefully. 

The resulting pressure force on the particle is the negative pressure gradient, can be computed first naively :
$$
\textbf{f}_i^{pres} = -\nabla p(r_i) = -\sum_j m_j {p_j\over \rho_j}\nabla\omega(r-r_j, h)
$$
Using the gradient of the smoothing kernel function. However due to the force not been symmetric between the two particles due to the gradient been zero at the centre thus Muller propses a simple solution where the pressures of either particle $i$ the iterated particle and $j$ each neighbour particle are averaged :
$$
\textbf{f}_i^{pres} = -\nabla p(r_i) = -\sum_j m_j {{p_i + p_j}\over 2\rho_j}\nabla\omega(r-r_j, h)
$$


We should make sure we don't eval self particle when computing pressure as the pressure kernel uses the length of the vector $||\vec{r}||$ and it will cause a divide by zero nan to occur so the resulting pressure gradient and thus force will be nan.

I don't think we can use $h < 1$ because it breaks the normalization of the Kernels, so this is a bit of an issue if the Scene is scaled such that 1 "unit" is too large we may have to scale all the scene up to be realativly scaled larger eg $[0,100]$ instead of $[0,10]$ such that we can use smaller kernel radius / smoothing length relative to the scale of the scene/fluid. This shouldn't be an issue as we just treat the scene meters as $0.1$ meter thus we use 40 instead of 4 etc. 

###### Issues : 

One implementation mistake I made was in the pressure gradient computation : 

```C++
// Fluid_Solver.cpp : Fluid_Solver::eval_forces(kernel_func w, kernel_grad_func w_g)
// For Particles Pt_i
//      For Particles Pt_j 
pressure_grad += (Pt_i.pressure + Pt_j.pressure / 2.f * Pt_j.density) * (this->*w_g)(Pt_i.P - Pt_j.P); // Wrong. 
// Incorrect, need to bracket out the Numer + Denom of the division duh !
pressure_grad += ((Pt_i.pressure + Pt_j.pressure) / (2.f * Pt_j.density)) * (this->*w_g)(Pt_i.P - Pt_j.P); // Correct.

```

Still having an issue of pressure gradient and thus force diverging to infinity and thus resulting in nans, but separate from the fixed above issue. 

____

#### Classes

I will follow the same idea as the Viewer Application based I used for the other 2 assignments, The Viewer App class here will be adopted for just 2D display. We keep the primitive class as we still may need to render mesh data the traditional way. Eg I'm still unsure on if boundaries will be mesh based for rendering or use implicit functions. For time sake we keep all vectors 3D and just omit the Z component. 

Oppose to using an approach with separate SPH Fluid Object and SPH Solver classes, I think for this project I will just keep it as a single class. However it might be better to keep them separate so all the render state  can be put inside the fluid object class while keeping the solver decoupled from the fluid state. The solver class will also be used for setting the boundaries. 

Spatial Hash Grid will also be its own class based on my UE4 Cloth solver hash grid implemetation (but in 2D).

##### Key Class List -

**Fluid_Object** : Contains Particle Array, Fluid State setting Shared Fluid State, Fluid Render Setup+Dispatch functions. Creates Fluid Particle Square at mouse / start location. 

**Fluid_Solver** : Contain solving operations within simulation timestep, Kernel functions etc. Contains ptr to Fluid_Object instance of whom's state it modifies. 

**Fluid_Collider** : Defines a basic polymorphic class who evaluates collisions of some inequality condition against passed particle array (from Fluid_Object, invoked within Fluid_Solver()). Use Primitive Class to render collider via GL_Lines with Line Width. 

**Hash_Grid** : Discretizes Fluids into Spatially coherent cells based on a 2D Spatial Hash Function for accelerating neighbour lookups. Only Cells that have been hashed to are allocated. 

**Viewer** : Contains the OpenGL,Input and GUI code as well as housing the solver and fluid state instances. 

____

##### Fluid Object

Currently emit a square of fluid

Will use random to add some jitter to the 2D positions to eliminate the grid discretisation the particles are visibly in. 

____

#### Hash Grid

As stated, the Hash Grid class is based on my UE4 Cloth Solver plugin where I was using it to accelerate particle-particle self collisions. The standard approach doesn't use a cell size but specifies the number of buckets (which is then mod'd with the XOR'd spatial coords with some large prime integers) based on the paper *[Optimized Spatial Hashing for Collision Detection of Deformable Objects. Teschner et al.]*. However I adopted it to specify both a cell size and a cell count. However I plan to modify it to specify a cell size and a grid size (which will be scaled over the 2D $10^2$ domain) to derive cell count from this. The Particle Positions are hashed and the resulting index ranges from $0\:\:to\:\:cellcount-1$. 

Particles are hashed and the resulting output is an index of the hash grid, so only cells which contain particles are allocated (Outer array contains pointers to inner dynamic arrays (eg std::vector)) while this does cause memory fragmentation it shouldn't be a big deal, each particle then stores the cell it lies within and can use the cell index to look up its neighbours to reduce the time complexity of particle-neighbour hood distance searching from $O(n^2) \mapsto O(nm)$. HashGrid is faster than a uniform/explicit grid, as we don't need to do a per cell gather step and transform the grid from index space, to world space etc and if we use an implementation with Cell Size specified for the HashGrid we don't have the disadvantage that we cannot specify a cell size like we can for an explicit uniform grid. We may need a uniform grid later for Surface Tension but will worry about that later. 

I'm using a modified Hash Grid implementation, so instead of specifying Cell Count or Cell count and Cell Size (as in my UE4 Version, which is pointless as cell_count should be based on size), we just specify cell size and the dimension size of the hash grid (which we assume is square and will be $[0,10] \isin \R_{x,y}^2$). From this we derive the cell count $cc = (dim / cs)^2$ where we compute the count for one dimension squared (2D Hash Grid). Using this total cell count we then allocate the outer 2D Array, which contains `std::vector<Particle*>*` pointers that are only allocated if the cell has some particle hashed into it.

Of course we need to be weary of the Hash Grid Cell count if $cs = 0.1$ we have $cc = (10/0.1)^2 = 1e04$ cells. But as the point is to use $cs = h$ where $h$ is the Smoothing Kernel radius, which should be much larger than this, I don't see it been a problem. 

The Hash Function is modified from [Teschner et al] to work wrt to the cell size : 

```C++
std::size_t Hash_Grid::HashPos(const glm::vec2 &PtPos) const
{
	std::size_t xx = static_cast<std::size_t>(PtPos.x * r_cell_size);
	std::size_t yy = static_cast<std::size_t>(PtPos.y * r_cell_size);
	std::size_t c = 73856093 * xx ^ 19349663 * yy;
	return c % cell_count;
}
```

Where we divide by the Particle Position components by the cell size (multiply by reciprocal in this case to save on division). We then return the cell index it hashes too. 

Of course there is risk of collisions (more cells ie lower cell size will reduce this, but ideally we want cell size to equal $h$ / kernel radius, so we all particles only search for neighbour particles in their current cell as oppose to needing to search current cell + adjacent cells particles). However I feel like we may need to reduce cell size to avoid too few cells and thus slow neighbour hood particle calculation. 

I'll do something in Fluid_Object to set particle colours based on cell index so we can viz it.

As in my UE4 Cloth solver, new hash grid will be created per frame, this is not ideal and ideally we can just reset the per cell particle lists /vectors, but it may be faster to just reallocate a new hash grid.

###### Issues : 

If Cell size matches Kernel Radius $h$ we end up with too large cells and the $O(nm)$ gains diminish. Ideally we'd have a smaller cell size (thus more hash cells) and search the particles current cell and its adjacent cells, which still should be less than searching a larger single cell that matches the size of the kernel radius. 

It becomes visible in the resulting density a hash grid is used as we have square regions of varying densities corresponding to each cell location, specially the nodes of the grid. This should be less of an issue with higher particle counts. However the other issue is that particles form into discrete cell groups as oppose to based on their smoothing kernel. 

___

#### Uniform 2D Grid

This class will be used as an alternate spatial acceleration structure as well as for rasterizing the particles for surface tension calculation and possible rendering via rasterized grid and marching squares from this. Thus will add methods to extract 2D textures later etc, primary implementation is for spatial acceleration. 

The benefit over Spatial Hash grid been we can access neighbouring cells of each particle (along with its current cell) which means we no longer need to ensure the cell size of the hash grid is larger than the kernel radius. 

However we still need to make sure that the Kernel radius is smaller than all adjacent cells combined, else we will get the same issues where the resulting fluid quantities are cropped within the bounds of grid cells and not smoothed correctly to the edges of the kernels radii. 

The solution to this is, rather than getting per particle adjacent cells, we generalize it to positions, so for each position, we return a list of cells, whom lie within the kernel radius, therefore we guarantee that we are use particles whom lie in cells, that are within the distance of the kernel radius, ofcourse this is per cell distance and not per particle. 

As stated above one possible fix of this is to increase scene / simulation scale. 

Need a gather step (per cell gather particles within cell, transformation from index space to world space, gather particles etc.), this is slower than hashing but will be worth it. 

Function to get neighbour grid cell indices / pointers which is easy to do. The grid array itself will be 1D flat and indexed using standard $i \cdot m + j$ function. 









____

#### Fluid Solver

##### Simulation Loop 

Deciding weather to use Per operation approach, or per particle approach, while the second one may work better if we need to eval forces multiple times and keep the previous force value local (without storing to an attrib on particles), we are only using the last steps particle state which is not ideal, it also makes it hard to multithread the outer $pt_i$, but the inner get neighbours and attribute eval loops could be. 

Per Operation approach (for all particles) 

```
GetNeighbours();
GetDensity();
EvalForces();
Integrate() 
...
```

vs. Per Particle (for all operations)

```
for particle p : 
list = GetNeighbours(P);
dens = GetDensity(P);
force = EvalForces(P);
Integrate(P) 
...
```

##### Simulation Loop Implementation

Ok so I just used the normal approach of having per operation functions, where each function itself loops through the particles, all resulting attributes are stored onto the particles themselves. 

Well for evaluation of forces, this is done per particle, so it can be called within a per particle for loop to eval forces twice during leapfrog integration.

A bit confusing mixing vec2 with vec3 ie particles use vec3 but kernels take in vec3s but use vec2s internally etc. Ideally we'd move the whole program to treat particles->vertices as 2D so attribute layout would change on the OGL side also. 

##### Fixed Kernel Radius

As Kernel Radius $h$ / smoothing length is set on Solver construction, so we can precompute the coefficients of the smoothing kernel functions, if we want to add GUI control to adjust it, we'd need to rebuild the solver which is doable and still makes more sense than having it as a free parameter where the smoothing kernel functions would have to be re-eval'd each call entirely as there coefficients could not be pre-computed, unless we did a caching approach where if Kernel radius is changed, then we compute the new coefficients once and store them and read them until its changed again. 

I implemented this and it works fine. The cost of rebuilding is hidden by the UI interaction anyway. 

Would be nice to visually draw the radius of the kernel via a circle, but this is not a prio. 

##### Integration

We are required to use the Leap Frog integration scheme which while been second order, has the property of been symplectic, which for SPH based solvers (and others like mass spring that have oscillatory Hamilton motion) creates much more stable solves as it can be evaluated forwards and backwards in time (preserves area in phase) even more so than using a higher order integrator. 

Re-Eval forces ? 

Oppose to doing eval forces as a sepreate per particle operation, its done within integration per particle loop via a func call taking in a single particle. 

Eval forces should be per particle so we don't need to spilt integration into two particle loops ? 

Do eval forces calls within integration (2 steps) dont even need to store force on pts now (but still will) could just return it directly to body of integrate callee

Min/Max ranges won't work no more, need to eval these separately over all pts.

##### Simulation behaviour

Been a few years since i've used an SPH solver so I forgot how unstable they are even using Houdini's old ParticleFluidSolver I managed to match the explosion behaviour of my solver which seems to be due to too small smoothing radius however my current issue is because smooth radius $h$ is tied to my cell size of the hashgrid increasing it increases to particle-particle cost of $O(nm)$, it also has the issue of visible cells been having the same particle attributes as oppose to smooth radii of particles within those cells hence using hash grid with coupled cell size from kernel radius is probs not ideal. As stated elsewhere I need to use either a uniform explicit grid instead where I can get adjacent cells, with smaller cell sizes (decoupled from $h$) or somehow figure out how to get adjacent cells of the hash grid, which is implictilly defined. 

Rest Density is a key parameter ofcourse, if there is a larger rest density, the pressure will be smaller as as particles cluster and their local density increases if it is still lower than the rest density the pressure won't be too high, however ideally the rest density should be reflective of the fluid and not using higher values as a compromise. 

Increasing Viscosity and Surface Tension forces won't stop expanding / collapsing behaviours they are usually a product of the smoothing kernel radius been incorrect. It could also be due to integration, initially I was testing with Explicit Euler but need to use Leapfrog. 

Houdini uses a mixed Runge Kutta 1 and 2 Method for integration which seems quite stable compared to leapfrog. 

##### Issues

Density or Pressure is tending to infinity after some x number of timesteps, not sure what's causing this, results in pressure force been incorrect and exploding...

____

#### Rendering Fluid

##### Rendering as Vertex Points :

###### 2D Particle Transforms

We can ethier manually transform the Particles + Tank into Clip Space via scaled down by $0.1$ and then offset to framebuffer bottom left origin. Scene is defined within a 0-10 Square cartesian range. Of course for rendering without Camera transforms all calculations are within clip space pre-rasterization so need to transform to screen space, can use model matrix for this strictly speaking its not a model transform but as its called this within Primitive class keep it as is. However it makes more sense to use a Projection transform still via a Orthogonal matrix, because then I don't need to worry about scaling into NDC space, then screen space manually. My actual simulation domain is going to be `[0,10]` (x,y) so use Ortho matrix to transform this into NDC with the (left,right) and (bottom,top) ranges been $[0,10]$. Using this approach with glm::ortho works great. 

I commented out the model matrix from `Primitive` and the shaders as I don't use it here at all, the only transformation is of the projection matrix described above which does the actual world-view/proj space transform. 

###### Particle --> Vertex attribute update Perf

In Debug mode we have problems are with the render time been bottleneck when particle count is increased, its not the GPU cost, but the update cost of the particle --> Vertex attributes within Primitive class because I made a new function to update Positions,Normals,Colours which all have to be written into the current vert_data array (it might be easier just to make a new vert_data array)

 Its more so the particle->vertex update that's so costly ie updating positions,normals,colour each tick. But to be fair in the release build, the optimization seems to speed it up greatly. 





###### Rendering via Implicit Functions in Fragment Shader 

###### Rendering via Grid Rasterize Density

###### Rendering via Marching Squares

