### COMP5823M - Animation and Simulation - Lecture Notes 16 : Lagrangian (SPH)

##### Niall Horn - University of Leeds - 26/11/2021

___
SPH : Each Particle has Mass, Density and influence of nearby area (depending on kernel radius)

Reference SCA Paper : Particle-based Fluid Simulation for Interactive Applications.

##### Smoothing Kernel Function 

Particle attributes computed at any location using Smoothing Kernel Function
$$
A_s(r) = \sum_j m_j {A_j\over \rho_j} W(r-r_j,h)
$$
Where J iterates over all particles, m is the  mass $r_j$ is the position, $\rho_j$ is the density and $A_j$ is the field quaintly at $r_j$. $h$ is the radius of the smoothing kernel itself.

The actual kernel function is the RHS, ie its a weighted average multiplied by the kernel function $W$ typically denoted $\omega$. 

Eg for computing density $\rho$ :
$$
\rho_s(r) = \sum_j m_j {\rho_j\over \rho_j} W(r-r_j,h)
$$
Derivative would then be :
$$
\nabla A_s(r) = \sum_j m_j {A_j\over \rho_j} \nabla W(r-r_j,h)
$$
Same for Laplacian (Second derivative)

##### Side Notes : 

Mass conservation "not needed" anymore. Well it kinda is, but its enforced via forces via smoothing kernel to particles as a result of pressure. He says Mass conservation is not needed, and reframes it as just applying the neg pressure gradient and RHS of the momentum equation (with the Laplacian viscosity term). We still need pressure which itself is a product of mass conservation, ie the density disparity itself causes pressure deltas, if mass conservation was not needed, this would not occur, as the density would not influence pressure and form different pressure areas so not a great explanation. 

Thus we calculate the pressure gradient using the above smoothing functions derivative.

"We don't deal with a velocity field anymore". Well we do, but the field is defined by the particles, as oppose to a fixed grid.  I don't mean to be picky, but I feel like his explanations are not good. 

Viscosity Laplacian of Smoothing Kernel applied to velocity, depends on velocity differences
$$
f_{visc} = u \nabla^2v(r_a) = \mu \sum_j m {v_j \over \rho_j}\nabla^2 W(r_i - r, h)
$$
Surface Tension applied to particles at interface. Not part of Navier Stokes as it only applicable to interface particles.  To determine the interface particles, they seem to use a mask/stencil  grid approach where we discretize particles to grid and mark air vs fluid cell, from this can get particles of interface if their cell is adjacent to an air cell. SDF Would be another approach.

##### Surface Tension Force

Calculate curvature using -Lap / N (grad). Cbt to this derivation [..] \sigma is coeff for Surf tension.

##### Kernel Functions

Need a good Kernel function for the Smoothing function to evaluate particles values within neighbourhood. 

* Need a smooth function.

* Typically will have vanishing derivatives at boundary, tends to be more stable. 
* Needs to have non zero derivatives at $W(0, h)$ *(to be able to use for Gradient and Lapacian for pressure and viscosity forces)*

$W(r, h)$

Where $r$ is the location of the particle to eval at, $h$ is the radius (confusing notation !).

***Poly6*** : Can't use for pressure as when function is near 0 (derivative is 0), thus if particle $r_i - r_j$ overlap (high density) gradient is zero when function evals to 0, which is incorrect and won't be corrected (will cluster), pressure should be highest here as density is atop. So we need a Kernel function whose derivative is smooth and not zero at any point to ensure correct pressure gradient force is calculated. 

[..]

***Spiky Kernel*** : is better approach for pressure as gradient results in high value (high derivative) as function evaluates to 0, but its second derivative and thus Laplacian becomes near zero, so not useful for viscosity. 

[..]

So can just use Poly6 for Viscosity (Laplacian) Kernel Function, use Spiky for Pressure Gradient Kernel.



Kernel function that has good  first and second derivatives (and thus can be used for both gradient and Laplacian for pressure and viscosity calc respectively) :

***Gaussian*** : nn

[..]

##### Leapfrog Method for Integration

$$
[..]
$$

Don't use Explicit Euler for Integration.

##### Acceleration Implementation

Use uniform grid or hash grid for neighbour hood lookup acceleration



##### Visualize Surface

Not required for assignment (render as particles)

Point Splatting (can be done in screen space)

Marching Cubes (would have to rasterize particle to grid first)
