### COMP5823M - Animation and Simulation - Lecture Notes 12 : Cloth Simulation

###### [To Clean-up] (as with all anim/sim module lecture notes currently !)

##### Niall Horn - University of Leeds - 12/11/2021
___
*Omitting all the trivial review stuff here. I've implemented Mass Spring cloth simulation before.*

##### Spring Force : 

$$
f_s = -k_s (L_c - L_r)( {p2-p1 \over ||p2-p1||})
$$

Negative stiffness coefficient $-k_s$ by the delta of the length with rest length between two points (massed), multiplied by the direction between the spring (Hooke's law based). 

Could implemented nonlinear stiffness coefficient $k_s$ varying by the current length (function of the length of the spring).

##### Damper Force

Depending on relative velocity between the two points (massed)
$$
f_d = -k_d (\dot{p_2} - \dot{p_1}) \cdot ( {p2-p1 \over ||p2-p1||})  ( {p2-p1 \over ||p2-p1||})
$$
Dot Product of the relative velocity and the unit vector of the position delta, multiplied with the unit vector itself of the position delta. Larger velocity delta, larger damping applied. 

##### Spring Damper Pair Force : 

$$
f = (-k_s(L_c - L_r)-k_d (\dot{p_2} - \dot{p_1}) \cdot ( {p2-p1 \over ||p2-p1||})) \:\:  ( {p2-p1 \over ||p2-p1||})
$$

Combined Spring and Damper force, multiplied by position delta unit vector. 

Leads to Mass-Spring-Damper model and Assignment 2. 

Formulation of apply Explicit Euler to integrate for velocity and position within Initial value problem 0. 

Have to account for masses of course, not assuming each point has uniform mass (1) here, even though its unlikely this system will need varying masses. 

Talks about not been appropriate for volumetric meshes, needing interior volume springs to maintain volume, as only edges are springs, thus inversion, compression etc can occur. Volume Springs, diagonal (shear) springs, Angular (Dihedral) springs etc. 

Angular Spring force :
$$
k_s (\theta(t) - \theta_r) - k_d \cdot{\theta}(t)
$$
Of course adding more springs (and thus forces) to solve, introduces to many parameters. Can be hard to tune, numerically unstable, over-constrained system especially with Explicit Euler for Integration of course (which I'm assuming they want us to use for this assignment). Could clip values (limit force / acceleration) but this just introduces more error in most cases. 

##### PID Control of Mass Spring Systems

Proportional Derivative control (PD) :
$$
u(t) = K_pe(t) + K_d{de(t) \over dt}
$$
Proportional Integral Control (PI) :
$$
u(t) = K_pe(t) + K_i \int_0^te(t^{\prime})dt^{\prime}
$$
Proportional Integral Derivate Control (PID) : 
$$
[..]
$$

##### Particle Systems

Basics about assumptions of particles .. 

Masses assumed to be lumped at points (point masses).

Particle attributes, collisions, states, lifespans etc ... 

Kind of zoned out here as you'd expect given my background !

##### Cloth Simulation (via Mass Spring)

Reformulates as energy based function
$$
E_S = k_s {1\over2}( {|v_1-v_2| - |v_1^* - v_2^*|\over |v_1^* - v_2^*|} )
$$
Now talking about preventing in plane skew, ie via Shear Springs and out plane skew via Dihedral / Angular Springs between faces.

Now talking about Explicit Euler's for integrating force, vel stability again, ie quadratic error $O(t^2)$. 

##### RK4 Derivation :

[..]

Evaluating RHS (Derivative) 4 times at quarter step intervals based on the previous evaluation. Move to half step, eval RHS Derivative (eg velocity), use this to step and then eval again based on this. Switches notation for h = $\Delta t$ not a fan of this derivation. 

##### Implicit Euler

$$
y_{n+1} = y_n + hf^{'}(x_{n+1}, y_{n+1})
$$

Solve linear system for y(n+1) on both LHS and RHS. 

Mentions Baraff and Witkin (Large Steps in cloth simulation) Paper (nice !)

##### Semi Implicit Euler

$$
y_{n+1} = y_n + h f^{\prime} ...
$$

[..] 



##### Stretching / Superelasticity 

Too many springs, overconstrained. 

Biphasic springs (nonlinear) allow initial stretching but becomes stiff after exceeding threshold (like a yield condition). 

Introduce Velocity damping to simulate air viscosity / air resistance. 

##### Cloth Collision Detection

Won't be covered, such a shame I've had fun last year implementing cloth collisions (both self and external). If I get time might implement collision detection for this assignment for some fun. 

Mentions Harmon et al Asynchronous Contact Mechanics paper (2011). 

Collision Response, Dampened Inelastic conditions, Restraining positions/velocities of colliding verts (PBD like Collision response using Spring with infinite stiffness).

Cloth Buckling (Compression and Twist based buckling) research...

