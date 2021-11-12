### COMP5823M - Animation and Simulation - Lecture Notes 11 : Rigid Body Constraints

###### [To Clean-up] (as with all anim/sim module lecture notes currently !)

##### Niall Horn - University of Leeds - 09/11/2021
___
Enforcing soft and hard constraints

Strictly enforced hard constraints (joint angles, penetrations, foot planting), numerically more challenging, the more constraints the more difficult to satisfy all (and more iterations needed to try and do so).

Soft constraints are preferred, (acceleration limits, not too rigid), can be formed as energy minimisation problem, where deviation causes non zero energy to minimise. 

##### Energy Minimisation

Desired motion can be formed as a non negative smooth function $E(\psi)$. Find the minima then constraints will be satisfied (fully or partially).
$$
F(0) = \psi
$$

$$
F(t_{i+1}) = F(t_i) - h \nabla E
$$

Ball Falling as energy minimisation problem example :

Initial distance 5m, initial velocity 0 $m/s$ . With simple energy function defined as : 
$$
E = |P-Q|^2
$$

$$
F(t_{i+1}) = F(t_i)- h \nabla E
$$

$P$ is the initial position and $Q$ is the position on a plane below. Essentially just the squared distance.

For timestep 0 $F(0) = H$ where h is the initial height of the ball. Because of this $\nabla E$ gradient is just 2p as Q is 0.

Lowest Energy is achieved when system reaches ideal state. Follows the negative gradient of energy. 



Three Useful functions :

$P(u,v)$ computes positions given u and v

$N(u,v)$ computes surface normal at u and v

$I(x)$ computes the signed distance to the surface. 

##### Example Energy Functions (to minimise)

##### Point-to-fixed-point: 

Attracting point $(P)$ to fixed point $(Q)$
$$
E = |P(u,v) - Q|^2
$$

##### Point-to-Point

Attracting two points together by minimisation, until $P^a$ and $P^b$ overlap where lowest energy is reached. 
$$
E = |P^a(u_a, u_b) - P^b(u_b, v_b)|^2
$$

##### Point-to-point locally abutting

Enforces that two points are 180 degrees apart (abutting) (assuming function is completely minimised to = 0)
$$
E = |P^a(u_a, u_b) - P^b(u_b, v_b)|^2 + N^a(u_a,v_a) \cdot N^b(u_b, v_b) + 1.0
$$

##### Floating Attachment

$\boldsymbol{I}$ is signed distance, energy function = 0, object is on the surface (floating on the surface). 
$$
E = (\boldsymbol{I}^b (P^a(u_a, v_a)))^2
$$
Can combine to the above 2 to get 

##### Floating Attachment locally abutting

[..]

##### Mass Spring Energy Minimisation

$$
E = s(||p2-p1|| - l)^2
$$

As distance decreases (length of spring decreases) the energy increases. The rest length (l) is equal to 1 with s been the stiffness coefficient.

Example with two springs connected at a mid point $X_m$  where $X_m^0 = 0.5$ 
$$
E = s(||p2^1-p1^1|| - l^1)^2 + s(||p2^2-p1^2|| - l^2)^2
$$

$$
E = s(||x_m-l_1|| - 1)^2 + s(||x_m-l_2|| - 1)^2 \\
= x_m^2 - 2x_m + 1 + x_m^2 - 2x_m + 1 \\
= {2x_m}^2 - 4x_m + 2
$$

So the energy gradient *(need to check this)*
$$
\nabla E = 4x_m - 4
$$

$$
F(t_1) = F(t_0) - h\nabla E\\
F(t_1) = 0.5 - 0.01 \cdot 4 \cdot 0.5 - 4 = -2\\
\dots
$$

Witkin and Kass : Space time constraints (Sig 1998). 

Given a particle 
$$
m\ddot{x}(t) - f(t) - mg = 0
$$
Given f the initial value problem. (Explicit Euler), we don't have f though.

Only have Inital and final position. $x(t_0)$ and $x(t_1)$ formulate as energy minimisation problem. Because of these position conditions we need to enforce them as Dirichlet BCs. 

R = $\int_{t0}^{t1} |f|^2 dt$ is the energy consumption given by $|f|^2$ 

Minimization can be formulated as 
$$
p_i = m {x_{i+1} - 2x_i + x_{i+1} \over h^2} - f - mg = 0
$$
Where
$$
c_a = |x_1 - a| = 0\\
c_b = |x_n - b| = 0
$$
$S_j$ values are the $x_i$ and$f_i$ values.

Where $\vec{S}_j$ minimises R irrespective to $C$. 

Need to solve for both the Jacobian and Hessian.
$$
J_{ij} = {\partial C_i \over \partial S_j}\\
H_{ij} = {\partial^2 R \over \partial S_i\partial S_j}
$$
Iteratively Solve : 
$$
- {\partial\over\partial S_j}(R) = \sum_j H_{ij}\vec{S}_j
$$

$$
-C_i =  \sum_j J_{ij}(\vec{S}_j + \vec{S}_i )
$$

Drives Cs to zero and project $\vec{S}_j $ to null space of $J$. 
