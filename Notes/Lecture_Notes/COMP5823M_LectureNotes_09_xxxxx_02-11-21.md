### COMP5823M - Animation and Simulation - Lecture Notes 09 : Rigid Body Dynamics

###### [To Clean-up] (as with all anim/sim module lecture notes currently !)

##### Niall Horn - University of Leeds - 02/11/2021
___
##### Rigid Body Dynamics (very) Basics

Newtonian physics, Rigid Bodies, Constraints (Collisions, etc).

Linear vs Angular : 6 DOF (3 DOF translation, 3 DOF Rotation)

##### Linear Motion

Centre of mass defined at one point (simplified model), weighted sum of mass points.

Linear Dynamics (Position, Velocity, Acceleration), basic time derivative formulation of displacement, velocity acceleration.  
$$
x(t) = x_t 
\\
\dot{x}(t) = {Dx\over\Delta t}
\\
\ddot{x}(t) = {D\dot{x}\over\Delta t}
$$


Force Newton second law. Net force is sum of forces. Linear Momentum (vel * mass). Relationship between momentum and accel. 
$$
p(t) = mv(t)
$$


Spring Force Coeff -k and damping Coeff -b (damping on velocity, so not really a force). 

Generalise to ODE formulation, for nth order derivative. Accel = Force / Mass uses notation to denote each time derivative (t, r(t), v(t)).

Shows basic analytical solution (integration) for gravity. v_ti+1 = v(ti) + (f_new(ti) / m) * dt, i.e. explicit Euler. 

##### Numerical Integration

Explicit Euler, step along tangent line of derivative by some timestep, divided into smaller timesteps to better approximate shape of function/curve and thus underlying function to integrate. Thus small timesteps are needed to reduce error. Only first order integration, rhs (derivative) is only evaluated once per timestep.  

Numerical Integration properties :

* Convergence

* Order
* Stability

Shows explicit Euler derived from Taylor Series, to show order and error term. 
$$
x(t_2) = x(t_1) + \dot{x}\Delta t + {1\over2} \ddot{x}\Delta t^2 + \dots
$$
Take terms after first order term, first remainder term is the error (second order term), hence for Explicit Euler is $O(\Delta t^2)$. Thus 

Explicit Euler *(with error term)* :  
$$
x(t_2) = x(t_1) + \dot{x}\Delta t + O(\Delta t^2)
$$
Error accumulates over time, reduces stability over time. Hence the need for higher order integration schemes.  Midpoint (RK2), high order Runge Kutta (RK4) etc. 

Implicit methods like Implicit Euler, form linear system to solve for both terms on LHS and RHS, allow for larger timesteps with stability. 

Velocity-less Verlet :


$$
x(t_2) = 2x(t_1) - x(t_0) + a(t1)\Delta t^2
$$
Second Order based scheme, integrates acceleration directly to position, error is $O(\Delta t^4)$ , can be derived to show it integrates second order central difference. Show derivation from Taylor series. 

Velocity Verlet :

Decouples second order accel-position integration, Evaluate velocity along half timestep, then eval acceleration along half step, and then do full velocity integration along full Dt. Like RK2 would be used for integrating position. Shows Integration for velocity from accel : 

##### Angular Motion

Moment Of Inertia (Rotational centre of mass), represented by matrix. 

Angular Velocity and Angular Acceleration derivatives. Vel = Delta theta / delta time. Accel = Delta omega / delta time. 

Torque is rotational force $N = r \cross F$ (denoted N), is result of a cross product. Multiplied by objects moment of inertia.  

Angular vel has both Tangential (Parallel) and Normal (Orthogonal) components. 

Shows solving angular vel using Explicit Euler, define angular accel as :
$$
\omega(t_2) = \omega(t_1) + {N_{net}(t_1) \over I} \Delta t
$$
Thus angular vel integrated (Explicit Euler):
$$
\theta(t_2) = \theta(t_1) + w(t_1)\Delta t
$$
Shows Velocity Verlet for integration of Angular Acceleration solving for Angular Velocity, using half step velocity approach.

[..]

Inertia Tensor (Moment of inertia) (3x3 matrix), inertia a long each axis.

##### Orientation

Fill in using lecture slides / Parent book. Integrate from angular acceleration to calculate timesteps body orientation quaternion. 

Use Quaternions for orientation typically, as defined by : 
$$
q = [w,x,y,z] = [cos({\theta\over2}), \vec{\boldsymbol{u}}\:{sin {\theta \over 2}}]
$$
$\vec{u}$  defines rotational axis of course, by angle $\theta$. Rotational axis itself is function of time, as rotational axis is changing over time. 
$$
\omega(t) = \omega_u(t)u(t) = \dot{\theta_u}(t)u(t)
$$
Linear relationship between torque and angular momentum, because :
$$
{d\over dt} I\omega(t)
$$
Angular momentum needs to be conserved for integrating to angular velocity. Angular velocity thus is inverse of Inertia Matrix by the torque N = L(t). L = angular momentum, Derive Angular Velocity Quaternion Integrate back to calc orientation.

