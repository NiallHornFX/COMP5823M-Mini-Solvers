### COMP5823M - Animation and Simulation - Lecture Notes 03

##### Niall Horn - University of Leeds - 05/10/2021

___

#### Controlling Motion using B-Splines

Curve fitting to positions at discrete timesteps, because its spatial temporal problem, use b-spline for this. 

Knot vector where n is number of points and k is number of control points.  Treat as N as a function  of time t. Solve linear system for the unknown locations of control points. All control points are linearly combined to yield each postion (data point) along the curve (which ensures the spline intersects each position). Same number of varibles and equations means unique solution. Else we have to use pseduo-inverse to get nearest solution. 

___

#### Interpolation of Orientations

Normally interpolate unit quaternions instead of Euler angles. However this results in non constant speed rotations. Because Arc lengths are not equal. Hence the use of SLERP (Smooth Linear Interpolation)

Slerp ..

Paths :

Orientation on path, Frenet frame. (w, u, v) ? Shouldn't it be (U,V,W)

Frenet Frame

W = P(s)'

U = P(s)' X P(s)''

V = u x w

If no P(s)'' acceleration / second derative. Use Interpolation to find last point where there was. Ie Line would yield a second derivative of 0 (as first would be constant).

No Up vector needed.

Discontinuity would yield non unqiue normal and thus it will flip. 

Centered of Interest (col) (Basically approximating tangent by using adjacent location on path, along with "Up" vector)

w = col -pos

u = w x (0, 1, 0)

v = u x w

For moving object can use

w = C(s) - P(s)

u = w x (U(s) - P(s))

w = u x w

Could use other methods like Parallel transport, where basis is only calculated once and then moved along curve.

##### Smoothing Paths

For some number of local neighbouring points, use essentially pieciewise smoothing : 

Linear Interoplation smoothing, Derivation

Cubic Interpolation smoothing, Derivation, also shows numerical approximation. 

For end points use parabolic curves. Again show analytical derivation and numerical approximation derivation. 

Convolution Kernels (Box, Tent, Gaussian). They are centerted at 0, symmetric and approx finite, integrate to 1. Shows Convolution integral (1D ofc). Shows example of tent, for contribution of each point when convolution tent kernel is applied. 

#### Interpolation Based Animation

2D Linear interpolation for points on a 2D polygon. 

Keyframing deformations via interpolation using point-point correspodnence (topology must be the same).

Interpolation of 2 curves : 

Sample equal points along curve, Interpolate corresponding points, Then sample points on resulting interoplant curve. 

Shows examples of scripting langagues for animation (MEL, Houdini (CHOPs) etc.)

#### Deforming Objects

Deformation with interpolations, ie cage lattice FFD, deform lattice and interpolate within its volume. Local coords of grid cell/lattice cell. Shows example of lerp between latice nodes (Bilinear for 2D Grid ofc, 3D is Trilinear).

Picking Pulling / Sculpting using neighbour hood kernel / convolution. 
