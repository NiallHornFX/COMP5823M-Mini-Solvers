

### COMP5823M - Animation and Simulation - Lecture Notes 1

##### Niall Horn - 28/09/2021

___

#### Module Information

**10 Weeks, 20 Lectures**

**12 hour labs (week 4, 7-11)**

Week 5 - Reading week.

3 assignemnts and self study (2/3 weeks) 100% total grade, no exam this year. 

Should start lab activties before lab session, as soon as released. 

Reference Textbook : Rick Parent

Implementation : FFD, IK, Cloth Simulation, Fluid Simulation. 

___

#### Interpolating Values : Basics 

Basic info about what is animation, specifying key frames and using interoplation to generate inbetween frames. Not writing notes on this.

Interpolation vs Approximation, difference been that interoplation, function intersects all points (keyframes) vs just lying close to them / intersecting only some (Including start and end points). 

Seeing joint angles as high dimensional space (but it never is, its always reduced to 2 dimensions, f(t) = angle for each joint axis, so not sure why this concept is even mentioned).

Linear Interpolation, Geometric (function of u or t). vs Algebaric form derivations, form linear system, to solve for coefficents or expad the coefficents out to matrix form and solve for P values directly.

Quadratic and Cubic Interoplation equations.

Using Algeberaic form as derivatives are easier to solve for u. Not a fan of his notation choices !

Types of curves to use, depending on complexity / order, Continuity.  Global Interoplation (Lagrange Interpolation) of whole curve  vs Local Interoplation between points (Parabolic, Catmull-Rom, Cubic Bezier, BSpline, etc)

Function repersentations 

Explicit, Implicit, Parametric. Ofc curves are parametric. (Lists pros and cons of each)... 

Polynomials named by highest order (degree) term. 

Conitnunity, Number of continous derivatives

Zeroth  Order (C_0) , continous but not differentiable 

C_1 First order derivative (Linear)

C_2 etc ..

Piece Wise functions, formed by segments, with possibly different continuties.

Paramtetric Contiunity != Geometric Contiunity

Paramteric : End tangent of first segment is equal to the one at the start of the seocnd segment.

Geometric : Direction would be the sam,e not the magnitude. 

Shared U^tMB

Differ M Matrices (designed, not derived) and Inputs (B vector) (Eg derivatives of points needed for hermite, not for catmull rom)

Hermite Interpolation : Cubic Curve, WIth known M Matrix.  Requires 2 points and first derivatives of 2 points form rhs B vector. Can also use in piecewise composite hermite. Know its continous because derivative is specified. Could use for set velocies on each point, because derivatives are specficed explcitlly. 

Catmull-Rom Spline, Cubic curve, M matrix , requires 4 input points in rhs B vector. First derivative of pi = (1/2)(pi_+1 - pi_-1). Advantage of not needing to calculate derivatives. 

Four point form, no way to gurantee contiunuity. 

Blended Parabolas for every 3 points fitted unqiue parabaloa Linear interoplaton overlap reigon.

Derivation of Tenison for parabolic interpolation of 3 points.  Negative tension softens parabola, postive sharpens.

Conintuity  of parabolic curve.

Bias shifting the peak point left and right along x to change shape of parabola.  Smaller left, larger right. 

Combined into TCB Control,  derivation of TCB for Left and right side. 

Bezier Interoploation, Cubic type, uses control points to control tangents of each point, to allow adjusting of slope of curve locally around each point. Only start and end points have 1 control point, intermediate points need two control points for positve and negative control. 

Casteljau construction of Bezier curves. 1/3  along per each previous curve. 

B Splines, decouple control point number from degree of polynomial like Bezier. Using Knot vector [0, 1 ... n+k -1 ] for k degree, n control point number. Segment of curbe defined by different sets of 4 points (Control points and function points). No Control point is interoploated itself. 

I asked about Radial Basis Functions for interpolation, but doesnt want to cover it as too complex, due to shape function choices etc. Couldn't help asking, as RBFs are used all over nowadays for animation, deformation etc. 



