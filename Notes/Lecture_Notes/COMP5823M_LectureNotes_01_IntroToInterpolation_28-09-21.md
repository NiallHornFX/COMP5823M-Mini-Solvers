### COMP5823M - Animation and Simulation - Lecture Notes 1
##### Niall Horn - University of Leeds - 28/09/2021
___

#### Module Information
##### 3 Assignments, ~33% each

* Inverse Kinematics
* Cloth Simulation
* Fluid Simulation

*Free Form Deformation used to be an assignment but was removed this year and is instead used as a lab exercise / warm up.*

**Primary Reference Literature** :

* **Computer Animation, Algorithms and Techniques**, 3rd Ed, (Parent, R)

Note while both the module material and my lecture notes are primarily based out of the reference textbook along with other sources, I may change up the variables and notation to what makes most sense to be based on my previous experience / use cases with such equations. An example of this is the textbook uses $u$ to denote the coefficient, where as it makes more sense to use $t$ as this is used is other sources most commonly. 

___

#### Interpolating Values : Basics 
**Interpolation vs Approximation** : The difference been interpolation and approximation is that the function intersects all data points as oppose to just lying close to them / intersecting only some (Including start and end points). 

##### Linear Interpolation

Most basic type of interpolation (first order) a line formed between two data points, interpolate along the line by some $t$ value. 

Can be described in Geometric or Algebraic form, we have two interpolants i.e. coefficients $t, (1-t)$ which always sum to 1. This property ensures the curve (line in this case) falls within the geometric convex hull of entities been interpolated. Geometrically this is denoted in the form of :
$$
P(t) = (1-t)P_0 + tP_1
$$
However in more general geometric form it is typically denoted as :
$$
P(t) = F_0(t)P_0 + F_1(t)P_1
$$
Where $F_0,F_1$ are blending functions between the explcitlly declared geometric points $P_0,P_1$ hence why this is called the geometric form. 

However it can also be re-written algebraically / in polynomial form, where terms are ordered (right to left) based on the coefficients with the variable raised to some power, with the final constant term been the rightmost, in the case of Linear Interpolation the highest degree / power is of course just 1, as Linear Interpolation is first order interpolation. This can be denoted : 
$$
P(t) = (P_1 - P_0)t + P_0 \\ P(t)= a_1t+a_0
$$
Both these forms (Geometric and Algebraic) can be put into Matrix representation, typically the algebraic form is used for this case, and is the form of each interpolation that will most likely be used to compute the resulting interpolated values for our use cases. And thus for matrix representations I will just use this form and omit the geometric version. 

Algebraic Matrix form of Linear Interpolation : 
$$
P(t) = [t, 1]\begin{bmatrix}a_1\\a_0\end{bmatrix} = U^T A
$$
(I may switch to using actual column vectors for all vectors oppose to using transpose notation). Which in its fully expanded form:
$$
P(t) = [t, 1]\begin{bmatrix}-1&1\\1&0\end{bmatrix} \begin{bmatrix}P_0\\P_1\end{bmatrix} = U^T MB
$$
Where $U$ (In the textbook) denotes $t$ the variable vector, M is the coefficient vector and B is the known data points we want to interpolate between. This essentially creates a small $2x2$ linear system. Which could be re-ordered in the form of $Ax=b$ to solve for the interpolants coefficients.  We'll get back to this later. 

##### Parameterisation by Arc Length

Even though the most basic interpolation is linear, there is not necessarily a linear relationship between the changes in the parameters and the distance travelled along the curve (arc length) , hence higher order interpolation schemes can be used, consisting of higher order non linear polynomials in algebraic forms. However this means that non linear curves used, can result in non monotonic / constant velocity like results, where as the interpolation coefficient increases, the distance travelled can become non linear, this is evident in certain interpolation methods / curves. 

Hence we move to methods when interpolation is paremtized by arc length as oppose to just linear length as with linear interpolation, this is vital for using a single non linear polynomial to interpolate data ethier globally or by piecewise. 

!!





Quadratic and Cubic Interpolation equations.

Using Algebraic form as derivatives are easier to solve for u. 

Eg (1-t)A + tB is Geometric formulation, vs Algebraic form ...

Types of curves to use, depending on complexity / order, Continuity.  Global Interpolation (Lagrange Interpolation) of whole curve  vs Local Interpolation between points (Parabolic, Catmull-Rom, Cubic Bezier, BSpline, etc)

Function repersentations 

Explicit, Implicit, Parametric. Ofc curves are parametric. (Lists pros and cons of each)... 

Polynomials named by highest order (degree) term. 

#### Continuity / Smoothness
Continuity, Number of continuous derivatives, however this is typically referred to as Smoothness. Ie a Continuous function has some level of smoothness defined by its C_x. 

(C_0) , continous but not differentiable 

C_1 First order derivative (Linear / First Order)

C_2 etc ..

Piece Wise functions, formed by segments, with possibly different continuties.

Paramtetric Contiunity != Geometric Contiunity

Paramteric : End tangent of first segment is equal to the one at the start of the seocnd segment.

Geometric : Direction would be the sam,e not the magnitude. 

Shared U^tMB

Differ M Matrices (designed, not derived) and Inputs (B vector) (Eg derivatives of points needed for hermite, not for catmull rom)

___

#### Common Methods of Interpolation in CG 

#### Hermite Interpolation
Hermite Interpolation : Cubic Curve, WIth known M Matrix.  Requires 2 points and first derivatives of 2 points form rhs B vector. Can also use in piecewise composite hermite. Know its continous because derivative is specified. Could use for set velocies on each point, because derivatives are specficed explcitlly. 

Was shown with Cubic Curve, although Hermite interpolation doesn't necessarily use Cubic Curves, it can use any. However in CG its typically using Cubic Curves. 

#### Catmull-Rom 
Catmull-Rom Spline, Cubic curve, M matrix , requires 4 input points in rhs B vector. First derivative of pi = (1/2)(pi_+1 - pi_-1). Advantage of not needing to calculate derivatives. 

Four point form, no way to gurantee contiunuity. 

Blended Parabolas for every 3 points fitted unqiue parabaloa Linear interoplaton overlap reigon.

Derivation of Tenison for parabolic interpolation of 3 points.  Negative tension softens parabola, postive sharpens.

Conintuity  of parabolic curve.

Bias shifting the peak point left and right along x to change shape of parabola.  Smaller left, larger right. 

Combined into TCB Control,  derivation of TCB for Left and right side. 

#### Bezier Curves
Bezier Interpolation, Cubic type, uses control points to control tangents of each point, to allow adjusting of slope of curve locally around each point. Only start and end points have 1 control point, intermediate points need two control points for positve and negative control. 

Casteljau construction of Bezier curves. 1/3  along per each previous curve. 

#### B Splines
B Splines, decouple control point number from degree of polynomial like Bezier. Using Knot vector [0, 1 ... n+k -1 ] for k degree, n control point number. Segment of curbe defined by different sets of 4 points (Control points and function points). No Control point is interoploated itself. 

Difference between BSpline and Bezier is BSpline is piecewise between data points, Bezier is global polynomial of data points. 

I asked about Radial Basis Functions for interpolation, but doesnt want to cover it as too complex, due to shape function choices etc. Couldn't help asking, as RBFs are used all over nowadays for animation, deformation etc. 