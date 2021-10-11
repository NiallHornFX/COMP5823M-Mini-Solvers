### COMP5823M - Animation and Simulation - Lecture Notes 02

##### Niall Horn - University of Leeds - 01/10/2021

___

#### Interpolating Values

Want to control higher order information aswell as just positions (ie using tangents of curve).

Points along curve may not be evenly distributed along curve, distance changes as non linearity of curve increases. 

##### Computing Arc Length between points along curve

Arc Length Integral Derivation [..] Approximate length of curve by line segments between points (data points/key-frames in this context), compute arc length between points (to approximate whole curve), this can be derived using the derivative of the curve integrated over n steps between each point, limited to infinity. Derivation from Euclidean distance, to arc length integral...

Controlling motion - computing arc length :

Analytically. (Example for cubic curve), points are functions of u (because ofc this defines there postion in cartesian space).

Distance = Arc length along curve. 

 Solve either of these in different context :

* Given ua, ub find lenght(ua, ub) (arc length)

* Given arch length s and a and parameter value ua,ub such hath length(ua, ub) = S (known arc length) finding solution of s - length(ua. ub ) = 0

Integrate to solve arc length over interval ua,ub by du., each component of p(u) is indepenent, thus decompose into seperate differential equations to integrate (Partial derivatives). Sub dp/du into the sqrt polynomial eq. But not every term will show up in polynomial depends on curve ofc. 

So First calc derivative of each and sub into integral equation.

Cannot solve closed form polynomial interpolation (via integration) beyond quartic ofc.  

Computing Arc Length - Forward Difference method aka f(x + h) - f(x) / h, but ofc would use discrete data points not function pt_i+1 - pt_i / h. Ie using segmentd deriv tangnent (approximating as linear segments) line along interval to compute arc length. Can be seen an linear interpolation between two points on curve esentially. 

Adaptive forward diffrencing, in smaller steps.

Central difference not useful because you want arc length between two points so using -,+ around each point would yield a derivative approx that would not be useful for approximating arc length between two points. Can cause over shooting / undershooting between points though. 

Computing Arc Length - Numerically (Simpsons, trapezodial integration, guassian quadrature)

Simpsons Integration : f(a) + f(m) + f(b) Approximate integration by [f(a) + 4f (a+b/2) + f(b)]

Trapeziodal Integration : Use average (midpoint) between f(a) and f(b) as triangle height (with a-b base) ofc a-b is subdivided over dx, so that the limit of the rect/trap count to infiinity yields solution. Because using midpoint of line between a-b can do poor for complex arcs. 

Gaussian-Quadrature : Approximate integral using two functions, g(x) is a polynomial while omega could be chebyshev-gauss or gauss-hermite etc. 

Adaptive Gaussian Integration is ^ subdivided. 

Formulate as nonlinear and use Newton method (s - length(ua, u) = 0, assuming u is given). Problem is no gurantee compute pn will be still on the curve due to solve error. Can find closest point to p and then use this. No longer need to integrate over segments / dx as its Ofc this is way more expesnive than integration (even gaussian quadrature and not really used for realtime ofc (Newtons method is too costly)).



Execrcise Forward difference arc length for some high order solve. 



#### Animation Curves

*(Easy for me as mapping values to curves was a big part of my job lol)

Distance time examples, have monotonic property (ideally, unless they overshoot/undershoot)

eg ease-in, ease-out

 (ease in, ease out) (eg a segment from sin (which is peridoic and ofc is constant easein, easeout)).

Showing pieciewise /semengted curves to allow ease-in - linear - ease-out. 

Enusre continous by caluclating first order deriatvies at each segmented start-end point.

Just using a cubic polynomial would create ease-in-ease-out anyway. 

Showing plots of DistTime, VelTime, AccelTime (Basic physics stuff, shows diffent curve shapes) splitting into these allows more control, that just DistTime curve. Assumes we are talking about postions over time, not rotations for now it seems, although could assume dist means distance from starting angle to end angle. 

Or instead start from VelTIme curve and integrate back to DistTIme (PosTime) curve. For users speciying velocitytime oppose to postime. 

Shows differnt anim curves.
