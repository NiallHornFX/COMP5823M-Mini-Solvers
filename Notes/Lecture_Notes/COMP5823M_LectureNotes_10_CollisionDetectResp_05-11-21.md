### COMP5823M - Animation and Simulation - Lecture Notes 10 : Rigid Body Collisions

###### [To Clean-up] (as with all anim/sim module lecture notes currently !)

*As with a lot of this stuff, I'm quite familiar with it, but still writing some notes, so notes may be sparse* also this lecture was not great. 

##### Niall Horn - University of Leeds - 05/11/2021
___
##### Collision Detection : 

Favour simple geometry (Spheres, boxes, capsules convex hulls etc.) or Signed Distance fields.

Sphere Collisions : 
$$
||s_0 - s_1|| <= r_1 + r_2
$$
AABBs + OBBs : AABBs much faster, just min,max of vertices of them to detect overlaps, only possible when parallel hence use of AABBs over OBBs. 

Discrete Oriented Polytope, pairs of parallel faces. 

Convex Decomposition of objects or compounds of collision primitives to approximate mesh. 

Separating Axis Theorem, if an axis can be found where projections are convex, shapes do not overlap. (using line segments projections (ray casting)).

Mentions tunnelling effect of using discrete collision detection. Not been able to resolve collisions due to large timesteps and not using CCD so trajectory/displacement or swept volume is not accounted for. 

Could mention to He about approach of using fixed physics timestep, but accumulating time using game delta time to subdivide timesteps (Glenn Fielder approach), but this lectures already running behind and we are 5 slides in. 

Acceleration of Collision Pair evaluation (BSP, HashGrid, BVH, KD-Tree etc). Need to reconstruct or re-use tree from previous timestep. Tree rebuilds can be costly, but not as costly as not using trees/acceleration structures to partition collision space. 

Collision Detection Phases : Broad, Mid, Narrow phase collision detection. Use of AABBs for Broad phase for initial detection and then test using smaller and more accurate collision detection down to individual primitive's for narrow phase if needed. 

##### Collision Response :

Energy $ E = V + T$ (Kinetic + Potential), need energy conservation. 

Impulse Collision response, shot instantaneous force or velocity applied to resolve penetration. Doesn't account for friction, approximates coefficient of restitution. 

Two moving bodies : Derivation of relative velocity before and after collision. Use restitution coefficient $\epsilon$ to add inverse velocity impulse to resolve collision along normal, accounting for momentum of each body. Change of linear momentum is same for both bodies. 

Shows example of plane collision, to calculate signed distance. 
$$
d = (p_a - p_b) \cdot \hat{n}
$$
Applied to resting contact, if < 0, distance is negative so collision response, projection is needed to resolve interpenetration distance. 

If force doesn't pass through centre of mass, will cause angular motion. 





