### COMP5823M - Animation and Simulation - Lecture Notes 05: Shape Interpolation and Kinematic Linkages

##### Niall Horn - University of Leeds - 12/10/2021
___
#### Interpolation Between nD Shapes

3D Shape Interpolation

* Surface Based : Vertex Matching, Vertex Based Interpolation, Difficult to handle holes in topology. (FFD, Pick-Pulling modify surfaces only)
* Volume Based : Blending Volumes deformations, less sensitive to surface topology, more expensive as requires full volumetric representation. 

##### Relative definition of topologies

Topology From pure mathematical point of view :

* Connectivity of Surface, Genus, etc. 
* Two objects are homeomorhpic if there continuous one-to-one and inverteable mapping between points of surfaces between two objects. Ie Torus --> Coffee Cup example. Points between mapping / morping position trajectory do not cross each other, hence continuous mapping. 

In Computer graphics, Topology is referring to the vertex/edge/face connectivity of the polyhedron and is invariant to the position or orientation of the object. 

3D Shape Interpolation Problems to solve : 

* Correspondence Problem, Between 2 sets of points / vertices
* Interpolation problem from initial / rest to target corresponding point. 

Siggraph Papers of recent research for 3D Shape Interpolation, for interpolating between meshes where both sets have ethier full correspondence or partial correspondence of topology. 

* Mobious Voting for surface correspondence

* Dense Human Body Correspondence (Interpolating between body scans with different topologies).

___

##### 2D Example of Shape Interpolation : 

Defining Same coordinate system on two objects, points on the same axis have correspondence, thus interpolation of the axes can be used, eg Interpolation between a square and a star, diagram shows linear interpolation result.

##### 3D Example of Shape Interpolation :  

Matching topology, same local coordinate system going through each shape, axial slices intersecting along mesh, defining correspondence between slices. Unlike 2D Orientation of interpolation needs to be considered. As topologies are the same its still just a pure correspondence problem. 

Connelly et al 2002, interpolating between points, without line segments colliding / intersecting. Uses energy functions to expand into convex shape (repulsive). In 3D : 

Advanced 3D Interpolation - An energy-Driven Motion planning for two distant postures (Wang Et Al) uses non linear interpolation sceheme, body is discretized into linkages, not purely a deformation problem. 

Other methods : 

* Again shows slicing 3D Shape, and then using 2D Interpolation for each slice. 

* Mapping objects to unit sphere. (Sig 1992, Shape Transformation for Polyhedral Objects)

* Wang, Harmonic Parameterisation by Electorstatics (He plugging a lot of his papers here ! ) Parameterisation for correspondence mapping using intermediate unit sphere representation. Then it becomes a simple sphere mapping problem for correspondence. 

##### 2D Morphing : 

Morphing one image to another, User defined correspondence, User controlled Transformation. User control makes interpolation diffucily ofcourse but is needed for artistic control. 

Nonlinear interpolation of 2D Coordinate Grid. Taking a source image and a user defined destination image, do immediate interpolation for grid time t. Then warp both source and destination image to match the intermediate grid (which is then implicllty used as a common coordinate system between each (Source, Target)) this then can be used to simplify the correspondence between each ie "Cross Disolve "like final interpolation. Two Pass Process. 

To form intermediate grid to use as the common coordinate system between the two. This is done using an Auxillary grid which uses x coords of Source and Y Coords of Target, this then forms the intermediate grid. To do this, we use horizontal scan line approach mapping each rows Pixel coords to the axuillary grid, grid cooridnates, For the first grid (Source) we use x coords, for target we use y coords.  Auxillary grid forms the intermediate grid. Ie a Coordinate system / common grid space of two source images User defines sample points, Curves between points is ofc interpolation based, that then leads to formation of aux grid. 

___

*Wanted to ask He if any of his research has been done in terms of morphing for SDFs, where there's no concern about maintaing topology, as the Levelset function can just be re-computed to correct post operation. Mention Implementation in VDB libary for morphing. Ofc creates issues because the output mesh then does not have consistent topology as Marching Cubes based meshing genereates temporally independent topology*.

I also mentioned the paper Jake Rice wrote about using optimal flow, for doing point - point correspondence (in the case of temportal correspondence of points with a particle fluid sim) from a statistical approach in the spirt of using different areas of maths applied to solve novel problems. 

___

#### Kinematic Linkages

Moving on from interpolation now.

Rigid Links connected by joints. Defined by some root node (Translation and rotation), with local rotation of each joint, to define pose for total linkage. Calculating position of end effector using rotation. Showing local vs global interpretation of joint angle rotations. 

Local $\theta$ rotation angle, relative to previous joint. Ie each joint angle is local with respect to the previous joint. 

Two approaches : 

* **Forward Kinematics** (Evaluated from root, to effector in forward order)

* **Inverse Kinematics** (Evaluated in reverse from end effector of last linkage, back to root). 

##### Forward Kinematics : 

Showing example in 2D using Homogeneous Coords $(x, y, w)$ with 2D rotation. In 3x3 Transformation matrix

(Clockwise) 2D Rotation in 3x3 Homogeneous Transformation Matrix : 
$$
\begin{bmatrix} \cos\theta&-sin\theta&0\\sin\theta&cos\theta&0\\0&0&1 \end{bmatrix}
$$


Stacking transforms from previous link, to compute joint angles and thus global position of joint. Thus Forward Kinematics is to  Calculate global coordinates of joints given local rotations. Ie successive transformation matrices applied from root to end effector via traversal along tree (Depth First).

Shows example Posing via rotation of joints in Maya, forward kinematics evaluation of bones to compute hand position in global (world space) based on local joint rotations. 

Child represented in local coordinate system of parent.

##### Joint Types

Revolute Joint : 1DOF

Prismatic Joint  : 1DOF

Ball and Socket : 3DOF

etc.

Human Skeleton Tree data structure representation. Each joint represented by node in a tree.

##### Inverse Kinematics : 

Solved inversley from known end effector global / world space location, calculate angles of each joing, back to the root node. Ie the inverse of forward kinematics. Will cover this next lecture. As this will from the first assignment. 

