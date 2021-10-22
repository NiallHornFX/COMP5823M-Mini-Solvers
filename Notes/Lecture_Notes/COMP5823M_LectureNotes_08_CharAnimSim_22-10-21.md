### COMP5823M - Animation and Simulation - Lecture Notes 08: Character Animation & Simulation

##### Niall Horn - University of Leeds - 22/10/2021
___
##### Intro

Perceptual point of view, i.e. avoiding the uncanny valley phenomena.

 High Complexity of human > 200 bones, > 600 Muscles approx. 200 DOFs, deformable skin, flesh, tissue, hair, clothing. 

##### Geometric Data Acquisition

Scanning (eg, structured light, depth based, photogrammetry etc) Point Clouds -> Patches -> Surfaces

4D Volumetric Capture. 

##### Geometry Deformation

Decomposed into multiple rigid body parts

Rigged Bones -> FK/IK solver. With skinning eg Linear Blending, Dual Quaternion etc. Artist defined vs computed skin weights etc. 

##### Layered Approach

Use Deformation methods like FFD to approximate internal volumetric motion from fat/flesh/muscle oppose to using simulation, weakly coupled deformation between them.

##### Skinning

Weighted combination of nearby joints. Skinned vertex would store joint indices and joint weights within its influence (final weight doesn't need to be stored as it can be computed by subtracting others from 1).  

Vertex Bound to joint by transforming into joint space. Joint is modified within joint space, and then transformed back to model space (model space here refers to world space i.e. the models current space).

1. Transform into Joint Space
2. Move joint into current pose
3. transform back into model space

Derivation of Skinning Matrix for simple case of single joint. 

[..] Simple Skinning Matrix

For multiple joints skinning would be weighted sum, ie weighted combination, shows linear blend skinning derivation (weight of joints are combined linearly). However its old and suffers from volume collapse. 

[..]  LBS Derivation

 Linear blending doesn't consider the boundary of the domain of the defined transformation space of joints, where as Dual Quaternion does, as it considers the arc around the transformation, i.e. similar to interpolation of quaternions. 

Delta Mush skinning seems quite hot now.

##### Joint Constraints

IK does not enforce joint limits, even if enforced, still under constrained. Can optimize so resulting motion is smooth.

Computing intermediate poses of positions, then do minimization of linear and angular acceleration, subject to valid joint ranges. Can do this via Quadratic programming and Lagragian Multiplier via equality constraints. 
$$
minimize \:\: {1\over2} X^TQX+c^TX
\\
subject\:\:to \:\: Ax \approx b
$$
Most complicated joints are shoulder and hands, cannot just approximate as typical ball socket joints, has other constrained degrees of freedom like translational components. 

##### Motion Matching

Reaching and grasping : Coordinated Motion, More DOFs involved. I.e. Motion planning Could use methods like Potential Fields for obstacle avoidance, cells within boundary of obstacles are given higher values than open areas. Couldn't SDFs be used for this, using their distances and gradients ? 

Global Path Planning : (Probabilistic Roadmaps, PRMs, Rapidly Exploring random trees).

Need to account for strength to generate force for manipulation. Given a posture needs to be able to evaluate the maximal torque and force output. 

Walking, Cyclic/Acyclic motions, has multiple purposes: transportation of centre of mass and keeping balance. 

Automated walking approaches eg Yin et al. Siggraph 2007 using finite state machine approach.

Deep Learning approaches  eg Holden (Ubisoft La Forge)

##### Garment / Clothing

Automated Garment Generation (panel generation) - Berthouzoz  et al. (SIggraph 2013).

Cloth Capture 

Cloth Simulation (FEM/Baraaf, PBD, Mass Spring etc)

##### Hair

Strand Based eg (Mass spring, PBD, Cosserat/Elastic Rods)

Data Driven Methods (Chai et al. Siggraph 2017).



