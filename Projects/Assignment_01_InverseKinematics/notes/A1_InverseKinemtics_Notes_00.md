#### COMP5823M - Assignment 1 - Inverse Kinematics - Notes

##### Niall Horn - University of Leeds - 2021

___

#### Initial Notes

Using tree structure to represent kinematic linkages of body. We have joints and links, joints been the actual rotational joints between links which are the kinematic linkages / appendages that are joined together by them. However we will change this terminology for the sake of the assignment because I'm going to be implementing a tree structure to represent the hierarchy of the skeleton defined by BVH file, and most implementations use the nodes of the tree to represent body parts / linkages and the links / connections of the tree define the joints themselves. This make sense as it allows multiple joints emanating and connecting to each linkage. 



Aim for project : A viewer application with a viewport, DearImGui based IM Mode GUI for control which I may implement via a controller class, with mouse pick-point to define end effector positions / position constraints, load BVH Files, some sort of timeline to scrub the animation. Control for which IK method is used, etc. Then for visualization of the skeleton bones as lines, but ideally as bone geo as well, along with spheres for joints and target positions/end effector. Have a ground plane / grid, and some sort of viewport fog/horizon if there's times will implement shadows from a headlight source light, basically its gonna be a tiny anim app (but you cannot set/store keyframes etc) its mainly for loading BVH Files and writing them back out based on user modified end effectors and resulting joints configuration. Might be a good idea to follow Model,View,Controller paradigm for this app. Eigen will be used for Linear Algebra, Modern OpenGL for rendering. 

While IK is used for solving the joint angles from the BVH joint configuration, FK is used to then define the transforms for rendering / viz. 



___

BHV File : 

BVH File defines Hierarchy of joints at each frame, for some number of frames. The Joints are defined in rest position, then at the bottom half, each line defines each joints offset per frame. It can be parsed as a recursive tree of joints starting from root, where each joint has its own child joints down to each end joint / leaf. 

Consists of Offset per joint and Channels per joint. Only difference is Root for the root joint and End Site defined for the end joint. There's no definition of bones, they are implicitly defined between joints. 

* Offset : Position offset in world space relative to parent joint.

* Channels : DOFs per joint



___

Skeleton Tree Data Structure

If we use nodes of a tree to represent joints as that would seem logical in terms of how joints are the nodes of bones in a skeleton, it would mean that we each bone thus tree segment could only be connected by two joints at most, however each bone can actually have multiple joints connecting to it, and thus it makes more sense to inverse this logic and have the nodes represent the bones and the segments represent the joints. 

Each Bone 

However for BVH it might make more sense to use a the former idea ? 