#### COMP5823M - Assignment 1 - Inverse Kinematics - Notes

##### Niall Horn - University of Leeds - 2021

___

#### Initial Notes

Using tree structure to represent kinematic linkages of body. We have joints and links, joints been the actual rotational joints between links which are the kinematic linkages / appendages that are joined together by them. However we will change this terminology for the sake of the assignment because I'm going to be implementing a tree structure to represent the hierarchy of the skeleton defined by BVH file, and most implementations use the nodes of the tree to represent body parts / linkages and the links / connections of the tree define the joints themselves. This make sense as it allows multiple joints emanating and connecting to each linkage. 

My Aim is to have a App with a viewport, DearImGui to for control which I may implement via a controller class, with mouse pick-point to define end effector positions, load BVH Files, some sort of timeline to scrub the animation. Control for which IK method is used, etc. Then for visualization of the skeleton as lines, but ideally as geo, have a ground plane / grid, and some sort of viewport fog/horizon if there's times will implement shadows from a headlight source, basically its gonna be a tiny anim app (but you cannot set/store keyframes etc) its mainly for loading BVH Files and writing them back out based on user modified end effectors and resulting joints. Might be a good idea to follow Model,View,Controller paradigm for this app. Eigen will be used for Linear Algbera, Modern OpenGL for rendering. 

BVH File defines Hierarchy of joints at each frame, for some number of frames. Parsing it shouldn't be too much work as it is ASCII based. 