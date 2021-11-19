#### COMP5823M - Assignment 1 - Inverse Kinematics - Notes

##### Niall Horn - University of Leeds - 2021

___

#### Misc Notes

Aim for project : A viewer application with a viewport, DearImGui based IM Mode GUI for control which I may implement via a controller class, with mouse pick-point to define end effector positions / position constraints, load BVH Files, some sort of timeline to scrub the animation. Control for which IK method is used, etc. Then for visualization of the skeleton bones as lines, but ideally as bone geo as well, along with spheres for joints and target positions/end effector. Have a ground plane / grid, and some sort of viewport fog/horizon if there's times will implement shadows from a headlight source light, basically its gonna be a tiny anim app (but you cannot set/store keyframes etc) its mainly for loading BVH Files and writing them back out based on user modified end effectors and resulting joints configuration. Might be a good idea to follow Model,View,Controller paradigm for this app. Eigen will be used for Linear Algebra, Modern OpenGL for rendering. 

While IK is used for solving the joint angles from the BVH joint configuration, FK is used to then define the transforms for rendering / viz. 

While I'd really love to write detail notes again this project is due in 2 weeks and I have so much to do. So rough notes and maybe write up later. 

##### External Libraries : 

* OpenGL (w/ GLFW + GLEW) : Rendering
* DearImGui : UI
* Eigen LinAlg

##### Main Components to do : 

* BVH File Parser, Loader, Writer.  
* Skeleton Data Structure from BVH
* Rendering of BVH File.
* GUI + OpenGL Application for viewing Bones and Joints of loaded BVH File
* Basic Pseudo-Inverse Inverse Kinematics Implementation to allow use modification of end effectors. 

___

#### BHV File 

BVH File defines Hierarchy of joints at each frame, for some number of frames. The Joints are defined in rest position, then at the bottom half, each line defines each joints offset per frame. It can be parsed as a recursive tree of joints starting from root, where each joint has its own child joints down to each end joint / leaf. 

The First half of the file denotes the skeleton hierarchy starting below the `HIERACHY` header, with the first joint been the root node (typically hips). Each joint is then a recursive hierarchy of child joints, until the `End Site` is reached i.e. the leaf node of that branch, this also has an offset and will be used for the end effector in IK context later. 

Technically they are segments with joints be defined after the joint keyword. But we just treat these segments/bones as joints (unless they are root or end site) and the bones are implicitly defined between them. 

Each joint, consists of Offset per joint and Channels per joint. There's no definition of bones / linkages themselves, they are implicitly defined between joints. 

* Offset : Position offset in world space relative to parent joint.
* Channels : DOFs per joint (Position and Rotation)

The second half of the BVH file denoted beneath the `MOTION` header is where the per frame animation data is listed. The total number of animation frames is listed, and then for each frame starting with `Frame Time:` the timestep of the frame, the resulting channel values are then listed below to define each joints channel values, these are listed on a single line and are in the order of the tree.

```
HIERARCHY
ROOT Hips
{
	OFFSET 0.000000 0.000000 0.000000
	CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation 
	JOINT LHipJoint
	{
		OFFSET 0.000000 0.000000 0.000000
		CHANNELS 3 Zrotation Yrotation Xrotation
		JOINT LeftUpLeg
		{
			OFFSET 1.363060 -1.794630 0.839290
			CHANNELS 3 Zrotation Yrotation Xrotation
			JOINT LeftLeg
			{
            	OFFSET 0.000000 0.000000 0.000000
           	 	CHANNELS 3 Zrotation Yrotation Xrotation
            	End Site
            	{
            		OFFSET 0.000000 0.000000 0.000000
            	}
			}
		}
	}
}
MOTION
Frames: 1
Frame Time: 0.041667
0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 
```

Typically only the root has 6 channels, with the children have 3 channels. 

___

#### BVH_Data Class :

`bvhdata` is where the `BVH_Data` class is declared, This file also is where joint and channel are declared. 

Plan to have both the Loading,Parsing and Writing within the same class, this class will also store the parsed state of the Skeleton / Tree. 

This will contain the time based state of the skeleton tree, in the form of per channel frame data loaded from the original BVH file.  This will later be modified from the IK Solve. 

Maybe I can serialize and extract this out and store in some separate skeleton class later. 

First part of the parsing is checking which part of the BVH File we are in, secondly we check if we are in a ROOT or JOINT like, if so we create a new joint, and then for each successive line we update its data, we need to create a local stack to push each new joints data into (because of the recursive nature of the hierarchy).

So because parsing is line by line, each new loop iteration has no knowledge of previous hence the need for a stack of joints.

Each joint has array of pointer to its channels.

If joint is end_site, its offset is the end postion. 

Each Channel stores its type (of the 6DOFs), an Index. 

##### Channel Per Frame Data : 

But then each channel has frame dependent data, I'm not sure if it would make sense to store this per frame, oppose to concatenating together. If the latter we know the offset per frame is just the number of channels. I think it might just be easier to store channels as single vector and then yeah, define the offset based on the current set frame. 

But we could store per channel, an array of per frame channel values directly. 

I think the sample code, uses index offsets to get the current channel data per frame

The example code maps the joint pointers to joint names etc, i'm gonna skip this for now and just id joints by indices along the tree. 

We can index channels as we encounter them in the tree, this should lead to their linear indices within the motion data per frame. Thus if we store channels in a per frame array, we should be able to look up each frame (outer array), each channel (by index), inner array.

##### Using Sample Code for Parsing

I'm thinking about using the sample code for parsing due to time limitations, but it would be better to write my own and possibly adapt it to make more sense for me. 

___

#### Skeleton Tree Data Structure

If we use the nodes of a tree to represent joints as that would seem logical in terms of how joints are the nodes of bones in a skeleton, it would mean that we each bone thus tree segment / link could only be connected by two joints at most. The textbook recommends you flip this logic and use nodes to represent bones (linkages) and the links to represent joints (explicit link objects), however for BVH I will use the former logic as I know each bone won't have more than two connections. 

___

OpenGL Renderer : 

Using Modern OpenGL with GLFW and GLEW, each class eg bone.h has its own draw/render code, global render context is defined within application code. 