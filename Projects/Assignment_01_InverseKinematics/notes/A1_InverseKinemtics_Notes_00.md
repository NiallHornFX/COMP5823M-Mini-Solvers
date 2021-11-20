#### COMP5823M - Assignment 1 - Inverse Kinematics - Notes

##### Niall Horn - University of Leeds - 2021

___

#### Initial Notes

Aim for project : A viewer application with a viewport, DearImGui based IM Mode GUI for control which I may implement via a controller class, with mouse pick-point to define end effector positions / position constraints, load BVH Files, some sort of timeline to scrub the animation. Control for which IK method is used, etc. 

Then for visualization of the skeleton bones as lines, but ideally as bone geo as well, along with spheres for joints and target positions/end effector. Have a ground plane / grid, and some sort of viewport fog/horizon if there's times will implement shadows from a headlight source light, basically its gonna be a tiny anim app (but you cannot set/store keyframes etc) its mainly for loading BVH Files and writing them back out based on user modified end effectors and resulting joints configuration. 

Might be a good idea to follow Model,View,Controller paradigm for this app. Eigen will be used for Linear Algebra, Modern OpenGL for rendering. 

I would of used Qt for the application, GUI, OpenGL etc. But for this project i'm gonna stick with Imgui. 

While IK is used for solving the joint angles from the BVH joint configuration, FK is used to then define the transforms for rendering / viz. 

While I'd really love to write detail notes again this project is due in 2 weeks and I have so much to do. So rough notes and maybe write up later. 

##### External Libraries : 

* OpenGL (w/ GLFW + GLEW) : Rendering, Dynamically linked (with static symbol libraries)
* GLM : OpenGL Related math
* DearImGui : UI
* Eigen IK Linear Algebra (Will convert to/from GLM where needed)

##### Main Components to do : 

* BVH File Parser, Loader, Writer.  
* Viewer Application, OpenGL State Setup
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

##### Rendering BVH Data

To Render the resulting BVH Data, we just use FK, ie we are apply successive rotations along each joint hierarchy within the space of the (relative to) parent joint, for pure BVH this is easy as we have the offset that defines the start of the joint (and thus bone) we know the next joint offset is the end of the bone (linkage) and we can then just look up the channel data for the current frame.

Starting from the root (which should be the first element of the joint array) do stack based depth traversal to render each bone, by applying offset and rotation relative to parents transform matrix (concatenation of all previous joint transforms).

___

#### BVH_Data Class :

`bvhdata` is where the `BVH_Data` class is declared, This file also is where joint and channel are declared. 

Plan to have both the Loading,Parsing and Writing within the same class, this class will also store the parsed state of the Skeleton / Tree. 

This will contain the time based state of the skeleton tree, in the form of per channel frame data loaded from the original BVH file.  This will later be modified from the IK Solve. 

Maybe I can serialize and extract this out and store in some separate skeleton class later. 

First part of the parsing is checking which part of the BVH File we are in, secondly we check if we are in a ROOT or JOINT like, if so we create a new joint, and then for each successive line we update its data, we need to create a local stack to push each new joints data into (because of the recursive nature of the hierarchy).

So because parsing is line by line, each new loop iteration has no knowledge of previous hence the need for a stack of joints (using a vector, so not really a stack). A new joint is started after ethier `ROOT` or `JOINT` keyword, and then For each subsequent line, if the starting char is `{` the current new joint is referenced, else if the line starting char is `}` the back vector joint is then extracted (ie top of the stack) and referenced untill the final enclosing `}` is reached which is the first joint added. Essentially each  `{` defines the start of the scope of the current joint, when the next `JOINT` is encountered a new joint is created and its parent is the previously created joint. 

We can see part of the parsing code here to explain this better, note joint_stack is just a std::vector<joint*> but it would make more sense to use a std stack container : 

```c++
// BVH_Data::Load() 
// [..]
// Start of Joint Block
if (strcmp(token, "{") == 0)
{

joint_stack.push_back(joint);
joint = new_joint;
continue;
}

// End of Joint Block
if (strcmp(token, "}") == 0)
{

joint = joint_stack.back();
joint_stack.pop_back();
is_site = false;
continue;
}

// Start of Joint Information
if ((strcmp(token, "ROOT") == 0) ||
(strcmp(token, "JOINT") == 0))
{

new_joint = new Joint();
new_joint->idx = joints.size();
new_joint->parent = joint;
// Set if end (end site)
new_joint->is_end = false;
// Set if root
if (strcmp(token, "ROOT") == 0) new_joint->is_root = true; else new_joint->is_root = false; 
new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
//new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;

// Append to joint array
joints.push_back(new_joint);

// If valid parent, add self as child joint. 
if (joint) joint->children.push_back(new_joint);


token = strtok(NULL, "");
while (*token == ' ')  token++;
new_joint->name = token;


//joint_index[new_joint->name] = new_joint;
continue;
}

// Start of end information
if ((strcmp(token, "End") == 0))
{
new_joint = joint;
is_site = true;
continue;
}
// [..]
```

Each joint has array of pointer to its channels. If joint is end_site, its offset is the end postion. 

Each Channel stores its type (of the 6DOFs), an Index. 

##### Channel Per Frame Data : 

But then each channel has frame dependent data, I'm not sure if it would make sense to store this per frame, oppose to concatenating together into single array of all channels (for all frames). If the latter we know the offset per frame is just the number of channels. I think it might just be easier to store channels as single vector and then define the offset based on the current set frame. 

But we could store per channel, an array of per frame channel values directly. 

The sample code, uses index offsets to get the current channel data per frame from the "motion" array, which is the per frame channel data. 

The example code maps the joint pointers to joint names etc, i'm gonna skip this for now and just id joints by indices along the tree. 

We can index channels as we encounter them in the tree, this should lead to their linear indices within the motion data per frame. Thus if we store channels in a per frame array, we should be able to look up each frame (outer array), each channel (by index), inner array.

The sample code decouples the channels themselves (per joint) from the per frame motion data, which is just stored as an array of doubles. The size of this is the number of frames  * number of channels (of all joints). Thus using the channel index we can dereference each channels data per frame via :

```c++
motion[frame * num_channel + channel_idx]
```

Because we know only the root has translation and we know the rotation channel order is (z, y, x) we don't need to worry about checking for each channel type when rendering, we can just follow this order to get each rotation component per joint (and thus bone segment).

##### Using Sample Code for Parsing

I'm thinking about using the sample code for parsing due to time limitations, but it would be better to write my own and possibly adapt it to make more sense for me. 

Ok yeah for now I am going to try and integrate this code, which was not even written by the University it seems to be from some Japanese due about 20 years ago. It has goto statements and some ugly syntax choices, but i'm so short on time I don't care too much at the moment. I'm not gonna feel bad as I think most other people have used this code to some extent, even projects on github using BVH files use this code. Would of been nice to write my own parser based on this, but due to time its ok. 

I don't have much (any) experience of parsing hierarchal files, its something I need to work on. 

Remember with Eigen to use [0],[1],[2] for 3D vector indices, as they are just typedefs' matrices there is no x,y,z components. 

##### File Input

Initially file will be passed via args, but would be cool to have string field in the GUI to quickly switch between BVH files. 

___

#### Skeleton Tree Data Structure

If we use the nodes of a tree to represent joints as that would seem logical in terms of how joints are the nodes of bones in a skeleton, it would mean that we each bone thus tree segment / link could only be connected by two joints at most. The textbook recommends you flip this logic and use nodes to represent bones (linkages) and the links to represent joints (explicit link objects), however for BVH I will use the former logic as I know each bone won't have more than two connections. 

For the BVH Loading, there is an array of joints, whose first joint should be root, starting from here we can loop down to the child joint, each channel per joint represents its DOFs (3 per joint (rotation) apart from root which also has translation / position DOFs).



Ideally I'd take the resulting BVH Joints + Channels and put it into a nicer tree structure that makes it easier to apply per joint transforms (rotations) atop of the existing BVH anim data per frame to add the modifications from the IK solve, however for now this might just be done, before drawing, based on a separate array of stored IK offsets for each joint. 

___

##### OpenGL Renderer : 

Using Modern OpenGL with GLFW and GLEW, each class eg bone.h has its own draw/render code, global render context is defined within application code. We then need to integrate DearImgui into this to it can pass the GUI data/buffers for rendering. Also need to make sure GLFW input polling is forwarded to DearImgui. 

This is based within the core application call `viewer.h` i'm still deciding how modular I want the code to be, ideally its as modular as possible so re-use on later projects is easier, however because of the limited time, modularity is not a priority. Can also refactor and breakup into more modular classes later (eg separate MVC classes, Abstracted OpenGL objects/constructs etc). Ie I could separate viewer into the application side, control side, renderer side classes, but for now will keep as one big file for time sake. The more modular, the harder it is to check the state is been updated and needing to pass state between classes etc, so need to find a balance. 

Ideally i'd have a core viewer app I could then link to, but I don't, so will just have to modify on a per project basis, right now I don't care about this code been reused for other projects, just need to get this assignment done and worry about defining a common viewer app base later on. 

Rendering won't be done within BVH_Data as I want to separate it out, Bone class will render single bone, based on data, (either as line or geo), skeleton class will whole skeleton (of bones). These render methods will then be called within OpenGL Context. This also makes sense as channel data may be modified from Inverse Kinematics of bones. 

##### Animation State

Because the application is ticking constantly, I don't want to couple the ticking of the viewer to the anim frame of the BVH. So I will have a "global" animation frame set, that either loops continually or is set by user input so a single frame. This will be continually incremented per tick to keep looping or stay static. 

I'm still deciding if the actual Anim Loading + IK calc will be called from within Viewer or do I do this is some other more abstract application_class. Ideally i'd decouple the viewer from this stuff, have the application do the tick itself (which calls render of viewer) with updated skeleton data based on current ticks modified (or not) BVH data, from IK solve. OR I just integrate all this into viewer class as a single application class that handles both the anim state and the viewer related tasks. 

Could use a separate class that's a member within Viewer (or nested class approach) to keep the Anim logic / state still within the viewer app, but separated, without separating the viewer class into separate app + renderer. 

##### Shader Class

Will have a separate shader class that defines both a vertex and fragment shader for each object. This will handle shader loading, compiling and uniform setting (along with texture samplers).

##### Primitive Class 

Will define a render object base class `primitive.h`, to define OpenGL calls eg Setup, Pass Texture/State, shaders, Render self. 

Because Primitive renders the object, we need to pass the camera transforms from viewer app to the primitive to set the uniforms within its shader. 

Primitive Class Base Members/Functions

Primitive Classes (Things needed in app to draw)

* Bone : Draws bone / linkage as either line or bone mesh
* Ground : Draws ground plane with tiled grid texture
* Gnomon : Draws world and local coord frame of joints axis gnomons
* Sphere : End Effector Sphere

##### Other Classes Used within Viewer

Skeleton Class is not inherited from Primitive, but instead contains the array of all bone primitives, defining the skeleton (fetched from BVH_Data), Skeleton will also apply medications to joints based on the IK Solve. 

___

User Interaction with Bones/Joints

Ray casting from mouse ? Selection from list of joints via GUI ? 