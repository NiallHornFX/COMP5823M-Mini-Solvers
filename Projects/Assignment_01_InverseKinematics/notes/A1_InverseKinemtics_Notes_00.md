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

Rotations need to be done locally (at origin, so subtract origin, do rotation then add back).

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

##### Animation State

Because the application is ticking constantly, I don't want to couple the ticking of the viewer to the anim frame of the BVH. So I will have a "global" animation frame set, that either loops continually or is set by user input so a single frame. This will be continually incremented per tick to keep looping or stay static. 

I'm still deciding if the actual Anim Loading + IK calc will be called from within Viewer or do I do this is some other more abstract application_class. Ideally i'd decouple the viewer from this stuff, have the application do the tick itself (which calls render of viewer) with updated skeleton data based on current ticks modified (or not) BVH data, from IK solve. OR I just integrate all this into viewer class as a single application class that handles both the anim state and the viewer related tasks. 

Could use a separate class that's a member within Viewer (or nested class approach) to keep the Anim logic / state still within the viewer app, but separated, without separating the viewer class into separate app + renderer. This could be how isolate the state of the solver for each project (using the same core viewer app) could define a base class like `Solver_Base` which is then derived into `Animation_Solver` for this project where the BVH + IK Solver is contained within the viewer app. 

For now I won't use polymorphism for the Solver class, just will have separate classes.

___

#### Viewer / Application Class

##### OpenGL Renderer : 

Using Modern OpenGL with GLFW and GLEW, each class eg bone.h has its own draw/render code, global render context is defined within application code. We then need to integrate DearImgui into this to it can pass the GUI data/buffers for rendering. Also need to make sure GLFW input polling is forwarded to DearImgui. 

This is based within the core application call `viewer.h` i'm still deciding how modular I want the code to be, ideally its as modular as possible so re-use on later projects is easier, however because of the limited time, modularity is not a priority. Can also refactor and breakup into more modular classes later (eg separate MVC classes, Abstracted OpenGL objects/constructs etc). Ie I could separate viewer into the application side, control side, renderer side classes, but for now will keep as one big file for time sake. The more modular, the harder it is to check the state is been updated and needing to pass state between classes etc, so need to find a balance. 

Ideally i'd have a core viewer app I could then link to, but I don't, so will just have to modify on a per project basis, right now I don't care about this code been reused for other projects, just need to get this assignment done and worry about defining a common viewer app base later on. 

Rendering won't be done within BVH_Data as I want to separate it out, Bone class will render single bone, based on data, (either as line or geo), skeleton class will whole skeleton (of bones). These render methods will then be called within OpenGL Context. This also makes sense as channel data may be modified from Inverse Kinematics of bones. 

___

##### OpenGL Issues /  Debugging

I had some crazy OpenGL issues, I think its because of lifetime / scope / context issues because of  the separation (object wrapping) of OpenGL constructs, more so that any previous program i've written. 

Keep running into Shader Not bound issues (Renderdoc), not sure if its due to some sort of issue with the context. 

Seems the fragment shader is not running thus the output colour of the geo is coming from the vertex shader? It could be Undefined Behaviour, there's no build / link errors.

Removed Shader loading in shader ctor. Use separate load() member function to load post construction. RAII could be cause of issues. Could also try using glDetachShader() after linking to decouple its lifetime. 

Render doc seems to sometimes report No Resource when using OOP approach, despite the fact there seems to be no issues. 

Could of been the primitive lifetime, and lack of proper copy ctor, from local var to member within viewer class. Ok it doesn't seem to be this.  But it does make sense if were not initializing key objects on construction is better just to store ptrs we can allocate later without needing to do default initialization of member and then copy / move (replacement) later on when real allocation is done via separate member functions which can create issues if the default initialization and destruction of the original initialized member data is not handled correctly.  If Primitive or mesh member is stack based and is not set (and thus default initialization is used (which is not initialized)) the errors occur. So it does make sense to go back to storing them as ptrs, so they can be allocated dynamically when needed as oppose to doing default initialization on construction, trying to handle this and then the then copy / move with assignment in separate call to replace the default initialized members which causes issues (not the mention the fact the default initialization of gl related objects causes invalid calls if missed). 

Don't terminate on invalid uniform, as may of just been optimized out, alright for debugging though, as this is what led me to realize the shader was not been ran (along with seeing a fixed colour on the triangle when no colour was set, very weird, didn't know OpenGL would still execute without shader guess it reverts to legacy).

So I think removing the shading loading + building from the Ctor, and doing it as a separate call (still called from Primitive class) fixed the issue. The issue been that the shader program was not executing at all, hence all the warnings from RenderDoc about the vertex shader, but no shader stage was running (modifying gl_Position did nothing).

If `glDeleteProgram(ID);` is called within Shaders Dtor I get an OpenGL 1281 error which is invalid value, but the shader execution still seems to work correctly, I'm not sure where this is occurring because the shader should have the same lifetime as the viewer (primitives currently created in viewer for debugging), wonder if its when `Primitive::set_shader()` replaces the default initialized shader with the user defined shader, Implicit copy assignment is done so maybe this causes the shader dealloc before the copy, but still seems to work. For now i've commented out the `glDeleteProgram(ID);`  call within the shader Dtor. 

I Think this may be what causes the error, the default initialized shader then copied via the implicit operator= keeps the same ID, but the shader state is invalid ? So I will do the same for the Texture object and all other classes calling GL related functions within the context, i.e. separate their operations (creating and destruction of GL resources) as separate member functions to be invoked by user, as oppose to happening within the Ctor on construction. 

Another issue, if you try and call any OpenGL code for testing, eg mesh loading, outside of the GLFW context (eg just creating a mesh class instances in main()) you'll get issues because the OpenGL context does not exist within this scope.  So can only test OpenGL related code in viewer class (because GLFW Context is created here).

So issues are a combo of  :

* GL Operations occurring in Ctor of classes (whom may not be in valid context OR not initialized to valid resources ). Specifically because the shader class is called within viewer::render() via its Primitive::render() member function, existing within another classes scope seems to cause issues. 

* Classes doing GL operations / allocations occurring on default initialization of stack based members (and later copy operations onto these members result in invalid state of GL objects/resources and calls). 

Things like this weren't an issue in SFGL for example, because the render context (GLFW Context) was created in main (via render context instance creation) and window ptr passed to the render object / solver. Thus its always within scope. I could create a similar context class, and init this in main, and pass to GLFW window to viewer app ? 

Texture bug - "Frame not in module", don't reinterpret_cast<void*> the texture_data as it changes the address (in this case) unlike static_cast which guarantees to preserve it. Still seem to be getting this issue now and again even though this initially fixed it. Seems to happen if UVs are out of bounds. Just turn off debugging, its not a bug, just a warning on the texture usage (via sampler in GLSL), signals invalid usage. Seems to be temperamentally occurring on and off which is annoying as their may be some underlying bug / mistake I cant find. 

Custom per primitive GL State set within passed lambda eg custom state set callback (to customize per primitive render state calls) ? Eg 32 MSAA Multisampling for ground / grid draw call, then revert to 2-4 MSAA via calls to the GLFW Framebuffer.

Weird issue where if Uniform is set within render loop of primitive/mesh it breaks, but if set from the scope of the viewer it works fine.  (Uniforms need to be reset when changed if uniform was set before operation). It causes the shader to become unbound (same issue that I seemed to solve last night, need to ensure shader is valid, not sure why uniform modification would cause this issue). Ok yeah its because I thought it was smart to add glUseProgram(0) to the end of each uniform set function to clear the bound shader, but when this is called after the shader is bound and then renders, the resulting shader is no disabled. 

Could just wrap uniform access in a getter and call from viewer render loop to ensure its called from within the gl context class (of the viewer class). 

Hmm, but it seems to work if its called, before the shader is active. 

##### GLFW State Callbacks

To be able to use Scroll and Mouse input along with Window resizing, we need to use GLFW callback free functions, however these have fixed parameters (eg width height, x offset y offset) and as they are defined as free functions they cannot modify member state of the Viewer Class (as I'm not using Viewer as a singleton, with static members to update, (which would solve it also)), so I write the callback data (changed GLFW State) from the callback into a struct defined (only) within viewer.cpp called GLFWState, the Viewer::UpdateCamera() member function which is called per tick, then reads from this struct (in global scope). Not ideal with the GLFWState struct been global but I need it as an in-between write from the call-back's (free functions in global namespace) to then read from within the viewer class implementation. 

Setting the GLFW callback function pointers to member functions (eg defined within Viewer per instance  (or even static)) doesn't work afaik hence the need for global namespace defined free functions. That we then need to pass data into the viewer class to update the internal width,height (for camera aspect ratio) and mouse offset (for camera yaw and pitch). Note Key presses are done internally of the class within the Camera::Update_Camera() member function via Polling the keys for key press state, but for mouse and window resizing polling does not make sense / not possible hence use of callbacks to write updated state. 

Because Mouse offset is then passed to Viewer::Update_Camera() which runs per tick, it will keep the previous state (if mouse is now static) from when the callback was last called and values set in the struct, so to avoid repetitive addition we need to check for per tick delta of the offset positions before adding to the camera pitch and yaw. 

##### Shader Class

Will have a separate shader class that defines both a vertex and fragment shader for each object. This will handle shader loading, compiling and uniform setting (along with texture samplers).

##### Primitive Class 

Will define a render object base class `primitive.h`, to define OpenGL calls eg Setup, Pass Texture/State, shaders, Render self. 

Because Primitive renders the object, we need to pass the camera transforms from viewer app to the primitive to set the uniforms within its shader. 

Primitive Class Base Members/Functions :

Member functions like render(), createbuffers() will be virtual to allow for overriding eg for mesh class.  

Oppose to having a big monolithic constructor, where all data is passed, this class will rely on basic initialization and then separate calls to pass mesh data, texture data, shader data etc via setter like member functions. 

In terms of rendering I'm not going to be using indexed drawing for now, will just render triangle soup via Draw Arrays, can switch to element buffers / indexed drawing later if need be. Could have separate primitive bases for indexed vs non indexed drawing but that's over abstraction / modularisation I don't have time for now. 

Primitive Derived Classes

* `Mesh` derives from `Primitive` and adds obj loading to get the mesh data to set and textures.

Primitive Based Classes : (Things needed in app to draw)

* Bone : Draws bone / linkage as either line or bone mesh - Derived from `Mesh`
* Ground : Draws ground plane with tiled grid texture - Derived from `Mesh`
* Gnomon : Draws world and local coord frame of joints axis gnomons - Derived from `Primitive` (Data passed directly).
* Sphere : End Effector Sphere - Derived from `Mesh` 

Need a better way to allocate texture units per mesh, maybe some sort of mesh manager class to check which texture units are available.  For shaders just rendering a single mesh with single texture, don't need texture units. 

##### Mesh vs Primitive Line Rendering

Note Primitive Draws line as direct lines : 

```C++
case (RENDER_LINES) :
{
	glDrawArrays(GL_LINES, 0, vert_count);
	break;
}
```

While Mesh Draws Lines as triangles (lines, thus wireframe) :

```C++
case (RENDER_LINES):
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawArrays(GL_TRIANGLES, 0, vert_count);
	break;
}
```

##### Skeleton Class

Skeleton Class is not inherited from Primitive, but instead contains the array of all bone mesh primitives, defining the skeleton (fetched from BVH_Data), Skeleton will also apply modifications to joints based on the IK Solve.  Its render call, calls render for each bone applying transformations based on joints + channels. 

It will exist within the animation solver class, (which houses the BVH_Loader instance, and the FK,IK operations, that are then applied to the skeletons bones), Bone class will also have an overriden render method to render as lines, where lines are not wireframe of bones, but infact just piecewise lines for each bone (joint to joint). 

Issue is, with having classes of primitive derived instances, is we have to pass the Camera matrices (which then pass to the internal primitive functions), so there is some forwarding needed for this. Eg Skeleton Render takes in cam matrices, which passes them to bone::set_cameraTransform() which itself then calls mesh and primitive cameraTransform() (on the bones mesh and primitive instances, depending which is rendered (lines or mesh)).

The Skeleton will contain the root bone transform (which has 6DOFs, the other bones should only have 3 DOFs ie Joint angles, with their start defined by offset vector rel to parent).

The Bones Transformation is then passed to its model matrix for resulting line / mesh rendering. One thing thats key is the length of the segments by the start/end offset of the current and previous joint (or current and next depending on how bones are created along tree).

___

##### Animation State Class

This is the class that contains the BVH Loading, the intermediate operations, FK and IK calculation and the resulting Skeleton/Bones to render, this class is responsible for maintain the animation state. 

Its embedded within the Viewer Class, and updates when needed (not always per tick). (Eg in Cloth Simulation project this class will be replaced with an equivalent "Cloth_State" class embedded within viewer app class) this then handles render state to call from viewer as well as forwarding inputs and GUI Controls --> Anim state actions (eg arrow keys to move anim state), meaning the Viewer class can still handle the GUI and Control side, and just forward to anim_state class as needed. 

IK types will be defined as Solvers, we need conversion to and from glm representations of transformations along with the BVH motion data.

___

#### Forward Kinematics (of BVH Data)

##### BVH Data --> Skeleton

There's two parts to this stage, the first is creating a bone for each joint pair in the BVH data, the second is updating the transforms per frame (successively through the tree) from the motion data (essentially Forward Kinematics)

Need to get each joint and the next joint to get the current and next offset to define the start and end position (offset) for each bone. Along with concating the rotation from the previous joints concat matrix * the current. 

We need correct traversal from the BVH Root joint to each end site regardless, this is done via depth first traversal (so all parent nodes along each branch can concatenate to the final transformation applied to each rendered bone of both offset and rotation (defined by BVH Motion/Channels data)).

Bone end should be the current joint offset, bone start should be the parent joint offset. So End would be current joint offset + parent offset, parent offset is the total accumulated offsets since root. 



...

Recursion approach each recursive call gets copy of previous (parent call) offset translation and rotation matrix which it then accumulates itself to before creating a bone. Replicates OpenGL Matrix Stack state they are using

Do start end calc inline, create bone from start end only (so dont need to do rotation internally of bone class?)

old approach of start + end pos defined by offsets, and then passing rot matrix to be applied atop as model matrix was not a good idea. precompute the final bone postion with both offset and rotation applied based on current state of transforms. 

If inversed for rotation needs to be rel to parent offset

inverse rel to parent rotation not just trans (offset...)

need to translate root joint  segment using its channel (6DOF, using its translation components as it has 0 offset)

glm axis angle rot creates scaling be from accumulation need to fix this. 

Just use 0 start, offset end for line start end, then put concat offset in matrix to translate rel to?

So when transform is applied we have parent offset + 0 for the start pos , and parent offset + child offset for end (with rotations included).

Inverts trans + rot of parent, apply my joint transform, then un inverse

think my issues might be due to root transform... Or something to do with branching children as single bone strip seems to work, is my recurrsive approach not correct for child rotations ? 



Need to WUP recurrsive approach, ie each call gets copy of parent transform (doesnt require stack approach or reverse traversal back to root). Also rather than creating bone at resulting offset start and end, their created in realtive local space, and then transformed. (Model matrix is then just used for post transformation scaling atop of this)



Do we contiune with approach of prebuilding rest pose with offsets then per frame / tick, correctly inverse do the local rotation rel to parent (to move bones via joints), or do we just reconstruct the bones per tick, with the transforms precomputed and passed directly as resulting start and end postions of the bone lines.The bones would be inverted to their parent joints (first point) transform to then apply their transform.



In the sample code they define a bone as a single mesh object (which my bone class can render as also) so they concat the translation (offset) to define the centre between two joints along with the rotation to define the bone transform. For my sake for now rendering as lines, the offsets define the line segments start and end and the rotation is applied atop of this via the transform matrix (which becomes the primitives model matrix), I'm thinking its correct that eg for a line ((0,0,0), (0,1,0)) the rotation origin should be at the centre of origin so maybe before the rotation is applied in the shader we should be inverting line verts to origin...

Or just create line procedurally based on distance of offsets, put offsets into matrix form internally with rotation matrix passed ? The rot transformation should be computed on the CPU side really, because otherwise the model matrix is applying just the rotation component, to the off-setted lines in the shader. This is why offset alone works fine (as it defines the line positions in WS relative to parent offsets) but not the rotations... Can still use model matrix to do the post transform scaling and translation (to scale resulting skeleton)

And thus pre computing it, we have access to the offset (translation data for line verts) we need to invert to LS for rotation and then uninvert. 

Or we could pack centre into matrix and do this on gpu side, but then we wouldnt be able to do post transform operations on the skel. 

But its not local space to bone its local space to rel parent, so make sure offset is used for ....

Sample code operates on drawing (from current joint (parent to child), my code gets current joint (from parent) so while there end segment is the child joint, in my loop, the end pos is the current joint offset + parent offset and start pos is the parent offset. 

They define offset and transform together so there origin vertex per bone is always (0,0,0)

##### Building Skeleton of bones from BVH Data 

(Starting with just rest pose with offsets, no joint transforms/channels) 

I found this hacky-ish way, where oppose to traversing from root, we get each joint, then get its parent and traverse back to root (going back to root is easier) accumulating the total offsets along the way, this then defines the start position of the bone segment (which is the current joints, parent offset (we accumulated)), the end position is then the current joint's offset + the parents offset : 

```C++
// anim_state::build_bvhSkeleton

// Hacky Way to get inital pose 
for (Joint *cur : bvh->joints)
{
	glm::vec3 par_offs(0.f);
	Joint *p = cur;
	while (p->parent) // Traverse back to root to get total parent offset
	{
		par_offs += p->parent->offset;
		p = p->parent;
	}
	// Start is then parent offset, end is the current joint offset +
	// parent offset (total parent offset along tree).
	// Append bone to Skeleton
	skel.add_bone(par_offs, (cur->offset + par_offs), glm::mat4(1));
}
```

This is not very efficient as it means doing backwards traversal multiple times over the same branches. But its conceptually quite simple and avoids recursion or lots of for loops. 

Similar approach could be used to apply transformations to each bone containing all previous parent transforms + current. However it would mean rotation matrix is concatenated in reverse which is not correct as matrix multipcation is not commutative unlike translation (for offsets) which works fine as addition is of course commutative. 

For reference approach could looks like this (it doesn't work though as per above) :

```c++
// Is it easier to rebuild the Skeleton per tick ? (with the current set anim 
// frame motion/channel angles retrived).

// Oppose to building skeleton and updating bone transform joints later 
// in seperate per tick step.
void Anim_State::build_per_tick()
{
	skel.reset(); // Testing calling this per tick, so reset...

	for (Joint *cur : bvh->joints)
	{
		// Get Parent Offset
		glm::vec3 par_offs(0.f);
		// Get Parent Transform
		glm::mat4 rot(1.f);

		Joint *p = cur;
		while (p->parent)
		{
				// Accumulate Channel Transform
			// Non root joints (3DOF, joint angles only).
			if (!p->parent->is_root)
            // Else just dont accumulate rotation for parent = root 
			{
			// Get Angles from motion data of current frame
			DOF3 angs = bvh->get_joint_DOF3(p->parent->idx, anim_frame);
			// Build Local Matrix to multiply accumlated with 
			glm::mat4 tmp(1.f);
			// Z Rotation 
			tmp = glm::rotate(tmp, float(std::get<0>(angs)), glm::vec3(0.f, 0.f, 1.f));
			// Y Rotation 
			tmp = glm::rotate(tmp, float(std::get<1>(angs)), glm::vec3(0.f, 1.f, 0.f));
			// X Rotation 
			tmp = glm::rotate(tmp, float(std::get<2>(angs)), glm::vec3(1.f, 0.f, 0.f));
			// Accumlate Rotation 
			rot *= tmp;
			}
            
			// Traverse up to parent 
			p = p->parent;
		}
// Start is then parent offset, end is the current joint offset + parent offset 
// (total parent offset along tree).
		skel.add_bone(par_offs, (cur->offset + par_offs), rot);
	}
}
```

##### Updating transforms

Bones are not mapped to joints, so updating transforms from joints to resulting bones is not ideal, we could rebuild the tree with offsets and transforms per tick (so joint transform is passed directly on bone construction, no need to then map joint transforms to bones per anim frame), but this is very inefficient, we should only build skeleton once and then update bone transforms per tick needing to define a mapping from joints to bones, to then update their transforms. 

The sample code kinda does this (it doesnt maintain a skeleton state, it re calcs per call to render directly using legacy GL.  It uses recursion approach for this starting from root). I could do a similar approach, where skel is rebuilt per call so transforms can be applied directly, but as above its not very effcient because reconstructing the same skeleton tree with changed rotations is a waste of time if we can find a way just to update joint rotations on pre-built skeleton. 

So need a robust way to handle updating bone transforms from BVH Joints, the joint is what defines the start of the bone ie in the above example, the parent of the current iterated joint. 

The Scaling + Translation only happens in the Bone Rendering calls (ie to Primitive Model matrix) so this should not affect the transforms of the bone itself, as they are applied after the bone transform matrix is passed to the primitive as its model matrix. 



____

*Above notes are before I implemented this approach (and then went back to trying to inverse and do per tick transform updates only). This approach is better for just drawing the bones as is from the BVH data, as oppose to trying to modify them also (using IK).*

##### Alternate Approach 

In hindsight I should of implemented this approach first, as it what all the BVH Loaders / Projects i've seen do to render the joints as bones, typically using Immediate Mode (Legacy) OpenGL. The sample code also follows this approach as follows : 

Starting from the root joint and an identity matrix we get the joints translation and apply it to the matrix, we get the joints rotation and apply it to the matrix, then for each of the joints children, we do a recursive call to this function passing the current matrix (cumulative transforms). We need to check if we have the root joint passed if so the translation comes from the channel / motion data, else the translation comes from the joint offset. The rotation data comes from the joint channel data regardless of course. 

```C++
// Pesudo C++ Code 
void build_from_root()
{
	Joint *root = joints[0];
	build(root, glm::mat4(1.f))
}

// Function to accumulate transform from root
void build(Joint *joint, glm::mat4 trs)
{
	if joint == root // trans from channel data
	   for (c in joint->channels) ... get transformation channel data -> glm::vec4(trans)
	      trs = glm::translate(trs, trans);
	
	else // (non root) trans from offset
	   trs = glm::translate(trs, joint->offset);
	   
	 for (c in joint->channels) ... get rotation channel data
	    trs = glm::rotate(trs, glm::radians(x), glm::vec3(1, 0, 0))
	    trs = glm::rotate(trs, glm::radians(y), glm::vec3(0, 1, 0))
	    trs = glm::rotate(trs, glm::radians(z), glm::vec3(0, 0, 1))
	
	for child in joints->children // Recurse
	   build(joint->child, trs)
}
```

Then if we want to draw a line for each joint, within this function we create two points one is at `(0, 0, 0)` which is transformed using the current accumulated transformation matrix to place it to the joint start position in WS relative to parent. The second point is from each of the current joints, children offset positions, these are then also transformed by the matrix, fed to some line primitive to render the bone.

```C++
// Pesudo C++ Code 
void build_from_root()
{
	Joint *root = joints[0];
	build(root, glm::mat4(1.f))
}

// Function to accumulate transform from root
void build(Joint *joint, glm::mat4 trs)
{
	if joint == root // trans from channel data
	   for (c in joint->channels) ... get transformation channel data -> glm::vec4(trans)
	      trs = glm::translate(trs, trans);
	
	else // (non root) trans from offset
	   trs = glm::translate(trs, joint->offset);
	   
     // Rotation Data for all joints comes from channels 
	 for (c in joint->channels) ... get rotation channel data
	    trs = glm::rotate(trs, glm::radians(x), glm::vec3(1, 0, 0))
	    trs = glm::rotate(trs, glm::radians(y), glm::vec3(0, 1, 0))
	    trs = glm::rotate(trs, glm::radians(z), glm::vec3(0, 0, 1))
         
   // Start vert of bone. Transformed via joint matrix. 
   glm::vec4 vert_0 = trs * (0,0,0,1); 
    
    // Check is joint end
    if joint == end
        glm::vec4 vert_end = trs * glm::vec4(joint->end_site, 1.0);
        draw_bone_line(vert_0, vert_end) // Draw as line
    
	for child in joints->children // Recurse
       // Create vertex at each child offset from parent, transformed. 
       glm::vec4 vert_child = trs * glm::vec4(child->offset, 1.0);
       draw_bone_line(vert_0, vert_child); // Draw as Line. 
	   build(joint->child, trs)
}
```

So this approach works and is a good way to directly draw the bones as lines one by one, however it has to be done per frame, as the transform data of course changes.  We see that the starting vertex is at world origin and then is transformed by the current matrix, the second vertex is then the child offset transformed by the current matrix. 

The recursive call for each child joint, then takes the current matrix as a copy (this is vital, do not reference it) so it can add its transformation for its children to it, this enables us to accumulate the joint transformation without using a stack (as we don't need to pop back to the parent matrix after each branch, we just end the recursion), each child call carries forward only its parent transforms, if it itself has no more children, the recursion ends and we don't need to recover the parent matrix to then traverse the other children branches, as they have already been evaluated in their own recursive calls (without recursion would need a stack approach, to recover parent transform before traversing next child branch (for joints with multiple children)). 

However Ideally I would like to go back to my initial approach of pre-building the skeleton tree using the offsets in the rest pose, and then per tick only update the transforms. Because otherwise its going to be difficult just using this approach where transforms are been pulled directly from the BVH Data each frame, to add in the  IK Functionality. 

However I will branch this code off, remove some of the skeleton code (as its not needed just to render FK might as well make it faster by removing this, and just directly render line primitives without using the skeleton concept, as their is no bone transforms passed (as transforms are set directly to vertices to define the bone line start/end  verts), then I can add a GUI to this code, and use it as backup FK / BVH Only code. In case IK is not done in time. I will branch this as a separate project stored locally for now. 









___

#### Inverse Kinematics

Eigen time

Pseudo inverse ... 

User interaction to move joints ...

Getting this working with the BVH Motion is going to be challenging





##### User Interaction with Bones/Joints

Get inputs from viewer class (Could just pass window to anim state class directly to query)

Ray casting from mouse ? Selection from list of joints via GUI ? 

Input like frame stepping will be done within viewer, (inc/decrementing anim frame eg.).