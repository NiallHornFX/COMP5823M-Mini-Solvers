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

Each joint has array of pointer to its channels. If joint is end_site, it stores both its offset AND the end_site offset position (which is offset from the joints offset, kinda like another embedded joint who starts at the end of the joint containing the end site). 

Each Channel stores its type (of the 6DOFs), an Index. Typically only root will ever have 6 DOF Channels, all other joints in the hierarchy will have 3 DOFs (rotational only). 

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

Strange new bug where some primitive/meshes model matrix diagonal gets zero'd on the GPU, uniform is set correctly on host, matrix is correct on host, but when checked on GPU via renderDoc the diagonal is zero'd and the resulting mesh is scaled to a point. This is so weird as the same primitive and mesh class operations are drawing other meshes fine in the calls before, but then for this mesh this occurs. 				 Ok I think this is my bad, as these mesh has its transform defined by the model matrix (the mesh input is obj file in LS), so I'm scaling the model  matrix per tick, hence its going to zero over time by the time renderdoc captures it, its already at 0. Makes sense, silly mistake on my behalf. This isn't an issue on the other prim/meshes as their model matrices are doing post transform scaling so thats needed per call, to correctly scale down from currently transformed WS position.

This is a good example of using Model matrices to define vertex transformations is not ideal, they should only be used for placing existing transforms onto the GPU. Otherwise if you modify the model transform per tick you get accumulation of transformations. 

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

____

##### ImGUI Setup

Context setup code needs to be called directly after GLFW Setup (within Window_Context() Setup Member function).

Render Loop of GUI needs to be within the OpenGL render loop directly, doesn't work when within VIewer::tick() (despite this been within the GLFW / GL Context and the main application loop, I think its because glClearColour needs to be called first and this is only done within Viewer::Render() which itself is called within Viewer::Tick()).

This can be safely encapsulated within a Member Function, but make sure the correct GL, GLFW calls have been done before its called within the correct scope. 

I had to add the direct path to the imgui headers, because the imgui source files include them (these need to be part of the project, or prebuilt and linked) otherwise i'd have to modify all include calls to find the imgui headers within my projects `../../ext/dearimgui/` header directory, its just easier to pass the path directly as an include search path, so I don't need to change all the includes. 



Make sure GUI Render calls within render loop are the last call, so the GUI Renders ontop, kind of a no brainer, but worth noting. 

GUI Shutdown calls within Viewer::gui_shutodwn() are called after the viewer application loop exits within viewer::exec().

Make char[] buffers for InputText (Input and Ouput Paths) either static local vars within the gui_render() member function or members so the are not reset on each frame/draw of the GUI. 

Could use ImGUI Tree widget (See demo window example) to list the joint/bone hierarchy. 



_____

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

____

***Ugly Notes as I figured out how to accumulate transform and draw bones***

Bone end should be the current joint offset, bone start should be the parent joint offset. So End would be current joint offset + parent offset, parent offset is the total accumulated offsets since root. 

Recursion approach each recursive call gets copy of previous (parent call) offset translation and rotation matrix which it then accumulates itself to before creating a bone. Replicates OpenGL Matrix Stack state they are using

Do start end calc inline, create bone from start end only (so dont need to do rotation internally of bone class?)

old approach of start + end pos defined by offsets, and then passing rot matrix to be applied atop as model matrix was not a good idea. precompute the final bone postion with both offset and rotation applied based on current state of transforms. 

If inversed for rotation needs to be rel to parent offset

inverse rel to parent rotation not just trans (offset...)

need to translate root joint  segment using its channel (6DOF, using its translation components as it has 0 offset)

Just use 0 start, joint offset end bone bone / line (start,end) then put concat offset in matrix to translate rel to? Oppose to doing inverse transform of prebuilt hierarchy ? So when transform is applied we have parent offset + 0 for the start pos , and parent offset + child offset for end (with rotations included).

Or other approach, prebuild bones from joint hierarchy, then per tick Inverts trans + rot of parent, apply my joint transform of current anim channel data, then un inverse back to offset location. 

think my issues might be due to root transform... Or something to do with branching children as single bone strip seems to work, is my recursive approach not correct for child rotations ? (It was because I was using parent as child, so my hierarchy transforms were off by one when rotation was applied, see below)

Do we continue with approach of prebuilding rest pose with offsets then per frame / tick, correctly inverse do the local rotation rel to parent (to move bones via joints motion channel (and eventually IK angles)), or do we just reconstruct the bones per tick, with the transforms precomputed and passed directly as resulting start and end positions of the bone lines. The bones would be inverted to their parent joints (first point) transform to then apply their transform.

Or just create line procedurally based on distance of offsets, put offsets into matrix form internally with rotation matrix passed ? The rot transformation should be computed on the CPU side really, because otherwise the model matrix is applying just the rotation component, to the offsetted lines in the shader. This is why offset alone works fine (as it defines the line positions in WS relative to parent offsets) but not the rotations... Can still use model matrix to do the post transform scaling and translation (to scale resulting skeleton). Yeah this was a terrible approach, why did I think this would work, both the offset (translation) and rotation need to be applied to the bone verts together, why did I think I could apply the rotation later using the model matrix. I'd have to inverse, re-fetch and then re-transform, which is what i'm trying to do but the transformations would be concatenated still unlike this approach where I split them. 

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

The sample code kinda does this (it doesnt maintain a skeleton state, it re calcs per call to render directly using legacy GL.  It uses recursion approach for this starting from root). I could do a similar approach, where skel is rebuilt per call so transforms can be applied directly, but as above its not very effcient because reconstructing the same skeleton tree with changed rotations is a waste of time,  (See Immediate Mode approach below). 

If we can find a way just to update joint rotations on pre-built skeleton. So need a robust way to handle updating bone transforms from BVH Joints, the joint is what defines the start of the bone ie in the above example, the parent of the current iterated joint. 

The Scaling + Translation only happens in the Bone Rendering calls (ie to Primitive Model matrix), these are post transform operations to re-scale the visual output ie bones to be smaller / closer to world origin, so this should not affect the transforms of the bone itself, as they are applied after the bone transform matrix is passed to the primitive as its model matrix. 

____

*Above notes are before I implemented this approach (and then went back to trying to inverse and do per tick transform updates only). This approach is better for just drawing the bones as is from the BVH data, as oppose to trying to modify them also (using IK).*

##### Alternate Approach (Pseudo Immediate Mode approach)

In hindsight I should of implemented this approach first, as it what all the BVH Loaders / Projects i've seen do to render the joints as bones, typically using Immediate Mode (Legacy) OpenGL with the matrix stack (however I do without a stack using recursion). The sample code also follows this approach as follows : 

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

The recursive call for each child joint, then takes the current matrix as a copy (this is vital, do not reference it) so it can add its transformation for its children to it, this enables us to accumulate the joint transformation without using a stack (as we don't need to pop back to the parent matrix after each branch, we just end the recursion), each child call carries forward only its parent transforms, if it itself has no more children, the recursion ends and we don't need to recover the parent matrix to then traverse the other children branches, as they have already been evaluated in their own recursive calls (without recursion would need a stack approach, to recover parent transform before traversing next child branch (for joints with multiple children)). Essentially we use recursion to build the traversal matrix stack for us, instead of creating a matrix stack manually and doing standard non recursive tree traversal. 

Cannot pass matrix to recursive call for children as reference as it means each child call will reference and write to (accumulate onto of) the same parent matrix, meaning each child branch writes to the same matrix, breaking the concept of the recursive stack of matrices along each branch of the joint hierarchy. 

My issues stemmed from the fact I was transforming the vertex positions of the bone lines, from the parent  position not the current joints children, with the starting position been transformed only by the parent matrix. So following the sample code where the joint bone starts from the parent and then ends at the child. So parent vert is local space (0,0,0) transformed to parent position via current matrix and end vert is the child joint offset transformed by the matrix. Originally I was doing the inverse, where I'd treat each joint as the child, and define the bone, from the parent (previous recursive call) to the current joint (as the child) however as I was using the current joint offset as the parent position (bone start position) I always had incorrect offsets and the rotations did not work properly (they were offset by one joint). So it makes more sense to define a bone from the current joint, to its children, oppose to the current joint to its parent. 

 My very first idea I was using start+end verts for the line but then doing the transform of rotation on the GPU via the model matrix, this of course will not work as the start+end verts themselves need to be transformed by this. This was my bad for separating the translation and rotation to separate data to accumulate, makes more sense to accumulate to single matrix and use this to transform the line/bone vertices. 

For rendering eg mesh spheres on joints, we do need the model transform as the input joint pos, we need to use model transform to move the mesh to this pos by setting translation to the joint pos on the model matrix. However the joint pos itself is calculated using the parent transform matrix externally (within anim_state class) this is unrelated but just to clarify, while  meshes need to use model matrix for transformations, for points and lines, they are just used for post scaling the input line/point vert positions. However we can also do post scaling on the mesh on the model matrix, just make sure the transformation is done atop the translation to the joint position of the model matrix.  For rendering joint spheres instancing would make more sense, oppose to individual meshes for each joint, but I don't have time to abstract away joint sphere, just so I can reuse the same mesh data and GPU Resources. For scaling the sphere radius (before model transform) just modify the vert data directly we know the obj file is a unit sphere can just scale the vert positions within here (but either way should be ok). More efficient approach would be use sphere texture quads / sprites. 

However Ideally I would like to go back to my initial approach of pre-building the skeleton tree using the offsets in the rest pose, and then per tick only update the transforms. Because otherwise its going to be difficult just using this approach where transforms are been pulled directly from the BVH Data each frame, to add in the  IK Functionality. Its not very fast re-building the skeleton each time, especially because of the recursion used also the allocation of GPU Resources is un-optimized, there's no-reuse at the moment because that sort of optimization I don't have time for. 

However I will branch this code off, remove the skeleton and bone code (as its not needed just to render FK might as well make it faster by removing this, and just directly render line primitives without using the skeleton concept, as their is no bone transforms passed (as transforms are set directly to vertices to define the bone line start/end  verts), then I can add a GUI to this code, and use it as backup FK / BVH Only code. In case IK is not done in time. I will branch this as a separate project stored locally for now. 

Once bones have been created, we need to update the joints that drive their transform, however as they depend on joints I need to map bones to joints or store joint idx within bones, so when a joint is updated, the joint's bone positions are re-set correctly. If I just update joints and re-build the bone hierarchy it defats the point of trying to separate the bone creation (once only) and bone transform (update only per tick). As noted above, a bone start pos is defined by the joint parent, with the end pos been the joint parent + joint offset transform.

____

##### Prebuild Bones From Joint Hierarchy, Update Bones (via joints) per tick :

I got this approach working after some trial and error, essentially the joint hierarchy is traversed and bones built initially by using the reverse traversal approach to build the skeleton based on the joint offsets, creating a bone for each. 

The update step uses a similar recursive approach to the immediate mode style approach where we accumulate the transform matrix from the root joint to each child joint, we then find the bone that the joint is part of (where the bone has the joint defining its start position because remember we traverse parent->child) we then pass the updated matrix to the bone, recalculate the bone start and end position in WS internally (by inversing along current bone start to origin, applying rotation of accumulated relative/local transform matrix of parent joints) and then re translating the joints back into WS using the translation column of the matrix. (Note we inverse using original bone start position (as this is rel to the joint), but we re-apply this translation post local space rotation from the accumulated translation of the matrix (of the updated joint transforms for the current frame), this applies the correct SRT (rotation, translation) order of operations.

Its interesting because even if the translation component is left in the matrix, when applied to the bone positions in local space at origin (after been inversed to origin via subtraction of the current start position (so start of bone is at origin and end of bone is offset along joint offset from origin)) the translation has no affect. Its only when the translation is extracted and added afterwards that it works correctly Ie :

```C++
// Bone::Update()
// [..]
// Invert to origin along bone start (orginal joint location/trans)
bone_start -= bone_ws;
bone_end   -= bone_ws;

// Do rotation in LS (Translation column of matrix left as is)
bone_start = no_trans * bone_start;
bone_end = no_trans * bone_end;

// Re apply new joint translation back to WS
bone_start += joint_trs[3];
bone_end += joint_trs[3];
// [..] Update bone vert GPU data
```

Yields the same result as : 

```C++
// Bone::Update()
// [..]
// Invert to origin along bone start (orginal joint location/trans)
bone_start -= bone_ws;
bone_end   -= bone_ws;

// Remove Translation from joint transform matrix
glm::mat4 no_trans = joint_trs;
no_trans[3] = glm::vec4(0.f, 0.f, 0.f, 1.f);

// Do rotation in LS
bone_start = no_trans * bone_start;
bone_end   = no_trans * bone_end;

// Re apply new joint translation back to WS
bone_start += joint_trs[3];
bone_end   += joint_trs[3];
// [..] Update bone vert GPU data
```

The latter is correct as only the rotation should be applied at origin, but I assumed that it would also do the translation back to WS also (within the matrix multipcation) so I wouldn't need to then add the translation column back to the start/end positions as a separate step, but I guess I do. The matrix seems to be correct, so I can't see why the translation multipcation wouldn't work, but doing it as a separate addition is not a big deal, just interesting that the translation has no affect within the matrix multipcation. Could debug further but no time at the moment and this is working as is. 

The performance of this approach is great, and this is even before I refactored the Skeleton -> anim data class, there's no joint-bone hash map (we search each bone for the current traversed joint) yet the perf is still great.

Theoretically each bone is defined by two joints (start joint is the "parent" joint of which the matrix we accumulate and update is used to update the bones start + end transformation, as the end point is defined as the parent joint transform + the child joints offset). However we treat it as if the bone stores a single joint index, which is the joint at the start of the bone, as per above this defines the transform of the bone itself, the end point is just this + the joint->offset. Hence we search bones for joint id which is the starting joint, and we update the correct bone according to which bone has this current traversed joint as its joint ID / starting (parent) joint in a parent->child like relationship. 

The approach itself is split into two parts, as state the bone hierarchy of joints is pre-built initially once (when BVH File is loaded), this is done by the non recursive approach, ie Loop over each joint, and accumulate translation / offset back to root (including root's translation), and add bones at these start and end locations. Where start position is the parent joints accumulated offset and end position is this offset + the joint's offset (parent-child order). Code for this can be seen here : 

```C++
// Fill out Skeleton from BVH Tree using offsets (only need to be done once per BVH file load, then update channels per anim frame)
void Anim_State::build_bvhSkeleton()
{
	// Get root offset  from Channel Data of 0th frame
	// Channel indices should be first 3 (as root is first joint in hierachy)
	glm::vec3 root_offs(bvh->motion[0], bvh->motion[1], bvh->motion[2]);

	for (Joint *joint : bvh->joints)
	{
		if (joint->is_root)
		{
			// Use fetched root offset
			skel.add_bone(glm::vec3(0.f), root_offs, glm::mat4(1), -1); // Bone has no starting parent joint (hence -1 index).
		}
		else // Regular Joint
		{
			// Get Parent Offset 
			glm::vec3 par_offs(0.f);
			Joint *p = joint;
			while (p->parent)
			{
				// Accumulate offset
				par_offs += p->parent->offset;

				// Traverse up to parent 
				p = p->parent;
			}
			// Add fetched Root offset
			par_offs += root_offs;

			// Add Bone to Skeleton
			std::size_t cur_par_idx = joint->parent ? joint->parent->idx : 0;
			skel.add_bone(par_offs, (joint->offset + par_offs), glm::mat4(1), joint->parent->idx); // Bone starts at parent joint. 
		}
	}
}
```

While it avoids recursion it may be a bit in-efficient as each joint has its hierarchy reverse traversed back to root, as oppose to just traversing the whole skeleton once from root via recursion like approach, but this seems to work fine for now and the joint offsets of parents is accumulated correctly, with the root offset been added also.  Note you don't need to add the root bone itself, but it can be done to viz the offset of the root from world origin, as this approach uses reverse traversal, there's no worry about not adding root but still needing to accumulate its offset, as root offset is added to all joints resulting accumulated offset anyway, but that doesn't mean we need to create a root bone itself (whose start would be at world origin and end at the root offset, which can look odd for a skeleton).  

Now even though the Skeleton is built from joints -> bones, we need to update the joints themselves and then from this update the bones (as oppose to traversing bone objects and then updating joints, because we'd still need to access joints to know where the children are as bones only store there starting joint index, so makes sense to update joints directly). So we have a recursive traversal of the joints from root, we accumulate the translation (offset) (which is only time dependent for the root of course as its offset comes from motion data (the first 3 values of motion data are the roots X,Y,Z translation)) and rotation which is time dependent, based on the current animation frame set within Anim_State.  We accumulate the rotation axis by axis and apply it / rotate it to the matrix (via glm::rotate) and then from here we ideally would have a stored mapping of <bone,joint> so we know which bone to update based on this new joint transformation, for the initial POC I just search the skeleton bone array for the matching joint (the joint is the starting joint of the bone which defines its offset rel to parent + the child joints offset which defines the end point of the bone). (Note for now I don't handle the end points of the joints, however this can be added back later, can add attribute to bones to define if they are end sites, but the process will be the same, for skeleton joint hierarchy end_sites are at (0) offset from the last joint itself). Code for this can be seen here :

```C++
// Update Bones based on joint data for current set anim_frame. 
void Anim_State::update_bvhSkeleton()
{
	 fetch_traverse(bvh->joints[0], glm::mat4(1.f));
} 

// Recursivly traverse through hierachy, update joints and their resulting bones... 
void Anim_State::fetch_traverse(Joint *joint, glm::mat4 trans)
{
	//  =========== Get Translation  ===========
	if (!joint->parent) // Root joint, translation from channels. 
	{
		glm::vec4 root_offs(0., 0., 0., 1.);

		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
			case ChannelEnum::X_POSITION:
			{
				float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.x = x_p;
				break;
			}
			case ChannelEnum::Y_POSITION:
			{
				float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.y = y_p;
				break;
			}
			case ChannelEnum::Z_POSITION:
			{
				float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
				root_offs.z = z_p;
				break;
			}
			}
		}

		trans = glm::translate(trans, glm::vec3(root_offs));
	}
	else if (joint->parent) // Non root joints, Translation is offset. 
	{
		trans = glm::translate(trans, joint->offset);
	}

	// =========== Get Rotation ===========
	glm::mat4 xx(1.), yy(1.), zz(1.);
	for (const Channel *c : joint->channels)
	{
		switch (c->type)
		{
		case ChannelEnum::Z_ROTATION:
		{
			float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
			trans = glm::rotate(trans, glm::radians(z_r), glm::vec3(0., 0., 1.));
			break;
		}
		case ChannelEnum::Y_ROTATION:
		{
			float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
			trans = glm::rotate(trans, glm::radians(y_r), glm::vec3(0., 1., 0.));
			break;
		}
		case ChannelEnum::X_ROTATION:
		{
			float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
			trans = glm::rotate(trans, glm::radians(x_r), glm::vec3(1., 0., 0.));
			break;
		}
		}
	}

	// Search for joint in bones and update transform of each end point.
	for (Bone &bone : skel.bones)
	{
		if (bone.joint_id == joint->idx)
		{
			bone.update(trans); // Pass Joint transform matrix to update bone
		}
	}

	/*
	// ==================== End Point ====================
	if (joint->is_end)
	{
		// Not Handeled currently, but we'd find bones who are end_sites
		// and update them the same way. 
	}
	*/

	// ==================== Children ====================
	// Pass each recurrsive call its own copy of the current (parent) transformations to then apply to children, creating stacked matrix accumulation. 
	for (std::size_t c = 0; c < joint->children.size(); ++c)
	{
		fetch_traverse(joint->children[c], trans);
	}
}
```

To Update the bone we then pass the accumulated transformation matrix to the `Bone::update()` member function which then recalculates the start and end positions by inverting the bone to local space (relative to the start position (and thus parent joint), with the end position offset from the origin by the joint itself offset (child offset)), we apply the rotation component of the matrix, and then move back into world space by adding the translation component of the matrix. We now have a update start + end position (and line defining the bone) transformed based on the current joint angles, accumulated along the joint hierarchy. Its a much better approach than calculating the start + end transforms inline and then passing to bone, it makes more sense for bone to  be passed its new transform matrix to then calculate the modified start + end positions from, thus the initial rest pose would be defined by an identify transform (as the accumulated rotation and translation would not effect the rest pose start end, defined by the original call to `Anim_State::build_bvhSkeleton`). Note Joint's themselves don't store any transformation state, this accumulated state is passed directly to bones for now of which the joint defines the start of, however when doing IK we may need a more stateful current tick representation of joint where we also store the current tick/anim frames transform in the joint class itself, for further joint operations (IK) without having to lookup the matrix we just passed to the joints bone. 

Code can be seen here : 

```c++
void Bone::update(const glm::mat4 &joint_trs)
{
	glm::vec4 bone_ws(start, 1.f);
	glm::vec4 bone_start(start, 1.f);
	glm::vec4 bone_end(end, 1.f);

	// Invert to origin along bone start (orginal joint location/trans)
	bone_start -= bone_ws;
	bone_end   -= bone_ws;

	// Remove translation component for local space rotation only. 
	glm::mat4 rot = joint_trs;
	rot[3] = glm::vec4(0.f, 0.f, 0.f, 1.f);

	// Do rotation in LS
	bone_start = rot * bone_start;
	bone_end = rot * bone_end;

	// Re apply new joint translation component back to WS
	bone_start += joint_trs[3];
	bone_end += joint_trs[3];

	// Update line Data
	std::vector<glm::vec3> pos_updt; pos_updt.resize(2);
	pos_updt[0] = bone_start;
	pos_updt[1] = bone_end;
	line->update_data_position(pos_updt);
}
```

We pass these new start+end positions to the the bones vertex data and update the vertex buffer via the `Primitive::update_data_position(std::vector<glm::vec3> &vert_positions)` member function which essentially just replaces the VBO data via updating the internal Primitive vetex data float array (by replacing position attributes per vertex along a single vertex stride) and then calling glBufferData to update the OpenGL resource. Code :

```C++
// Only sets position of vertices (allows for updating positions per tick)
void Primitive::update_data_position(const std::vector<glm::vec3> &posData)
{
	for (std::size_t v = 0; v < vert_count; ++v)
	{
		std::size_t i = v * 11; // Vert Index, Position. 
		vert_data[i++] = posData[v].x;
		vert_data[i++] = posData[v].y;
		vert_data[i++] = posData[v].z;
	}

	// Refill Buffer
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, (vert_count * 11 * sizeof(float)), vert_data.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
```

Compare this approach to the "Immediate Mode" approach where new bone render primitives are re-built per frame, within the recursive traversal of the joint hierarchy, requiring new OpenGL resources to be allocated, spun up old ones deallocated etc it was slow (hence why that approach made more sense to use actual immediate mode OpenGL, and not me replicating it with Modern OpenGL and recursive matrix stack). 

###### Improvements :

Merge the BVH_Data class and Anim_State together as mentioned elsewhere in these notes, and create a hash map to map joints to bones, without needing to search all bones joint indices members for the current joint been transform been updated within the `Anim_State::fetch_traverse()` call. This would mean we could eliminate the need for the search :

```c++
// Anim_State::fetch_traversal()
// [..]
	// Search for joint in bones and update transform of each end point.
	for (Bone &bone : skel.bones)
	{
		if (bone.joint_id == joint->idx)
		{
			bone.update(trans); // Pass Joint transform matrix to update bone
		}
	}
// [..]	
```

We could just get the `std::map<joint*, bone*>` bone and pass trans matrix directly to it, saving some time for sure. 

Would be nice to implement this approach of updating the joint transforms (and thus bone vert positions) into my fork BVH_Viewer project, and doing a little writeup about how the immediate mode approach of re-creating the bone GPU resources each frame vs just updating the vert positions from new accumulated joint transforms (inversing back to LS to apply rotation etc) is far faster. 

___

#### Inverse Kinematics - Handling Combination with FK / Input BVH Motion

Getting this working with the BVH Motion is going to be challenging, because the BVH Anim will need to be paused, joint selected, joint effector created, moved by user, get affected joints back up to the root, calc joint angles and then do we just override the BVH Motion data for these joints channels, or do we do a keyframe like approach going from the BVH joint angle (motion) data to the new IK based angle data (which may or may not vary temporally, but these will be calculated per tick, not cached), or do we interpolate between the IK Angles and the original BVH FK angles. Many different ways to approach this. 

Bones map back to joints, modify joints, calculate IK on joints, map back to bone position delta. The joint of each bone, is the joint that is located at the start (parent) of the bone (the other is offset by the joint child offset).

User selects a joint, creates a sphere to be the goal end effector which is positioned in 3D Space, the the joints hierarchy (back up its parents to root) is modified so its per frame angles come from the solved IK and not the BVH motion data. 

For time sake, we could just hardcode end effectors to operate on sets of joints eg left arm, right arm etc. And then have UI interaction to move the end effectors. Oppose to trying to have a procedural approach with user selection of bones, mapping back to the correct joints etc, we could just hard code pre-defined (rig like) controls (assuming the same joint hierarchy is used between BVH files, which we know for the humanoid files we got, is the case). In this case we don't need the Skeleton and Bone classes really, as we are just modifying sets of joints directly and then their primitives will be updated for drawing, so I could maybe use the FK / BVH Viewer fork as a starting point, oppose to going back to the original assignment code base ? Depends if I want to try and do a more efficient bone primitive update (oppose to re-building per tick).

Recursive build step, recursive update step (pass new transform matrix to bones, recalc bone start/end points internally updating vertex buffers without recreating) (see above section).

Need to create a hash map of Bones to joints, so we can efficiently do recursive traversal of joint hierarchy for updates to joint transforms, and then pass them to the bones of each joint, we don't want to have to search each bone for the correct bone matching the current JointID each time. This is making me think I could abandon the BVH Class, and combine it all into the anim_state class. Thus all the original BVH Data, joints, channels will live together and will make mapping between joints and bones easier. I will also get rid of the skeleton class and instead keep bones within an array in the anim_state class (as all skeleton class does is encapsulate an array of bones and render them, we can implement this in anim_state).

As for storing the BVH motion data, for now we will use the same approach, but I may need to abstract the motion data array a bit, so I can more easily overwrite joint angles if IK is used for certain joints. 

Remove the mesh object from bone class, not gonna have time to render mesh bones for now. 

Makes sense to merge BVH_Data class with Anim_Data so joints are directly part of class members, can do mapping between bones and joints + storing the motion data directly within, so IK can overwrite affected joints per frame motion data, with resulting IK angles. However It's gonna lead to a giant class and loss of modularity. 

To map between Joints and Bones without merging classes we can use friend classes of Anim_State,BVH_Data and Skeleton. However as Skeleton class currently stores bones as a vector of bone objects I changed this to vector of pointer to bones, so I can easily map each joint ptr (pointing to joints in the BVHData class joint array, to Bone pointers within the Skeleton class bone array via `std::map<joint*, bone*>`) or I could of used the map to store index of bone to joint ptr, instead of the bone ptr directly. Make sure the bone joint ptr, is the parent joint of the current joint (same for defining the bones joint index member) :

```C++
// Anim_State::build_bvhSkeleton()
// [..]
// Add Bone to Skeleton
std::size_t cur_par_idx = joint->parent ? joint->parent->idx : -1;
skel.add_bone(par_offs, (par_offs + joint->offset), glm::mat4(1), joint->parent->idx); 
// Bone starts at parent joint. 
// Add Joint-Bone Mapping (Joint is within BVH_Data, Bone within Skeleton Class)
//                       Joint*         Bone*
joint_bone.insert({ joint->parent, skel.bones.back() });
// [..]
```

If a bone from the root joint is not created then there will be no bone that maps from the root joint ptr, to a bone ptr, so we need to be careful of this, before this was handled by just discarding the transform update for the joint if no bone was found with the joint index, but doing a direct mapping means we have to check this case still. Bear in mind multiple bones will have the root joint as their joint index (where the bone starts) as the root joint has multiple children ofcourse, this is more nicely handled in the search approach then direct mapping because again we need to check if the joint in the map, maps to the root joint, however because in this case no bone has been created for the root joint (for viz sake we abandon it) it breaks the mapping of these bones who have root joint as their start/joint parent. The search approach directly maps these to the 0th index of the joints array, but the map approach doesn't because no mapping exists. 

For the sake of time, I'm going to not use a hash map to map joints to bones, searching for bones with the current parent joint to pass the joint transform update to was working fine, and the perf is not too costly. Its not like we are searching 10000s of elements.  

____

##### Interfacing Between Inverse Kinematics - Anim_State 

Input glm based transforms within Joints, internally the IK_Solver class uses eigen, joint angles which we then apply using the traversal methods described above, to apply the joint angles to the joints, accumulating the transformations to define bone transforms, in the same FK approach we use for the pure BVH Motion. The resulting joint angles should be the same format, possibly within the same array, so we're just doing the same as before, apart from some joints have new angles computed as part of the IK Solve, while others are based on their original offsets defined in the BVH Motion channel data. 

Eigen will only be used internally of the Inverse Kinematics Solver classes, if I had time i'd define a polymorphic base class IK Solver, to then implement for each type of IK solver, but as I'm probs only going to have time for one type of IK, i'll keep it simple for now and avoid polymorphism approach, can always add later. 

I've made the decision that IK will directly operator on joints, not bones (and through bones, their joints) thus we will do the same as the FK update, joints updated via recursive traversal, accumulation of transformations rel to parent from root, from this find bones affected and pass updated transform matrices to bones, to update bone start/end vert postions. Thus bones are really just a rendering construct, operations for both FK and IK are done on joints and motion data directly. 

I don't plan on needing to store joint transforms on joints directly, as most likely we just return the joint angles to then update the motion data array (replacing or copying from the BVH File import data) with the joint angle data solved from IK.

Input is an effector (which is just a position and a mesh primitive wrapped into a class, so we can move it about to define a IK effector) and the joints to affect by IK, this may just be one joint, we then need to determine affected joints by traversing back to root in reverse and collection parent joints, or user could just specify affected joints (so whole hierarchy branch is not indirectly affected by IK, like a depth parameter). Then based on these inputs we calculate the jacobian pseudo inverse so that the joint angles iteratively tend towards the end effector. Return array of joint angles, or write directly back to motion data array ? 

So this specifies an IK Chain. The effector class also stores a vector of joint pointers, to the joints it effects, these will be the joints of the IK chain also. Will create Member Function to collect joints back along hierarchy to define IK Chain.  We could also implement some method to get IK chains from end_site joints back up to some depth (or to root) but will worry about that later, for now will define hardcoded IK chains of joints to solve. 

Could do with another recursive function to get joint position (concat offsets from root), unless we store these onto joints when building bone skeleton (ie just a zero vec4 multiplied with the current joint transform matrix, should given the current WS pos).  I need this so I can use it as the end effector starting position. This is where using bones as the base would make more sense as we have the positions of bones in WS calculated already, however then we'd have to access joints via bones, maybe that makes sense, for now I'm avoiding this approach though. 

For test sake the IK Setup will be done within Anim_State on a predefined set of joints (eg the left hand back to the root) by some end effector. 

Not sure in future if its ideal to store IK chain joints within effector, effector should probs just store the joint its targeting + offset from it (to define the target pos for the joints to tend to) IK chain array of joints probs just passed to IK solve calls directly.

Joint positions are now stored/updated within joint position member, per tick assuming update_skeleton is called per tick.

 We will also need to calculate IK per tick and integrate the solved joint angles. Or we use an iterative approach and try and solve the whole motion needed to reach the end effector on the current tick / frame. 

We discard end effector orientations, only care about the positions, thus our Jacobian dimensions will be $3 \times N$ where 3DOFs for the end effector and N is the number of angles of all joints in the IK chain. 

Trying not to confuse Effector class with End Site / End Effector of joints.  Effector could be used to define an end effector, or some arbitrary target. Technically the end site would define the start location for the end effector which would then be moved by user interaction. The intention of the effector class is to define the target for the joints end effector / end site, to move towards and thus all linked joints from that end site.

Would make more sense for IK chains to start at end sites joints of the hierarchy, not just random joints, however not a big deal as even if child joints were not affected by IK, they would still be transformed correctly (should be) with relative offsets to there IK transformed parents, along as the resulting IK and original BVH joint angles are accumulated transforms and applied together. 

While we refer to the current effector position and target position, technically this is only for the end effector of the joint, we are actually referring to the current position vs target position of the Joint. All positions for Jacobian evaluation are treated as within World Space, hence why we need to update the Joint hierarchy first to get the current joint WS positions from accumulated relative transforms, before calling IK update related functions. 

##### User Interaction with Bones/Joints Ideas

As per above, most likely won't have time for this now, these were my initial ideas. 

Get inputs from viewer class (Could just pass window to anim state class directly to query).

Ray casting from mouse ? Selection from list of joints via GUI ?  Input like frame stepping will be done within viewer, (inc/decrementing anim frame eg.). Worst case, select joints using keyboard. Will then need to use keys to move or mouse input (which will rule out using free camera as we need to mouse pos to calc offset).

We will need some way to move the effectors (ie add offset to their target positions which is most likely a joint, so user can move end effector so it still follows joint target with some offset) to create disparity and thus using IK to solve for the joints to continue targeting the effector. Ideally I want to keep all input polling within Viewer and forward to Anim_State via MFs as I did for the input state for the FK stuff. 

____

#### Inverse Kinematics (IK_Solver)

Build Jacobian from input Joints and Effector, where input joints define known joint positions and angles including joint end effector along with a target effector defines known target position. 

The follow IK Calculations assume we know all joint positions and the chain end effector (which I will use end sites to define, but theoretically could be any joint defined as the "end" joint in the chain) along with the target for the end effector (I denote as target effector ). These positions are all expected to be within the same coordinate space, and because we now store the accumulated transforms of the joints from the motion / channel array per tick in World Space (via their accumulated local space / relative transforms to parent) we can assume all positions for Joints and Effectors are in world space and will treat as such. 

One thing i'm unsure about is, most texts only cover revolute joints, ie rotational joints with only 1DOF, this is not the case here, all joints have 3DOFs. I've seen mention of the Perturbation method for computing the Jacobian, oppose to the method shown in Rick Parents book and the URegina article, where you use a more typical FDM numerical approach solving as an FK problem, this seems a bit more convoluted and would mean it might make more sense to do this directly in Anim_State class, but I'll come back to this approach if needed. 

Ideally we should be checking if Target Effector position is reachable - To check if effector position is reachable by joint chain, check for $||L_1 - L_2||$ and $||L_1 + L_2||$ min and max distances (eg for 2 bones (lengths of deltas between joint positions)). 

We assume there is only a single chain of joints and a single end effector in this case, if I had time we would allow for multiple end effectors, for multiple chains over the system, eg for each limb, but time wise I'm just focused on single chains. 

Note that the delta defines the amount of displacement the Jacobian will solve for hence the final transformation of the joints is done over multiple frames. Larger h will result in, inaccurate Finite differences and thus resulting joint angles / transformation. Or you evaluate it for multiple iterations within the same frame, successively applying the resulting angles (obtained from the inverse) so that the solution converges within a single frame, as oppose to over time / frames, at the cost of longer per frame computation times. 

_____

#### Jacobian Matrix Formulation & Evaluation

##### Jacobian Formulation

From Rick Parent's textbook we know that : 

```
The rotation at a rotational joint induces an instantaneous linear direction of travel at the end effector.
```

However, Rick Parent's book as carefully picked situation ie 3 Joint system, with single DOF revolute joints which allows for a $3 \times 3 $ system, so its not the best reference for real world situations. 

For a single joint we can view the problem as :
$$
t-c = \textbf{J}(\theta_2 - \theta_1)
$$
We set the LHS $v$ vector be (t-c).

Where $t$ is the *target* position of the effector, $c$ is the *current* position of the joint / end effector (if joint is end site) these are $3\times1$ vectors, as we can discard the target effectors orientation as we only care about matching its position not rotation hence the DOFs of the end effector is also 3. We can see that the Jacobian will define the instantaneous linear velocity to move the joints from $c$ to $t$, as the velocities are calculated by differentiating the target effectors DOFs to the Joints DOFs (angles). Thus we will use the inverse of this to derive the joint angles to do so.  

The unknown joint angles to solve for via the Jacobian these are $N\times 1$ vectors where $N$ is number of joints  by the  Joints DOFs (stacked rotational angles into single vector) $ N \times (j \cdot dof)$. 

###### Jacobian refresher 

For system of $n$ equations of $y = f(x)$. To differentiate any $y_i$ we use the chain rule : 
$$
\partial y_i = {\partial f_i \over \partial x_1} \partial x_1, {\partial f_i \over \partial x_2} \partial x_2, {\partial f_i \over \partial x_3} \partial x_3, \dots, {\partial f_i \over \partial x_n} \partial x_n
\\\vdots\\
\partial y_n = {\partial f_n \over \partial x_1} \partial x_1, {\partial f_n \over \partial x_2} \partial x_2, {\partial f_n \over \partial x_3} \partial x_3, \dots, {\partial f_n \over \partial x_n} \partial x_n
$$
For all $f_i$, thus the Jacobian would define each $\partial y_i$ as rows of the matrix where 
$$
J(x_1, \dots x_2) = 
\begin{bmatrix}
{\partial f_1 \over \partial x_1} & {\partial f_1 \over \partial x_2} & \dots & {\partial f_1 \over \partial x_n}\\
{\partial f_2 \over \partial x_1} & \ddots && \vdots \\
\vdots & & \ddots & \vdots\\
{\partial f_n \over \partial x_1}  & {\partial f_n \over \partial x_2}  & \dots & {\partial f_n \over \partial x_n}
\end{bmatrix}
$$
Then combining with the vectors of partial derivatives on ethier side, we can solve for the derivatives $y_i$ given $x_i$ : 
$$
\begin{bmatrix}
\partial y_1 \\
\partial y_2 \\
\vdots \\
\partial y_n \\
\end{bmatrix}
 = 
\begin{bmatrix}
{\partial f_1 \over \partial x_1} & {\partial f_1 \over \partial x_2} & \dots & {\partial f_1 \over \partial x_n}\\
{\partial f_2 \over \partial x_1} & \ddots && \vdots \\
\vdots & & \ddots & \vdots\\
{\partial f_n \over \partial x_1}  & {\partial f_n \over \partial x_2}  & \dots & {\partial f_n \over \partial x_n}
\end{bmatrix}

\begin{bmatrix}
\partial x_1 \\
\partial x_2 \\
\vdots \\
\partial x_n \\
\end{bmatrix}
$$

##### Applied to Inverse Kinematics : 

In terms of inverse kinematics the input variables $x_i$ are the joint angles with the output variables $y_i$ representing the target end effector position (and orientation if used). However we want the inverse of this, but we'll worry about that in a bit. For IK this is typically formulated as : 
$$
\begin{bmatrix}
v_x \\ v_y \\ v_z \\ \omega_x \\ \omega_y \\ \omega_z 
\end{bmatrix}
= 
J(\theta_1, \dots, \theta_n)
\begin{bmatrix}
\dot{\theta_1} \\
\dot{\theta_2} \\
\vdots \\
\dot{\theta_n} \\
\end{bmatrix}
$$
Where $v_x,v_y,v_z$ are the desired linear velocities of the effector, $\omega_x,\omega_y,\omega_z$ are the desired angular velocities of the end effector (which are calculated from the joint angles velocities) $\theta_i$ are the joint angles and $\dot{\theta}_i$ are the unknown joint angle velocities. $V$ is the end effector velocity, thus to solve for $V$ we have the equation :
$$
V = J\dot\theta
$$
The Jacobian relates the change in end effector position (ie velocity) to the change in joint angles (ie their velocities via partial derivatives, target effector velocity / joint angle velocity) we will want the inverse of this later. It defines the linear instantaneous rate of change hence for accuracy it needs to be re-calculated for multiple iterations so the resulting inverse gives us incremental joint angle velocities to rotate the joints such that the end effector of the joint chain moves towards the desired goal position. 

We formulate this Jacobian in terms of the velocities denoting $\partial P_i = V_i$. However we can disregard the (end) effector orientation and just solve for the positions (and thus linear velocities), so the row count goes from 6 to 3 now.

Thus the resulting Jacobian matrix is  thus size of $3\times N$ where each positional component is differentiated with respect to each joint angle. Where $N$ is number of Joints * Joints DOFs (stacked rotational angles into single vector). Each column defines a single DOF / component differentiated with respect to a single positional component of the end effector, which means for any system of joints larger than 3 we can see the Jacobian matrix is not square. 
$$
J(\theta_1, \dots, \theta_n) = 
\begin{bmatrix}
{\partial P_x \over \partial \theta_1} & {\partial P_x \over \partial \theta_2} & {\partial P_x \over \partial \theta_3} & \dots & {\partial P_x \over \partial \theta_n}
\\
{\partial P_y \over \partial \theta_1} & {\partial P_y \over \partial \theta_2} & {\partial P_y \over \partial \theta_3} & \dots & {\partial P_y \over \partial \theta_n}
\\
\vdots & \vdots & \vdots & \vdots & \vdots \ \\
{\partial P_n \over \partial \theta_1} & {\partial P_n \over \partial \theta_2} & {\partial P_n \over \partial \theta_3} & \dots & {\partial P_n \over \partial \theta_n}
\end{bmatrix}
$$


Eg for the case of 2 joints (each with DOFs / Angles) and the end effector position only the Jacobian would be $3 \times 6$ :
$$
J(\theta_1, \dots, \theta_6) = 
\begin{bmatrix}
{\partial P_x \over \partial \theta_1} & {\partial P_x \over \partial \theta_2} & {\partial P_x \over \partial \theta_3} & {\partial P_x \over \partial \theta_4} & {\partial P_x \over \partial \theta_5} & {\partial P_x \over \partial \theta_6}
\\
{\partial P_y \over \partial \theta_1} & {\partial P_y \over \partial \theta_2} & {\partial P_y \over \partial \theta_3} & {\partial P_y \over \partial \theta_4} & {\partial P_y \over \partial \theta_5} & {\partial P_y \over \partial \theta_6}
\\
{\partial P_z \over \partial \theta_1} & {\partial P_z \over \partial \theta_2} & {\partial P_z \over \partial \theta_3} & {\partial P_z \over \partial \theta_4} & {\partial P_z \over \partial \theta_5} & {\partial P_z \over \partial \theta_6}
\end{bmatrix}
$$

Of course the Joint angles are unknown and we want to solve for them, not for the end effector position hence the Inverse of the Jacobian is needed.  Will cover that later. 

##### Relating the Jacobian to the Target End Effector Position : 

I initially struggled to understand how the Jacobian (which is the derivative of the joint chains current end effector position wrt the current joint DOF / angles) is then related to the target effector position, as P is the current location of the end effector of the chain. But if we look at the system we are trying to solve, we explicitly define the LHS of the system $V$ the velocities vector which for a single end effector is just a $3\times 1$ vector as we discard its orientation. 
$$
V = J(\theta_i)\dot\theta \\ (t-c) = J(\theta_i)\dot\theta 
$$
We relate the desired velocity (ie the velocity of the end effector from its current position to the target position), to the Jacobian (which is the current linear velocity of the end effector wrt to each joint angle / DOF hence the $J(\theta_i)$ notation). Then we have the time derivative / angle velocities $\dot{\theta}$. 

The LHS and RHS are vectors ofcourse, but this notation is more compact. $V = t-c$ is a $3 \times 1$ vector as mentioned for the single end effector.  The Jacobian is ofcourse as $3 \times (j \cdot 3)$ matrix assuming the end effector only has position and each joint $j$ has 3 DOFs. 

Don't confuse $(\theta_i)$ which is the current joint angles of the joints DOFs as is, with $\dot{\theta}$ which is the angle velocities / angular velocities delta. 

Thus where the Inverse is 
$$
\dot{\theta} = {J(\theta_i)}^{-1}V
$$
We set $V = t-c$ and we have the $J^{-1}$ and we get the $\dot{\theta}$ (also may be denoted $\Delta \theta$) as the output, which is the delta in angular velocity to apply to the joint angles, such that they tend towards the target $V$ end effector velocity. 

Thus we treat the velocity $V$ of the end effector we are solving, as $t-c$ and we want to solve so that the resulting velocity matches as closet as possible this goal condition. So it becomes a bit like a Newton Solve where we are linearizing the system and using this to advance the solution iteratively using the inverse, because the actual end effector position as a function of joint angles as you can imagine is highly nonlinear. 

This is where we use either (or both) a time integration approach, to evaluate the the Jacobian multiple times over a number of frames so that the end effector position tends towards the target position gradually over time or, we can do multiple Jacobian + application steps per frame (where resulting $\dot{\theta}$ are applied back to angles directly) using an iterative approach at the cost of longer calculation times per frame (as multiple iterations of Jacobian Calculation, Inversion and Application are needed per frame), however this would remove the need to temporally integrate gradual joint angle deltas over time as it could ensure the end effector reaches the target end effector position, within the current frame. 

An iterative step like approach  could be formulated using a simple error function, where we minimise the error s.t the resulting $V$ approaches $t-c$ 

Something like :
$$
||t-c|| > eps
$$
Where eps / tol is the tolerance of the position deltas we will allow for. 

We apply the resulting $\dot{\theta}$ to the system using standard FK approach as before, traversing through the chain (using the same method for calculation the end effector perturbations see below) but this time applying the resulting delta angles to each joints rotational axis / DOF. 

##### Building the Jacobian Matrix : Analytically with "The Cross Product Method"

This is the method described in the modules textbook (Rick Parent's book), and while in the book its not described as analytical, because it uses a closed form solution to each column of the Jacobian it's typically denoted as such. The book shows a system of 3 joints where each joint is revolute and only has a single DOF, which makes this method not very useful for our problem where each joint defined from the input BVH joint hierarchy has 3 DOFs for each rotational axis. 

We know the joints axis of rotation (by defining an arbitrary axis based on its channels), we can use this with cross product of the vector to the end effector of the joint chain to define : 
$$
Z_i \cross (E - J_i)
$$
Defines the instantaneous linear velocity of the chains end effector, hence this is what we use to define each column of the Jacobian. Where $Z_i$ is the i'th joint axis of rotation, $E$ is the chains end effectors position and $J_i$ is the i'th joint position. So this can be see as the cross product of the Joint Axis and the direction from the joint to the end effector. Each of these defines a column in the Jacobian, with each component (DOF) been on each row hence the matrix is of size $3 \times (j \cdot 3)$. 

So eg $\partial P_z \over \partial \theta_1$ is the velocity x component of the end effector, by the rotation of joint 1's DOF. Which is simply just the x component of the above cross product where the $i$th joint is Joint 1 $J_1$ (Don't confuse j notation with Jacobian) ! 

However we can see now that this formulation only makes sense for joints with 1 DOF ? As each column defines a single joint, with a single DOF, How would it be possible if each joint has multiple DOFs as they do in our case ? All texts I see also assume $Z_i$ has a single DOF eg the axis $(0, 0, 1)$ where as in my case each joints 3 DOF axis is arbitrary.

##### Building the Jacobian Matrix : Numerically with "The Perturbation Method"

However this method may not work for joints with more than 1 DOF ie rotational joints in my case and not single DOF joints like Revolute or Prismatic joints which the above method for deriving the Jacobian seems to be limited to, hence an alternate approach is needed to allow differentiation for each joint with respect to each DOFs for all its rotational angles as the end effector position is a function of the joint angles, hence we want to differentiate it against each individual joint, DOF.  

One such approach is a "perturbation" approach to differentiation to calculate the Jacobian. Where the end effector position (and rotation if used) is differentiated with respect to the joint angles for all the joints DOFs, in a forward finite difference like approach, ie using perturbations of $\Delta \theta$ to solve for the unknown partial derivatives numerically. 

So we can define a single term of the Jacobian as : 
$$
{\partial P_x \over \partial \theta_1} \approx {\Delta P_x \over \Delta \theta_1} = 
{P2_x - P1_x \over \Delta \theta_1}
$$
However to get $P2_x$ we need to formulate it as a FDM problem, where $P2_x$ is $P1_x$ are positions which are functions of joint angle (for each DOF) but with the perturbation $h$ added. Which is forward differencing  where $h = \Delta \theta$. 

Thus we get : 
$$
{P(\theta_1+\Delta\theta)_x - P_{x}(\theta_1) \over \Delta \theta}
$$
Where $\theta_1$ is the joint, DOF, current angle, with $\Delta \theta$ been the perturbation applied to form a Forward difference term to evaluate the Jacobian element numerically. Thus we solve the instantaneous rate of change ie the Partial derivative, as the joint angle changes to relate it to the instantaneous change of the end effector position. 

So a single column *(note transpose notation)* of the Jacobian would be : 
$$
{\partial P \over \partial \theta_1} = 
\begin{bmatrix} 
{\partial P_x \over \partial \theta_1}, 
{\partial P_y \over \partial \theta_1}, 
{\partial P_z \over \partial \theta_1} 
\end{bmatrix}
^T
= 
\begin{bmatrix} 
{\Delta P_x \over \Delta \theta_1}, 
{\Delta P_y \over \Delta \theta_1}, 
{\Delta P_z \over \Delta \theta_1} 
\end{bmatrix}
^T
$$
 $P$ of course denotes the joint end effector of the chain, or Joint position, if just a single joint is used component, with respect to each angle defined by the joints DOFs, which for a column is a single joint DOF $\theta_1$.  

(*Removed transpose notation for my autistic brain to better view the matrix in its actual form*), this would represent a single joint's 3DOFs (thus 3 columns).
$$
\begin{bmatrix} 
{\partial P_x \over \partial \theta_1} & {\partial P_x \over \partial \theta_2} & {\partial P_x \over \partial \theta_3} &   \: \dots 
\\ 
{\partial P_y \over \partial \theta_1} & {\partial P_y \over \partial \theta_2} & {\partial P_y \over \partial \theta_3} & \: \dots 
\\
{\partial P_z \over \partial \theta_1} & {\partial P_z \over \partial \theta_2} & {\partial P_z \over \partial \theta_3} & \: \dots \:
\\

\end{bmatrix}
$$
So here we are differentiating $P_x$ with respect to $\theta_1$ so if we apply the $\Delta \theta_1$ to $\theta_1$ with for the $x$ degree of freedom, which we can do by using Forward Kinematics, holding all other joints constant / as is, but perturbing the current joint along its x DOF. 

That means for the perturbations though we need to do FK traversal for each joint adding the perturbation/delta to single joints while holding the rest constant (we cant just add perturbation to each in a single traversal), so this is going to require $N$ joints, evaluations of the joint hierarchy to do the same transformation accumulation (accumulation of each joints Local Space relative transforms from motion data, to put into WS) as before, but this time we are changing a single joints, single degree of freedom $\theta$ by the perturbation factor $\Delta \theta$ each time to relate the change to the change if end effector position of the chain / joint system for each Jacobian matrix element (as expensive as it sounds, and we've still got the inverse to worry about later) ! However as noted below, we only need to do this for joints in the chain (ie joints we want to be affected by the resulting IK solve, in most cases this won't be the whole joint hierarchy, most likely just a chain of joints) however it's not just per joint, its per joint DOF, which is 3 for each joint. 

To test this out I will implement within Anim_State to build the Jacobian first, the method to perturb each joint will be similar to `Anim_State::fetch_traverse(Joint *joint, glm::mat4 trans)` but with as *joint now defines the starting joint of some joint chain that will be passed to a caller function, along with some delta factor to define the perturbation amount per joint. However we only need to do this along the joint chain we are solving for, I can discard the rest of the joint system/hierarchy if I don't care about IK not affecting it, so I only need to do perturbations for joints along the joint chain I want to solve for. Eg in the test case, the joints that make up the right arm (from root). So the input would be (start joint, perturb joint, perturb factor) and then the return value should be the end effector position for each perturbed joint DOF, ie $P_2$ we can use with the original position end effector position $P_1$ without perturbation of the joint DOF / angle to define : 
$$
P_2 - P_1 \over \Delta \theta_i
$$
This result will then define the value of each Jacboian column, ie each DOF angle (of each joint) is perturbed for each $P_(x,y,z)$ component (remember we don't care about differentiating respect to end effector orientation). Thus a single column would look like this,  for ($N \cdot 3$ columns (joints * joint DOFs for each $\theta_i$)) making up the whole matrix, all other joints than the currently perturbed joint DOF are the same / held constant so we can measure the instantaneous change relative to each. 
$$
\begin{bmatrix} 
{P2_x - P1_x \over \Delta \theta_i} \\
{P2_y - P1_y \over \Delta \theta_i} \\
{P2_z - P1_z \over \Delta \theta_i} \\
\end{bmatrix}
$$
So we need to gather each joints, DOF perturbation's resulting end effector position, when accumulated along the joint chain hierarchy. This involves using the recursive traversal approach from before, but now we need to do it for each joints, degree of freedom, we update joint positions as we go (although it could be faster not too I guess as the non perturbed joint positions, should be the same as the joint positions currently are) until we get back to the end site, after each call we collect the new end_site of the chain (end effector) position with each perturbed angle / dof per joint, and gather them into an array to define $P_2 - P_1 \over \Delta \theta_i$ for each Jacobian entry / column per joint dof. 

Actually yeah DO not modify Joint positions unless they are the perturbed joint, we also need to reset all joints after each call, to their original un-perturbed WS positions before doing the next joint,DOF perturbation.  

I eval each joint at a time, for all 3 of its DOFs, then querying each component of $P_2$ after the perturbation to form the column of the Jacobian, the Jacobian could be constructed within this func, or I just return the resulting perturbed end effector positions for each column and then use it for $P_2 - P_1 \over \Delta \theta_i$ element wise construction per column of Jacobian in a separate function. 

Remember that each column is a single DOF of the joint, not a single joint, as we want each DOF with respect to each effector position component (hence we only change the effector position component on each row within the col, the DOF its differentiated against (and thus perturbed by) remains the same).

Problem is, the Traversal function starts from root, and will continually evaluate the whole joint hierarchy, not just the chain, so Maybe I can replace the current traversal recursive logic, with an iterative approach, seen as we have a bounded joint chain defined anyway. Should be fine to use iterative approach because we can assume each joint just has one child because we are just traversing along a linear chain (ie we don't need recursive call to each current joints, children). 

##### Perturbation based Jacobian Matrix Implementation test

###### Gather End Effector Perturbations : 

So in this test case we construct a joint chain from the `r_thumb` end site, back up to the root joint, this defines a chain of 9 joints. That's 9 joints, each with 3 DOFs, meaning 27 columns (3 x 27) Jacobian matrix will be constructed. So we use the perturbation to return $P1,P2$ for each column / joint DOF differentiated each with respect to each component of the end effector position, we we can then use to build the Jacobian columns in the form of : 
$$
\begin{bmatrix} 
{P2_x - P1_x \over \Delta \theta_i} \\
{P2_y - P1_y \over \Delta \theta_i} \\
{P2_z - P1_z \over \Delta \theta_i} \\
\end{bmatrix}
$$
This part just amounts to getting and returning the perturbed end effector positions for each joint DOF, the construction of the Jacobian for the above form, will be done separately using Eigen with the resulting position pairs $(P1, P2)$. 

The Code is split into 3 functions, first we define the Joint Chain, the Target Position (this will be needed later, for now we just care about the current end effector / end site). We know that the `r_thumb` joint is an end_site so this defines the current end effector : 

```C++
void Anim_State::ik_test_setup()
{
	// Create target effector (based on LThumbs Location + offset) 
	Joint *l_thumb = bvh->find_joint("LThumb");
	assert(l_thumb);
	Effector *eff_arm_r = new Effector(l_thumb->position, 0);
	eff_arm_r->target = l_thumb;
	eff_arm_r->target_offset = glm::vec3(-20.0f, 8.f, 0.f);
	// Will use this later to define the LHS ...

	// Create IK Chain based at RThumb (end_site)
	Joint *r_thumb = bvh->find_joint("RThumb");
	assert(r_thumb && r_thumb->is_end);

	// Define IK Chain of joints
	std::vector<Joint*> chain; 
	// Gather Joints back up to some depth (or up to root by default).
	gather_joints(r_thumb, chain);
	
	// Get Perturbed Postions of end effector, with respect to each joint 
	// dof to use for formulation of  Jacobian Matrix columns (P2 - P1 / Dtheta) 
	// End Joint Effector = r_thumb, Start Joint = root. 
	// Each Pair is the P1,P2 vals for each column.
	std::vector<std::pair<glm::vec3, glm::vec3>> cols_p1p2 = perturb_joints(chain, r_thumb, nullptr, 0.01f);
	
	// We know this chain has 9 joints. Each joint has 3 DOFs (rotational) hence 
	// we will have 27 sets of P1,P2 values, and thus a 3 x 27 Jacobian 
	// (as we only eval the 3 Postional components of the end effector) 
	
	// [..]
}
```

We have the call to `gather_joints(Joint *start, std::vector<Joint*> &chain, int32_t depth=-1)` which is what's used to then initiate the calls to the joint traversal function, perturbing each DOF / Joint angle per joint, storing the non-perturbed and resulting perturbed end effector positions with respect to each (defining 27 Columns of $(P1,P2)$ vec3s :

```C++
std::vector<std::pair<glm::vec3, glm::vec3>> Anim_State::perturb_joints(std::vector<Joint*> &chain, Joint *end_effec, Joint *start_joint, float perturb_factor)
{
	// Set start joint to root if passed nullptr. 
	Joint *start = start_joint == nullptr ? bvh->joints[0] : start_joint;
	
	// Number of Columns (theta_0 ... theta_n) (size = j * 3) 
	std::size_t dof_c = chain.size() * 3;  
	// We know number of rows is (size = 3) (3 DOFs in end effector) pos (P_x, P_y, P_z)	
	glm::vec3 end_pos = end_effec->position; 

	// Vector to store Orginal and preturbed postions of end effector, 
    // for each perturbed joint (for all rot DOFs). 
	// (Defines single col of Jacobian in the form of P2 - P1 / Dtheta)
	std::vector<std::pair<glm::vec3, glm::vec3>> pertrub_pos;
	
	// For each joint, for each DOF, traverse the chain hierachy, 
    // perturbing only the cur DOF, and then deriving the resulting end site 
    // postion, note that end site position == last joint (in chain) postion 
    // as there is no offset on the end_site locations. 

	// We want to return pairs of P1 and P2 (where P1 is non-perturbed, P2 is perturbed),
	// 	with respect to each joint DOF. We can then use these to build the Jacobian
	// 	element wise (P2 - P1 / Dtheta).
	for (Joint *perturb_joint : chain)
	{
		// Orginal Pos before perturbation of joint. 
		glm::vec3 org_pos = perturb_joint->position; 

		for (std::size_t c = 0; c < 3; ++c) // 0-2 (X-Z rotation DOFs)
		{
			// Get current DOF to peturb, for joint. 
			ChannelEnum DOF = static_cast<ChannelEnum>(c); 
			
			// Reset Joint Pos back to orginal (to remove last perturbation)
			perturb_joint->position = org_pos; 

			// Column wise Un-perturbed and Perturbed Vectors to store results in. 
			glm::vec3 P1 = end_effec->position; 
			glm::vec3 P2(0.f); 

			float delta_theta = perturb_factor; 

			// Function traverses the joint from start of chain
            // (assumed to be root for now), and when we reach the preturb joint, 
            // we perturb its DOF. We contiune accumulating the resulting transform 
            // along the chain and store the resulting end effector end joint
            // position. 
			perturb_traverse(chain, perturb_joint, DOF, delta_theta);

			// Query Resulting Modified End Effector Postion component
			P2 = end_effec->position;

			//assert(P1 != P2); // Check something is actually happeneing. 

			// Now P1 defines orginal effector pos, P2 defines perturbed 
            // effector pos, for the current DOF (rotational channel).
			pertrub_pos.push_back(std::pair<glm::vec3, glm::vec3>(P1, P2));
		}
	}

	// Return vector of pairs of un-perturbed and perturbed end effector postions
	// for each joint angle / DOF perturbation. 
	return pertrub_pos;
}

```

We probs don't need to store P1 (un-perturbed position) for each column, as it should the same system wide (ie the non perturbed end effector position), I just thought it was cool to store them in pairs, maybe I'm over engineering it. But for the following example code I'll leave it in. 

Then the actual traversal over the chain of joints, is similar to the approach we use for FK, apart from its not recursive, as we don't blindly accumulate the transform along all joint children passing a copy of the trans matrix to each, we do it, iteratively along the linear joint chain, hence we can use an iterative approach, accumulating the transformation for each iteration / joint. When we get to the joint whom is to be perturbed, we look at the current DOF passed to perturb and add the Perturbation factor to that angle, and continue accumulating the transform. Only when we get to the end effector to we update its position with the new resulting joint transformation matrix (with some perturbed joint DOF within it) this then defines the $P2$ perturbed end effector position, for each joint DOF :

```C++
// chain - chain to traverse and apply perturbations
// perturb_joint - the joint in the chain we want to perturb, 
// (check if current joint, == perturb joint).
// dof - the DOF / axis angle we want to perturb
// perturb_fac - Perturbation amount

void Anim_State::perturb_traverse(std::vector<Joint*> &chain, Joint *perturb_joint, ChannelEnum dof, float perturb_fac)
{
	glm::mat4 trans(1.f); // Accumulated Transform

	for (Joint *joint : chain)
	{
		//  Translation is the same. Preturbation only occurs on rotational DOFs 
		if (!joint->parent) // Root joint, translation from channels. 
		{
			glm::vec4 root_offs(0., 0., 0., 1.);

			for (const Channel *c : joint->channels)
			{
				switch (c->type)
				{
				// Translation
				case ChannelEnum::X_POSITION:
				{
					float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.x = x_p;
					break;
				}
				case ChannelEnum::Y_POSITION:
				{
					float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.y = y_p;
					break;
				}
				case ChannelEnum::Z_POSITION:
				{
					float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.z = z_p;
					break;
				}
				}
			}

			trans = glm::translate(trans, glm::vec3(root_offs));
		}
		else if (joint->parent) // Non root joints, Translation is offset. 
		{
			trans = glm::translate(trans, joint->offset);
		}

		if (perturb_joint) // Perturbed Joint Rotation 
		{
			for (const Channel *c : joint->channels)
			{
				switch (c->type)
				{
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					// Preturb for DOF Rot_Z
					if (dof == ChannelEnum::Z_ROTATION) z_r += perturb_fac;
					trans = glm::rotate(trans, glm::radians(z_r), glm::vec3(0., 0., 1.));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					// Preturb for DOF Rot_Y
					if (dof == ChannelEnum::Y_ROTATION) y_r += perturb_fac;
					trans = glm::rotate(trans, glm::radians(y_r), glm::vec3(0., 1., 0.));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					// Preturb for DOF Rot_X
					if (dof == ChannelEnum::X_ROTATION) x_r += perturb_fac;
					trans = glm::rotate(trans, glm::radians(x_r), glm::vec3(1., 0., 0.));
					break;
				}
				}
			}
		}
		else // Constant Joint Rotation
		{
			for (const Channel *c : joint->channels)
			{
				switch (c->type)
				{
				case ChannelEnum::Z_ROTATION:
				{
					float z_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					trans = glm::rotate(trans, glm::radians(z_r), glm::vec3(0., 0., 1.));
					break;
				}
				case ChannelEnum::Y_ROTATION:
				{
					float y_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					trans = glm::rotate(trans, glm::radians(y_r), glm::vec3(0., 1., 0.));
					break;
				}
				case ChannelEnum::X_ROTATION:
				{
					float x_r = bvh->motion[anim_frame * bvh->num_channel + c->index];
					trans = glm::rotate(trans, glm::radians(x_r), glm::vec3(1., 0., 0.));
					break;
				}
				}
			}
		}
		// ONLY transform the end_site, we don't care about actually 
		// transforming the other joints aslong as we have their transforms
		// accumulated (to propgate to the end site joint as it is what 
		// we are measuring the delta of). 

		// Assumes last chain joint is an end_site / effector. 
		if (joint->is_end) 
		{
			joint->position = glm::vec3(trans * glm::vec4(0.f, 0.f, 0.f, 1.f));
		}
	}
}
```

Not oppose to using switch over each joint channel, we could gather the motion data per channel for all joint channels and then extract them element wise, but for now this approach is what I use in the recursive joint tree traversal for FK rendering of the BVH bones, so it's what i'm basing this on (apart from this is iterative traversal). 

For reference, `gather_joints()` to build the joint chain, looks like this : 

```C++
// Used to create chain of joints for IK, gathering joints from starting joint, 
// back to root, up to some depth. 
void Anim_State::gather_joints(Joint *start, std::vector<Joint*> &chain, int32_t depth)
{
	// if depth -1, gather joints all the way back to root. 
	std::size_t c = 0; 
	
	Joint *joint = start;
	while (joint->parent)
	{
		chain.push_back(joint); // Append to chain

		if (depth > 0 && depth == c) break;

		joint = joint->parent;
		c++; 
	}

	if (depth <= 0) chain.push_back(bvh->joints[0]); // Also add root joint. 

	// Reverse joints order in chain, so root (or joint closest to root) 
    // is first, end_site joint is last. 
	std::reverse(chain.begin(), chain.end());
}
```

 I'm wondering if I should do this in double precision as the perturbation factor $\Delta \theta$ is small so $P2-P1$ deltas are very small, and likely to accumulate FP error overtime just using single precision fp32. 

##### Build Jacobian from Perturbations : 

As stated before each perturbation vector will define a column of the Jacobian following the Forward Difference form of : 
$$
\begin{bmatrix} 
{P2_x - P1_x \over \Delta \theta_i}& \dots \\
{P2_y - P1_y \over \Delta \theta_i}& \dots \\
{P2_z - P1_z \over \Delta \theta_i}& \dots \\
\end{bmatrix}
$$
So now we know $(P1, P2)$ for each we can actually build the Jacobian. This is done for now within Anim_State class like the perturbation gathering is, but later I will put it back within the IK_Solver class. 

We build an Eigen Matrix in of size $ 3 \times (j \cdot 3)$ which in this case is of size $3 \times 27$ as we have 9 joints in the chain. Make sure matrix is col major, we then will iterate column wise and build each elements FDM value as per above.  Loop Col wise, down each row, splitting the $(P1, P2)$ component wise.  For now I implemented this within `Anim_State::ik_test_setup()` Member function : 

```C++
// Anim_State::ik_test_setup()
// [..] Create Chain, Get End Effector, Get Perturbed Values 

	// ============ Jacobian Construction ============
	// Encap into Some class for Eigen use, for now will do inline.
	// Now Construct Jacobian. (3 x (j * 3)) j = number of joints in chain.
	std::size_t r = 3, c = chain.size() * 3; 
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> J;  
	J.resize(r, c); J.setZero();

	// Loop column wise
	std::size_t col_ind = 0; 
	std::stringstream r0, r1, r2; 
	for (auto col : J.colwise())
	{
		// Each Column get Non-Perturbed (P1) and Perturbed (P2) end 
        // effector postion vector3s. 
		std::pair<glm::vec3, glm::vec3> &perturb = cols_p1p2[col_ind];

		// Split into Effector Positional components (x,y,z) form ((P2 - P1) / Dtheta) 
        // for each col,row element. 
		col(0) = (perturb.second.x - perturb.first.x) / delta_theta;
		col(1) = (perturb.second.y - perturb.first.y) / delta_theta;
		col(2) = (perturb.second.z - perturb.first.z) / delta_theta;

		// Dbg stream output
		r0 << col(0) << ", "; r1 << col(1) << ", "; r2 << col(2) << ", ";

		col_ind++;
	}
	// Dbg Matrix output
	std::cout << "\n======== DEBUG::JACOBIAN_MATRIX::BEGIN ========\n"
		<< "Rows = " << J.rows() << " Cols = " << J.cols() << "\n"
		<< "|" << r0.str() << "|\n" << "|" << r1.str() << "|\n" << "|" << r2.str() << "|\n"
		<< "======== DEBUG::JACOBIAN_MATRIX::END ========\n";
// [..]		
```

This seems to be correct, apart from the last column (26th in this case) is all zeros, this would be the z'th rotation DOF perturbation of the final joint in the chain `r_thumb` ? The final joint is the end effector, so its differeinating its own rotation with respect to its own position (not with respect to itself, ie should not equal 1, as we are differentiation rotation of DOFs to position), thus if there is no change in the resulting end effector delta because of the rotation with what we are doing $P2 - P1 / \Delta \theta$ we get 0, which makes sense. The only case this occurs on is the 26th col for Z Rotation DOF of the `r_thumb` joint / end effector. I believe its correct that the end effector itself should be part of the Jacobian so this seems to make sense. 

Of course we know the matrix is not square, however this means it does not have a determinant to check. 

##### Iteration of Jacobian to Target Effector 

Now we can build the Jacobian via fetching perturbed end effector positions, we need to do it iteratively, so we can satisfy 
$$
V = J(\theta_i)\dot\theta \ \\ (t-c) = J(\theta_i)\dot\theta
$$
This involves calculating the Jacobian, Inversing it, applying the resulting delta angle velocities to the joints in the system and either by integration over time / a number of frames or successively for $k$ iterations within a single frame. 

___

#### Jacobian Matrix (Pseudo) Inverse

From the Jacobian calculation, we have at the velocities of the end effector wrt to each joints DOFs angles in the form of : 
$$
V = J\dot\theta \
$$
However we want the Joint Angle velocities, which we can apply to the joints to reach the target effector hence we need to inverse the Jacobian to get :
$$
\dot\theta = J^{-1}V
$$
Where $\dot{\theta}$ is the angular velocity of the joints to integrate to the joint angles, also denoted as $\Delta \theta$ but I omit that as it may be confused with the above perturbation factor of $\Delta \theta$. Where as derived above, $V = t-c$ the delta of the target vs the current end effector position. 

However we know that for systems of joints larger than 3, the Jacobian is non square, meaning it has no actual inverse as it has no deterimant. 

There may also be singularities that occur, that are impossible to avoid, depending on the joint configuration which may lead to an over constrained system where their are too few solutions or under constrained where their are too many solutions and no single unique solution, because of this the Pseudo-Inverse is used instead.

##### First - Approximating Inverse with Transpose

For simple initial case we could just use the Jacobian transpose as the inverse and thus return the $\dot{\theta}$ values as :
$$
\dot{\theta} = J^T  V
$$
For the test case described above, where $J$ is a matrix of dimensions $3 \times 27$ for 3 End Effector positional DOFs and 9 joints with 3 rotational DOFs, $V$ is a column vector of dimensions $3 \times 1$ for the 3 positional DOFs of the end effector. Thus $\dot{\theta}$ is a $27 \times 1$ vector of the resulting joint angle velocities. 

We know this is correct because $J^T$ is of dimensions $27 \times 3$ and $V$ of dimensions $3 \times 1$ so we have the matrix vector product : $(27 \times 3) \cdot (3 \times 1)$  with a matching inner pair $3,3$ that validates the matrix vector product is possible and the outer pair $27, 1$ defines the resulting vector size. We then take each set of 3 DOF Angles and use them to perform Forward Kinematics on the joint chain to modify their angles and thus transforms (and thus transform of the end effector to tend towards the target effector position).

##### Pseudo-Inverse 

Apart from the 3 Joint case (where each joint has a single DOF) and we only care about the end effector position (which defines a $3\times3$ matrix) we cannot just use the transpose as the inverse and get a stable result.  For low rank matrices where most columns are not linearly independent the transpose becomes much less of an approximation of the Inverse and we need to use another method to approximate the inverse, hence the Pseudo-Inverse.

Depending on the Rank of the Jacobian (Number of linearly independent columns) will depend on how the Pseudo-Inverse is approached, typically calculating the Moore-Penrose Pseudoinverse on a matrix with linearly independent columns is defined as
$$
A^+ = (A^TA)^{-1} A^T
$$


$(A^TA)$ should result in a square matrix, which then allows for the inverse to occur. This is then multiplied by the transpose again (of the original matrix).

Note Eigen `matrix.transpose()` doesn't actually reshape the matrix, is just swaps the pointers internally, so the size still stays the same, however It seems to result in square matrix result when printed, however it may not be the same internally, because the Inverse method asserts the sizes are the same, but because they are dynamic matrices and it seems even though the member functions (rows(), cols()) return the correct square size, internally they are still the size they were first resized() to Maybe just resize() ?. 



May need other methods like calculating SVD of $A$ and using for Pseudo-Inverse calc. 

_____

##### Applying / Integrating the Resulting Joint Angle Velocities

Ideally we can re-use the perturbation joint chain traversal code, but instead of perturbing each joint DOF / angle we are applying the resulting angle velocities to them, however we want to update all joint positions in the chain (not just the end effector) in this case from the accumulated transforms from root / start joint in the chain. 

Whether we do this as a multi-iteration process within a single frame, then re-calculating the Jacobian, Inversing, Applying the resulting angle velocities for $k$ iterations Or if we just integrate the resulting single iteration over multiple frames is optional. 





____

##### Abstraction back into IK_Solver class

IK Solver for each Joint chain, target effector ...

Also need to update bone transform method so it doesn't just fetch the BVH Data and overwrite the joints within the IK Chain/s deltas. Need to branch for joints still using BVH Transforms vs ones using IK transforms. Or do we write into the motion data array then we can unify the joint and bone update.

We need to update Joint Positions also (which are the resulting World Space transforms from the concatenated parent transform matrices) because the Joint's WS Positions, specifically the end effector of the chain, is needed to be updated for each new Jacobian iteration. We could avoid updating the rest of the joints position members, because bone rendering does not use Joint positions (its passed the transform matrix directly, to transform the start + end verts of the bones of course). 

Anim_State passed with friend access to IK_Solver so IK related Joint traversal functions (over joint chain can be encapsulated within, without needing to keep these in Anim_State). 

Need to make it easier to add visualizer Points/Prims to verify Joint / End Effector positions are been updated per tick / frame. 

Also will switch to double precision for Jacobian, Inverse and possibly the FK Joint traversal for both BVH and IK Angles.  

IK_Solver will take copy of joint chain, so we don't need to store chains within Anim_State vectors/arrays. Same for effector/s. Note it doesn't matter that these are copies as they contain pointers to the joints defining them (so essentially its just copies of pointers), which will be updated via Anim_State tick or within IK_Solve itself. 

I'm going to refactor the code quite a bit, getting rid of the switch statements in the joint hierarchy traversal functions and putting the IK code within the solver class, with only the chain creation and possibly the resulting angle integration happening within Anim_State.  

Also will remove the Pair<P1,P2> perturbation end effector positions per DOF, as the P1 is the same for all columns of the Jacobian, so is convoluted storing them as a pair. 

Actually might make more sense to keep the chains within Anim_State, so when we do the Transformation Accumulation traversal and resulting joint positions and bone updates, we can query each joint to check if its part of an IK Chain, if so, we fetch its joint angles from there, else we use the BVH Data. Note that the initial state of the IK Joints will be written from the loading of the BVH File, the IK angle updates should then add / integrate on top of this. 









____

##### BVH Exporter

As this was part of the assignment I implemented this quickly, currently it just reads the motion data from the BVH Loader arrays itself, later on it would read from my custom IK modified joint angles (and resulting motion values per channel DOF). I use the same recursive approach described above to loop over the joint hierarchy, writing out each Joint name, offset and channels to a stringstream, which is then passed as reference to the joint children in a recursive call for each for them to write into and so on, until we reach the end site where recursion ends (as joint has no more children) and we also write the end site into the stream the same way.

A key thing is to make sure the tab number at each joint in the hierarchy is correct, starting from Root which has 0 tab indent, so this is easy to do by just using the joint index to define the number of tabs to indent, and then for the joint offset/channel info we do another tab indent on top of this so its within the body of the braces defined as the current joint. We finally do a closing bracket at the end of the function (so when each recursive call is popped) to end the joint scope, and result in the tab indents decrementing back to the left of the file. 

We then read the motion data array (again this is just using the BVH Data for now as is), which we split into lines for all channels, per frame.  The .bvh file format has a newline at the end, this is needed.

Thus we have two member functions `void Anim_State::write_bvh(const char *out_path)` and `void Anim_State::write_bvh_traversal(Joint *joint, std::stringstream &stream)`.

Where the callee function for writing is :

```C++
void Anim_State::write_bvh(const char *out_path)
{
	// ============== Check output file is good ==============
	std::ofstream out(out_path);
	if (!out.is_open())
	{
		std::cerr << "ERROR::Writing BVH File::" << out_path << "::Path is not valid" << std::endl;
		std::terminate();
	}

	// ============== Init Write Stream ==============
	std::stringstream stream; 
	stream << "HIERARCHY\n";

	// ============== Get Joint Hierachy ==============
	write_bvh_traverse(bvh->joints[0], stream);

	// ============== Get Motion Data ==============
	stream << "MOTION\nFrames: " << bvh->num_frame << "\nFrame Time: " << bvh->interval << "\n";

	for (std::size_t c = 0; c < (bvh->num_channel * bvh->num_frame); ++c)
	{
		stream << std::fixed << std::setprecision(6) << bvh->motion[c]; 
		//if (c != 0 && c % bvh->num_channel == 0) stream << "\n"; else stream << " ";
		if ((c+1) % bvh->num_channel == 0) stream << "\n"; else stream << " ";
	}

	stream << "\n";

	// Debug
	//std::cout << "Stream Output :\n\n" << stream.str() << "\n";
	// ============== Write to file ==============
	out << stream.str();
	out.close(); 
}
```

And the Traversal function itself is similar to what was used in the Immediate Mode approach to rendering bones and joints, however it only looks up joint translation (offset) data :

```C++
void Anim_State::write_bvh_traverse(Joint *joint, std::stringstream &stream)
{
	// Get tab indent for joint based on index in tree. 
	std::string tab; for (std::size_t i = 0; i < joint->idx; ++i) tab += "\t";
	
	if (joint->is_root) // Root joint, translation from channels. 
	{
		glm::vec4 root_offs(0., 0., 0., 1.);

		for (const Channel *c : joint->channels)
		{
			switch (c->type)
			{
				// Translation
				case ChannelEnum::X_POSITION:
				{
					float x_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.x = x_p;
					break;
				}
				case ChannelEnum::Y_POSITION:
				{
					float y_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.y = y_p;
					break;
				}
				case ChannelEnum::Z_POSITION:
				{
					float z_p = bvh->motion[anim_frame * bvh->num_channel + c->index];
					root_offs.z = z_p;
					break;
				}
			}
		}

		// We know root has 6 DOFs so we can hardcode these. 
		stream << "ROOT " << joint->name << "\n{\n\t"
			<< "OFFSET " << std::fixed << std::setprecision(6)
			<< root_offs.x << " " << root_offs.y << " " << root_offs.z << "\n\t"
			<< "CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation\n";
	}
	else // Standard Joint
	{
		stream << tab.c_str() << "JOINT " << joint->name << "\n" << tab.c_str() << "{\n" << tab.c_str() << "\t" 
			<< "OFFSET " << std::fixed << std::setprecision(6)
			<< joint->offset.x << " " << joint->offset.y << " " << joint->offset.z << "\n" << tab.c_str() << "\t"
			<< "CHANNELS 3 Zrotation Yrotation Xrotation\n";

		// End Site
		if (joint->is_end)
		{
			stream << tab.c_str() << "\tEnd Site\n" << tab.c_str() << "\t{\n" << tab.c_str() << "\t\t"
				<< "OFFSET " << joint->end.x << " " << joint->end.y << " " << joint->end.z 
				<< "\n" << tab.c_str() << "\t}\n";
		}
	}
	for (Joint *child : joint->children)
	{
		write_bvh_traverse(child, stream);
	}
	stream << tab.c_str() << "}\n";
}
```

The formatting of the write into the string stream is hardcoded to match the BVH format as it should, this took a bit of trial and error and running diff against original input BVH files, but it seems to be working well now. 

Note that we pass the string stream by reference so its not copied for each recursive call, and the next call appends to previous write. We then read the final stringstream out to the ofstream bvh file output.

So this works pretty good, but its just reading the data from the BVH_Data class itself (within Anim_State class) ideally we will have modified joint angles (and thus motion data section) so the we will be reading modified channel motion data which may be modified from the input BVH Data directly or copied and internally stored to preserve the original BVH motion data per channel.

However there is a precision issue, because the output writes float data extracted from the Motion Data from the original BVH file, so some level of precision is lost, ideally we should use doubles. 

___

#### Bugs / Issues : 

##### Release Mode : Viewer Exec Loop doesn't run (fine in debug mode)

In release mode within viewer::exec() the application loop condition ` !esc_pressed()` seems to be getting optimized out , leading to the condition failing and the application / viewer closing immediately : 

```C++
void Viewer::exec()
{
	// ==== Init Operations ====
	render_prep();

	// ==== Application Loop ====
	while (!glfwWindowShouldClose(window) && !esc_pressed())
	{
		// Tick viewer application
		tick();
	} 
}	
```

 The GLFW call seems to work fine alone, but when the `&& !esc_pressed()` is present it leads the condition to be false when in release mode (works fine in debug mode), maybe its the resulting rvalue from `esc_pressed()` gets optimized out, not ideal. Ok Yeah I think this is the case because : 

```C++
void Viewer::exec()
{
	// ==== Init Operations ====
	render_prep();

	// ==== Application Loop ====
	bool esc = false; 
	while (!glfwWindowShouldClose(window) && !esc)
	{
		// Tick viewer application
		tick();

		// Query Esc key
		esc = esc_pressed();
	} 
}
```

Fixes the issue. I guess that's my bad for using an rvalue bool to check the condition against, but I'd of thought the compiler would of known not to optimize out an rvalue used within a loop condition. 

##### Depending if in Release or Debug mode, the camera start position (and thus direction) is different.

Not had time to look into this, but it seems the start direction is slightly different when switching between release and debug builds, i've not looked into this as its not a big deal, but there is possibly something going on again in terms of optimization where the initial basis of the camera is incorrect (or different) due to some optimization occurring. 

____

##### References

* BVH File Reference : 
  * research.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/BVH.html
* Inverse Kinematics Reference : 
  * Rick Parents Book
  * www2.cs.uregina.ca/~anima/408/Notes/Kinematics/InverseKinematics.htm