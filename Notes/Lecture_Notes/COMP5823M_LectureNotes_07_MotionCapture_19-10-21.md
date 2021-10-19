### COMP5823M - Animation and Simulation - Lecture Notes 07: Motion Capture

##### Niall Horn - University of Leeds - 19/10/2021
___
Body Capture, Facial Capture, Hand capture/tracking. Or Capturing deformable objects, ie cloth/garments or skin deformation directly. 

##### Sensing Methods

* **Optical** : Infrared camera or colour camera reciving reflected marker light.
  * Pros : accurate (high speed, typically 120FPS-240FPS), small error accumulation (is calculated per frame)
  * Cons : occlusions (markers hidden from camera/s), errors caused by moving sensors, slow post processing, errors caused by moving mocap suit across body. 
* **Electro-magnetic** : Sensors attached to body based sensor unit using local magnetic fields.
  * Pros : No occlusion, faster
  * Cons : Bigger errors, devoid of magnetic field disortations, cumbersome, no metal nearby eg pipes ! 
* **Intertial**
  * Pros : Lightweight, Fast, cheaper, doesn't need optical tracking (eg can be done onset more easily)
  * Cons : Accumulative errors from intertial data integration to retrive postion and orientation. 

*Doesn't metion on-set / incamera tracking based approach to motion capture, using a couple of witness cameras or even a single camera, i guess he'd classify this as optical*

##### Optical Tracking : Processing

Locate Markers :

* Markers put near / on joints.
* Static Background (Plain suit)
* Track each marker through time, cross frame coherency. 

Camera Callibration 

* Pinhole method - good  enough for CG
  * One world coordinate system for multiple cameras
  * Known camera focal lengths, image centre and aspect ratio
  * Need the postions and orientations of the cameras

Mapping marker from image space of each camera into 3D space via projection. Shows derivation for linear solve [..]. Also derive local coordiante system for each marker.

Not even gonna write notes for now, will fill this in later, also want to research correspondece methods myself.

Shows Camera callibarion tool, record points with known world coordinates (points on tool), known distances. Solve least square problem for callibartion. Solve for each camera.

3D position reconstructing using epipolar line like method, find closest point of rays that intersect from image space marker, looking for shortest segments when they cross / and thus are orthogonal. 

Two markers typically go on ethier side of joint to form a plane, not always correct as wrist-elbow-shoulder degenerates when colinear.

Hence the need to constrain foot sliding. Can use inverse kinematics to solve for this. 

#### Manipulating Mocap Data

Error Prone : Ideal 

* Recapture
* Post Processing (Mocap TDs etc)

Prcoessing the signals

* Signal Processing, smoothing 
  * Each joint trajceotry can be seen as time series data

Motion Warping

Breifly touches on retargeting to different skeleton types. Shows some siggraph papers, I doubt are still used, most retargeting is a combination of manual and solve based.

Deep Learning based re-trageting, current research. 



