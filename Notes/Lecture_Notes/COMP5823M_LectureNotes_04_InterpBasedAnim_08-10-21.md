### COMP5823M - Animation and Simulation - Lecture Notes 04 : Interpolation Based Animation

##### Niall Horn - University of Leeds - 08/10/2021
___
#### Interpolation Based Animation

Picking and pulling  recap.  Known displacement of one vert, needs to be propogated to neighbouring verts by some influence / kernel function by their distance. Shows 1-(i/n+1)^-k+1 function or 1-(i/n+1)^k. Where k is just the kernel radius / falloff, used to scale the displacement of the picked point displacement onto other verts. 

Deforming an Embeeding Space ie FreeForm Deformation : Deformation of lattice / grid nodes. Interpolation of local mesh verts from rest grid space  to deformed grid space by computing bilinear interpolation (2D Example). Solve for uknowns of the interpolation coefficents (u,v), thus interpolate between deformed grid cell nodes, apply the resulting deformation of grid cell to vertices within these cells.

Shows 1D Example of using polyline as the embedding space to deform. Uses segments of polyline to divide the space bounding the mesh to deform (within 2D Space). Use 2 distances of each vertex to each polyline segment starting vertex, these are used as the coeffiecents for interpolation of the resulting deformation.

Global deformation by applying transformation matrix to vertices, eg twisting, bending etc. Ie Parametric deformation.

##### Extensions

Hiercahal FFD Deformation, so local parts of mesh can have finer grid resolution / higher resolution displacement. 

Mean Value Coordinates, Extend Barycentric coordinate concept to aribitary polygonal shape. All points within polygon represented by linear combination of boundary points, by angles around each vertex. 

Harmonic Cooridnates : Fixes problem of deformation not been affected by the boundaries of the cage mesh. 

Green Coordinates : Allows some deformation to lie outside the boundary of the cage by a controllable factor reducing volume loss from restriction to lie within cage boundary. 