## COMP5823M - Mini Solver Projects - 2021-2022

____

### Summary

This repository contains 3 projects which I completed as coursework for COMP5823M taught by Dr He Wang. These are based around the topic animation and simulation within computer graphics. This module was taken during my MSc at University of Leeds in 2021-2022. 

**Note:** Due to my prior experience writing solvers, I found the scope of these projects quite simple. However the code for these projects may be helpful to beginners, and also useful for me to look back on my coding style from this era! 

_____

### Projects

Each project has a `README.md` which provides more details about their original papers, algorithms used and implementation into C++ code. 

### Kinematic Viewer

<img src="D:\DEV\_University\MSc_HPG_21-22\Modules\COMP5823M_Animation-Simulation\_public\A1_Screenshot.png" alt="A1_Screenshot" style="zoom:30%;" />

This application implements a small utility to view BVH motion capture data, which is assembled utilising forward kinematics. 

### 3D Mass-Spring Cloth Solver

<img src="D:\DEV\_University\MSc_HPG_21-22\Modules\COMP5823M_Animation-Simulation\_public\A2_Screenshot.png" alt="A2_Screenshot" style="zoom:30%;" />

A pretty simple mass-spring based cloth solver, implementing parametric collisions, basic friction handling, hybrid time-stepping [1] and a basic modern OpenGL based renderer. 

### 2D Smoothed Particle Hydrodynamics (SPH) Fluid Solver

<img src="D:\DEV\_University\MSc_HPG_21-22\Modules\COMP5823M_Animation-Simulation\_public\A3_Screenshot.png" alt="A3_Screenshot" style="zoom:30%;" />

A 2D SPH fluid solver, utilising a hash + explicit grid [3] for particle neighbour searching, Leapfrog Symplectic Integration, Wyvill/Gaussian based 'Metaball' [4] surfacing and a selection of different kernel functions for approximating particle derivatives. Pressure is computed utilising a simple equation of state. Modern OpenGL is used for rendering. 

____

### External Dependencies

* GLM - Vector/Matrix DS's + Linear Algebra. 
* GLEW - OpenGL Function Loading (could be done manually)
* GLFW - OpenGL Context, Windowing and Input managment. 
* DearImGUI - Intermediate Mode GUI 

____

### References 

1. [Fix Your Timestep, Fiedler.G, 2004] : https://gafferongames.com/post/fix_your_timestep/

2. [Particle-Based Fluid Simulation for Interactive Applications, Muller et al, SCA 2003] : https://matthias-research.github.io/pages/publications/sca03.pdf

3. [Optimized Spatial Hashing for Collision Detection of Deformable Objects, Teschner et al, 2003] : https://matthias-research.github.io/pages/publications/tetraederCollision.pdf

4. [Data Structure for Soft Objects, Wyvill et al, 1986] : http://webhome.cs.uvic.ca/~blob/publications/datastructures.pdf
