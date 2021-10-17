### COMP5823M - Animation and Simulation - Lecture Notes 06: Inverse Kinematics

##### Niall Horn - University of Leeds - 15/10/2021
___
#### Inverse Kinematics

Desired position and possibly orientation of end effector/s are given. We need to compute the joint angles of links/bones going back to the root node, incrementally to reach this end effector. There might be 0, 1 or many solutions.

For simple systems of a couple or few joints, its possible to derive an analytical solution. 

##### Example using 2 linkages in 2D space : 

For 2 links, $L_1, L_2$ with joint angles $\theta_1, \theta_2$ where we have a target angle $\theta_T$ of the end effector. We can analytically calculate the angles for $\theta_1,\theta_2$​ using cosine rule. (see Parent, R Textbook, page 172-174).



Jacobi Pseudoinverse using transpose (assuming columns are independent from each other) Expand and use LU Decomposition.

 From Ax=b. A = LU Decomp (A). 

LUx = B

Lc = b so Ux = c

solve Lc = b (Gaussian Elim)

solve Ux = c (Gaussian Elim)

Could use just pure transpose by some alpha coeff , but not very accurate as doing the proper pseudo inverse. 

___

#### Different Ways of Solving IK

n