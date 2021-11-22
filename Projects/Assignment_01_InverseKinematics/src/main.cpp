// COMP5930M : Inverse Kinematics A1 : Niall Horn - main.cpp

// Project Headers
#include "BVHData.h"
#include "viewer.h"
#include "mesh.h"

// Ext Headers 

// Std Headers

int main(int argc, char **argv)
{
	/*
	// Test - Load BVH File :
	if (argc != 2)
	{
		std::cerr << "ERROR:: Incorrect Arguments Passed, pass single .bvh file path." << std::endl; 
	}
	BVH_Data bvh(argv[1]);
	*/

	// Obj Loading Test
	Mesh test_mesh("Test", "triplane.obj");
	test_mesh.load_obj(true);


	// Create Viewer Application
//	Viewer app(1024, 1024, "Viewer Application");

	// Exec
//	app.exec();

	return 0; 
}

