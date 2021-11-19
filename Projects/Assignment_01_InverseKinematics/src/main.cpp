#include "BVHData.h"

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		std::cerr << "ERROR:: Incorrect Arguments Passed, pass single .bvh file path." << std::endl; 
	}

	// Test - Load BVH File :
	BVH_Data bvh(argv[1]);


	// Create Viewer Application

	// Exec

	return 0; 
}

