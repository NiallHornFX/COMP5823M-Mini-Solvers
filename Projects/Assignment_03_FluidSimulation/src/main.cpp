// COMP5823M - A1 : Niall Horn - main.cpp

// Project Headers
#include "viewer.h"


int main(int argc, char **argv)
{
	// Create Viewer Application Instance
	Viewer app(1024, 1024, "COMP5823M_A3 SPH Fluid Solver");

	// Exec
	app.exec();

	return 0; 
}

