// Implements
#include "bvhdata.h"

// Std Headers
#include <fstream>
#include <stack>
#include <sstream>

BVH_Data::BVH_Data(std::string filename)
{
	std::ifstream file(filename);
	// Check file exists
	if (!file.is_open())
	{
		std::cerr << "ERROR::BVH_Data could not load file" << std::endl; 
		return; 
	}

	// Parse Line by line. 
	std::string line;
	while (std::getline(file, line))
	{
		std::stack<joint*> joint_stack; 

		if (line == "ROOT" || line == "JOINT") 
		{
			std::stringstream ss(line);
			joint Joint;
			std::string name;
			ss >> name; 
			std::cout << line << std::endl;

		}
	}


	file.close();
}