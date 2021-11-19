// BVHData Class, responsible for loading, parsing, writing and maintaing the resulting skeleton state.

// Std Includes
#include <iostream>
#include <string>
#include <vector>

// Extern Includes
// Eigen
#include "ext/Eigen/Core"

// FDs
struct joint;
struct channel;


enum channelType
{
	x_pos = 0, y_pos, z_pos,
	x_rot, y_rot, z_rot
};

struct channel
{
	joint *sJoint;         // self joint of channel
	channelType type;      // DOF type
	std::size_t idx;       // index.
};

struct joint
{
	std::string name; 
	std::size_t idx; 

	bool is_root = false;
	bool is_end = false;

	Eigen::Vector3f offset; 


	joint *parent;
	std::vector<joint*> children; 

	std::vector<channel*> channels; 

};

class BVH_Data
{
public:
	BVH_Data(std::string filename);
	~BVH_Data();


private:
	std::vector<joint*> joints; 
	std::vector<channel*> channels; 

};