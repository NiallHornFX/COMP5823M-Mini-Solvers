// BVHData Class, responsible for loading, parsing, writing and maintaing the resulting skeleton state.

// Std Includes
#include <iostream>
#include <string>
#include <vector>

// Extern Includes
// Eigen
#include "ext/Eigen/Core"

// FDs
struct Joint;
struct Channel;

// Type Def
using real = double; 


enum ChannelEnum
{
	X_ROTATION = 0, Y_ROTATION, Z_ROTATION,
	X_POSITION, Y_POSITION, Z_POSITION
};

struct Channel
{
	Joint *joint;            // self joint of channel
	ChannelEnum type;        // DOF type
	std::size_t index;       // index.
};

struct Joint
{
	std::string name; 
	std::size_t idx; 

	bool is_root, is_end;

	Eigen::Vector3d offset; 

	Joint *parent;
	std::vector<Joint*> children; 

	std::vector<Channel*> channels; 

};

class BVH_Data
{
public:
	BVH_Data() = delete;
	BVH_Data(std::string fileName);
	~BVH_Data();


	void Load();
	void Clear(); 
	void Debug(bool to_file = false);

private:
	std::string filename;
	std::vector<Joint*>   joints; 
	std::vector<Channel*> channels; 

	std::size_t num_channel;
	std::size_t num_frame;
	real interval;
	real *motion; 
	

};