#ifndef BVH_DATA_H
#define BVH_DATA_H

// Std Headers
#include <iostream>
#include <string>
#include <vector>

// Ext Headers
#include "ext/Eigen/Core" // Eigen

// Type Def
using real = double; 

// FDs
struct Joint;
struct Channel;

enum ChannelEnum
{
	X_ROTATION = 0, Y_ROTATION, Z_ROTATION,
	X_POSITION, Y_POSITION, Z_POSITION
};

// Info : Structs for Joint and Channel Objects 

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

// Info : BVHData Class, responsible for loading, parsing BVH Animation file to Joints and Channel Data. 

class BVH_Data
{
public:
	BVH_Data() = delete;
	BVH_Data(std::string fileName);
	~BVH_Data();

	// Load and Parse BVH File into Joints and Channels
	void Load(); 
	// Reset Data
	void Clear(); 
	// Dump State
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


#endif