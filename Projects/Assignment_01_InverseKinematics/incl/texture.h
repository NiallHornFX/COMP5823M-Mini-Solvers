#ifndef TEXTURE_H
#define TEXTURE_H

// Std Headers
#include <iostream>
#include <string>
#include <sstream>

using byte = unsigned char;


class Texture
{
public:
	enum filter_type
	{
		NEAREST = 0, LINEAR
	};
public:
	Texture(const char *tex_path, const char *name);
	~Texture() = default; 

	// State Setup
	void SetTex_Params(filter_type filter);

	// Tick Calls
	void Activate_Tex(std::size_t unit_id);
	void Bind_Tex();

	std::ostringstream debug();

public:
	int32_t width, height, nChannels; 
 	std::string name; 
	unsigned int ID;
	filter_type activeFilter; 
};


#endif