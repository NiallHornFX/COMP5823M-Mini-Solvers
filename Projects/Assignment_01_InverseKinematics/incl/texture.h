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
	Texture(const char *name, const char *tex_path);
	Texture() {};
	~Texture() = default; 

	// State Setup
	void load();
	void SetTex_Params(filter_type filter);

	// Tick Calls
	void Activate_Tex(std::size_t unit_id);
	void Bind_Tex();

	std::ostringstream debug();

public:
	bool valid_state; 
	int32_t width, height, nChannels; 
	std::string name, filePath;
	unsigned int ID;
	filter_type activeFilter; 
};


#endif