// Implements 
#include "texture.h"

// Std Headers
#include <iostream>
#include <fstream>
#include <string>

// Ext Headers
// GLEW
#include "ext/GLEW/glew.h" 
// stb
#include "ext/stb/stb_image.h"


// Creation of texture and loading, now decoupled from Ctor, for completeness of OpenGL state. 

Texture::Texture(const char *Name, const char *tex_path)
	: name(Name), filePath(tex_path)
{
	valid_state = false; 
	ID = -1; 
	activeFilter = filter_type::NEAREST;
}

void Texture::load()
{
	// Check File path
	std::ifstream file(filePath);
	if (!file.is_open())
	{
		std::cerr << "ERROR::Texture::" << name << ":: file does not exist." << std::endl;
		return;
	}
	file.close();

	// Load via Stb image : 
	stbi_set_flip_vertically_on_load(true); // Flip Texture Y Axis on Load. 
	byte *tex_data = stbi_load(filePath.c_str(), &width, &height, &nChannels, 0);

	// Check for invalid state
	if (!tex_data)
	{
		std::cerr << "ERROR::Texture::" << name << ":: Failed to load texture." << std::endl;
		return;
	}
	if (nChannels < 3)
	{
		std::cerr << "ERROR::Texture::" << name << ":: Incorrect number of channels." << std::endl;
		return;
	}

	// Gen Texture and Bind
	glGenTextures(1, &ID);
	glBindTexture(GL_TEXTURE_2D, ID);

	switch (nChannels)
	{
	case 3:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, reinterpret_cast<void*>(tex_data));
		break;
	case 4:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, reinterpret_cast<void*>(tex_data));
	}
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	SetTex_Params(activeFilter);
	valid_state = true; 
}

void Texture::SetTex_Params(filter_type filter)
{
	glBindTexture(GL_TEXTURE_2D, ID);
	switch (filter)
	{
		case 0 : 
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			break;

		case 1 : 
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			break;

		default: 
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}

	// Enforce mirrored repat. 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT); // X (S)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT); // Y (T)
	glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::Activate_Tex(std::size_t unit_id)
{
	switch (unit_id)
	{
		case 0:	
			glActiveTexture(GL_TEXTURE0);
			break;
		case 1:	
			glActiveTexture(GL_TEXTURE1);
			break;
		case 2:	
			glActiveTexture(GL_TEXTURE2);
			break;
		case 3:
			glActiveTexture(GL_TEXTURE3);
			break;
		case 4:
			glActiveTexture(GL_TEXTURE4);
			break;
		case 5:
			glActiveTexture(GL_TEXTURE5);
			break;
		case 6:
			glActiveTexture(GL_TEXTURE6);
			break;
		case 7:
			glActiveTexture(GL_TEXTURE7);
			break;

		default: std::cerr << "ERROR::Texture::" << name << ":: texture unit out of bounds" << std::endl;
	}
}

void Texture::Bind_Tex()
{
	glBindTexture(GL_TEXTURE_2D, ID);
}


std::ostringstream Texture::debug()
{
	std::ostringstream out;
	out << "======== DEBUG::Texture::" << name << "::BEGIN ========\n"
		<< "ID = " << ID
		<< "\nWidth = " << width << " Height = " << height << " NumChannels = " << nChannels;
	out << "======== DEBUG::Texture::" << name << "::END ========\n";

	return out;
}