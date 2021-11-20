// Implements 
#include "shader.h" 

// Project Headers
#include <GLEW/glew.h>

Shader::Shader(const char *vertpath, const char *fragpath)
{
	// String objects to hold GLSL Code from File.
	std::string vertexCode;
	std::string fragmentCode;

	// ifstream object to load Shader Files. 
	std::ifstream vShaderFile;
	std::ifstream fShaderFile;

	// Enable Expection Checking Bits Before MFs Getting Files 
	// Allows the Catch Statement to look out for these expections and catch. 
	vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// Open Files
		vShaderFile.open(vertpath); // Load Vert Shader txt from Func Input Path.
		fShaderFile.open(fragpath); // Load Frag Shader txt from Func Input Path.
									// Use StringStream and String to handle ifStream buffer to string conversion. 
		std::stringstream vShaderStream, fShaderStream; // SStreams objs to convert to strings.
														// Read File Buffers (pointers) into StringStream objects - 
		vShaderStream << vShaderFile.rdbuf();
		fShaderStream << fShaderFile.rdbuf();
		// Close file objs
		vShaderFile.close();
		fShaderFile.close();
		// Convert From (string)Streams Into  Strings using sstream str() MF.
		vertexCode = vShaderStream.str();
		fragmentCode = fShaderStream.str();

		// Set Vertex/Frag Class Members to ^^ Strings.
		vert_shader_code = vertexCode; 
		frag_shader_code = fragmentCode;
	}
	catch (std::ifstream::failure err) // Catch ifstream failure error to object err
	{
		// Handle Excpetion, cerr Error to Consle.
		std::cerr << "ERROR::SHADER::FILE_NOT_READ \n";
	}

	// Pass back to Char String Literal (Char Pointer).
	const char *vShaderCode = vertexCode.c_str();
	const char *fShaderCode = fragmentCode.c_str();

	
	// Shader Creation & Compilation 
	unsigned int vertexS, fragmentS;

	// Vertex Shader Creation & Compilation -
	vertexS = glCreateShader(GL_VERTEX_SHADER);
	// Pass Charptr code into shadersource. 
	glShaderSource(vertexS, 1, &vShaderCode, NULL);
	// Compile - 
	glCompileShader(vertexS);
	// Call Shader Error Check Func - 
	check_compile(vertexS, "Vertex_Shader");

	// Fragment Shader Creation & Compilation -
	fragmentS = glCreateShader(GL_FRAGMENT_SHADER);
	// Pass Charptr code into shadersource. 
	glShaderSource(fragmentS, 1, &fShaderCode, NULL);
	// Compile - 
	glCompileShader(fragmentS);
	// Call Shader Error Check Func - 
	check_compile(fragmentS, "Fragment_Shader");

	// Shader Program Creation
	ID = glCreateProgram(); // Store in Shader Member uint ID. 
	glAttachShader(ID, vertexS);
	glAttachShader(ID, fragmentS);
	glLinkProgram(ID);
	// Call Shader Linking Error Check Func - 
	check_link();

	// Now Shaders are Compiled/Linked, Delete them.
	glDeleteShader(vertexS);
	glDeleteShader(fragmentS);
	
} // End of Shader::Constructor. 

Shader::~Shader()
{
	std::cout << "Shader Destructor Called" << "\n";
}

void Shader::use()
{
	glUseProgram(ID);
}

// Set Uniform Member Functions - 

void Shader::setBool(const std::string &name, bool value) const
{
	glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value); // Cast Value to int.
}

void Shader::setInt(const std::string &name, int value) const
{
	glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setFloat(const std::string &name, float value) const
{
	glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setVec(const std::string &name, const glm::vec3 value) const
{
	glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, value);
}

// Shader Compile Error Check Function -
/*
Take in uint ref param to define shader to check for glComplieShader
errors. Also Input Char Pointer string of shader type so I can print out what shader error'd.
Compiled Shader ID is not Member ID (that is linked Shader ID).
*/

void Shader::check_compile(unsigned int &shader, char *type)
{
	const int len = 512; // length of error log char array. 
	int sucess;
	char err_log[len];

	// Check For Compile Status via GetShaderiv func, set to sucess int.
	glGetShaderiv(shader, GL_COMPILE_STATUS, &sucess);

	// If Not Sucess (thus sucess undefined/non-intialzed) print error log out (via cout)
	if (!sucess)
	{
		glGetShaderInfoLog(shader, len, NULL, err_log);
		std::cout << "ERROR:SHADER:" << type << "COMPILE_FAILED" << std::endl;
		std::cout << err_log << std::endl;
	}
	else
	{
		std::cout << "DEBUG:SHADER " << shader << " COMPLIE SUCEEDED" << std::endl;
	}
}

// Shader Link Error Check Function -
/*
Take in uint ref param to define shader to check for ShaderProgram Link Errors
*/

void Shader::check_link()
{
	// ID IS Shader ID Member (Shader Program to check for Linking Errors).
	unsigned int shaderprog = ID;

	int sucess;
	const int len = 512;
	char err_log[len];

	// Check For ShaderProg Status via glGetProgramiv func, set to sucess int.
	glGetProgramiv(shaderprog, GL_LINK_STATUS, &sucess);

	// If Not Sucess (thus sucess undefined/non-intialzed) print error log out (via cout)
	if (!sucess)
	{
		glGetProgramInfoLog(shaderprog, len, NULL, err_log);
		std::cout << "ERROR:SHADER:PROGRAM:" << "LINKAGE_FAILED" << std::endl;
		std::cout << err_log << std::endl;
	}
	else
	{
		std::cout << "DEBUG:SHADER:PROGRAM " << shaderprog << " LINKAGE SUCEEDED" << std::endl;
	}
}

void Shader::debug_vert()
{
	if (vert_shader_code.size() >= 1)
	{
		std::cout << "DEBUG: VERTEX SHADER: " << vert_shader_code; 
	}
	else
	{
		std::cout << "ERROR: VERTEX SHADER FAILED TO STRING \n";
	}
}

void Shader::debug_frag()
{
	if (frag_shader_code.size() >= 1)
	{
		std::cout << "DEBUG: FRAGMENT SHADER: " << frag_shader_code;
	}
	else
	{
		std::cout << "ERROR: FRAGMENT SHADER FAILED TO STRING \n";
	}
}

unsigned int Shader::get_id()
{
	return ID;
}

// End of Shader Member Functions Implmentation.