// Implements 
#include "shader.h" 

// Ext Headers
#include "ext/GLEW/glew.h" // GLEW

#define BUFFER_SIZE 512

Shader::Shader(const char *sname, const char *vertpath, const char *fragpath)
{
	name = sname; 

	std::ifstream vShaderFile, fShaderFile;
	vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		vShaderFile.open(vertpath); 
		fShaderFile.open(fragpath); 

		std::stringstream vShaderStream, fShaderStream; 

		vShaderStream << vShaderFile.rdbuf();
		fShaderStream << fShaderFile.rdbuf();

		vShaderFile.close();
		fShaderFile.close();

		vert_shader_code = vShaderStream.str();
		frag_shader_code = fShaderStream.str();
	}
	catch (std::ifstream::failure err) // Catch ifstream failure error to object err
	{
		std::cerr << "ERROR::Shader::" << name << "::Failed to load shader code\n";
		std::terminate();
	}

	const char *vShaderCode = vert_shader_code.c_str();
	const char *fShaderCode = frag_shader_code.c_str();

	// Intermediate Shaders (Vertex, Fragment) handles. 
	unsigned int vertexS, fragmentS;

	// Build Vertex Shader
	vertexS = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexS, 1, &vShaderCode, NULL);
	glCompileShader(vertexS);
	check_compile(vertexS, "Vertex_Shader");

	// Build Fragment Shader
	fragmentS = glCreateShader(GL_FRAGMENT_SHADER);
	// Pass Charptr code into shadersource. 
	glShaderSource(fragmentS, 1, &fShaderCode, NULL);
	// Compile - 
	glCompileShader(fragmentS);
	// Call Shader Error Check Func - 
	check_compile(fragmentS, "Fragment_Shader");

	// Build Shader Program
	ID = glCreateProgram(); 
	glAttachShader(ID, vertexS);
	glAttachShader(ID, fragmentS);
	glLinkProgram(ID);
	check_link();

	// Delete Intermediate Shaders
	glDeleteShader(vertexS);
	glDeleteShader(fragmentS);
	
} // End of Shader::Constructor. 

Shader::~Shader()
{
	glDeleteProgram(ID);
}

void Shader::use()
{
	glUseProgram(ID);
}

// ========================== Set Uniform Member Functions ========================== 
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
	glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
}

// ========================== Check Compile and Link ========================== 

void Shader::check_compile(unsigned int &shader, std::string type)
{
	int sucess;
	char err_log[BUFFER_SIZE];

	glGetShaderiv(shader, GL_COMPILE_STATUS, &sucess);

	if (!sucess)
	{
		glGetShaderInfoLog(shader, BUFFER_SIZE, NULL, err_log);
		std::cout << "ERROR::Shader::" << name << " " << type << "::Compile Failed : " 
			<< err_log << std::endl;
	}
}

void Shader::check_link()
{
	int sucess;
	char err_log[BUFFER_SIZE];

	glGetProgramiv(ID, GL_LINK_STATUS, &sucess);

	if (!sucess)
	{
		glGetProgramInfoLog(ID, BUFFER_SIZE, NULL, err_log);
		std::cerr << "ERROR::Shader::" << name << "::Linkage Failed : " 
			<< err_log << std::endl;
	}
}

// ========================== Debug / Output Shader Code ========================== 
void Shader::debug_vert()
{
	if (vert_shader_code.size() >= 1)
	{
		std::cout << "DEBUG::Shader_" << name 
			<< "\n==== Vertex Shader Code BEGIN ====\n" 
			<< vert_shader_code
			<< "\n==== Vertex Shader Code END ====\n";
	}
	else
	{
		std::cerr << "ERROR:Shader_" << name
			<< " Invalid Vertex Shader Code" << std::endl;
	}
}

void Shader::debug_frag()
{
	if (frag_shader_code.size() >= 1)
	{
		std::cout << "DEBUG::Shader_" << name
			<< "\n==== Fragment Shader Code BEGIN ====\n"
			<< frag_shader_code
			<< "\n==== Fragment Shader Code END ====\n";
	}
	else
	{
		std::cerr << "ERROR:Shader_" << name
			<< " Invalid Fragment Shader Code" << std::endl;
	}
}