#include "GObject.h"
#include <iostream>
#include <fstream>
#include <QOpenGlFunctions_3_0>
#include "Log.h"
#include "MyMath.h"
using namespace Crane_Graphics;
using namespace std;

CGObject::CGObject()
	:m_vbo(QOpenGLBuffer::VertexBuffer)
{

}


CGObject::~CGObject()
{
	m_vbo.destroy();
	m_vao.destroy();
}

int CGObject::init(std::string file, float scaleCo)
{

	if (loadObj(file) != 0)
		return -1;
	scale(scaleCo);
	if (computeFaceNormals() != 0)
		return -2;

	if (m_vertNormals.empty())
		if (computeVertNormals() != 0)
			return -3;

	if (m_showColor && m_colors.empty())
		computeVertexColors();

	if (initBuffer() != 0)
		return -4;

	

	return 0;
}

int CGObject::normalise()
{
	glm::vec3 mid(0, 0, 0);

	for (glm::vec3 vert : m_vertices)
		mid += vert;
	mid /= m_vertices.size();

	vector<float> distances;

	for (glm::vec3 vert : m_vertices)
		distances.push_back(glm::length(vert - mid));
	
	float it = *std::max_element(distances.begin(), distances.end());
	
	for (int i = 0; i < m_vertices.size(); ++i)
	{
		glm::vec3 v = m_vertices[i] - mid;

		v /= glm::length(v) / it;

		m_vertices[i] = v;

	}

	return 0;
}
int CGObject::computeVertexColors()
{
	if (m_vertices.empty())
		return 0;

	m_colors.resize(m_vertices.size());

	for (int i = 0; i < m_colors.size(); ++i)
	{
		//m_colors[i].x = ((float)rand() / (RAND_MAX));
		//m_colors[i].y = ((float)rand() / (RAND_MAX));
		//m_colors[i].z = ((float)rand() / (RAND_MAX));

		//m_colors[i] /= m_colors[i].length();
		m_colors[i].r = 0;
		m_colors[i].g = 1;
		m_colors[i].b = 0;
	}

	return 0;
}
int CGObject::initBuffer()
{
	int m_unitSize = 6;
	if (m_showColor)
		m_unitSize += 3;
	if (m_showTexture)
		m_unitSize += 2;
	
	int bufferSize = m_triangles.size() * 3 * m_unitSize;
	m_buffer.resize(bufferSize);

	for (int i = 0; i < m_triangles.size(); ++i)
	{
		int v1 = m_triangles[i].x;
		int v2 = m_triangles[i].y;
		int v3 = m_triangles[i].z;

		int offset = 0;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v1].x;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v1].y;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v1].z;

		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v1].x;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v1].y;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v1].z;

		if (m_showColor)
		{
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v1].r;
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v1].g;
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v1].b;
		}

		if (m_showTexture)
		{
			m_buffer[i*m_unitSize * 3 + offset++] = m_vertTexCoords[v1].x;
			m_buffer[i*m_unitSize * 3 + offset++] = m_vertTexCoords[v1].y;
		}

		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v2].x;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v2].y;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v2].z;

		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v2].x;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v2].y;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v2].z;

		if (m_showColor)
		{
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v2].r;
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v2].g;
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v2].b;
		}

		if (m_showTexture)
		{
			m_buffer[i*m_unitSize * 3 + offset++] = m_vertTexCoords[v2].x;
			m_buffer[i*m_unitSize * 3 + offset++] = m_vertTexCoords[v2].y;
		}

		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v3].x;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v3].y;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertices[v3].z;

		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v3].x;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v3].y;
		m_buffer[i*m_unitSize * 3 + offset++] = m_vertNormals[v3].z;

		if (m_showColor)
		{
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v3].r;
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v3].g;
			m_buffer[i*m_unitSize * 3 + offset++] = m_colors[v3].b;
		}

		if (m_showTexture)
		{
			m_buffer[i*m_unitSize * 3 + offset++] = m_vertTexCoords[v3].x;
			m_buffer[i*m_unitSize * 3 + offset++] = m_vertTexCoords[v3].y;
		}
	}


	m_vao.create();
	assert(m_vao.isCreated());
	QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
	
	m_vbo.create();
	m_vbo.bind();
	m_vbo.allocate(&m_buffer[0], m_buffer.size() * sizeof(float));


	
	QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
	f->glEnableVertexAttribArray(0);
	f->glEnableVertexAttribArray(1);
	if (m_showColor)
		f->glEnableVertexAttribArray(2);
	if (m_showTexture)
		f->glEnableVertexAttribArray(3);

	f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, m_unitSize * sizeof(float), reinterpret_cast<void*>(0));
	f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, m_unitSize * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));

	if (m_showColor)
		f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, m_unitSize * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
	if (m_showTexture)
		f->glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, m_unitSize * sizeof(float), reinterpret_cast<void*>(9 * sizeof(float)));
	m_vbo.release();

	return 0;
}
int CGObject::render()
{

	QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
	
	glDrawArrays(GL_TRIANGLES, 0, m_triangles.size()*3);
	GLenum err = glGetError();
	assert(err == GL_NO_ERROR);

	if (m_showNormals)
	{
		glBegin(GL_LINES);
		for (int i = 0; i < m_vertices.size(); ++i)
		{
			glVertex3fv(&m_vertices[i].x);
			glm::vec3 n = m_vertNormals[i] + m_vertices[i];
			glVertex3fv(&n.x);
		}
		glEnd();
	}

	return 0;
}

int CGObject::scale(float scale)
{
	if (scale <= 0 || m_vertices.empty())
		return -1;

	for (int i = 0; i < m_vertices.size(); ++i)
	{
		m_vertices[i] *= scale;
	}
	
	return 0;
}

int CGObject::loadObj(std::string file)
{
	ifstream inputFile(file.c_str(), ios::in);
	if (!inputFile.is_open()) {
		return -1;
	}
	m_file = file;
	char line[256];

	int textureCounter = 0;
	int normalCounter = 0;

	while (inputFile.getline(line, 256))
	{
		// the first letter of every line tells us what to do with the rest 
		// of the line
		glm::vec3 vec(0, 0, 0);
		switch (line[0])
		{
			char dummy[10];
			
		case 'o':
			char name[256];
			sscanf(line, "%s %s", dummy, name);
			m_name = string(name);
			break;
		case 'v': // new vertex of some type
				
			switch (line[1]) // switch on second char to determine which kind
			{
			case ' ': // plain vertex
				// fill the values with the data in the line
				sscanf(line, "%s %f %f %f", dummy, &(vec.x), &(vec.y), &(vec.z));
				m_vertices.push_back(vec);
				break;
			case 't': // texture vertex
				sscanf(line, "%s %f %f %f", dummy, &(vec.x), &(vec.y), &(vec.z));
				m_vertTexCoords.push_back(vec);
				break;
			case 'n': // vertex normal
				sscanf(line, "%s %f %f %f", dummy, &(vec.x), &(vec.y), &(vec.z));
				m_vertNormals.push_back(vec);
				break;
			}
			break;

		case 'f': // a new face

			Triangle tri;

			int lPos, rPos;

			string strLine = string(line);

			//strLine = strLine.substr(0, strLine.length() - 1); // chomp the newline

			//												   // for each word that follows until the end of the line, those are
			//												   // our new array indices for the face
			vector<int> vertInd;
			lPos = strLine.find(" ");
			while (lPos != string::npos)
			{
				string curWord;
				lPos++; // get the lPos past the space

				rPos = strLine.find(" ", lPos);
				if (rPos != string::npos)
					curWord = strLine.substr(lPos, rPos - (lPos));
				else
					curWord = strLine.substr(lPos, strLine.length());

				// ignore any extra spaces
				if (curWord != " " && !curWord.empty())
				{
					int vertexIndex;
					int posSlash1, posSlash2;
					posSlash1 = curWord.find("/");
					posSlash2 = curWord.find("/", posSlash1 + 1);
					sscanf(curWord.substr(0, posSlash1).c_str(), "%d", &vertexIndex);
					vertInd.push_back(vertexIndex);
				}
				lPos = strLine.find(" ", lPos);
			}

			tri.x = vertInd[0] - 1;
			tri.y = vertInd[1] - 1;
			tri.z = vertInd[2] - 1;

			m_triangles.push_back(tri);

			break;
		}
	}
	inputFile.close();
	return 0;
}
int CGObject::writeFile(std::string objFile)
{
	ofstream output(objFile);
	if (!output.is_open())
		return -1;

	output << "o " << m_name << std::endl;

	for (glm::vec3 pt : m_vertices)
	{
		output << "v " << pt.x << " " << pt.y << " " << pt.z << std::endl;
	}

	for (glm::vec3 n : m_vertNormals)
	{
		output << "vn " << n.x << " " << n.y << " " << n.z << std::endl;
	}

	for (glm::vec2 t: m_vertTexCoords)
	{
		output << "vt " << t.x << " " << t.y << std::endl;
	}

	for (Crane_Graphics::Triangle tri : m_triangles)
	{
		output << "f " << tri.x + 1 << " " << tri.y + 1 << " " << tri.z + 1 << std::endl;
	}

	output.close();
	return 0;
}
int CGObject::computeFaceNormals(bool recompute)
{
	if (!recompute && !m_faceNormals.empty())
		return 0;

	m_faceNormals.clear();

	vector<int> degeneratedTris;

	for (int i = 0; i < m_triangles.size(); ++i)
	{
		Triangle tri = m_triangles[i];
		glm::vec3 v1 = m_vertices[tri.y] - m_vertices[tri.x];
		glm::vec3 v2 = m_vertices[tri.z] - m_vertices[tri.x];

		v1 = glm::normalize(v1);
		v2 = glm::normalize(v2);

		glm::vec3 n = glm::normalize(glm::cross(v1, v2));
		if (n.x != n.x || n.y != n.y || n.z != n.z)
		{
			ostringstream o;
			o << "Triangle [(" << m_vertices[tri.x].x << " " << m_vertices[tri.x].y << " " << m_vertices[tri.x].z << ")("
				<< m_vertices[tri.y].x << " " << m_vertices[tri.y].y << " " << m_vertices[tri.y].z << ")("
				<< m_vertices[tri.y].x << " " << m_vertices[tri.y].y << " " << m_vertices[tri.y].z << ")";
			LOG_ERROR_MSG(o.str());

			LOG_ERROR_MSG("Invalid normal computed, remove the triangle");

			degeneratedTris.push_back(i);

			continue;
		}

		m_faceNormals.push_back(n);
	}


	for (int i = 0; i < degeneratedTris.size(); ++i)
	{
		LOG_DEBUG("Removing degenerated triangles ", degeneratedTris[i], "");
		m_triangles.erase(m_triangles.begin() + degeneratedTris[i] - i);
	}

	return 0;
}
int CGObject::computeVertNormals()
{
	computeFaceNormals();
	m_vertNormals.clear();
	for (int i = 0; i < m_vertices.size(); ++i)
	{
		vector<int> triangles;
		vector<float> areas;
		glm::vec3 normal(0, 0, 0);

		for (int j = 0; j < m_faceNormals.size(); ++j)
		{
			if (m_triangles[j].x == i || m_triangles[j].y == i || m_triangles[j].z == i)
			{
				triangles.push_back(j);
				glm::vec3 a = m_vertices[m_triangles[j].y] - m_vertices[m_triangles[j].x];
				glm::vec3 b = m_vertices[m_triangles[j].z] - m_vertices[m_triangles[j].x];

				areas.push_back(glm::length(glm::cross(a, b)) * 0.5);
			}
		}

		//assert(areas.size() > 0);

		float total = std::accumulate(areas.begin(), areas.end(), static_cast<CNReal>(0.0));

		for (int j = 0; j < triangles.size(); ++j)
		{
			if (!CMyMath::isFinite(m_faceNormals[triangles[j]].x)
				|| !CMyMath::isFinite(m_faceNormals[triangles[j]].y)
				|| !CMyMath::isFinite(m_faceNormals[triangles[j]].z))
			{
				LOG_ERROR_MSG("Found invalid face normals");
				return -1;
			}

			if (!CMyMath::isFinite(areas[j]) || !CMyMath::isFinite(total))
			{
				LOG_ERROR_MSG("Found invalid face area");
				return -1;
			}

			if(areas[j] != 0 && total != 0)
				normal += m_faceNormals[triangles[j]] * areas[j] / total;
		}

		float l = glm::length(normal);
		if(l != 0)
			normal = glm::normalize(normal);

		if (!CMyMath::isFinite(normal.x) || !CMyMath::isFinite(normal.y) || !CMyMath::isFinite(normal.z))
		{
			LOG_ERROR_MSG("invalid vertex normal computed");
			return -1;
		}

		m_vertNormals.push_back(normal);
	}

	return 0;
}
