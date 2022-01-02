// COMP5823M - A3 : Niall Horn - fluid_collider.cpp

// Implements
#include "fluid_collider.h"
#include "fluid_object.h"

// ================================== Cloth_Collider Class Implementation ===============================
Fluid_Collider::Fluid_Collider(const char *Name)
	: name(Name), eps(1e-02f), friction(0.25f)
{
	render = false; 
}

// ================================== Cloth_Collider_Plane Implementation ===============================

Fluid_Collider_Plane::Fluid_Collider_Plane(const char *name, const glm::vec3 &Q, const glm::vec3 &Normal, const glm::vec2 &WH)
	: Fluid_Collider(name), q(Q), N(Normal), length(WH.x), height(WH.y)
{
	if (length != 0.f) type = Type::HORIZONTAL; else type = Type::VERTICAL;

	render_setup();
}

void Fluid_Collider_Plane::render_setup()
{
	prim = new Primitive((name + "Prim").c_str());

	if (type == Type::HORIZONTAL)
	{
		// Assume q is centroid. 
		float x_min = q.x - length; 
		float x_max = q.x + length; 
		float data[11 * 2]
		{
			x_min, q.y, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f,
			x_max, q.y, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f
		};
		prim->set_data_mesh(data, 2);
		prim->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
		prim->mode = Render_Mode::RENDER_LINES;
	}
	else
	{
		float y_min = q.y - length;
		float y_max = q.y + length;
		float data[11 * 2]
		{
			q.x, y_min, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f,
			q.x, y_max, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f
		};
		prim->set_data_mesh(data, 2);
		prim->set_shader("../../shaders/basic.vert", "../../shaders/colour.frag");
		prim->mode = Render_Mode::RENDER_LINES;
	}
}

// Info : Project positions of particles directly to satisfy the Plane collision inequality condition : (P-Q) * N >= 0
// Within specified bounds (if horizontal or vertical)
void Fluid_Collider_Plane::eval_collision(std::vector<Particle> &particles)
{
	for (Particle &curPt : particles)
	{
		float dist = glm::dot(curPt.P, N);

		switch (type)
		{
			case Type::HORIZONTAL:
			{

				break;
			}

			case Type::VERTICAL:
			{

				break;
			}
		}

		if (dist <= 1e-03f) curPt.P += -dist * N;
	}
}

