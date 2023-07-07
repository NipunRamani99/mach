#ifndef __CIRCLE_RIGID_BODY_HPP__
#define __CIRCLE_RIGID_BODY_HPP__
#include <glm/glm.hpp>
#include "RigidBody.hpp"
class CircleRigidBody : public RigidBody {
public:
	float radius = 0.0f;
	CircleRigidBody(glm::vec2 position = { 0.0f,0.0f }, float radius = 2.0f, float rotation = 0.0f, float mass = 1.0f, float restitution = 0.0f, glm::vec3 color = { 1.0f,1.0f,1.0f }, bool is_static = false)
	{
		this->type = Type::CIRCLE;
		this->position = position;
		this->radius = radius;
		this->angle = rotation;
		this->color = color;
		this->mass = mass;
		this->is_static = is_static;
		if (!is_static) {
			this->inv_mass = 1.0f / mass;
		}
		this->static_friction = 0.6;
		this->dynamic_friction = 0.4;
		calculateInertia();
	}

	void calculateInertia() {

		this->inertia = 0.5f * mass * radius * radius;
		this->inv_inertia = 1.0f / inertia;
	}
};
#endif // !__CIRCLE_RIGID_BODY_HPP

