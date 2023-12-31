#ifndef __BOX_RIGID_BODY_HPP__
#define __BOX_RIGID_BODY_HPP__
#include <glm/glm.hpp>
#include "Math.hpp"
#include <vector>
#include "RigidBody.hpp"
#include <iostream>
class BoxRigidBody  : public RigidBody {
public:
	glm::vec2 size = { 0.0f,0.0f };


	BoxRigidBody(glm::vec2 position = { 0.0f,0.0f }, glm::vec2 size = { 20.0f,20.0f }, float rotation = 0.0f, float mass = 1.0f, float restitution = 0.0f,  glm::vec3 color = { 1.0f,1.0f,1.0f }, bool is_static = false)
	{
		static uint32_t current_id = 0;
		id = current_id++;
		this->type = Type::BOX;
		this->position = position;
		this->size = size;
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
		float max = std::max(size.x, size.y);
		aabb.size = glm::vec2(1.5f * max, 1.5f * max);
	}

	std::vector<glm::vec2> getVertices() {
		//calculate vertex positions
		glm::vec2 half_size = 0.5f * size;
		glm::vec2 tl = -half_size;
		glm::vec2 br = half_size;
		glm::vec2 tr{br.x, tl.y};
		glm::vec2 bl{tl.x, br.y};
		std::vector<glm::vec2> vertices = { bl, tl, tr, br };
		for (int i = 0; i < 4; i++) {
			vertices[i] = rotateVec2Radians(vertices[i],angle);
			vertices[i] = vertices[i] + position;
		}
		return vertices;
	}

	void step(float dt, int iterations) {
		
		dt /= (float)iterations;
		updateVelocity(dt);
		updateAngularVelocity(dt);

	}
	void updateAngularVelocity(float dt) {
		if(this->is_static){
				angular_velocity = 0.0f;
			return;
		}
		angle += angular_velocity * dt;
	}

	void updateVelocity(float dt) {
		if(this->is_static) {
			linear_velocity = {0.0f,0.0f};
			return;
		}
		glm::vec2 acceleration = force / mass;
		linear_velocity += acceleration * dt;
		position += linear_velocity*dt;
		force = { 0.0f,0.0f };

	}

	float getAngularVelocity() {
		if (is_static) return 0.0f;
		else
			return angular_velocity;
	}

	glm::vec2 getVelocity() {
		if (is_static) return { 0.0f,0.0f };
		return linear_velocity;
	}

	void setVelocity(glm::vec2 vel) {
		linear_velocity = vel;
	}

	void move(glm::vec2 velocity) {
		this->position += velocity;
	}

	void moveTo(glm::vec2 pos) {
		this->position = pos;
	}

	void accelerate(glm::vec2 acceleration) {
		if(is_static) return;
		force += acceleration * mass;
	}

	void calculateInertia() {
		if (!is_static) {
			float new_inertia = (mass * (size.x * size.x + size.y * size.y)) / 12.0f;
			this->inertia = new_inertia;
			this->inv_inertia = 1.0f / (new_inertia);
			inv_mass = 1.0f / mass;
		} else {
			inertia = std::numeric_limits<float>::max();
			inv_inertia = 0.0f;
			mass = std::numeric_limits<float>::max();
			inv_mass = 0.0f;
		}
	}

	void calculateAABB() {
		aabb.position = position - 0.5f * aabb.size;
	}

	bool testAABBOverlap(RigidBody * b) {
		
	}

	bool checkIfInside(glm::vec2 p) {
		int intersections = 0;
		auto vertices = getVertices();
		for (size_t i = 0; i < vertices.size(); i++) {
			glm::vec2 v1 = vertices[i];
			glm::vec2 v2 = vertices[(i + 1) % vertices.size()];


			if(((( v1.y <= p.y && p.y < v2.y) ||
				(v2.y <= p.y && p.y < v1.y)) && 
				(p.x < (v2.x - v1.x) * ((p.y - v1.y) / (v2.y - v1.y )) + v1.x))){
				intersections++;
			}
		}
		return intersections % 2 == 1;
	}
};
#endif