#ifndef __BOX_RIGID_BODY_HPP__
#define __BOX_RIGID_BODY_HPP__
#include <glm/glm.hpp>
#include "Math.hpp"
#include <vector>
struct BoxRigidBody {
	glm::vec2 position_current = { 0.0f, 0.0f };
	glm::vec2 position_old = { 0.0f, 0.0f };
	glm::vec2 acceleration = { 0.0f, 0.0f };
	glm::vec2 size = { 0.0f,0.0f };
	float rotation_old = 0.0f;
	float rotation_current = 0.0f;
	float angular_acceleration = 0.0f;
	glm::vec3 color = { 1.0f,1.0f,1.0f };
	bool is_static = false;
	BoxRigidBody(glm::vec2 position = { 0.0f,0.0f }, glm::vec2 size = { 20.0f,20.0f }, float rotation = 0.0f, glm::vec3 color = { 1.0f,1.0f,1.0f }) {
		this->position_current = position;
		this->position_old = position;
		this->size = size;
		this->rotation_current = rotation;
		this->rotation_old = rotation;
		this->color = color;
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
			vertices[i] = rotateVec2Radians(vertices[i],rotation_current);
			vertices[i] = vertices[i] + position_current;
		}
		return vertices;
	}


	void updateAngularVelocity(float dt) {
		const float last_update_rotation = rotation_current - rotation_old;
		const float new_rotation = rotation_current + last_update_rotation + (angular_acceleration) * (dt * dt);
		rotation_old = rotation_current;
		rotation_current = new_rotation;
		angular_acceleration = 0.0f;
	}

	void updateVelocity(float dt) {
		const glm::vec2 last_update_move = position_current - position_old;
		const glm::vec2 new_position = position_current + last_update_move + (acceleration - last_update_move * 40.0f) * (dt * dt);
		position_old = position_current;
		position_current = new_position;
		acceleration = { 0.0f,0.0f };
	}

	void setVelocity(glm::vec2 vel, float dt) {
		position_old = position_current - vel * dt;
	}

	void accelerate(glm::vec2 acceleration) {
		this->acceleration += acceleration;
	}

	void setAngularVelocity(float angular_velocity, float dt) {
		rotation_old = rotation_current - angular_velocity * dt;
	}

	void angularAccelerate(float angular_acceleration) {
		this->angular_acceleration += angular_acceleration;
	}
};
#endif