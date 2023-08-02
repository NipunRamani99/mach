#ifndef __MOUSE_JOINT_HPP__
#define __MOUSE_JOINT_HPP__
#include "Joint.hpp"
#include "RigidBody.hpp"
class MouseJoint {
private:
	glm::vec2 target;
	RigidBody* body;
	float max_force = 0.0f;
	float stiffness = 0.0f;
	float damping = 0.0f;
	float beta = 0.0f;
	float gamma = 0.0f;
	glm::vec2 impulse;
	glm::vec2 r;
	glm::vec2 localCenter;
	float invMass;
	float invI;
	glm::mat2 mass;
	glm::vec2 C;

public:
	MouseJoint(RigidBody* body, glm::vec2 target, glm::vec2 localCenter)
		:
		body(body),
		target(target)
	{
		this->localCenter = localCenter - body->position;
		glm::mat2 rot = rotationMatrix(body->angle);
		glm::mat2 rotT = glm::transpose(rot);
		this->localCenter = this->localCenter * rotT;

		max_force = 100.0f * body->mass;
		stiffness = 500;
		damping = 40;
		beta = 0;
		gamma = 0;
		impulse = { 0.0f,0.0f };
		r = glm::vec2{ 0.0f,0.0f };
		invMass = 0.0f;
		invI = 0.0f;

	}

	void setTarget(glm::vec2 target) {
		this->target = target;
	}

	void initialize(const float dt) {
		glm::vec2 pos1 = body->position;
		glm::vec2 v1 = body->getVelocity();
		float w = body->getAngularVelocity();
		invMass = body->inv_mass;
		invI = body->inv_inertia;
		glm::mat2 rot1 = rotationMatrix(body->angle);
		float d = damping;
		float k = stiffness;
		float h = dt;
		gamma = h * (d + h * k);
		beta = h * k * gamma;
 		r = (localCenter) * rot1;
		glm::mat2 K;
		K[0][0] = invMass + invI * r.y * r.y + gamma;
		K[0][1] = -invI * r.x * r.y;
		K[1][0] = K[0][1];
		K[1][1] = invMass + invI * r.x * r.x + gamma;

		mass = invert(K);

		C = pos1 + r - target;
		//C *= beta;
		

	}

	void applyImpulse(const float dt) {
		glm::vec2 v1 = body->getVelocity();
		float w = body->getAngularVelocity();

		glm::vec2 Cdot = v1 + cross(w, r);
		glm::vec2 impulse = -(Cdot + C + gamma * this->impulse) * mass;

		glm::vec2 oldImpulse = impulse;
		this->impulse = impulse;
		float maxImpulse = dt * max_force;
		float mag = glm::dot(this->impulse, this->impulse);
		if (mag > maxImpulse) {
			this->impulse *= maxImpulse / std::sqrtf(mag);
		}

		//impulse = this->impulse - oldImpulse;

		v1 += invMass * this->impulse;
		w += invI * cross(r, this->impulse);
		body->linear_velocity = v1;
		body->angular_velocity = w;
	}
};

#endif // !__MOUSE_JOINT_HPP__
