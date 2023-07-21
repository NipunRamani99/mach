#ifndef __JOINT_HPP__
#define __JOINT_HPP__
#include "Math.hpp"
#include <glm/glm.hpp>
#include "RigidBody.hpp"
struct Joint {
	RigidBody* body1 = nullptr;
	RigidBody* body2 = nullptr;
	glm::vec2 P{0.0f, 0.0f};
	glm::vec2 anchor{0.0f, 0.0f};
	glm::vec2 r1;
	glm::vec2 r2;
	glm::vec2 localAnchorPoint1{};
	glm::vec2 localAnchorPoint2{};
	glm::vec2 bias{};
	glm::mat2 M{};
	float softness = 0.0f;
	float biasFactor = 0.2f;

	Joint(RigidBody* body1, RigidBody* body2, glm::vec2 anchor) 
		:
		body1(body1),
		body2(body2),
		anchor(anchor)
	{
		glm::mat2 rot1(rotationMatrix(body1->angle));
		glm::mat2 rot2(rotationMatrix(body2->angle));
		glm::mat2 rot1T = glm::transpose(rot1);
		glm::mat2 rot2T = glm::transpose(rot2);
		localAnchorPoint1 = (anchor - body1->position) * rot1T;
		localAnchorPoint2 = (anchor - body2->position) * rot2T;
		
		P = { 0.0f, 0.0f };
		biasFactor = 0.2f;
	}

	void preStep(const float inv_dt) {
		glm::mat2 rot1(rotationMatrix(body1->angle));
		glm::mat2 rot2(rotationMatrix(body2->angle));

		r1 = localAnchorPoint1 * rot1;
		r2 = localAnchorPoint2 * rot2;
		const float invMass1 = body1->inv_mass;
		const float invMass2 = body2->inv_mass;
		const float invInertia1 = body1->inv_inertia;
		const float invInertia2 = body2->inv_inertia;

		glm::mat2 k1{0.0f};
		k1[0][0] = invMass1 + invMass2;
		k1[0][1] = 0.0f;
		k1[1][0] = 0.0f;
		k1[1][1] = invMass1 + invMass2;
		glm::mat2 k2;
		k2[0][0] = r1.y * r1.y;
		k2[0][1] = -r1.x * r1.y;
		k2[1][0] = k2[0][1];
		k2[1][1] = r1.x * r1.x;
		k2 = invInertia1 * k2;
		
		glm::mat2 k3;
		k3[0][0] = r2.y * r2.y;
		k3[0][1] = -r2.x * r2.y;
		k3[1][0] = k3[0][1];
		k3[1][1] = r2.x * r2.x;
		k3 = invInertia2 * k3;

		glm::mat2 K = k1 + k2 + k3;

		M = invert(K);

		glm::vec2 p1 = body1->position + r1;
		glm::vec2 p2 = body2->position + r2;

		glm::vec2 dp = p2 - p1;
		bias = -biasFactor * inv_dt * dp;
		P = { 0.0f, 0.0f };
	}

	void applyImpulse() {
		glm::vec2 dv = body2->getVelocity() + cross(body2->angular_velocity, r2) - body1->getVelocity() - cross(body1->getAngularVelocity(), r1);

		glm::vec2 impulse{0.0f, 0.0f};
		impulse = M * (bias - dv - softness * P);

		body1->linear_velocity -=  body1->inv_mass * impulse;
		body1->angular_velocity -= body1->inv_inertia * cross(r1, impulse);

		body2->linear_velocity += body2->inv_mass * impulse;
		body2->angular_velocity += body2->inv_inertia * cross(r2, impulse);

		P += impulse;
	}
};

#endif