#ifndef __DISTANCE_JOINT_HPP__
#define __DISTANCE_JOINT_HPP__
#include "Math.hpp"
#include "RigidBody.hpp"
#include "Joint.hpp"
struct DistanceJoint : public Joint {
	RigidBody* body1 = nullptr;
	RigidBody* body2 = nullptr;

	float minLength = 0.0f;
	float maxLength = FLT_MAX;
	float stiffness = 0.0f;
	float damping = 0.0f;
	float restLength = 1.0f;
	float currentLength = 0.0f;
	glm::vec2 P{0.0f, 0.0f};
	glm::vec2 localAnchorA{0.0f, 0.0f};
	glm::vec2 localAnchorB{0.0f, 0.0f};
	glm::vec2 r1 {0.0f, 0.0f};
	glm::vec2 r2 {0.0f, 0.0f};
	glm::vec2 u {0.0f, 0.0f};
	float bias = 0.0f;
	float invEffectiveMass = 0.0f;
	float effectiveMass = 0.0f;
	float gamma = 0.0f;
	float softMass = 0.0f; 
	float impulse = 0.0f;
	float lowerImpulse = 0.0f; 
	float upperImpulse = 0.0f;
	DistanceJoint(RigidBody * body1, RigidBody * body2, glm::vec2 anchorA, glm::vec2 anchorB, float minL, float maxL, float restL, float stiffness, float damping) {
		localAnchorA = anchorA - body1->position;
		localAnchorB = anchorB - body2->position;
		this->body1 = body1;
		this->body2 = body2;
		minLength = minL;
		maxLength = maxL;
		restLength = restL;
		this->stiffness = stiffness;
		this->damping = damping;
		bias = 0.0f;
		invEffectiveMass = 0.0f;
		effectiveMass = 0.0f;
		this->type = DISTANCE;
		lowerImpulse = 0.0f;
		upperImpulse = 0.0f;
		currentLength = 0.0f;
	}

	void initialize(const float dt) {
		glm::vec2& pos1 = body1->position;
		glm::vec2& pos2 = body2->position;
		glm::vec2 v1 = body1->getVelocity();
		glm::vec2 v2 = body2->getVelocity();
		float w1 = body1->getAngularVelocity();
		float w2 = body2->getAngularVelocity();
		float invM1 = body1->inv_mass;
		float invM2 = body2->inv_mass;
		float invI1 = body1->inv_inertia;
		float invI2 = body2->inv_inertia;

		glm::mat2 rot1 = rotationMatrix(body1->angle);
		glm::mat2 rot2 = rotationMatrix(body2->angle);

		r1 = localAnchorA * rot1;
		r2 = localAnchorB * rot2;
		u = pos2 + r2 - pos1 - r1;
		currentLength = glm::length(u);
		u *= 1.0f / currentLength;
		float crAu = cross(r1, u);
		float crBu = cross(r2, u);
		invEffectiveMass = invM1 + invI1 * crAu * crAu + invM2 + invI2 * crBu * crBu;
		effectiveMass = invEffectiveMass != 0.0f ? 1.0f / invEffectiveMass : 0.0f;
		
		if (stiffness > 0.0f && minLength < maxLength)
		{
			// soft
			float C = currentLength - restLength;

			float d = damping;
			float k = stiffness;

			// magic formulas
			float h = dt;

			// gamma = 1 / (h * (d + h * k))
			// the extra factor of h in the denominator is since the lambda is an impulse, not a force
			gamma = h * (d + h * k);
			gamma = gamma != 0.0f ? 1.0f / gamma : 0.0f;
			bias = C * h * k * gamma;

			invEffectiveMass += gamma;
			softMass = invEffectiveMass != 0.0f ? 1.0f / invEffectiveMass : 0.0f;
		}
		else
		{
			// rigid
			gamma = 0.0f;
			bias = 0.0f;
			softMass = effectiveMass;
		}
		

		impulse = 0.0f;
	}

	void applyImpulse(const float dt) {
		glm::vec2 vA = body1->getVelocity();
		glm::vec2 vB = body2->getVelocity();
		float invM1 = body1->inv_mass;
		float invM2 = body2->inv_mass;
		float invI1 = body1->inv_inertia;
		float invI2 = body2->inv_inertia;

		float wA = body1->getAngularVelocity();
		float wB = body2->getAngularVelocity();
	
		if (minLength < maxLength) {
			if (stiffness > 0.0f) {
				glm::vec2 vpA = vA + cross(wA, r1);
				glm::vec2 vpB = vB + cross(wB, r2);
				float cDot = glm::dot(u, vpB - vpA);
				float impulse = -softMass * (cDot + bias + gamma * this->impulse);
			//	this->impulse += impulse;
				glm::vec2 P = u * impulse;
				vA -= invM1 * impulse;
				vB += invM2 * impulse;
				wA -= invI1 * cross(P, r1);
				wB += invI2 * cross(P, r2);
			}
			// lower
			{
				float C = currentLength - minLength;
				float bias = std::max(0.0f, C) * 1.0f / dt;
				glm::vec2 vpA = vA + cross(wA, r1);
				glm::vec2 vpB = vB + cross(wB, r2);
				float cDot = glm::dot(u, vpB - vpA);
				float impulse = -effectiveMass * (cDot + bias);
				float oldImpulse = lowerImpulse;
				lowerImpulse = std::max(0.0f, lowerImpulse + impulse);
				impulse = lowerImpulse - oldImpulse;
				glm::vec2 P = impulse * u;
				vA -= invM1 * impulse;
				vB += invM2 * impulse;
				wA -= invI1 * cross(P, r1);
				wB += invI2 * cross(P, r2);
			}
			// upper
			{
				float C = maxLength - currentLength ;
				float bias = std::max(0.0f, C) * 1.0f / dt;
				glm::vec2 vpA = vA + cross(wA, r1);
				glm::vec2 vpB = vB + cross(wB, r2);
				float cDot = glm::dot(u, vpB - vpA);
				float impulse = -effectiveMass * (cDot + bias);
				float oldImpulse = upperImpulse;
				upperImpulse = std::max(0.0f, upperImpulse + impulse);
				impulse = upperImpulse - oldImpulse;
				glm::vec2 P = -impulse * u;
				vA -= invM1 * impulse;
				vB += invM2 * impulse;
				wA -= invI1 * cross(P, r1);
				wB += invI2 * cross(P, r2);
			}
		}



		body1->linear_velocity = vA;
		body2->linear_velocity = vB;
		body1->angular_velocity = wA;
		body2->angular_velocity = wB;
	}
};
#endif
