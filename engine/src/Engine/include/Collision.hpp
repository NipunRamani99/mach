#ifndef __COLLISIONS_HPP__
#define __COLLISIONS_HPP__
#include "Math.hpp"
#include <vector>
#include <utility>
#include <array>
#include "BoxRigidBody.hpp"
#include "CircleRigidBody.hpp"
#include "ContactPoint.hpp"

	struct CollisionManifold {
		RigidBody * bodyA;
		RigidBody * bodyB;
		std::vector<ContactPoint> contacts;


		CollisionManifold(RigidBody* bodyA, RigidBody* bodyB)
			:
			bodyA(bodyA),
			bodyB(bodyB)
		{
			if (bodyA->type == RigidBody::CIRCLE && bodyB->type == RigidBody::BOX) {
 				contacts = Collide((BoxRigidBody*)bodyB, (CircleRigidBody*)bodyA);
			}
			else if (bodyA->type == RigidBody::BOX && bodyB->type == RigidBody::CIRCLE) {
				contacts = Collide((BoxRigidBody*)bodyA, (CircleRigidBody*)bodyB);
			}
			else if(bodyA->type == RigidBody::BOX && bodyB->type == RigidBody::BOX) {
				contacts = Collide((BoxRigidBody*)bodyA, (BoxRigidBody*)bodyB);
			}
			else if (bodyA->type == RigidBody::CIRCLE && bodyB->type == RigidBody::CIRCLE) {
				contacts = Collide((BoxRigidBody*)bodyA, (BoxRigidBody*)bodyB);
			}
		}

		void preStep(float dt) {
			const float allowedPenetration = 0.1f;
			float biasFactor = 0.1f;
			//std::array<glm::vec2, 2> contacts = { contact1, contact2 };
			for (int i = 0; i < contacts.size(); i++) {
				ContactPoint& c = contacts[i];
				c.r1 = contacts[i].position - bodyA->position;
				c.r2 = contacts[i].position - bodyB->position;

				//precompute normal mass, tangent mass, and bias
				float rn1 = glm::dot(c.r1, c.normal);
				float rn2 = glm::dot(c.r2, c.normal);
				float kNormal = bodyA->inv_mass + bodyB->inv_mass;
				kNormal += bodyA->inv_inertia * (glm::dot(c.r1, c.r1) - rn1 * rn1) + bodyB->inv_inertia * (glm::dot(c.r2, c.r2) - rn2 * rn2);
				c.massNormal = 1.0f / kNormal;

				glm::vec2 tangent = { -c.normal.y, c.normal.x };
				float rt1 = glm::dot(c.r1, tangent);
				float rt2 = glm::dot(c.r2, tangent);
				float kTangent = bodyA->inv_mass + bodyB->inv_mass;
				kTangent += bodyA->inv_inertia * (glm::dot(c.r1, c.r1) - rt1 * rt1) + bodyB->inv_inertia * (glm::dot(c.r2, c.r2) - rt2 * rt2);
				c.massTangent = 1.0f / kTangent;
				c.bias = -biasFactor / dt * std::min(0.0f, (c.separation) + allowedPenetration);

				glm::vec2 P = c.Pn * c.normal + c.Pt * tangent;
				bodyA->linear_velocity -= P * bodyA->inv_mass;
				bodyA->angular_velocity -= bodyA->inv_inertia * cross(c.r1, P);
				bodyB->linear_velocity += P * bodyB->inv_mass;
				bodyB->angular_velocity += bodyB->inv_inertia * cross(c.r2, P);
			}
		}
		glm::vec2 _cross(float a, glm::vec2 b) {
			glm::vec2 result;
			result.x = -a * b.y;
			result.y = a * b.x;
			return result;
		}
		void applyImpulse() {

			for (int i = 0; i < contacts.size(); i++) {
				ContactPoint & c = contacts[i];
				c.r1 = c.position - bodyA->position;
				c.r2 = c.position - bodyB->position;

				//relative velocity at contact
				glm::vec2 dv = bodyB->linear_velocity + _cross(bodyB->angular_velocity, c.r2) - bodyA->linear_velocity - _cross(bodyA->angular_velocity, c.r1);

			
				//compute normal impulse
				float vn = glm::dot(dv, c.normal);

				float dPn = c.massNormal * (-vn + c.bias );

				
				float Pn0 = c.Pn;
				c.Pn = std::max(Pn0 + dPn, 0.0f);
				dPn = c.Pn - Pn0;

				//apply contact impulse
				glm::vec2 _Pn =  dPn * c.normal;
				bodyA->linear_velocity -= _Pn * bodyA->inv_mass;
				bodyA->angular_velocity -= bodyA->inv_inertia * cross(c.r1, _Pn);
				bodyB->linear_velocity += _Pn * bodyB->inv_mass;
	    		bodyB->angular_velocity += bodyB->inv_inertia * cross(c.r2, _Pn);


				//relative velocity at contact
				dv = bodyB->linear_velocity + _cross(bodyB->angular_velocity, c.r2) - bodyA->linear_velocity - _cross(bodyA->angular_velocity, c.r1);

				//compute tangent impulse
				float vt = glm::dot(dv, { -c.normal.y, c.normal.x });
				float dPt = c.massTangent * (-vt);

				float friction = std::sqrtf(bodyA->static_friction * bodyB->static_friction);

				float maxPt =  friction* c.Pn;
				float oldTangentImpulse = c.Pt;
				c.Pt = glm::clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
				dPt = c.Pt - oldTangentImpulse;

				
				//apply contact impulse
				glm::vec2 _Pt = dPt * glm::vec2{ -c.normal.y, c.normal.x };
				bodyA->linear_velocity -= _Pt * bodyA->inv_mass;
				bodyA->angular_velocity -= bodyA->inv_inertia * cross(c.r1, _Pt);
				bodyB->linear_velocity += _Pt * bodyB->inv_mass;
				bodyB->angular_velocity += bodyB->inv_inertia * cross(c.r2, _Pt);
			}
		
		}

	};


#endif