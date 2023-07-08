#ifndef __MACH_HPP__
#define __MACH_HPP__
#include <iostream>
#include "BoxRigidBody.hpp"
#include <vector>
#include <array>
#include "Collision.hpp"

class Mach {
private:
	std::vector<BoxRigidBody> staticObjects;
	std::vector<BoxRigidBody> dynamicObjects;
	std::vector<RigidBody*> rigidBodies;
	std::vector<Collisions::CollisionManifold> contactList;
	const int num_iterations = 10;
	glm::vec2 gravity = glm::vec2(0.0f,10.0f);
	float angularAcceleration = 0.0f;
public:
	Mach() {

	}
	
	Mach(Mach& mach) = delete;
	
	~Mach() {};
	
	std::vector<BoxRigidBody>& getStaticObjects() {
		return staticObjects;
	}

	std::vector<BoxRigidBody>& getDynamicObjects() {
		return dynamicObjects;
	}

	std::vector<Collisions::CollisionManifold>& getContactList() {
		return contactList;
	}
	
	std::vector<RigidBody*>& getRigidBodies() {
		return rigidBodies;
	}

	void addStaticObject(BoxRigidBody rigidBody) {
		rigidBody.is_static = true;
		staticObjects.push_back(rigidBody);
	}

	void addDynamicObject(RigidBody * rigidBody) {
		rigidBodies.push_back(rigidBody);
	}

	void update(float dt) {		
		applyGravity();
		applyForce(dt);
		for (int i = 0; i < num_iterations; i++) {
			contactList.clear();
			narrowPhase();
		}
		step(dt);
	}

	void applyGravity() {
		for (RigidBody * rigidBody : rigidBodies) {
			rigidBody->accelerate(gravity);
		}
	}

	void applyForce(float dt) {
		for(RigidBody * rigidBody : rigidBodies) {
			rigidBody->applyForce(dt);
		}
	}

	void step(float dt) {
		for(RigidBody * rigidBody : rigidBodies) {
			rigidBody->integrate(dt);
		}
	}

	void updateRotation(float dt) {
		for (auto& dynamicObject : dynamicObjects) {
			dynamicObject.updateAngularVelocity(dt);
		}
	}

	void updatePosition(float dt) {
		for (auto& dynamicObject : dynamicObjects) {
			dynamicObject.updateVelocity(dt);
		}
	}

	void solveRigidBodyCollision(BoxRigidBody& rigidBodyA, BoxRigidBody& rigidBodyB, Collisions::CollisionManifold& record) {
		glm::vec2 normal = record.normal;
		float depth = record.depth;

		glm::vec2 relativeVelocity = rigidBodyB.getVelocity() - rigidBodyA.getVelocity();
		if (glm::dot(relativeVelocity, normal) > 0) {
			return;
		}

		float e = std::min(rigidBodyA.restitution, rigidBodyB.restitution);
		float j = -(1.0f + e) * glm::dot(relativeVelocity, normal);
		j /= (rigidBodyA.mass + rigidBodyB.mass);

		glm::vec2 impulse = j * normal;

		rigidBodyA.linear_velocity -= rigidBodyA.inv_mass * impulse;
		rigidBodyB.linear_velocity += rigidBodyB.inv_mass * impulse;
	}

	static float cross(glm::vec2 a, glm::vec2 b) {
		return a.x * b.y - a.y * b.x;
	}

	//void solveRigidBodyCollisionWithInertia(Collisions::CollisionManifold& record) {
	//	BoxRigidBody& rigidBodyA = record.bodyA;
	//	BoxRigidBody& rigidBodyB = record.bodyB;
	//	glm::vec2 normal = record.normal;
	//	glm::vec2 contactPoint1 = record.contact1;
	//	glm::vec2 contactPoint2 = record.contact2;
	//	size_t contactCount = record.contactCount;
	//	float depth = record.depth;
	//	glm::vec2 zero = { 0.0f,0.0f };
	//	//float e = std::min(rigidBodyA.restitution, rigidBodyB.restitution);
	//	std::array<glm::vec2, 2> contactPoints = { contactPoint1, contactPoint2 };
	//	std::array<glm::vec2, 2> impulses = { zero,zero };
	//	std::array<glm::vec2, 2> raList = { zero, zero };
	//	std::array<glm::vec2, 2> rbList = { zero, zero };

	//	float e = std::min(rigidBodyA.restitution, rigidBodyB.restitution);

	//	for (size_t i = 0; i < contactCount; i++) {
	//		glm::vec2 ra = contactPoints[i] - rigidBodyA.position;
	//		glm::vec2 rb = contactPoints[i] - rigidBodyB.position;
	//		raList[i] = ra;
	//		rbList[i] = rb;

	//		glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
	//		glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

	//		glm::vec2 angularVelocityA = rigidBodyA.getAngularVelocity() * raPerp;
	//		glm::vec2 angularVelocityB = rigidBodyB.getAngularVelocity() * rbPerp;
	//		glm::vec2 relativeVelocity = rigidBodyB.getVelocity() + angularVelocityB - rigidBodyA.getVelocity() - angularVelocityA;

	//		float contactVelocityMag = glm::dot(relativeVelocity, normal);

	//		if(contactVelocityMag > 0) {
	//			return;
	//		}

	//		float raPerpDotNormal = glm::dot(raPerp, normal);
	//		float rbPerpDotNormal = glm::dot(rbPerp, normal);

	//		float denom = glm::dot(normal,normal) * (rigidBodyA.inv_mass + rigidBodyB.inv_mass) + rigidBodyA.inv_inertia * (raPerpDotNormal * raPerpDotNormal) + rigidBodyB.inv_inertia * (rbPerpDotNormal * rbPerpDotNormal);

	//		float j = -(1.0f + 1.00f) * contactVelocityMag;
	//		j /= denom;
	//	    j /= (float)contactCount;

	//		glm::vec2 impulse = j * normal;
	//	    impulses[i] = impulse;

	//	}

	//	for (size_t i = 0; i < record.contactCount; i++) {
	//		if (!rigidBodyA.is_static) {
	//			rigidBodyA.linear_velocity -= rigidBodyA.inv_mass * impulses[i];
	//			float new_angular_velocityA = rigidBodyA.getAngularVelocity() - rigidBodyA.inv_inertia * cross(raList[i], impulses[i]);
	//			rigidBodyA.angular_velocity = new_angular_velocityA;
	//		}
	//		if (!rigidBodyB.is_static) {
	//			rigidBodyB.linear_velocity += rigidBodyB.inv_mass * impulses[i];
	//			float new_angular_velocityB = rigidBodyB.getAngularVelocity() + rigidBodyB.inv_inertia * cross(rbList[i], impulses[i]);
	//			rigidBodyB.angular_velocity = new_angular_velocityB;
	//		}

	//	}
	//}


	void solveRigidBodyCollisionWithInertiaAndFriction(Collisions::CollisionManifold& record) {
		RigidBody * rigidBodyA = record.bodyA;
		RigidBody * rigidBodyB = record.bodyB;
		glm::vec2 normal = record.normal;
		glm::vec2 contactPoint1 = record.contact1;
		glm::vec2 contactPoint2 = record.contact2;
		size_t contactCount = record.contactCount;
		float depth = record.depth;
		glm::vec2 zero = { 0.0f,0.0f };
		//float e = std::min(rigidBodyA->restitution, rigidBodyB->restitution);
		std::array<glm::vec2, 2> contactPoints = { contactPoint1, contactPoint2 };
		std::array<glm::vec2, 2> impulses = { zero,zero };
		std::array<glm::vec2, 2> frictionImpulses = { zero,zero };
		std::array<glm::vec2, 2> raList = { zero, zero };
		std::array<glm::vec2, 2> rbList = { zero, zero };
		std::array<float, 2> jList = { 0.0f, 0.0f };

		float e = std::min(rigidBodyA->restitution, rigidBodyB->restitution);
		float sf = rigidBodyA->static_friction + rigidBodyB->static_friction;
		sf *= 0.25f;
		float df = rigidBodyA->dynamic_friction + rigidBodyB->dynamic_friction;
		df *= 0.25f;

		for (size_t i = 0; i < contactCount; i++) {
			glm::vec2 ra = contactPoints[i] - rigidBodyA->position;
			glm::vec2 rb = contactPoints[i] - rigidBodyB->position;
			raList[i] = ra;
			rbList[i] = rb;

			glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
			glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

			glm::vec2 angularVelocityA = rigidBodyA->getAngularVelocity() * raPerp;
			glm::vec2 angularVelocityB = rigidBodyB->getAngularVelocity() * rbPerp;
			glm::vec2 relativeVelocity = rigidBodyB->getVelocity() + angularVelocityB - rigidBodyA->getVelocity() - angularVelocityA;

			float contactVelocityMag = glm::dot(relativeVelocity, normal);

			if (contactVelocityMag > 0) {
				return;
			}

			float raPerpDotNormal = glm::dot(raPerp, normal);
			float rbPerpDotNormal = glm::dot(rbPerp, normal);

			float denom = glm::dot(normal, normal) * (rigidBodyA->inv_mass + rigidBodyB->inv_mass) + rigidBodyA->inv_inertia * (raPerpDotNormal * raPerpDotNormal) + rigidBodyB->inv_inertia * (rbPerpDotNormal * rbPerpDotNormal);

			float j = -(1.0f + 1.00f) * contactVelocityMag;
			j /= denom;
			j /= (float)contactCount;
			jList[i] = j;
			glm::vec2 impulse = j * normal;
			impulses[i] = impulse;
		}

		for (size_t i = 0; i < record.contactCount; i++) {
			if (!rigidBodyA->is_static) {
				rigidBodyA->linear_velocity -= rigidBodyA->inv_mass * impulses[i];
				float new_angular_velocityA = rigidBodyA->getAngularVelocity() - rigidBodyA->inv_inertia * cross(raList[i], impulses[i]);
				rigidBodyA->angular_velocity = new_angular_velocityA;
			}
			if (!rigidBodyB->is_static) {
				rigidBodyB->linear_velocity += rigidBodyB->inv_mass * impulses[i];
				float new_angular_velocityB = rigidBodyB->getAngularVelocity() + rigidBodyB->inv_inertia * cross(rbList[i], impulses[i]);
				rigidBodyB->angular_velocity = new_angular_velocityB;
			}

		}

		for (size_t i = 0; i < contactCount; i++) {
			glm::vec2 ra = contactPoints[i] - rigidBodyA->position;
			glm::vec2 rb = contactPoints[i] - rigidBodyB->position;
			raList[i] = ra;
			rbList[i] = rb;

			glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
			glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

			glm::vec2 angularVelocityA = rigidBodyA->getAngularVelocity() * raPerp;
			glm::vec2 angularVelocityB = rigidBodyB->getAngularVelocity() * rbPerp;
			glm::vec2 relativeVelocity = rigidBodyB->getVelocity() + angularVelocityB - rigidBodyA->getVelocity() - angularVelocityA;

			glm::vec2 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
			if(tangent.length() < 0.0001f) {
				continue;
			}

			tangent = glm::normalize(tangent);
			float raPerpDotTangent = glm::dot(raPerp, tangent);
			float rbPerpDotTangent = glm::dot(rbPerp, tangent);
			
			float denom = glm::dot(normal, normal) * (rigidBodyA->inv_mass + rigidBodyB->inv_mass) + rigidBodyA->inv_inertia * (raPerpDotTangent * raPerpDotTangent) + rigidBodyB->inv_inertia * (rbPerpDotTangent * rbPerpDotTangent);

			float jt = -glm::dot(relativeVelocity, tangent);
			jt /= denom;
			jt /= (float)contactCount;

			

			glm::vec2 frictionImpulse;

			float j = jList[i];

			if(std::abs(j)<=jt * sf) {
				frictionImpulse = jt * tangent;
			}
			else {
				frictionImpulse = -j * tangent * df;
			}

			frictionImpulses[i] = frictionImpulse;
		}

		for (size_t i = 0; i < record.contactCount; i++) {
			if (!rigidBodyA->is_static) {
				rigidBodyA->linear_velocity -= rigidBodyA->inv_mass * frictionImpulses[i];
				float new_angular_velocityA = rigidBodyA->getAngularVelocity() - rigidBodyA->inv_inertia * cross(raList[i], frictionImpulses[i]);
				rigidBodyA->angular_velocity = new_angular_velocityA;
			}
			if (!rigidBodyB->is_static) {
				rigidBodyB->linear_velocity += rigidBodyB->inv_mass * frictionImpulses[i];
				float new_angular_velocityB = rigidBodyB->getAngularVelocity() + rigidBodyB->inv_inertia * cross(rbList[i], frictionImpulses[i]);
				rigidBodyB->angular_velocity = new_angular_velocityB;
			}

		}
	}


	void checkCollisions() {
		for (size_t i = 0; i < rigidBodies.size(); i++) {
			if(rigidBodies[i]->is_static)continue;
			for (size_t j = 0; j < rigidBodies.size(); j++) {
				if (i == j) continue;
				Collisions::IntersectionRecord intersectionRecord = Collisions::findIntersection(rigidBodies[i], rigidBodies[j]);
				if(intersectionRecord.intersecting){
					Collisions::CollisionManifold collisionManifold(rigidBodies[i], rigidBodies[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);
					seperateBodies(rigidBodies[i], rigidBodies[j], intersectionRecord);
					Collisions::findContactPoint(collisionManifold);
					
				}
			}
		}


		//for (size_t i = 0; i < dynamicObjects.size(); i++) {
		//	std::vector<glm::vec2> verticesA = dynamicObjects[i].getVertices();
		//	if (dynamicObjects[i].is_static)continue;
		//	for (size_t j = i + 1; j < dynamicObjects.size(); j++) {
		//		std::vector<glm::vec2> verticesB = dynamicObjects[j].getVertices();
		//		Collisions::IntersectionRecord intersectionRecord = Collisions::polygonIntersection(verticesA, verticesB);
		//		if (intersectionRecord.intersecting) {
		//			Collisions::CollisionManifold collisionManifold(dynamicObjects[i], dynamicObjects[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);
		//			Collisions::findPolygonContactPoints(verticesA, verticesB, collisionManifold);
		//			contactList.push_back(collisionManifold);
		//		}
		//	}
		//}
	}
	
	

	void seperateBodies(RigidBody * rigidBodyA, RigidBody * rigidBodyB, Collisions::IntersectionRecord & intersection) {
		if (rigidBodyA->is_static && rigidBodyB->is_static) return;
		if (rigidBodyA->is_static) {
			rigidBodyB->position -= (intersection.depth) * intersection.axis;
		}
		else if (rigidBodyB->is_static) {
			rigidBodyA->position -= (intersection.depth) * intersection.axis;
		}
		else {
			rigidBodyA->position -= (intersection.depth / 2.0f) * intersection.axis;
			rigidBodyB->position += (intersection.depth / 2.0f) * intersection.axis;
		}
	}

	void narrowPhase() {
		for (size_t i = 0; i < rigidBodies.size(); i++) {
			if (rigidBodies[i]->is_static)continue;
			for (size_t j = 0; j < rigidBodies.size(); j++) {
				if (i == j) continue;
				Collisions::IntersectionRecord intersectionRecord = Collisions::findIntersection(rigidBodies[i], rigidBodies[j]);
				if (intersectionRecord.intersecting) {
					Collisions::CollisionManifold collisionManifold(rigidBodies[i], rigidBodies[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);
					seperateBodies(rigidBodies[i], rigidBodies[j], intersectionRecord);
					Collisions::findContactPoint(collisionManifold);
					solveRigidBodyCollisionWithInertiaAndFriction(collisionManifold);
				}
			}	
		}
	}
};

#endif
