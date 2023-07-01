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
	std::vector<Collisions::CollisionManifold> contactList;

	glm::vec2 gravity = glm::vec2(0.0f, 0.0f);
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
	
	void addStaticObject(BoxRigidBody rigidBody) {
		rigidBody.is_static = true;
		staticObjects.push_back(rigidBody);
	}

	void addDynamicObject(BoxRigidBody rigidBody) {
		dynamicObjects.push_back(rigidBody);
	}

	void update(float dt) {
		applyGravity();
		applyAngularAcceleration(dt);
		contactList.clear();
		checkCollisions();
		checkCollisionWithStaticBodies();
		solveCollisions(dt);
		updateRotation(dt);
		updatePosition(dt);

	}

	void applyGravity() {
		for (auto& dynamicObject : dynamicObjects) {
			dynamicObject.accelerate(gravity);
		}
	}

	void applyAngularAcceleration(float dt) {
		for (auto& dynamicObject : dynamicObjects) {
			dynamicObject.setAngularVelocity(angularAcceleration, dt);
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

		rigidBodyA.position_current -= rigidBodyA.inv_mass * impulse;
		rigidBodyB.position_current += rigidBodyB.inv_mass * impulse;
	}

	static float cross(glm::vec2 a, glm::vec2 b) {
		return a.x * b.y - a.y * b.x;
	}

	void solveRigidBodyCollisionWithInertia(Collisions::CollisionManifold& record, float dt) {
		BoxRigidBody& rigidBodyA = record.bodyA;
		BoxRigidBody& rigidBodyB = record.bodyB;
		glm::vec2 normal = record.normal;
		glm::vec2 contactPoint1 = record.contact1;
		glm::vec2 contactPoint2 = record.contact2;
		size_t contactCount = record.contactCount;
		float depth = record.depth;

		float e = std::min(rigidBodyA.restitution, rigidBodyB.restitution);
		std::array<glm::vec2, 2> contactPoints = { contactPoint1, contactPoint2 };
		std::array<glm::vec2, 2> impulses;
		std::array<glm::vec2, 2> raList;
		std::array<glm::vec2, 2> rbList;
		for (size_t i = 0; i < contactCount; i++) {
			glm::vec2 ra = contactPoints[i] - rigidBodyA.position_current;
			glm::vec2 rb = contactPoints[i] - rigidBodyB.position_current;
			raList[i] = ra;
			rbList[i] = rb;

			glm::vec2 raPerp = glm::vec2(-ra.y, ra.x);
			glm::vec2 rbPerp = glm::vec2(-rb.y, rb.x);

			glm::vec2 angularVelocityA = rigidBodyA.getAngularVelocity() * raPerp;
			glm::vec2 angularVelocityB = rigidBodyB.getAngularVelocity() * rbPerp;
			glm::vec2 relativeVelocity = rigidBodyB.getVelocity() + angularVelocityB - rigidBodyA.getVelocity() - angularVelocityA;

			float contactVelocityMag = glm::dot(relativeVelocity, normal);

			if(contactVelocityMag > 0) {
				return;
			}

			float raPerpDotNormal = glm::dot(raPerp, normal);
			float rbPerpDotNormal = glm::dot(rbPerp, normal);

			float denom = rigidBodyA.inv_mass + rigidBodyB.inv_mass + rigidBodyA.inv_inertia * (raPerpDotNormal * raPerpDotNormal) + rigidBodyB.inv_inertia * (rbPerpDotNormal * rbPerpDotNormal);

			float j = -(1.0f + e) * contactVelocityMag;
			j /= denom;
			j /= (float)contactCount;

			glm::vec2 impulse = j * normal;
			impulses[i] = impulse;

		}
		
		for (size_t i = 0; i < impulses.size(); i++) {
			rigidBodyA.position_current -= rigidBodyA.inv_mass * impulses[i];
			float new_angular_velocityA = rigidBodyA.getAngularVelocity() - rigidBodyA.inv_inertia * cross(raList[i], impulses[i]);
			rigidBodyA.setAngularVelocity(new_angular_velocityA, dt);
			rigidBodyB.position_current += rigidBodyB.inv_mass * impulses[i];
			float new_angular_velocityB = rigidBodyB.getAngularVelocity() + rigidBodyB.inv_inertia * cross(rbList[i], impulses[i]);
			rigidBodyB.setAngularVelocity(new_angular_velocityB, dt);
		}
	}

	void solveRigidBodyCollisionWithStaticBodies(BoxRigidBody& dynamicObjectA, BoxRigidBody& staticObjectB, Collisions::CollisionManifold& record) {
		dynamicObjectA.position_current -= (record.depth) * record.normal;
	}

	void checkCollisions() {
		for (size_t i = 0; i < dynamicObjects.size(); i++) {
			std::vector<glm::vec2> verticesA = dynamicObjects[i].getVertices();
			for (size_t j = i + 1; j < dynamicObjects.size(); j++) {
				std::vector<glm::vec2> verticesB = dynamicObjects[j].getVertices();
				Collisions::IntersectionRecord intersectionRecord = Collisions::polygonIntersection(verticesA, verticesB);
				if (intersectionRecord.intersecting) {
					Collisions::CollisionManifold collisionManifold(dynamicObjects[i], dynamicObjects[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);
					Collisions::findContactPoints(verticesA, verticesB, collisionManifold);
					contactList.push_back(collisionManifold);
				}
			}
		}
	}

	void checkCollisionWithStaticBodies() {
		for (size_t i = 0; i < dynamicObjects.size(); i++) {
			std::vector<glm::vec2> verticesA = dynamicObjects[i].getVertices();
			for (size_t j = 0; j < staticObjects.size(); j++) {
				std::vector<glm::vec2> verticesB = staticObjects[j].getVertices();
				Collisions::IntersectionRecord intersectionRecord = Collisions::polygonIntersection(verticesA, verticesB);
				if (intersectionRecord.intersecting) {
					Collisions::CollisionManifold collisionManifold(dynamicObjects[i], staticObjects[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);
					Collisions::findContactPoints(verticesA, verticesB, collisionManifold);
					contactList.push_back(collisionManifold);
				}
			}
		}
	}

	void seperateBodies(Collisions::CollisionManifold & manifold) {
		BoxRigidBody & rigidBodyA = manifold.bodyA;
		BoxRigidBody & rigidBodyB = manifold.bodyB;
		if (rigidBodyA.is_static) {
			rigidBodyB.position_current -= (manifold.depth) * manifold.normal;
		}
		else if (rigidBodyB.is_static) {
			rigidBodyA.position_current -= (manifold.depth) * manifold.normal;
		}
		else {
			rigidBodyA.position_current -= (manifold.depth / 2.0f) * manifold.normal;
			rigidBodyB.position_current += (manifold.depth / 2.0f) * manifold.normal;
		}
	}


	void solveCollisions(float dt) {
		
		for (auto& contact : contactList) {
			
			seperateBodies(contact);
			std::vector<glm::vec2> verticesA = contact.bodyA.getVertices();
			std::vector<glm::vec2> verticesB = contact.bodyB.getVertices();
			Collisions::findContactPoints(verticesA, verticesB, contact);
			solveRigidBodyCollisionWithInertia(contact, dt);
		}
	}
};

#endif
