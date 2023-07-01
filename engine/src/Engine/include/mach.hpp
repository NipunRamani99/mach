#ifndef __MACH_HPP__
#define __MACH_HPP__
#include <iostream>
#include "BoxRigidBody.hpp"
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
		solveCollisions();
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
		rigidBodyA.position_current -= (record.depth / 2.0f) * record.normal;
		rigidBodyB.position_current += (record.depth / 2.0f) * record.normal;
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
					contactList.push_back(collisionManifold);
				}
			}
		}
	}

	void solveCollisions() {
		for (auto& contact : contactList) {
			if (contact.bodyA.is_static) {
				solveRigidBodyCollisionWithStaticBodies(contact.bodyB, contact.bodyA, contact);
			}
			else if (contact.bodyB.is_static) {
				solveRigidBodyCollisionWithStaticBodies(contact.bodyA, contact.bodyB, contact);
			}
			else {
				solveRigidBodyCollision(contact.bodyA, contact.bodyB, contact);
			}
		}
	}
};

#endif
