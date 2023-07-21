#ifndef __MACH_HPP__
#define __MACH_HPP__
#include <iostream>
#include "BoxRigidBody.hpp"
#include <vector>
#include <array>
#include "Collision.hpp"
#include "Joint.hpp"

class Mach {
private:
	std::vector<BoxRigidBody> staticObjects;
	std::vector<BoxRigidBody> dynamicObjects;
	std::vector<RigidBody*> rigidBodies;
	std::vector<Collisions::CollisionManifold> contactList;
	std::vector<Joint*> joints;
	const int num_iterations = 10;
	glm::vec2 gravity = glm::vec2(0.0f,10.0f);
	float angularAcceleration = 0.0f;
	long long unsigned int iterationCount = 0;
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

	std::vector<Joint*>& getJoints() {
		return joints;
	}

	void addStaticObject(BoxRigidBody rigidBody) {
		rigidBody.is_static = true;
		staticObjects.push_back(rigidBody);
	}

	void addDynamicObject(RigidBody * rigidBody) {
		rigidBodies.push_back(rigidBody);
	}

	void addJoint(Joint* joint) {
		joints.push_back(joint);
	}

	void update(float dt) {		
		//dt/=10.0f;
		
		contactList.clear();
		broadPhase();
		applyGravity();
		applyForce(dt);
		preStep(dt);
		for (int i = 0; i < num_iterations; i++) {
			size_t list_size = contactList.size();
			for(size_t j = 0; j < list_size; j++) {
				contactList[j].applyImpulse();
				iterationCount++;
				
			}
			list_size = joints.size();
			for (size_t j = 0; j < list_size; j++) {
				joints[j]->applyImpulse();
			}
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

	
	static float cross(glm::vec2 a, glm::vec2 b) {
		return a.x * b.y - a.y * b.x;
	}

	

	void broadPhase() {
		for (size_t i = 0; i < rigidBodies.size(); i++) {
			if (rigidBodies[i]->is_static)continue;
			for (size_t j = 0; j < rigidBodies.size(); j++) {
				if (i == j) continue;
				if (rigidBodies[i]->type == RigidBody::Type::BOX && rigidBodies[j]->type == RigidBody::Type::BOX) {
					bool found = false;
					for (Collisions::CollisionManifold & manifold : contactList) {
						if (manifold.bodyA == rigidBodies[i] && manifold.bodyB == rigidBodies[j] || manifold.bodyB == rigidBodies[i] && manifold.bodyA == rigidBodies[j]) {
							found = true;
							break;
						}
					}
					if (found) continue;
					Collisions::CollisionManifold collisionManifold(rigidBodies[i], rigidBodies[j]);
						if (collisionManifold.contacts.size() > 0) {
							contactList.push_back(collisionManifold);
						}
				}
			}
		}
	}

	void preStep(float dt) {
		if (contactList.size() > 0) {
			//std::cout << "yolo2";
		}
		for (size_t i = 0; i < contactList.size(); i++) {
			contactList[i].preStep(dt);
		}
		for (Joint* j : joints) {
			j->preStep(1.0f / dt);
		}
	}

	void checkCollisions() {
		for (size_t i = 0; i < rigidBodies.size(); i++) {
			if(rigidBodies[i]->is_static)continue;
			for (size_t j = 0; j < rigidBodies.size(); j++) {
				if (i == j) continue;
				Collisions::IntersectionRecord intersectionRecord = Collisions::findIntersection(rigidBodies[i], rigidBodies[j]);
				if(intersectionRecord.intersecting){
			/*		Collisions::CollisionManifold collisionManifold(rigidBodies[i], rigidBodies[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);*/
					seperateBodies(rigidBodies[i], rigidBodies[j], intersectionRecord);
				//	Collisions::findContactPoint(collisionManifold);
					
				}
			}
		}
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
					/*Collisions::CollisionManifold collisionManifold(rigidBodies[i], rigidBodies[j], intersectionRecord.intersecting, intersectionRecord.depth, intersectionRecord.axis);*/
					seperateBodies(rigidBodies[i], rigidBodies[j], intersectionRecord);
				//	Collisions::findContactPoint(collisionManifold);
					//solveRigidBodyCollisionWithInertiaAndFriction(collisionManifold);
				}
			}	
		}
	}
};

#endif
