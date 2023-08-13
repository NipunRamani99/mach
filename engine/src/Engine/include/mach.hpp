#ifndef __MACH_HPP__
#define __MACH_HPP__
#include <iostream>
#include "BoxRigidBody.hpp"
#include <vector>
#include <array>
#include "Collision.hpp"
#include "Joint.hpp"
#include "MouseJoint.hpp"
class Mach {
private:
	std::vector<RigidBody*> rigidBodies;
	std::vector<Collisions::CollisionManifold> contactList;
	std::vector<Joint*> joints;
	MouseJoint* mouse_joint = nullptr;
	const int num_iterations = 8;
	glm::vec2 gravity = glm::vec2(0.0f, 10.0f);
	float angularAcceleration = 0.0f;
	std::vector<std::pair<RigidBody*, RigidBody*>> collisionPairs;
public:
	Mach() {

	}

	Mach(Mach& mach) = delete;

	~Mach() {};

	std::vector<Collisions::CollisionManifold>& getContactList() {
		return contactList;
	}

	std::vector<RigidBody*>& getRigidBodies() {
		return rigidBodies;
	}

	std::vector<Joint*>& getJoints() {
		return joints;
	}

	void setMouseJoint(MouseJoint* joint) {
		this->mouse_joint = joint;
	}

	void removeMouseJoint() {
		this->mouse_joint = nullptr;
	}

	void addDynamicObject(RigidBody* rigidBody) {
		rigidBodies.push_back(rigidBody);
	}

	void addJoint(Joint* joint) {
		joints.push_back(joint);
	}

	void update(float dt) {
		contactList.clear();
		collisionPairs.clear();
		broadPhase();
		narrowPhase();
		applyGravity();
		applyForce(dt);
		preStep(dt);
		for (int i = 0; i < num_iterations; i++) {
			size_t list_size = contactList.size();
			for (size_t j = 0; j < list_size; j++) {
				contactList[j].applyImpulse();
			}
			list_size = joints.size();
			for (size_t j = 0; j < list_size; j++) {
				joints[j]->applyImpulse();
			}
			if (mouse_joint != nullptr) {
				mouse_joint->applyImpulse(dt);
			}
		}
		step(dt);
	}

	void applyGravity() {
		for (RigidBody* rigidBody : rigidBodies) {
			rigidBody->accelerate(gravity);
		}
	}

	void applyForce(float dt) {
		for (RigidBody* rigidBody : rigidBodies) {
			rigidBody->applyForce(dt);
		}
	}

	void step(float dt) {
		for (RigidBody* rigidBody : rigidBodies) {
			rigidBody->integrate(dt);
		}
	}


	void broadPhase() {
		for (RigidBody* rigidBody : rigidBodies) {
			rigidBody->calculateAABB();
		}

		for (size_t i = 0; i < rigidBodies.size(); i++) {
			if (rigidBodies[i]->is_static)continue;
			for (size_t j = 0; j < rigidBodies.size(); j++) {
				if (i == j) continue;
				if (rigidBodies[i]->groupId != -1) {
					if (rigidBodies[i]->groupId == rigidBodies[j]->groupId) continue;
				}
				if (rigidBodies[i]->aabb.isOverlapping(rigidBodies[j]->aabb)) {
					collisionPairs.push_back({ rigidBodies[i], rigidBodies[j] });
				}
			}
		}
	}

	void narrowPhase() {
		for (auto [bodyA, bodyB] : collisionPairs) {
			bool found = false;
			for (Collisions::CollisionManifold& manifold : contactList) {
				if (manifold.bodyA == bodyA && manifold.bodyB == bodyB || manifold.bodyB == bodyA && manifold.bodyA == bodyB) {
					found = true;
					break;
				}
			}
			if (found) continue;
			Collisions::CollisionManifold collisionManifold(bodyA, bodyB);
			if (collisionManifold.contacts.size() > 0) {
				contactList.push_back(collisionManifold);
			}
		}
	}

	void preStep(float dt) {
		for (size_t i = 0; i < contactList.size(); i++) {
			contactList[i].preStep(dt);
		}
		for (Joint* j : joints) {
			j->preStep(1.0f / dt);
		}
		if (mouse_joint != nullptr) {
			mouse_joint->initialize(dt);
		}
	}
};

#endif
