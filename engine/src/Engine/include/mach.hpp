#ifndef __MACH_HPP__
#define __MACH_HPP__
#include <iostream>
#include "BoxRigidBody.hpp"
#include <vector>
#include <array>
#include <set>
#include "Collision.hpp"
#include "RevoluteJoint.hpp"
#include "MouseJoint.hpp"
#include "DistanceJoint.hpp"
class Mach {
private:
	std::vector<RigidBody*> rigidBodies;
	std::vector<CollisionManifold> contactList;
	std::vector<Joint*> joints;
	MouseJoint* mouse_joint = nullptr;
	const int num_iterations = 8;
	glm::vec2 gravity = glm::vec2(0.0f, 10.0f);
	float angularAcceleration = 0.0f;
	std::set<std::pair<RigidBody*, RigidBody*>> collisionPairs;
public:
	Mach() {

	}

	Mach(Mach& mach) = delete;

	~Mach() {};

	std::vector<CollisionManifold>& getContactList() {
		return contactList;
	}

	std::set<std::pair<RigidBody*, RigidBody*>> getCollisionPairs() {
		return collisionPairs;
	}

	std::vector<RigidBody*>& getRigidBodies() {
		return rigidBodies;
	}

	std::vector<Joint*>& getJoints() {
		return joints;
	}

	MouseJoint* getMouseJoint() {
		return mouse_joint;
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
		applyGravity();
		applyForce(dt);
		narrowPhase();
		preStep(dt);
		for (int i = 0; i < num_iterations; i++) {
			size_t list_size = contactList.size();
			for (size_t j = 0; j < list_size; j++) {
				contactList[j].applyImpulse();
			}
			list_size = joints.size();
			for (size_t j = 0; j < list_size; j++) {
				joints[j]->applyImpulse(dt);
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
		//Compute the AABB for every rigid body
		for (RigidBody* rigidBody : rigidBodies) {
			rigidBody->calculateAABB();
		}

		for (size_t i = 0; i < rigidBodies.size(); i++) {
			//Skip if rigid body is static
			if (rigidBodies[i]->is_static)continue;
			for (size_t j = 0; j < rigidBodies.size(); j++) {
				//Skip self-overlap test
				if (i == j) continue;
				//If the rigid body belongs to the same group, ignore.
				if (rigidBodies[i]->groupId != -1) {
					if (rigidBodies[i]->groupId == rigidBodies[j]->groupId) continue;
				}
				
				//Check if bounding box overlaps, sort the index and insert it into a vector of collision pairs.
				if (rigidBodies[i]->aabb.isOverlapping(rigidBodies[j]->aabb)) {
					int min = std::min(i, j);
					int max = std::max(i, j);
					collisionPairs.insert({ rigidBodies[max], rigidBodies[min] });
				}
			}
		}
	}

	void narrowPhase() {
		for (auto [bodyA, bodyB] : collisionPairs) {
			CollisionManifold collisionManifold(bodyA, bodyB);
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
			j->initialize(dt);
		}
		if (mouse_joint != nullptr) {
			mouse_joint->initialize(dt);
		}
	}

	void clear() {
		for (Joint* joint : joints) {
			delete joint;
		}
		joints.clear();
		for (RigidBody* body : rigidBodies) {
			delete body;
		}
		rigidBodies.clear();
	}
};

#endif
