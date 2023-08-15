#include <gtest/gtest.h>
#include "include/mach.hpp"  

//Test Broadphase
TEST(MachTest, BroadphaseTest) {
    Mach mach;

	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);
	boxA.calculateAABB();

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = { 80.0f, 80.0f };
	boxB.angle = 0.0f;
	boxB.position = { 110,110 };
	boxB.calculateAABB();

	mach.addDynamicObject(&boxA);
	mach.addDynamicObject(&boxB);
	
	mach.broadPhase();

	auto collisionPairs = mach.getCollisionPairs();
	ASSERT_EQ(collisionPairs.size(), 1);
}

//Test Narrowphase
TEST(MachTest, NarrowPhase) {
	Mach mach;
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);
	boxA.calculateAABB();

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = { 80.0f, 80.0f };
	boxB.angle = 0.0f;
	boxB.position = { 110,110 };
	boxB.calculateAABB();

	mach.addDynamicObject(&boxA);
	mach.addDynamicObject(&boxB);

	mach.broadPhase();

	mach.narrowPhase();
	auto contactList = mach.getContactList();
	ASSERT_EQ(contactList.size(), 1);
}

