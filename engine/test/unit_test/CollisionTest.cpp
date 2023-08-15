#include <gtest/gtest.h>
#include "include/Collision.hpp"
#include "include/RigidBody.hpp"
#include "include/ContactPoint.hpp"



TEST(CollisionTest, BasicIntersectingBoxes) {
	
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100,100);

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = { 80.0f, 80.0f };
	boxB.angle = 0.0f;
	boxB.position = {110,110 };

	CollisionManifold manifold(&boxA, &boxB);

	ASSERT_EQ(manifold.contacts.size(), 2);
}

TEST(CollisionTest, NonIntersectingBoxes) {
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = { 80.0f, 80.0f };
	boxB.angle = 0.0f;
	boxB.position = { 200,200 };


	CollisionManifold manifold(&boxA, &boxB);

	ASSERT_EQ(manifold.contacts.size(), 0);
}


TEST(CollisionTest, RotatedIntersectingBoxes) {

	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = { 80.0f, 80.0f };
	boxB.angle = pi / 4.0f;
	boxB.position = { 110,110 };
	CollisionManifold manifold(&boxA, &boxB);

	ASSERT_EQ(manifold.contacts.size(), 2);
}

TEST(CollisionTest, BasicCircleBoxIntersecting) {
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);

	CircleRigidBody circleRigidBody = CircleRigidBody();
	circleRigidBody.radius = 10.0f;
	circleRigidBody.angle = 0.0f;
	circleRigidBody.position = glm::vec2(140.0f, 100.0f);
	CollisionManifold manifold(&boxA, &circleRigidBody);

	ASSERT_EQ(manifold.contacts.size(), 1);
}

TEST(CollisionTest, CircleBoxNonIntersecting) {
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);

	CircleRigidBody circleRigidBody = CircleRigidBody();
	circleRigidBody.radius = 10.0f;
	circleRigidBody.angle = 0.0f;
	circleRigidBody.position = glm::vec2(170.0f, 100.0f);
	CollisionManifold manifold(&boxA, &circleRigidBody);

	ASSERT_EQ(manifold.contacts.size(), 0);
}

TEST(CollisionTest, RotatedBoxCircleIntersectingTest) {
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(100, 100);

	CircleRigidBody circleRigidBody = CircleRigidBody();
	circleRigidBody.radius = 10.0f;
	circleRigidBody.angle = 0.0f;
	circleRigidBody.position = glm::vec2(140.0f, 100.0f);
	CollisionManifold manifold(&boxA, &circleRigidBody);

	ASSERT_EQ(manifold.contacts.size(), 1);
}
