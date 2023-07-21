#include <gtest/gtest.h>
#include "include/Collision.hpp"
#include "include/RigidBody.hpp"
#include "include/Joint.hpp"

/*
* Create 2 overlapping polygons and check if they intersect using Collisions.intersectPolygon
* 
* Triangle A: (0.0f, 0.0f), (0.5f, 1.0f), (1.0f,0.0f)
* Triangle B: (0.5f,0.5f), (2.0f,2.0f),  (2.0f,0.0f)
*/
TEST(CollisionTest, IntersectingPolygons) {
	std::vector<glm::vec2> verticesA = {
		{0.0f, 0.0f},
		{0.5f, 1.0f},
		{1.0f, 0.0f}
	};
	std::vector<glm::vec2> verticesB = {
		{0.5f, 0.5f},
		{2.0f, 2.0f},
		{2.0f, 0.0f}
	};
	
	Collisions::IntersectionRecord intersecting = Collisions::polygonIntersection(verticesA, verticesB);

	ASSERT_TRUE(intersecting.intersecting);
}

TEST(CollisionTest, NonIntersectingPolygons) {
	std::vector<glm::vec2> verticesA = {
		{0.0f, 0.0f},
		{0.5f, 1.0f},
		{1.0f, 0.0f}
	};
	std::vector<glm::vec2> verticesB = {
		{3.0f, 1.5f},
		{2.0f, 2.0f},
		{2.0f, 0.0f}
	};

	Collisions::IntersectionRecord intersecting = Collisions::polygonIntersection(verticesA, verticesB);

	ASSERT_FALSE(intersecting.intersecting);
}

TEST(CollisionTest, IntersectingSquare) {
	std::vector<glm::vec2> verticesA = {
		{0.0f, 0.0f},
		{0.0f, 1.0f},
		{1.0f, 1.0f},
		{1.0f, 0.0f}
	};
	std::vector<glm::vec2> verticesB = {
		{0.50f, 0.970f},
		{0.50f, 2.0f},
		{1.50f, 2.0f},
		{1.50f, 1.0f}
	};
	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70.0f, 70.0f);
	boxA.angle = 0.0f;
	boxA.position = glm::vec2(600.000122f, 915.217041f);

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = { 1900.0f, 100.0f };
	boxB.angle = 0.0f;
	boxB.position = { 970.0f , 1000.0f };

	verticesA = boxA.getVertices();
	verticesB = boxB.getVertices();

	Collisions::CollisionManifold cm1(&boxA, &boxB);

	ASSERT_EQ(cm1.contacts.size(), 2);
}

TEST(CollisionTest, CollisionPoint) {
	//BoxRigidBody boxA = BoxRigidBody();
	//boxA.size = glm::vec2(70,70);
	//boxA.angle = 0.0f;
	//boxA.position = glm::vec2(600.0f,915.010132f);

	//BoxRigidBody boxB = BoxRigidBody();
	//boxB.size = glm::vec2(70, 70);
	//boxB.angle = 0.0f;
	//boxB.position = glm::vec2(635.0f, 845.433472f);

	BoxRigidBody boxA = BoxRigidBody();
	boxA.size = glm::vec2(70, 70);
	boxA.angle = 0.0379352309;
	boxA.position = glm::vec2(599.160339, 904.810547);

	BoxRigidBody boxB = BoxRigidBody();
	boxB.size = glm::vec2(70, 70);
	boxB.angle = 0.0380495787;
	boxB.position = glm::vec2(628.335876, 876.075806);

	
	std::vector<ContactPoint> contactPoints = Collide(&boxA, &boxB);

	/*ASSERT_LE(std::abs(contactPoints[0].x - 3.9f), 0.0001f);
	ASSERT_LE(std::abs(contactPoints[0].y - 3), 0.0001f);
	ASSERT_LE(std::abs(contactPoints[1].x - 3.9f), 0.0001f);
	ASSERT_LE(std::abs(contactPoints[1].y - 1), 0.0001f);*/

	const float eps = 0.0001f;

	ASSERT_EQ(contactPoints.size(), 2);
	ASSERT_LE(std::abs(contactPoints[0].normal.x - 0.999281), 0.0001f);
	ASSERT_LE(std::abs(contactPoints[0].normal.y - 0.0379261), 0.0001f);


	//ASSERT_LE(std::abs(contactPoints[0].normal.x ), eps);
	//ASSERT_LE(std::abs(contactPoints[0].normal.y + 1.0f), eps);
	//ASSERT_LE(std::abs(contactPoints[0].separation + 0.414214), eps);
}

TEST(CollisionTest, JointTest) {
	BoxRigidBody bodyA = BoxRigidBody({ 0.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, FLT_MAX, 0.0f, { 1.0f,1.0f,1.0f }, false);
	BoxRigidBody bodyB = BoxRigidBody({ 5.0f,0.0f }, { 1.0f, 1.0f }, 0.0f, 1.0f, 0.0f, { 1.0f,1.0f,1.0f }, false);
	bodyA.linear_velocity = { -1.0f, 0.0f };
	bodyA.angular_velocity = 0.0f;
	bodyB.linear_velocity = { 1.0f,0.0f };
	bodyB.angular_velocity = 0.0f;
	bodyA.inv_mass = 0.0f;
	bodyA.inv_inertia = 0.0f;
	bodyA.inertia = FLT_MAX;

	Joint joint(&bodyA, &bodyB, bodyA.position);
	float inv_dt = 60.0f;
	joint.preStep(inv_dt);
	joint.applyImpulse();

	ASSERT_FLOAT_EQ(joint.P.x, -2.0f);
	ASSERT_FLOAT_EQ(joint.P.y, 0.0f);

	ASSERT_FLOAT_EQ(bodyA.linear_velocity.x, -1.0f);
	ASSERT_FLOAT_EQ(bodyA.linear_velocity.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyA.angular_velocity, 0.0f);

	ASSERT_FLOAT_EQ(bodyB.linear_velocity.x, -1.0f);
	ASSERT_FLOAT_EQ(bodyB.linear_velocity.y, 0.0f);
	ASSERT_FLOAT_EQ(bodyB.angular_velocity, 0.0f);
}