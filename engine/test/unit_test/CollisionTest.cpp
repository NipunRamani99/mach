#include <gtest/gtest.h>
#include "include/Collision.hpp"

/*
* Create 2 overlapping polygons and check if they intersect using Collisions.intersectPolygon
* 
* Triangle A: (0.0f, 0.0f), (0.5f, 1.0f), (1.0f,0.0f)
* Triangle B: (0.5f,0.5f), (2.0f,2.0f),  (2.0f,0.0f)
*/
TEST(CollisonTest, IntersectingPolygons) {
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

TEST(CollisonTest, NonIntersectingPolygons) {
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

TEST(CollisonTest, IntersectingSquare) {
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
