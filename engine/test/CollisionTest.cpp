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
	Collisions::IntersectionRecord intersecting = Collisions::polygonIntersection(verticesA, verticesB);
	ASSERT_TRUE(intersecting.intersecting);
	ASSERT_LE(float(std::abs(intersecting.depth)), 0.03);
	ASSERT_EQ(intersecting.axis.x, 0.0f);
	ASSERT_EQ(intersecting.axis.y, 1.0f);
}
