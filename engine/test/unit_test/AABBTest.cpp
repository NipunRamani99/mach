#include <gtest/gtest.h>
#include "include/AABB.hpp"

TEST(AABB, isOverlapping) {
	AABB box1;
	AABB box2;
	box1.position = glm::vec2(0, 0);
	box1.size = glm::vec2(4, 4);
	box2.position = glm::vec2(2, 2);
	box2.size = glm::vec2(6, 6);
	ASSERT_TRUE(box1.isOverlapping(box2));
	AABB box3 = { glm::vec2(10, 10), glm::vec2(4, 4) };
	AABB box4 = { glm::vec2(15, 15), glm::vec2(4, 4) };
	ASSERT_FALSE(box3.isOverlapping(box4));
}