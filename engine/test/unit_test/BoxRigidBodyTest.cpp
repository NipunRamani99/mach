#include <gtest/gtest.h>
#include "include/BoxRigidBody.hpp"
TEST(BoxRigidBody, getVertices) {
	glm::vec2 position_current{100.0f, 100.0f};
	glm::vec2 size{10.0f, 10.0f};
	
	std::vector<glm::vec2> expected = {
		{95.0f,95.0f},
		{105.0f,95.0f},
		{95.0f,105.0f},
		{105.0f,105.0f},
	};
	
	BoxRigidBody boxRigidBody;
	boxRigidBody.position = position_current;
	boxRigidBody.size = size;
	std::vector<glm::vec2> actual = boxRigidBody.getVertices();
	for (size_t i = 4; i--;) {
		ASSERT_EQ(expected[i].x, actual[i].x);
		ASSERT_EQ(expected[i].y, actual[i].y);
	}
}

TEST(BoxRigidBody, checkIfInside) {
	glm::vec2 position_current{100.0f, 100.0f};
	glm::vec2 size{10.0f, 10.0f};

	std::vector<glm::vec2> expected = {
		{95.0f,95.0f},
		{105.0f,95.0f},
		{95.0f,105.0f},
		{105.0f,105.0f},
	};

	BoxRigidBody boxRigidBody;
	boxRigidBody.position = position_current;
	boxRigidBody.size = size;
	glm::vec2 p {110, 100};
	ASSERT_FALSE(boxRigidBody.checkIfInside(p));
}


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