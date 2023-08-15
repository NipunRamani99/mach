#include <gtest/gtest.h>
#include "include/BoxRigidBody.hpp"
TEST(BoxRigidBody, getVertices) {
	glm::vec2 position_current{100.0f, 100.0f};
	glm::vec2 size{10.0f, 10.0f};
	
	std::vector<glm::vec2> expected = {
		{95.0f,105.0f},
		{95.0f,95.0f},
		{105.0f,95.0f},	
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

