#include <gtest/gtest.h>
#include "include/CircleRigidBody.hpp"

TEST(CircleRigidBody, checkIfInside) {
	glm::vec2 position_current{100.0f, 100.0f};
	glm::vec2 p_inside{100.0f, 100.0f};
	glm::vec2 p_outside{200.0f, 100.0f};
	float radius = 10.0f;
	float rotation = 0.0f;
	float mass = 1.0f;
	float restitution = 0.6f;
	glm::vec3 color{255.0f, 0.0f, 0.0f};
	CircleRigidBody circleRigidBody(position_current,radius,rotation, mass, restitution, color, false);
	ASSERT_TRUE(circleRigidBody.checkIfInside(p_inside));
	ASSERT_FALSE(circleRigidBody.checkIfInside(p_outside));
}

