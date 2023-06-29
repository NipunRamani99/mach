#include "include/Math.hpp"
#include <gtest/gtest.h>

TEST(MathTest, test_rotate_vec2_radians) {
	glm::vec2 vertex{1.0f, 0.0f};
	float rotate_angle = 3.141592653f / 2.0f;

	glm::vec2 rotated = rotateVec2Radians(vertex, rotate_angle);
	ASSERT_LE(std::abs(rotated.y - 1.0f), eps);
	ASSERT_LE(std::abs(rotated.x), eps);
}

TEST(MathTest, test_rotate_vec2_degrees) {
	glm::vec2 vertex{1.0f, 0.0f};
	float rotate_angle = 90.0f;
	glm::vec2 rotated = rotateVec2Degrees(vertex, 90.0f);
	ASSERT_LE(std::abs(rotated.y - 1.0f), eps);
	ASSERT_LE(std::abs(rotated.x), eps);
}

TEST(MathTest, test_calculatePolygonCentroid) {
	// Set up common data for all test cases
	std::vector<glm::vec2> vertices = {glm::vec2(1.0f, 1.0f), glm::vec2(2.0f, 3.0f), glm::vec2(4.0f, 2.0f)};
	glm::vec2 centroid = calculatePolygonCentroid(vertices);
	EXPECT_LE(std::abs(centroid.x - 2.33333f), eps);
	EXPECT_LE(std::abs(centroid.y - 2.0f), eps);
}