#ifndef __MATH_HPP__
#define __MATH_HPP__
#include <glm/glm.hpp>
#include <vector>
const float eps = 0.0001f;
static glm::vec2 rotateVec2Radians(glm::vec2 vertex, float radians) {
	float _cos = glm::cos(radians);
	float _sin = glm::sin(radians);
	float rx = vertex.x * _cos - vertex.y * _sin;
	float ry = vertex.x * _sin + vertex.y * _cos;
	return glm::vec2(rx, ry);
}

static glm::vec2 rotateVec2Degrees(glm::vec2 vertex, float degrees) {
	return rotateVec2Radians(vertex, degrees * 0.0174533f);
}

static glm::vec2 calculatePolygonCentroid(std::vector<glm::vec2>& vertices) {
	glm::vec2 centroid{0.0f, 0.0f};

	for (glm::vec2& pt : vertices) {
		centroid += pt;
	}
	centroid /= glm::vec2(vertices.size(), vertices.size());
	return centroid;
}

#endif