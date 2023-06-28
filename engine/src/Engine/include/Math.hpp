#ifndef __MATH_HPP__
#define __MATH_HPP__
#include <glm/glm.hpp>

static glm::vec2 rotate_vec2(glm::vec2 vertex, float radians) {
	float _cos = glm::cos(radians);
	float _sin = glm::sin(radians);
	float rx = vertex.x * _cos - vertex.y * _sin;
	float ry = vertex.x * _sin + vertex.y * _cos;
	return glm::vec2(rx, ry);
}

#endif