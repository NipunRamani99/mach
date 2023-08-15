#ifndef __MATH_HPP__
#define __MATH_HPP__
#include <glm/glm.hpp>
#include <vector>
const float eps = 0.0001f;
const glm::vec2 gravity{0.0f, 500.0f};
const float pi = 3.141592653;
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

static float cross(glm::vec2 a, glm::vec2 b) {
	return a.x * b.y - a.y * b.x;
}

static glm::vec2 cross(float a, glm::vec2 b) {
	return { -a * b.y, a * b.x };
}
static glm::mat2 rotationMatrix(float radians) {
	float s = glm::sin(radians);
	float c = glm::cos(radians);
	return {c, -s, s, c};
}

static glm::vec2 applyRotationMatrix(glm::vec2 v, glm::mat2 rot) {
	return {
		rot[0][0] * v.x - rot[0][1] * v.y,
		rot[1][0] * v.x + rot[1][1] * v.y
	};
}

static glm::mat2 invert(glm::mat2& A) {
	glm::mat2 B;
	float a = A[0][0], b = A[0][1], c = A[1][0], d = A[1][1];
	float det = a*d - b*c;
	assert(det != 0.0f);
	det = 1.0f / det;
	B[0][0] = d * det;
	B[0][1] = -b * det;
	B[1][0] = -c * det;
	B[1][1] = a * det;
	return B;
}

#endif