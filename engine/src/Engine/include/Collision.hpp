#ifndef __COLLISIONS_HPP__
#define __COLLISIONS_HPP__
#include "Math.hpp"
#include <vector>
#include <utility>
class Collisions {
public:
	static bool polygonIntersection(std::vector<glm::vec2> verticesA, std::vector<glm::vec2> verticesB) {
		for (int i = 0; i < verticesA.size(); i++) {
			glm::vec2 va = verticesA[i];
			glm::vec2 vb = verticesA[(i+1)%verticesA.size()];
			glm::vec2 edge = vb - va;
			glm::vec2 axis = { -edge.y, edge.x };
			std::pair<float, float> overlapA = projectVertices(verticesA, axis);
			std::pair<float, float> overlapB = projectVertices(verticesB, axis);

			float minA = overlapA.first;
			float maxA = overlapA.second;
			float minB = overlapB.first;
			float maxB = overlapB.second;

			if (minA >= maxB || minB >= maxA) {
				return false;
			}
		}

		for (int i = 0; i < verticesB.size(); i++) {
			glm::vec2 va = verticesB[i];
			glm::vec2 vb = verticesB[(i + 1) % verticesB.size()];
			glm::vec2 edge = vb - va;
			glm::vec2 axis = { -edge.y, edge.x };
			std::pair<float, float> overlapA = projectVertices(verticesA, axis);
			std::pair<float, float> overlapB = projectVertices(verticesB, axis);

			float minA = overlapA.first;
			float maxA = overlapA.second;
			float minB = overlapB.first;
			float maxB = overlapB.second;

			if (minA >= maxB || minB >= maxA) {
				return false;
			}
		}
	}

	static std::pair<float,float> projectVertices(std::vector<glm::vec2>& vertices, glm::vec2 axis) {
		float min = std::numeric_limits<float>::max();
		float max = std::numeric_limits<float>::min();

		for (int i = 0; i < vertices.size();i++) {
			glm::vec2 v = vertices[i];
			float proj = glm::dot(v, axis);
			if (proj < min) { min = proj; };
			if (proj > max) {
				max = proj;
			}
		}
		return { min,max };
	}
};

#endif