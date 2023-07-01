#ifndef __COLLISIONS_HPP__
#define __COLLISIONS_HPP__
#include "Math.hpp"
#include <vector>
#include <utility>
#include "BoxRigidBody.hpp"
class Collisions {
public:
	struct IntersectionRecord {
		bool intersecting = false;
		float depth = 0.0f;
		glm::vec2 axis{0.0f};
	};

	struct CollisionManifold {
		BoxRigidBody& bodyA;
		BoxRigidBody& bodyB;
		float depth = 0.0f;
		glm::vec2 normal{ 0.0f };
		glm::vec2 contact1{ 0.0f };
		glm::vec2 contact2{0.0f};
		size_t contactCount = 0;
		bool intersecting = false;

		CollisionManifold(BoxRigidBody& bodyA, BoxRigidBody& bodyB, bool intersecting = false, float depth = 0.0f, glm::vec2 normal = { 0.0f, 0.0f }, glm::vec2 contact1 = { 0.0f, 0.0f }, glm::vec2 contact2 = { 0.0f, 0.0f }, size_t contactCount = 0) : bodyA(bodyA), bodyB(bodyB), depth(depth), normal(normal), contact1(contact1), contact2(contact2), contactCount(contactCount),intersecting(intersecting) {}
	};
public:
	static IntersectionRecord polygonIntersection(std::vector<glm::vec2> verticesA, std::vector<glm::vec2> verticesB) {
		IntersectionRecord record;
		record.depth = std::numeric_limits<float>::max();
		for (int i = 0; i < verticesA.size(); i++) {
			glm::vec2 va = verticesA[i];
			glm::vec2 vb = verticesA[(i+1)%verticesA.size()];
			glm::vec2 edge = vb - va;
			glm::vec2 axis = { -edge.y, edge.x };
			auto [minA, maxA] = projectVertices(verticesA, axis);
			auto [minB, maxB] = projectVertices(verticesB, axis);

			if (minA >= maxB || minB >= maxA) {
				record.intersecting = false;
				return record;
			}

			float axisDepth = std::min(maxB - minA, maxA - minB);
			
			if (axisDepth < record.depth) {
				record.depth = axisDepth;
				record.axis = axis;
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
				record.intersecting = false;
				return record;
			}

			float axisDepth = std::min(maxB - minA, maxA - minB);

			if (axisDepth < record.depth) {
				record.depth = axisDepth;
				record.axis = axis;
			}
		}

		record.depth = record.depth / glm::length(record.axis);
		record.axis = glm::normalize(record.axis);

		glm::vec2 centerA = calculatePolygonCentroid(verticesA);
		glm::vec2 centerB = calculatePolygonCentroid(verticesB);

		glm::vec2 direction = centerB - centerA;

		if (glm::dot(direction, record.axis) < 0.0f) {
			record.axis = -1.0f * record.axis;
		}

		record.intersecting = true;
		return record;
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