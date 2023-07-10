#ifndef __COLLISIONS_HPP__
#define __COLLISIONS_HPP__
#include "Math.hpp"
#include <vector>
#include <utility>
#include <array>
#include "BoxRigidBody.hpp"
#include "CircleRigidBody.hpp"
#include "ContactPoint.hpp"
class Collisions {
public:
	struct IntersectionRecord {
		bool intersecting = false;
		float depth = 0.0f;
		glm::vec2 axis{0.0f};
	};

	struct CollisionManifold {
		RigidBody * bodyA;
		RigidBody * bodyB;
		std::vector<ContactPoint> contacts;


		CollisionManifold(RigidBody* bodyA, RigidBody* bodyB) 
			:
			bodyA(bodyA),
			bodyB(bodyB)
		{
			contacts = Collide((BoxRigidBody*)bodyA, (BoxRigidBody*)bodyB);
		}

		void preStep(float dt) {
			const float allowedPenetration = 0.01f;
			float biasFactor = 0.2f;
			//std::array<glm::vec2, 2> contacts = { contact1, contact2 };
			for (int i = 0; i < contacts.size(); i++) {
				ContactPoint& c = contacts[i];
				c.r1 = contacts[i].position - bodyA->position;
				c.r2 = contacts[i].position - bodyB->position;

				//precompute normal mass, tangent mass, and bias
				float rn1 = glm::dot(c.r1, c.normal);
				float rn2 = glm::dot(c.r2, c.normal);
				float kNormal = bodyA->inv_mass + bodyB->inv_mass;
				kNormal += bodyA->inv_inertia * (glm::dot(c.r1, c.r1) - rn1 * rn1) + bodyB->inv_inertia * (glm::dot(c.r2, c.r2) - rn2 * rn2);
				c.massNormal = 1.0f / kNormal;

				glm::vec2 tangent = { -c.normal.y, c.normal.x };
				float rt1 = glm::dot(c.r1, tangent);
				float rt2 = glm::dot(c.r2, tangent);
				float kTangent = bodyA->inv_mass + bodyB->inv_mass;
				kTangent += bodyA->inv_inertia * (glm::dot(c.r1, c.r1) - rt1 * rt1) + bodyB->inv_inertia * (glm::dot(c.r2, c.r2) - rt2 * rt2);
				c.massTangent = 1.0f / kTangent;
				c.bias = -biasFactor / dt * std::min(0.0f, (c.separation) + allowedPenetration);

				glm::vec2 P = c.Pn * c.normal + c.Pt * tangent;
				bodyA->linear_velocity -= P * bodyA->inv_mass;
				bodyA->angular_velocity -= bodyA->inv_inertia * cross(c.r1, P);
				bodyB->linear_velocity += P * bodyB->inv_mass;
				bodyB->angular_velocity += bodyB->inv_inertia * cross(c.r2, P);
			}
		}
		glm::vec2 _cross(float a, glm::vec2 b) {
			glm::vec2 result;
			result.x = -a * b.y;
			result.y = a * b.x;
			return result;
		}
		void applyImpulse() {

			for (int i = 0; i < contacts.size(); i++) {
				ContactPoint & c = contacts[i];
				c.r1 = c.position - bodyA->position;
				c.r2 = c.position - bodyB->position;

				//relative velocity at contact
				glm::vec2 dv = bodyB->linear_velocity + _cross(bodyB->angular_velocity, c.r2) - bodyA->linear_velocity - _cross(bodyA->angular_velocity, c.r1);

			
				//compute normal impulse
				float vn = glm::dot(dv, c.normal);

				float dPn = c.massNormal * (-vn + c.bias );

				
				float Pn0 = c.Pn;
				c.Pn = std::max(Pn0 + dPn, 0.0f);
				dPn = c.Pn - Pn0;

				//apply contact impulse
				glm::vec2 _Pn =  dPn * c.normal;
				bodyA->linear_velocity -= _Pn * bodyA->inv_mass;
				bodyA->angular_velocity -= bodyA->inv_inertia * cross(c.r1, _Pn);
				bodyB->linear_velocity += _Pn * bodyB->inv_mass;
	    		bodyB->angular_velocity += bodyB->inv_inertia * cross(c.r2, _Pn);


				//relative velocity at contact
				dv = bodyB->linear_velocity + _cross(bodyB->angular_velocity, c.r2) - bodyA->linear_velocity - _cross(bodyA->angular_velocity, c.r1);

				//compute tangent impulse
				float vt = glm::dot(dv, { -c.normal.y, c.normal.x });
				float dPt = c.massTangent * (-vt);

				float friction = std::sqrtf(bodyA->static_friction * bodyB->static_friction);

				float maxPt =  friction* c.Pn;
				float oldTangentImpulse = c.Pt;
				c.Pt = glm::clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
				dPt = c.Pt - oldTangentImpulse;

				
				//apply contact impulse
				glm::vec2 _Pt = dPt * glm::vec2{ -c.normal.y, c.normal.x };
				bodyA->linear_velocity -= _Pt * bodyA->inv_mass;
				bodyA->angular_velocity -= bodyA->inv_inertia * cross(c.r1, _Pt);
				bodyB->linear_velocity += _Pt * bodyB->inv_mass;
				bodyB->angular_velocity += bodyB->inv_inertia * cross(c.r2, _Pt);
			}
		
		}

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
	
	static IntersectionRecord findIntersection(RigidBody* a, RigidBody* b) {
		IntersectionRecord intersectionRecord;
		if (a->type == RigidBody::Type::CIRCLE && b->type == RigidBody::Type::CIRCLE) {
			CircleRigidBody* circleA = static_cast<CircleRigidBody*>(a);
			CircleRigidBody* circleB = static_cast<CircleRigidBody*>(b);
			auto [intersecting, depth, axis] = circleIntersection(*circleA, *circleB);
			intersectionRecord = { intersecting, depth, axis };
		}
		else if (a->type == RigidBody::Type::CIRCLE && b->type == RigidBody::Type::BOX) {
			CircleRigidBody* circleA = static_cast<CircleRigidBody*>(a);
			BoxRigidBody* boxB = static_cast<BoxRigidBody*>(b);
			auto [intersecting, depth, axis] = circlePolygonIntersection(*circleA, *boxB);
			intersectionRecord = { intersecting, depth, axis };
		}
		else if (a->type == RigidBody::Type::BOX && b->type == RigidBody::Type::CIRCLE) {
			CircleRigidBody* circleB = static_cast<CircleRigidBody*>(b);
			BoxRigidBody* boxA = static_cast<BoxRigidBody*>(a);
			auto [intersecting,depth,axis] = circlePolygonIntersection(*circleB, *boxA);
			intersectionRecord = { intersecting, depth, axis };
		}
		else if (a->type == RigidBody::Type::BOX && b->type == RigidBody::Type::BOX) {
			BoxRigidBody* boxA = static_cast<BoxRigidBody*>(a);
			BoxRigidBody* boxB = static_cast<BoxRigidBody*>(b);
			intersectionRecord = polygonIntersection(boxA->getVertices(), boxB->getVertices());
		}
		return intersectionRecord;
	}

	static glm::vec2 findCirclePolygonContactPoint(CircleRigidBody & circleRigidBodyA, BoxRigidBody & boxRigidBodyB) {
		glm::vec2 contactPoint = { 0.0f, 0.0f };
		glm::vec2 center = circleRigidBodyA.position;
		std::vector<glm::vec2> vertices = boxRigidBodyB.getVertices();
		float minDistance = std::numeric_limits<float>::max();
		for (size_t i = 0; i < vertices.size(); i++) {
			glm::vec2 va = vertices[i];
			glm::vec2 vb = vertices[(i + 1) % vertices.size()];
			auto [distance, contact] = pointSegmentDistance(center, va, vb);
			if (distance < minDistance) {
				minDistance = distance;
				contactPoint = contact;
			}
		}
		return contactPoint;
	}

	static void findContactPoint(CollisionManifold& collisionManifold) {
		//RigidBody * rigidBodyA = collisionManifold.bodyA;
		//RigidBody * rigidBodyB = collisionManifold.bodyB;
		//if(rigidBodyA->type == RigidBody::Type::CIRCLE && rigidBodyB->type == RigidBody::Type::CIRCLE) {
		//	CircleRigidBody* circleRigidBodyA = static_cast<CircleRigidBody*>(rigidBodyA);
		//	CircleRigidBody* circleRigidBodyB = static_cast<CircleRigidBody*>(rigidBodyB);
		//	glm::vec2 contactPoint = findCirclesContactPoint(*circleRigidBodyA, *circleRigidBodyB);
		//	collisionManifold.contact1 = contactPoint;
		//	collisionManifold.contactCount = 1;
		//}
		//else if (rigidBodyA->type == RigidBody::Type::CIRCLE && rigidBodyB->type == RigidBody::Type::BOX) {
		//	CircleRigidBody* circleRigidBodyA = static_cast<CircleRigidBody*>(rigidBodyA);
		//	BoxRigidBody* boxRigidBodyB = static_cast<BoxRigidBody*>(rigidBodyB);
		//	glm::vec2 contactPoint = findCirclePolygonContactPoint(*circleRigidBodyA, *boxRigidBodyB);
		//	collisionManifold.contact1 = contactPoint;
		//	collisionManifold.contactCount = 1;
		//}
		//else if (rigidBodyA->type == RigidBody::Type::BOX && rigidBodyB->type == RigidBody::Type::CIRCLE) {
		//	BoxRigidBody* boxRigidBodyA = static_cast<BoxRigidBody*>(rigidBodyA);
		//	CircleRigidBody* circleRigidBodyB = static_cast<CircleRigidBody*>(rigidBodyB);
		//	glm::vec2 contactPoint = findCirclePolygonContactPoint(*circleRigidBodyB, *boxRigidBodyA);
		//	collisionManifold.contact1 = contactPoint;
		//	collisionManifold.contactCount = 1;
		//}
		//else if (rigidBodyA->type == RigidBody::Type::BOX && rigidBodyB->type == RigidBody::Type::BOX) {
		//	BoxRigidBody* boxRigidBodyA = static_cast<BoxRigidBody*>(rigidBodyA);
		//	BoxRigidBody* boxRigidBodyB = static_cast<BoxRigidBody*>(rigidBodyB);
		//	std::vector<glm::vec2> verticesA = boxRigidBodyA->getVertices();
		//	std::vector<glm::vec2> verticesB = boxRigidBodyB->getVertices();
		//	findPolygonContactPoints(verticesA, verticesB, collisionManifold);
		//}
	}

	static std::pair<float, glm::vec2> pointSegmentDistance(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
		glm::vec2 contact = a;
		glm::vec2 ab = b - a;
		glm::vec2 pa = p - a;
		float t = glm::dot(pa, ab) / glm::dot(ab, ab);
		if (t <= 0.0f) {
			contact = a;
		} else if (t >= 1.0f) {
			contact = b;
		}
		else {
			contact = a + t * ab;
		}
		return { glm::length(contact - p), contact };
	}

	static glm::vec2 findCirclesContactPoint(CircleRigidBody& bodyA, CircleRigidBody& bodyB) {
		glm::vec2 ab = bodyB.position - bodyA.position;
		glm::vec2 dir = glm::normalize(ab);
		return bodyA.position + bodyA.radius * dir;
	}

	static void findPolygonContactPoints(std::vector<glm::vec2>& verticesA, std::vector<glm::vec2>& verticesB, CollisionManifold& collisionManifold) {
		glm::vec2 contactPoint1 = { 0.0f, 0.0f };
		glm::vec2 contactPoint2 = { 0.0f, 0.0f };
		int contactPoints = 0;
		float minDistance = std::numeric_limits<float>::max();
		for (size_t i = 0; i < verticesA.size(); i++) {
			glm::vec2 p = verticesA[i];
			for (size_t j = 0; j < verticesB.size(); j++) {
				glm::vec2 va = verticesB[j];
				glm::vec2 vb = verticesB[(j + 1) % verticesB.size()];
				auto [distance, contact] = pointSegmentDistance(p, va, vb);
				if (std::abs(distance - minDistance) < 0.1f && contactPoint1 != contact) {
					contactPoint2 = contact;
					contactPoints = 2;
				}else if(distance < minDistance) {
					minDistance = distance;
					contactPoint1 = contact;
					contactPoints = 1;
				} 
			}
		}
		for (size_t i = 0; i < verticesB.size(); i++) {
			glm::vec2 p = verticesB[i];
			for (size_t j = 0; j < verticesA.size(); j++) {
				glm::vec2 va = verticesA[j];
				glm::vec2 vb = verticesA[(j + 1) % verticesA.size()];
				auto [distance, contact] = pointSegmentDistance(p, va, vb);
				if (std::abs(distance - minDistance) < 0.1f && contactPoint1 != contact) {
					contactPoint2 = contact;
					contactPoints = 2;
				}
				else if (distance < minDistance) {
					minDistance = distance;
					contactPoint1 = contact;
					contactPoints = 1;
				}
			}
		}
		//collisionManifold.contact1 = contactPoint1;
		//collisionManifold.contact2 = contactPoint2;
		//collisionManifold.contactCount = contactPoints;
	}

	static std::tuple<bool,float, glm::vec2> circleIntersection(CircleRigidBody& bodyA, CircleRigidBody& bodyB) {
		glm::vec2 normal = { 0.0f,0.0f };
		float depth = 0.0f;
		float distance = glm::length(bodyA.position - bodyB.position);
		float radiusSum = bodyA.radius + bodyB.radius;
		if (distance > radiusSum) {
			return { false, 0.0f, {0.0f,0.0f} };
		}
		else {
			normal = glm::normalize(bodyB.position - bodyA.position);
			depth = radiusSum - distance;
			return { true, depth, normal };
		}
	}

	static std::tuple<bool, float, glm::vec2> circlePolygonIntersection(CircleRigidBody& bodyA, BoxRigidBody& bodyB) {
		glm::vec2 normal = { 0.0f,0.0f };
		float depth = std::numeric_limits<float>::max();
		std::vector<glm::vec2> vertices = bodyB.getVertices();
		glm::vec2 axis = glm::vec2(0.0f, 0.0f);
		float axisDepth = 0.0f;
		for (size_t i = 0; i < vertices.size(); i++) {
			glm::vec2 va = vertices[i];
			glm::vec2 vb = vertices[(i + 1) % vertices.size()];
			glm::vec2 edge = vb - va;
			glm::vec2 edgeNormal = glm::normalize(glm::vec2(-edge.y, edge.x));
			auto [minA, maxA] = projectCircle(bodyA, edgeNormal);
			auto [minB, maxB] = projectVertices(vertices, edgeNormal);
			if (minA >= maxB || minB >= maxA) {
				return { false, depth, normal };
			}
			axisDepth = std::min(maxB - minA, maxA - minB);
			if (axisDepth < depth) {
				depth = axisDepth;
				normal = edgeNormal;
			}
		}
		int cpIndex = findClosestPointOnPolygon(vertices, bodyA.position);
		glm::vec2 cp = vertices[cpIndex];
		axis = glm::normalize(cp - bodyA.position);

		auto [min1,max1] = projectVertices(vertices, axis);
		auto [min2,max2] = projectCircle(bodyA, axis);

		if(min1 >= max2 || min2 >= max1) {
			return { false, depth, normal };
		}
		axisDepth = std::min(max2 - min1, max1 - min2);
		if (axisDepth < depth) {
			depth = axisDepth;
			normal = axis;
		}

		glm::vec2 direction = glm::normalize(bodyB.position - bodyA.position);

		if(glm::dot(direction, normal) < 0.0f) {
			normal = -normal;
		}

		return { true, depth, normal };
	}

	static int findClosestPointOnPolygon(std::vector<glm::vec2>& vertices, glm::vec2 point) {
		float minDistance = std::numeric_limits<float>::max();
		int closestPoint = -1;
		for (size_t i = 0; i < vertices.size(); i++) {
			float distance = glm::length(vertices[i] - point);
			if (distance < minDistance) {
				minDistance = distance;
				closestPoint = i;
			}
		}
		return closestPoint;
	}

	static std::pair<float, float> projectCircle(CircleRigidBody& body, glm::vec2 axis) {
		glm::vec2 direction = glm::normalize(axis);
		glm::vec2 directionAndRadius = direction * body.radius;
		glm::vec2 p1 = body.position + directionAndRadius;
		glm::vec2 p2 = body.position - directionAndRadius;
		float min = std::min(glm::dot(p1, axis), glm::dot(p2, axis));
		float max = std::max(glm::dot(p1, axis), glm::dot(p2, axis));
		return { min,max };
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