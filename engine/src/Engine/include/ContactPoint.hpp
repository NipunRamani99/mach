#ifndef _CONTACT_POINT_H_
#define _CONTACT_POINT_H_

#include <glm/glm.hpp>
#include "RigidBody.hpp"
#include "BoxRigidBody.hpp"
#include "CircleRigidBody.hpp"
#include <vector>

union FeaturePair
{
	struct Edges
	{
		char inEdge1;
		char outEdge1;
		char inEdge2;
		char outEdge2;
	} e;
	int value;
};

/*
* Taken from Box2D Lite
*/
struct ContactPoint {
public:
	glm::vec2 position = glm::vec2{ 0.0f,0.0f };
	glm::vec2 normal = glm::vec2{ 0.0f,1.0f };
	glm::vec2 r1 = glm::vec2{ 0.0f, 0.0f }, r2 = glm::vec2{ 0.0f };
	float separation = 0.0f;
	float Pn = 0.0f;        // accumulated normal impulse
	float Pt = 0.0f;        // accumulated tangent impulse
	float Pnb = 0.0f;       // accumulated normal impulse for position bias
	float massNormal = 0.0f, massTangent = 0.0f;
	float bias = 0.0f;
	FeaturePair feature;
};

std::vector<ContactPoint> Collide(BoxRigidBody* bodyA, BoxRigidBody* bodyB);
std::vector<ContactPoint> Collide(BoxRigidBody* bodyA, CircleRigidBody* bodyB);
std::vector<ContactPoint> Collide(CircleRigidBody* bodyA, CircleRigidBody* bodyB);

#endif