#ifndef _CONTACT_POINT_H_
#define _CONTACT_POINT_H_

#include <glm/glm.hpp>
#include "RigidBody.hpp"
#include "BoxRigidBody.hpp"
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
	RigidBody* body1;
	RigidBody* body2;
	glm::vec2 position;
	glm::vec2 normal;
	glm::vec2 r1, r2;
	float separation;
	float Pn;        // accumulated normal impulse
	float Pt;        // accumulated tangent impulse
	float Pnb;       // accumulated normal impulse for position bias
	float massNormal, massTangent;
	float bias;
	FeaturePair feature;
};

static std::vector<ContactPoint> Collide(BoxRigidBody* bodyA, BoxRigidBody* bodyB);
#endif