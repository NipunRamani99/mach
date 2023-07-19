#include "include/ContactPoint.hpp"


enum Axis
{
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

enum EdgeNumbers
{
	NO_EDGE = 0,
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4
};

struct ClipVertex
{
	ClipVertex() { fp.value = 0; }
	glm::vec2 v;
	FeaturePair fp;
};

void Flip(FeaturePair& fp)
{
	std::swap(fp.e.inEdge1, fp.e.inEdge2);
	std::swap(fp.e.outEdge1, fp.e.outEdge2);
}

glm::vec2 abs(glm::vec2 v) {
	return { fabs(v.x), fabs(v.y) };
}
glm::mat2 abs(glm::mat2 m) {
	return { abs(m[0]),  abs(m[1]) };
}

glm::mat2 rotatation_matrix2(float angle) {
	float c = cos(angle);
	float s = sin(angle);
	return glm::mat2(c, -s, s, c);
}

int clipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2], const glm::vec2& normal, float offset, char clipEdge) {
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	float distance0 = glm::dot(normal, vIn[0].v) - offset;
	float distance1 = glm::dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge;
			vOut[numOut].fp.e.inEdge2 = NO_EDGE;
		}
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge;
			vOut[numOut].fp.e.outEdge2 = NO_EDGE;
		}
		++numOut;
	}

	return numOut;
}

static void ComputeIncidentEdge(ClipVertex c[2], const glm::vec2& h, const glm::vec2& pos,
	const glm::mat2& rot, const glm::vec2& normal)
{
	glm::mat2 rotT = glm::transpose(rot);
	glm::vec2 n = -(normal * rotT);
	glm::vec2 nAbs(glm::abs(n.x), glm::abs(n.y));

	if (nAbs.x > nAbs.y) {
		if (glm::sign(n.x) > 0.0f) {
			c[0].v = { h.x, -h.y };
			c[0].fp.e.inEdge2 = EDGE3;
			c[0].fp.e.outEdge2 = EDGE4;

			c[1].v = { h.x, h.y };
			c[1].fp.e.inEdge2 = EDGE4;
			c[1].fp.e.outEdge2 = EDGE1;
		}
		else {
			c[0].v = { -h.x, h.y };
			c[0].fp.e.inEdge2 = EDGE1;
			c[0].fp.e.outEdge2 = EDGE2;

			c[1].v = { -h.x, -h.y };
			c[1].fp.e.inEdge2 = EDGE2;
			c[1].fp.e.outEdge2 = EDGE3;
		}
	}
	else {
		if (glm::sign(n.y) > 0.0f) {
			c[0].v = { h.x, h.y };
			c[0].fp.e.inEdge2 = EDGE4;
			c[0].fp.e.outEdge2 = EDGE1;

			c[1].v = { -h.x, h.y };
			c[1].fp.e.inEdge2 = EDGE1;
			c[1].fp.e.outEdge2 = EDGE2;
		}
		else {
			c[0].v = { -h.x, -h.y };
			c[0].fp.e.inEdge2 = EDGE2;
			c[0].fp.e.outEdge2 = EDGE3;

			c[1].v = { h.x, -h.y };
			c[1].fp.e.inEdge2 = EDGE3;
			c[1].fp.e.outEdge2 = EDGE4;

		}
	}
	c[0].v = pos + c[0].v * rot;
	c[1].v = pos + c[1].v * rot;
}



std::vector<ContactPoint> Collide(BoxRigidBody* bodyA, BoxRigidBody* bodyB) {

	//setup
	glm::vec2 hA = 0.5f * bodyA->size;
	glm::vec2 hB = 0.5f * bodyB->size;

	glm::vec2 pA = bodyA->position;
	glm::vec2 pB = bodyB->position;

	glm::mat2 rotA = rotatation_matrix2(bodyA->angle);
	glm::mat2 rotB = rotatation_matrix2(bodyB->angle);

	glm::mat2 rotAT = glm::transpose(rotA);
	glm::mat2 rotBT = glm::transpose(rotB);

	glm::vec2 dp = pB - pA;
	glm::vec2 dA = dp * rotAT ;
	glm::vec2 dB = dp * rotBT;

	glm::mat2 C = rotAT * rotB;
	glm::mat2 absC = abs(C);
	glm::mat2 absCT = glm::transpose(absC);

	//box A faces
	glm::vec2 faceA = glm::abs(dA) - hA - hB * absC;
	if (faceA.x > 0.0f || faceA.y > 0.0f) {
		return std::vector<ContactPoint>();
	}

	//box B faces
	glm::vec2 faceB = glm::abs(dB) - hB - hA * absCT;
	if (faceB.x > 0.0f || faceB.y > 0.0f) {
		return std::vector<ContactPoint>();
	}
	//return { faceA, faceB };


	//find best axis
	Axis axis;
	float separation;
	glm::vec2 normal;

	//box A faces
	axis = FACE_A_X;
	separation = faceA.x;
		normal = dA.x > 0.0f ? rotAT[0] : -rotAT[0];
	
	//return { normal, {0.0f,0.0f} };

	float relativeTol = 0.95f;
	const float absoluteTol = 0.01f;

	if (faceA.y > relativeTol * separation + absoluteTol * hA.y) {
		axis = FACE_A_Y;
		separation = faceA.y;
		normal = dA.y > 0.0f ? rotAT[1] : -rotAT[1];
	}

	//box B faces
	if (faceB.x > relativeTol * separation + absoluteTol * hB.x) {
		axis = FACE_B_X;
		separation = faceB.x;
		normal = dB.x > 0.0f ? rotBT[0] : -rotBT[0];
	}
	if (faceB.y > relativeTol * separation + absoluteTol * hB.y) {
		axis = FACE_B_Y;
		separation = faceB.y;
		normal = dB.y > 0.0f ? rotBT[1] : -rotBT[1];
	}

//	return { normal, {0.0f,0.0f} };

	//setup clipping plane data based on the separating axis
	glm::vec2 frontNormal, sideNormal;
	ClipVertex incidentEdge[2];
	float front, negSide, posSide;
	char negEdge, posEdge;

	// Compute the clipping lines and the line segment to be clipped.
	switch (axis) {
	case FACE_A_X:
	{
		frontNormal = normal;
		front = glm::dot(pA, frontNormal) + hA.x;
		sideNormal = rotAT[1];
		float side = glm::dot(pA, sideNormal);
		negSide = -side + hA.y;
		posSide = side + hA.y;
		negEdge = EDGE3;
		posEdge = EDGE1;
		ComputeIncidentEdge(incidentEdge, hB, pB, rotB, frontNormal);

		break;
	}
	case FACE_A_Y: {
		frontNormal = normal;
		front = glm::dot(pA, frontNormal) + hA.y;
		sideNormal = rotAT[0];
		float side = glm::dot(pA, sideNormal);
		negSide = -side + hA.x;
		posSide = side + hA.x;
		negEdge = EDGE2;
		posEdge = EDGE4;
		ComputeIncidentEdge(incidentEdge, hB, pB, rotB, frontNormal);
		break;
	}
	case FACE_B_X: {
		frontNormal = -normal;
		front = glm::dot(pB, frontNormal) + hB.x;
		sideNormal = rotBT[1];
		float side = glm::dot(pB, sideNormal);
		negSide = -side + hB.y;
		posSide = side + hB.y;
		negEdge = EDGE3;
		posEdge = EDGE1;
		ComputeIncidentEdge(incidentEdge, hA, pA, rotA, frontNormal);
		break;
	}
	case FACE_B_Y: {
		frontNormal = -normal;
		front = glm::dot(pB, frontNormal) + hB.y;
		sideNormal = rotBT[0];
		float side = glm::dot(pB, sideNormal);
		negSide = -side + hB.x;
		posSide = side + hB.x;
		negEdge = EDGE2;
		posEdge = EDGE4;
		ComputeIncidentEdge(incidentEdge, hA, pA, rotA, frontNormal);
		break;
	}
	}

	ClipVertex clipPoints1[2];
	ClipVertex clipPoints2[2];
	int np; 

	np = clipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

//	return {incidentEdge[0].v, incidentEdge[1].v};

	if (np < 2)
		return {};

	np = clipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

	if (np < 2)
		return {};

	std::vector<ContactPoint> contacts;
	for (int i = 0; i < 2; i++) {
		float seperation = glm::dot(frontNormal, clipPoints2[i].v) - front;
		if (seperation <= 0.0f) {
			ContactPoint contactPoint;
			contactPoint.separation = separation;
			contactPoint.normal = normal;
			contactPoint.position = clipPoints2[i].v - separation * frontNormal;
			contactPoint.feature = clipPoints2[i].fp;
			if (axis == FACE_B_X || axis == FACE_B_Y)
				Flip(contactPoint.feature);
			contacts.push_back(contactPoint);
		}
	}

	return contacts;
}
