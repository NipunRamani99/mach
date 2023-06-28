#ifndef __PHYSICS_OBJECT_HPP__
#define __PHYSICS_OBJECT_HPP__
#include "Math.hpp"
struct PhysicsObject {
	glm::vec2 position_current;
	glm::vec2 position_old;
	glm::vec2 acceleration;
	PhysicsObject();
};
#endif