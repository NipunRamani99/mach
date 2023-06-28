#ifndef __BOX_RIGID_BODY_HPP__
#define __BOX_RIGID_BODY_HPP__
#include <glm/glm.hpp>
#include <vector>
struct BoxRigidBody {
	glm::vec2 position_current;
	glm::vec2 position_old;
	glm::vec2 acceleration;
	glm::vec2 size;
	std::vector<glm::vec2> getVertices() {
		//calculate vertex positions
		glm::vec2 half_size = 0.5f * size;
		glm::vec2 tl = position_current - half_size;
		glm::vec2 br = position_current + half_size;
		glm::vec2 tr{br.x, tl.y};
		glm::vec2 bl{tl.x, br.y};
		return { tl,tr,bl,br };
	}
};
#endif