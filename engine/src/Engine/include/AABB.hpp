#ifndef __AABB_HPP__
#define __AABB_HPP__
#include <glm/glm.hpp>
struct AABB {
	glm::vec2 position;
	glm::vec2 size;

	bool isInside(glm::vec3 p) {
		glm::vec2 u_left = position;
		glm::vec2 l_right = position + size;
		return (p.x >= u_left.x && p.x <= l_right.x) && (p.y >= u_left.y && p.y <= l_right.y);
	}
	
	bool isOverlapping(AABB& b) {
		glm::vec2 b_ul = b.position;
		glm::vec2 b_lr = b.position + b.size;
		glm::vec2 u_left = position;
		glm::vec2 l_right = position + size;
		// Check for overlap along the x-axis
		bool x_overlap = (b_ul.x <= l_right.x) && (b_lr.x >= u_left.x);

		// Check for overlap along the y-axis
		bool y_overlap = (b_ul.y <= l_right.y) && (b_lr.y >= u_left.y);

		// Check if there is overlap along both x and y axes
		return x_overlap && y_overlap;
	}
};
#endif