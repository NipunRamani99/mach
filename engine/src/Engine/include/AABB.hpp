#ifndef __AABB_HPP__
#define __AABB_HPP__
#include <glm/glm.hpp>
struct AABB {
	glm::vec2 position;
	glm::vec2 size;

	bool isInside(glm::vec2 p) {
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

	bool Overlapping(AABB& a, AABB& b) {
		glm::vec2 b_ul = b.position;  //upper left corner of b
		glm::vec2 b_lr = b.position + b.size; //lower right corner of b
		glm::vec2 a_ul = a.position; //upper left corner of a
		glm::vec2 a_lr = a.position + a.size; // lower right corner of a
		// Check for overlap along the x-axis
		bool x_overlap = (b_ul.x <= a_lr.x) && (b_lr.x >= a_ul.x);

		// Check for overlap along the y-axis
		bool y_overlap = (b_ul.y <= a_lr.y) && (b_lr.y >= a_ul.y);

		// Check if there is overlap along both x and y axes
		return x_overlap && y_overlap;
	}
};
#endif