#ifndef _JOINT_HPP_
#define _JOINT_HPP_
#include "Math.hpp"
struct Joint {
	enum Type {
		REVOLUTE,
		MOUSE,
		DISTANCE
	} type;
	virtual void initialize(float dt) = 0;
	virtual void applyImpulse(float dt) = 0;
};
#endif