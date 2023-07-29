#ifndef __SCENE_HPP__
#define __SCENE_HPP__
#include "include/mach.hpp"
class Scene {
protected:
	Mach& mach;
public:
	Scene(Mach & mach) 
		:
		mach(mach)
	{
	}
	virtual void initialize() = 0;

	void teardown() {
		mach.getJoints().clear();
		mach.getRigidBodies().clear();
	}

	virtual void processInput() = 0;

	~Scene() {
		teardown();
	}
};
#endif