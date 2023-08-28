#ifndef __SCENE_HPP__
#define __SCENE_HPP__
#include "include/mach.hpp"
#include <SFML/Graphics.hpp>
class Scene {
protected:
	Mach& mach;
	MouseJoint* mouse_joint = nullptr;
	bool is_lmb_pressed = false;
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
		mach.setMouseJoint(nullptr);
	}

	virtual void processInput(sf::RenderWindow & window) = 0;

	void processMouseInput(sf::RenderWindow& window) {
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			if (mouse_joint == nullptr) {
				sf::Vector2i p = sf::Mouse::getPosition(window);
				sf::Vector2f p_coord = window.mapPixelToCoords(p);
				glm::vec2 pt = { float(p_coord.x), float(p_coord.y) };
				std::vector<RigidBody*> bodies =  mach.getRigidBodies();
				for (RigidBody* body : bodies) {
					if (!body->is_static)
					{
						if (body->aabb.isInside(pt) && body->checkIfInside(pt)) {
							glm::vec2 target = pt;
							mouse_joint = new MouseJoint(body, target, target);
							mach.setMouseJoint(mouse_joint);
							is_lmb_pressed = true;
							break;
						}
					}
				}
			}
			else if (mouse_joint && is_lmb_pressed) {
				sf::Vector2i p = sf::Mouse::getPosition(window);
				sf::Vector2f p_coord = window.mapPixelToCoords(p);
				glm::vec2 target = { float(p_coord.x), float(p_coord.y) };
				mouse_joint->setTarget(target);
				is_lmb_pressed = true;
			}
		}
		if (!sf::Mouse::isButtonPressed(sf::Mouse::Left) && is_lmb_pressed) {
			mach.removeMouseJoint();
			is_lmb_pressed = false;
			if (mouse_joint) {
				delete mouse_joint;
				mouse_joint = nullptr;
			}
		}
	}

	~Scene() {
		teardown();
	}
};
#endif