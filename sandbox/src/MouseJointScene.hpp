#pragma once
#include "Scene.hpp"
#include "Constants.hpp"
class MouseJointScene : public Scene {
private:
	MouseJoint* joint = nullptr;
	RigidBody* b = nullptr;
	bool is_lmb_pressed = false;
public:
	MouseJointScene(Mach& mach) 
		:
		Scene(mach)
	{
	}
	// Inherited via Scene
	void initialize() override {

		BoxRigidBody* boxRigidBody = new BoxRigidBody();
		boxRigidBody->size = { 1900.0f, 50.0f };
		boxRigidBody->position = { 970.0f , 1000.0f };
		boxRigidBody->angle = 0.0f;
		boxRigidBody->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody->is_static = true;
		boxRigidBody->aabb.size = glm::vec2{ 1950.0f, 100.0f };
		boxRigidBody->calculateInertia();
		boxRigidBody->calculateAABB();

		BoxRigidBody* boxRigidBody2 = new BoxRigidBody();
		boxRigidBody2->size = { 1900.0f, 50.0f };
		boxRigidBody2->position = { 970.0f , 0.0f };
		boxRigidBody2->angle = 0.0f;
		boxRigidBody2->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody2->is_static = true;
		boxRigidBody2->calculateInertia();
		boxRigidBody2->aabb.size = glm::vec2{ 1950.0f, 150.f };

		BoxRigidBody* slope1 = new BoxRigidBody();
		slope1->size = { 800.0f, 50.0f };
		slope1->position = { 500.0f , 600.0f };
		slope1->angle = 0.0f;
		slope1->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope1->is_static = true;
		slope1->calculateInertia();
		slope1->calculateAABB();
		slope1->aabb.size = { 800.0f, 400.0f };

		BoxRigidBody* boxRigidBody4 = new BoxRigidBody();
		boxRigidBody4->size = { 1920.0f, 50.0f };
		boxRigidBody4->position = { 1900.0f , 350.0f };
		boxRigidBody4->angle = PI / 2.0f;
		boxRigidBody4->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody4->is_static = true;
		boxRigidBody4->calculateInertia();
		boxRigidBody4->aabb.size = { 100.0f, 1950.0f };

		BoxRigidBody* boxRigidBody5 = new BoxRigidBody();
		boxRigidBody5->size = { 1920.0f, 50.0f };
		boxRigidBody5->position = { 20.0f , 350.0f };
		boxRigidBody5->angle = PI / 2.0f;
		boxRigidBody5->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody5->is_static = true;
		boxRigidBody5->calculateInertia();
		boxRigidBody5->aabb.size = { 100.0f, 1970.0f };
		mach.addDynamicObject(boxRigidBody);
		mach.addDynamicObject(boxRigidBody2);
		mach.addDynamicObject(boxRigidBody4);
		mach.addDynamicObject(boxRigidBody5);
		mach.addDynamicObject(slope1);
		glm::vec2 pos = { 650, 800 };
		glm::vec2 size = { 50,50 };
		b = new BoxRigidBody(pos, size, 0.0f, 10.0f, 0.5f, getRainbow(2), false);
		b->linear_velocity = { 0.0f, 0.0f };//getRandomVelocity();
		b->angular_velocity = 0.0f;// getRandomAngularVelocity();
		b->is_static = false;
		mach.addDynamicObject(b);
	
	}
	void processInput(sf::RenderWindow& window) override {
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			if (joint == nullptr) {
				sf::Vector2i p = sf::Mouse::getPosition(window);
				sf::Vector2f p_coord = window.mapPixelToCoords(p);
				if (b->checkIfInside({ float(p_coord.x),float(p_coord.y) })) {
					glm::vec2 target = { float(p_coord.x), float(p_coord.y) };
					joint = new MouseJoint(b, target, target);
					mach.setMouseJoint(joint);
					is_lmb_pressed = true;
				}
				
			}
			else if(joint && is_lmb_pressed) {
				sf::Vector2i p = sf::Mouse::getPosition(window);
				sf::Vector2f p_coord = window.mapPixelToCoords(p);
				glm::vec2 target = { float(p_coord.x), float(p_coord.y) };
				joint->setTarget(target);
				is_lmb_pressed = true;
			}
		}
		if (!sf::Mouse::isButtonPressed(sf::Mouse::Left) && is_lmb_pressed) {
			mach.removeMouseJoint();
			is_lmb_pressed = false;
			if (joint) {
				delete joint; 
				joint = nullptr;
			}
		}
	}
};