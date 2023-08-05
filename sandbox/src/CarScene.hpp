#ifndef __CAR_SCENE_HPP__
#define __CAR_SCENE_HPP__
#include "Scene.hpp"
#include "Constants.hpp"
#include "include/CircleRigidBody.hpp"
#include <SFML/Graphics.hpp>
#include <random>

class CarScene : public Scene {
private:
	RigidBody* wheel1 = nullptr;
	RigidBody* wheel2 = nullptr;
	RigidBody* body = nullptr;
	MouseJoint* mouse_joint = nullptr;
	bool is_lmb_pressed = false;
public:
	CarScene(Mach& mach) : Scene(mach) {}

	void initialize() noexcept override {
		BoxRigidBody* boxRigidBody = new BoxRigidBody();
		boxRigidBody->size = { 1900.0f, 100.0f };
		boxRigidBody->position = { 970.0f , 1000.0f };
		boxRigidBody->angle = 0.0f;
		boxRigidBody->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody->is_static = true;
		boxRigidBody->aabb.size = glm::vec2{ 1950.0f, 150.0f };
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
		glm::vec2 pos = { 650, 800 };
		glm::vec2 pos2 = { 850, 800 };
		glm::vec2 size = { 70, 70 };
		glm::vec2 bodySize = { 200, 50 };
		glm::vec2 bodyPos = { 750, 800 };
		body = new BoxRigidBody(bodyPos, bodySize, 0.0f, 10.0f, 0.5f, getRainbow(6), false);
		body->linear_velocity = { 0.0f, 0.0f };//getRandomVelocity();
		body->angular_velocity = 0.0f;// getRandomAngularVelocity();
		body->is_static = false;
		body->groupId = 1;
		mach.addDynamicObject(body);
		wheel1 = new CircleRigidBody(pos, 50.0f, 0.0f, 1.0f, 0.5f, getRainbow(5), false); // new BoxRigidBody(pos, size, 0.0f, 10.0f, 0.5f, getRainbow(2), false);
		wheel1->linear_velocity = { 0.0f, 0.0f };//getRandomVelocity();
		wheel1->angular_velocity = 0.0f;// getRandomAngularVelocity();
		wheel1->is_static = false;
		wheel1->groupId = 1;
		mach.addDynamicObject(wheel1);
		wheel2 = new CircleRigidBody(pos2, 50.0f, 0.0f, 1.0f, 0.5f, getRainbow(5), false);
		wheel2->linear_velocity = { 0.0f, 0.0f };//getRandomVelocity();
		wheel2->angular_velocity = 0.0f;// getRandomAngularVelocity();
		wheel2->is_static = false;
		wheel2->groupId = 1;
		mach.addDynamicObject(wheel2);
		Joint* j1 = new Joint(wheel1, body, pos);
		mach.addJoint(j1);
		Joint* j2 = new Joint(wheel2, body, pos2);
		mach.addJoint(j2);

		BoxRigidBody* slope1 = new BoxRigidBody();
		slope1->size = { 800.0f, 50.0f };
		slope1->position = { 500.0f , 600.0f };
		slope1->angle = PI / 9.0f;
		slope1->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope1->is_static = true;
		slope1->calculateInertia();
		slope1->calculateAABB();
		slope1->aabb.size = { 800.0f, 400.0f };
		BoxRigidBody* slope2 = new BoxRigidBody();
		slope2->size = { 800.0f, 50.0f };
		slope2->position = { 1200.0f , 350.0f };
		slope2->angle = -PI / 9.0f;
		slope2->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope2->is_static = true;
		slope2->calculateInertia();
		slope2->aabb.size = { 800.0f, 400.0f };
		mach.addDynamicObject(slope1);
		mach.addDynamicObject(slope2);
		/*CircleRigidBody* circle = new CircleRigidBody({ 1000,800 }, 20.0f, 0.0f, 1.0f, 0.5f, getRainbow(5), false);
		circle->radius = 45.0f;
		mach.addDynamicObject(circle);*/
	}
	void processInput(sf::RenderWindow& window) noexcept override {
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			if (mouse_joint == nullptr) {
				sf::Vector2i p = sf::Mouse::getPosition(window);
				sf::Vector2f p_coord = window.mapPixelToCoords(p);
				if (body->checkIfInside({ float(p_coord.x),float(p_coord.y) })) {
					glm::vec2 target = { float(p_coord.x), float(p_coord.y) };
					mouse_joint = new MouseJoint(body, target, target);
					mach.setMouseJoint(mouse_joint);
					is_lmb_pressed = true;
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
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
			wheel1->angular_velocity =  wheel1->angular_velocity > 100.0f? 100.0f:wheel1->angular_velocity+1.0f;
			wheel2->angular_velocity = wheel2->angular_velocity > 100.0f ? 100.0f : wheel2->angular_velocity + 1.0f;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
			wheel1->angular_velocity = wheel1->angular_velocity < -100.0f ? -100.0f : wheel1->angular_velocity - 1.0f;
			wheel2->angular_velocity = wheel2->angular_velocity < -100.0f ? -100.0f : wheel2->angular_velocity - 1.0f;
		}
	}
};

#endif // !__CAR_SCENE_HPP__

