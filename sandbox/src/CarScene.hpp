#ifndef __CAR_SCENE_HPP__
#define __CAR_SCENE_HPP__
#include "Scene.hpp"
#include "Constants.hpp"
#include "include/CircleRigidBody.hpp"
#include <SFML/Graphics.hpp>
#include <random>

class Car {
private:
	RigidBody* wheel1 = nullptr;
	RigidBody* wheel2 = nullptr;
	RigidBody* body = nullptr;
	
public:
	void initialise(glm::vec2 pos, Mach & mach) {
		unsigned int id = 2;
		glm::vec2 wheel1_pos = pos + glm::vec2{30,25};
		glm::vec2 wheel2_pos = pos + glm::vec2{-30, 25};
		glm::vec2 body_pos = pos;
		glm::vec2 size = { 70, 70 };
		glm::vec2 bodySize = { 100, 50 };
		body = new BoxRigidBody(body_pos, bodySize, 0.0f, 10.0f, 0.5f, getRainbow(6), false);
		body->linear_velocity = { 0.0f, 0.0f };
		body->angular_velocity = 0.0f;
		body->is_static = false;
		body->groupId = id;
		wheel1 = new CircleRigidBody(wheel1_pos, 10.0f, 0.0f, 1.0f, 0.5f, getRainbow(5), false);
		wheel1->linear_velocity = { 0.0f, 0.0f };
		wheel1->angular_velocity = 0.0f;
		wheel1->is_static = false;
		wheel1->groupId = id;
		mach.addDynamicObject(wheel1);
		wheel2 = new CircleRigidBody(wheel2_pos, 10.0f, 0.0f, 1.0f, 0.5f, getRainbow(5), false);
		wheel2->linear_velocity = { 0.0f, 0.0f };
		wheel2->angular_velocity = 0.0f;
		wheel2->is_static = false;
		wheel2->groupId = id;
		mach.addDynamicObject(wheel2);
		mach.addDynamicObject(body);
		RevoluteJoint* j1 = new RevoluteJoint(wheel1, body, wheel1_pos);
		mach.addJoint(j1);
		RevoluteJoint* j2 = new RevoluteJoint(wheel2, body, wheel2_pos);
		mach.addJoint(j2);
	}
	void processInput() {
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
			wheel1->angular_velocity = wheel1->angular_velocity > 100.0f ? 100.0f : wheel1->angular_velocity + 1.0f;
			wheel2->angular_velocity = wheel2->angular_velocity > 100.0f ? 100.0f : wheel2->angular_velocity + 1.0f;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
			wheel1->angular_velocity = wheel1->angular_velocity < -100.0f ? -100.0f : wheel1->angular_velocity - 1.0f;
			wheel2->angular_velocity = wheel2->angular_velocity < -100.0f ? -100.0f : wheel2->angular_velocity - 1.0f;
		}
	}
};

class CarScene : public Scene {
private:

	MouseJoint* mouse_joint = nullptr;
	bool is_lmb_pressed = false;
	Car* car = nullptr;
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
		glm::vec2 pos = { 680, 795 };
		glm::vec2 pos2 = { 620, 795 };
		glm::vec2 size = { 70, 70 };
		glm::vec2 bodySize = { 100, 50 };
		glm::vec2 bodyPos = { 650, 770 };
		
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
		car = new Car();
		car->initialise({ 300,300 }, mach);
	}

	void processInput(sf::RenderWindow& window) noexcept override {
		processMouseInput(window);
		car->processInput();
	}
};

#endif // !__CAR_SCENE_HPP__

