#ifndef __TRAIN_SCENE_HPP__
#define __TRAIN_SCENE_HPP__
#include "Scene.hpp"
#include "Constants.hpp"
#include "include/CircleRigidBody.hpp"
#include <SFML/Graphics.hpp>
#include <random>

class TrainCar {
public:
	RigidBody* wheel1 = nullptr;
	RigidBody* wheel2 = nullptr;
	RigidBody* body = nullptr;
	void initialise(glm::vec2 pos, Mach& mach) {
		unsigned int id = 2;
		glm::vec2 wheel1_pos = pos + glm::vec2{30, 25};
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

class TrainScene : public Scene {
private:

	MouseJoint* mouse_joint = nullptr;
	bool is_lmb_pressed = false;
	TrainCar* car1 = nullptr;
	TrainCar* car2 = nullptr;
	TrainCar* car3 = nullptr;
public:
	TrainScene(Mach& mach) : Scene(mach) {}

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
		slope1->size = { 480.0f, 10.0f };
		slope1->position = { 150.0f , 480.0f };
		slope1->angle = 0.0f;
		slope1->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope1->is_static = true;
		slope1->calculateInertia();
		slope1->calculateAABB();
		slope1->aabb.size = { 800.0f, 400.0f };
		BoxRigidBody* slope2 = new BoxRigidBody();
		slope2->size = { 450.0f, 10.0f };
		slope2->position = { 1745.0f , 475.0f };
		slope2->angle = 0.0f;
		slope2->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope2->is_static = true;
		slope2->calculateInertia();
		slope2->aabb.size = { 800.0f, 400.0f };
		mach.addDynamicObject(slope1);
		mach.addDynamicObject(slope2);
		car1 = new TrainCar();
		car1->initialise({ 100,300 }, mach);
		car2 = new TrainCar();
		car2->initialise({ 215,300 }, mach);

		car3 = new TrainCar();
		car3->initialise({ 330,300 }, mach);
		RevoluteJoint* j1 = new RevoluteJoint(car1->body, car2->body, car2->body->position);
		RevoluteJoint* j2 = new RevoluteJoint(car2->body, car3->body, car3->body->position);
		mach.addJoint(j1);
		mach.addJoint(j2);
		
		glm::vec2 hScreenRes = { SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f };
		hScreenRes -= glm::vec2(550, 50.0);
		glm::vec2 boxPos1 = hScreenRes + glm::vec2{-500.0f, 0.0f};
		size_t num = 12;
		size = { 90, 25 };
		glm::vec2 gap = { 90,0 };

		RigidBody* b1 = slope1;
		for (size_t i = 0; i < num; i++) {
			glm::vec2 x = float(i) * gap + hScreenRes + glm::vec2(50, 0);
			BoxRigidBody* b = new BoxRigidBody(x, size, 0.0f, 0.50f, 0.5f, getRainbow(i), false);
			mach.addDynamicObject(b);
			glm::vec2 anchor = hScreenRes + float(i) * gap;
			RevoluteJoint* j = new RevoluteJoint(b1, b, anchor);
			mach.addJoint(j);
			b1 = b;
		}
		
		RevoluteJoint* j = new RevoluteJoint(slope2, b1, b1->position);
		mach.addJoint(j);
	}

	void processInput(sf::RenderWindow& window) noexcept override {
		processMouseInput(window);
		car1->processInput();
		car2->processInput();
		car3->processInput();
	}
};
#endif __TRAIN_SCENE_HPP__