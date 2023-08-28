#ifndef __REVOLUTE_JOINT_SCENE__
#define __REVOLUTE_JOINT_SCENE__
#include "include/mach.hpp"
#include "Scene.hpp"
#include "Constants.hpp"
#include "SFML/Graphics.hpp"
#include <random>

class RevoluteJointScene : public Scene {
private:
	bool is_key_b_pressed = false;
	bool is_key_c_pressed = false;
	static std::random_device rd;

public:
	RevoluteJointScene(Mach& mach)
		:
		Scene(mach)
	{
	}

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

		glm::vec2 hScreenRes = { SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f };
		hScreenRes -= glm::vec2(0, 300);
		glm::vec2 boxPos1 = hScreenRes + glm::vec2{-100.0f, 0.0f};
		size_t num = 8;
		glm::vec2 size = { 75, 25 };
		glm::vec2 gap = { 90,0 };

		RigidBody* b1 = boxRigidBody;
		for (size_t i = 0; i < num; i++) {
			glm::vec2 x = float(i) * gap + hScreenRes + glm::vec2(50, 0);
			BoxRigidBody* b = new BoxRigidBody(x, size, 0.0f, 0.50f, 0.5f, getRainbow(i), false);
			mach.addDynamicObject(b);
			glm::vec2 anchor = hScreenRes + float(i) * gap;
			RevoluteJoint* j = new RevoluteJoint(b1, b, anchor);
			mach.addJoint(j);
			b1 = b;
		}
	}

	void processInput(sf::RenderWindow & window) noexcept override {
		processMouseInput(window);
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::B) && !is_key_b_pressed) {
			static int i = 0;
			is_key_b_pressed = true;
			BoxRigidBody* dynamicBoxRigidBody = new BoxRigidBody({ sf::Mouse::getPosition().x, sf::Mouse::getPosition().y }, getRandomSize(), 0.0f, 10.0f, 0.5f, getRainbow(i++));
			dynamicBoxRigidBody->linear_velocity = { 0.0f, 0.0f };
			dynamicBoxRigidBody->angular_velocity = 0.0f;
			dynamicBoxRigidBody->is_static = false;
			mach.addDynamicObject(dynamicBoxRigidBody);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::C) && !is_key_c_pressed) {
			static int i = 0;
			is_key_c_pressed = true;

			CircleRigidBody* dynamicBoxRigidBody = new CircleRigidBody({ sf::Mouse::getPosition().x, sf::Mouse::getPosition().y }, 10.0f, 0.0f, 10.0f, 0.5f, getRainbow(i++));
			dynamicBoxRigidBody->linear_velocity = { 0.0f, 0.0f };
			dynamicBoxRigidBody->angular_velocity = 0.0f;
			dynamicBoxRigidBody->is_static = false;
			mach.addDynamicObject(dynamicBoxRigidBody);
		}
		if (!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::B)) {
			is_key_b_pressed = false;
		}
		if (!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C)) {
			is_key_c_pressed = false;
		}
	}
};

#endif