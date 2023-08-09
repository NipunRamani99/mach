#ifndef __REVOLUTE_JOINT_SCENE__
#define __REVOLUTE_JOINT_SCENE__
#include "include/mach.hpp"
#include "Scene.hpp"
#include "Constants.hpp"
#include "SFML/Graphics.hpp"
#include <random>

class RevoluteJointScene : public Scene {
private:
	bool is_button_pressed = false;
	bool is_key_pressed = false;
	static std::random_device rd;

public:
	RevoluteJointScene(Mach& mach)
		:
		Scene(mach)
	{
	}

	void initialize() noexcept override {
		BoxRigidBody* boxRigidBody = new BoxRigidBody();
		boxRigidBody->size = { 1900.0f, 50.0f };
		boxRigidBody->position = { 970.0f , 1000.0f };
		boxRigidBody->angle = 0.0f;
		boxRigidBody->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody->is_static = true;
		boxRigidBody->calculateInertia();

		BoxRigidBody* boxRigidBody2 = new BoxRigidBody();
		boxRigidBody2->size = { 1900.0f, 50.0f };
		boxRigidBody2->position = { 970.0f , 0.0f };
		boxRigidBody2->angle = 0.0f;
		boxRigidBody2->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody2->is_static = true;
		boxRigidBody2->calculateInertia();

		BoxRigidBody* slope1 = new BoxRigidBody();
		slope1->size = { 800.0f, 50.0f };
		slope1->position = { 500.0f , 600.0f };
		slope1->angle = PI / 9.0f;
		slope1->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope1->is_static = true;
		slope1->calculateInertia();

		BoxRigidBody* slope2 = new BoxRigidBody();
		slope2->size = { 800.0f, 50.0f };
		slope2->position = { 1200.0f , 350.0f };
		slope2->angle = -PI / 9.0f;
		slope2->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope2->is_static = true;
		slope2->calculateInertia();

		BoxRigidBody* boxRigidBody4 = new BoxRigidBody();
		boxRigidBody4->size = { 1920.0f, 50.0f };
		boxRigidBody4->position = { 1900.0f , 350.0f };
		boxRigidBody4->angle = PI / 2.0f;
		boxRigidBody4->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody4->is_static = true;
		boxRigidBody4->calculateInertia();

		BoxRigidBody* boxRigidBody5 = new BoxRigidBody();
		boxRigidBody5->size = { 1920.0f, 50.0f };
		boxRigidBody5->position = { 20.0f , 350.0f };
		boxRigidBody5->angle = PI / 2.0f;
		boxRigidBody5->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody5->is_static = true;
		boxRigidBody5->calculateInertia();

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
			Joint* j = new Joint(b1, b, anchor);
			mach.addJoint(j);
			b1 = b;
		}
	}

	void processInput(sf::RenderWindow & window) noexcept override {
		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left) && !is_button_pressed) {
			static int i = 0;
			is_button_pressed = true;
			BoxRigidBody* dynamicBoxRigidBody = new BoxRigidBody({ sf::Mouse::getPosition().x, sf::Mouse::getPosition().y }, getRandomSize(), 0.0f, 10.0f, 0.5f, getRainbow(i++));
			dynamicBoxRigidBody->linear_velocity = { 0.0f, 0.0f };
			dynamicBoxRigidBody->angular_velocity = 0.0f;
			dynamicBoxRigidBody->is_static = false;
			mach.addDynamicObject(dynamicBoxRigidBody);
		}
		if (!sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
			is_button_pressed = false;
		}
	}

	float getRandomAngle() {
		return (rand() % 360) * PI / 180.0f;
	}

	float getRandomAngularVelocity() {
		return -1.0f + 2.0f * (rand() % 2);
	}

	glm::vec2 getRandomVelocity() {
		float x = 0.0f;
		float y = 0.0f;
		while (x == 0.0f) {
			x = -10.0f + (rand() % 20);
		}
		while (y == 0.0f) {
			y = -10.0f + (rand() % 20);
		}
		return glm::vec2(x, y);
	}

	float getRandomMass() {
		return 1.0f + 10.0f * (rand() % 10);
	}

	float getRandomRadius() {
		return 10.0f + 2.0f * (rand() % 10);
	}

	glm::vec2 getRandomPosition() {

		float x = 100.0f + (rand() % (int)(SCREEN_WIDTH - 200.0f));
		float y = 100.0f + (rand() % (int)(SCREEN_HEIGHT - 600.0f));
		return glm::vec2(x, y);
	}

	glm::vec2 getRandomSize() {
		float size = 50.0f + 10.0f * (rand() % 3);
		return glm::vec2(size, size);
	}

	glm::vec3 getRainbow(float t)
	{
		const float r = sin(t);
		const float g = sin(t + 0.33f * 2.0f * PI);
		const float b = sin(t + 0.66f * 2.0f * PI);
		return { static_cast<uint8_t>(255.0f * r * r),
				static_cast<uint8_t>(255.0f * g * g),
				static_cast<uint8_t>(255.0f * b * b) };
	}
};

#endif