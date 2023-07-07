#ifndef __PHYSICS_HPP__
#define __PHYSICS_HPP__
#include <include/mach.hpp>
#include "Constants.hpp"
#include <random>
#include <filesystem>
namespace fs = std::filesystem;
struct Physics {
	Mach mach;
	
	fs::file_type type;
	size_t num_dynamic_objects = 0;

	static std::random_device rd;
	bool is_button_pressed = false;
	bool is_key_pressed = false;
	Physics() {
		BoxRigidBody * boxRigidBody = new BoxRigidBody();
		boxRigidBody->size = { 1900.0f, 100.0f };
		boxRigidBody->position = { 970.0f , 1000.0f };
		boxRigidBody->angle = 0.0f;
		boxRigidBody->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody->is_static = true;

		BoxRigidBody * boxRigidBody2 = new BoxRigidBody();
		boxRigidBody2->size = { 1900.0f, 50.0f };
		boxRigidBody2->position = { 970.0f , 0.0f };
		boxRigidBody2->angle = 0.0f;
		boxRigidBody2->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody2->is_static = true;


		BoxRigidBody * slope1 = new BoxRigidBody();
		slope1->size = { 800.0f, 50.0f };
		slope1->position = { 500.0f , 600.0f };
		slope1->angle = PI / 9.0f;
		slope1->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope1->is_static = true;

		BoxRigidBody * slope2 = new BoxRigidBody();
		slope2->size = { 800.0f, 50.0f };
		slope2->position = { 1200.0f , 350.0f };
		slope2->angle = -PI / 9.0f;
		slope2->color = { 0.0f,255.0f * 0.92f,0.0f };
		slope2->is_static = true;

		BoxRigidBody * boxRigidBody4 = new BoxRigidBody();
		boxRigidBody4->size = { 1920.0f, 50.0f };
		boxRigidBody4->position = { 1900.0f , 350.0f };
		boxRigidBody4->angle = PI / 2.0f;
		boxRigidBody4->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody4->is_static = true;

		BoxRigidBody * boxRigidBody5 = new BoxRigidBody();
		boxRigidBody5->size = { 1920.0f, 50.0f };
		boxRigidBody5->position = { 20.0f , 350.0f };
		boxRigidBody5->angle = PI / 2.0f;
		boxRigidBody5->color = { 0.0f,255.0f * 0.92f,0.0f };
		boxRigidBody5->is_static = true;

		
		//BoxRigidBody dynamicBoxRigidBody2({ 300.0f,100.0f }, getRandomSize(), 0.0f, 5, 0.5f, getRainbow(0));
		//BoxRigidBody dynamicBoxRigidBody3({ 1100.0f,100.0f }, getRandomSize(), 0.0f,6, 0.5f, getRainbow(1));
		/*dynamicBoxRigidBody2->linear_velocity = getRandomVelocity();
		dynamicBoxRigidBody3.linear_velocity = getRandomVelocity();
		dynamicBoxRigidBody2->angular_velocity = getRandomAngularVelocity();
		dynamicBoxRigidBody3.angular_velocity = getRandomAngularVelocity();*/

		for (int i = 0; i < num_dynamic_objects; i++) {
			BoxRigidBody * dynamicBoxRigidBody = new BoxRigidBody(getRandomPosition(), getRandomSize(), getRandomAngle(), 0.005, 0.5f, getRainbow(i));
			dynamicBoxRigidBody->linear_velocity = getRandomVelocity();
			dynamicBoxRigidBody->angular_velocity = getRandomAngularVelocity();
			mach.addDynamicObject(dynamicBoxRigidBody);
		}

		//mach.addDynamicObject(dynamicBoxRigidBody2);
		//mach.addDynamicObject(dynamicBoxRigidBody3);
		mach.addDynamicObject(boxRigidBody);
		//mach.addDynamicObject(boxRigidBody3);
		mach.addDynamicObject(boxRigidBody2);
		mach.addDynamicObject(boxRigidBody4);
		mach.addDynamicObject(boxRigidBody5);
		mach.addDynamicObject(slope1);
		mach.addDynamicObject(slope2);


	};
	
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
		float size = 50.0f + 10.0f * (rand() %3);
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

	void processInput(sf::Event & e, float dt) {
		//if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
		//	mach.getDynamicObjects()[0].move({0.0f, -10.0f});
		//}
		//if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
		//	mach.getDynamicObjects()[0].move({ 0.0f, 10.0f });
		//}
		//if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {
		//	mach.getDynamicObjects()[0].move({ -10.0f,0.0f });
		//}
		//if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {
		//	mach.getDynamicObjects()[0].move({ 10.0f,0.0f });
		//}
		
		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left) && !is_button_pressed) {
			static int i = 0;
			is_button_pressed = true;
			BoxRigidBody * dynamicBoxRigidBody = new BoxRigidBody({sf::Mouse::getPosition().x, sf::Mouse::getPosition().y}, getRandomSize(), getRandomAngle(), 0.05f, 0.5f, getRainbow(i++));
				dynamicBoxRigidBody->linear_velocity = getRandomVelocity();
				dynamicBoxRigidBody->angular_velocity = getRandomAngularVelocity();
				dynamicBoxRigidBody->is_static = false;
				mach.addDynamicObject(dynamicBoxRigidBody);
		}
		if (!sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
			is_button_pressed = false;
		}

		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C) && !is_key_pressed) {
			static int j = 0;
			CircleRigidBody* dynamicBoxRigidBody = new CircleRigidBody({ sf::Mouse::getPosition().x, sf::Mouse::getPosition().y }, getRandomRadius(), getRandomAngle(), 0.05f, 0.5f, getRainbow(j++));
			dynamicBoxRigidBody->linear_velocity = getRandomVelocity();
			dynamicBoxRigidBody->angular_velocity = getRandomAngularVelocity();
			dynamicBoxRigidBody->is_static = false;
			mach.addDynamicObject(dynamicBoxRigidBody);
			is_key_pressed = true;
		}
		if(!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C) ) {
			is_key_pressed = false;
		}
	}

	void update(float dt) {
		mach.update(dt);
	}


};
#endif // !__PHYSICS_HPP__
