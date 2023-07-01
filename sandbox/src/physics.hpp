#ifndef __PHYSICS_HPP__
#define __PHYSICS_HPP__
#include <include/mach.hpp>
#include "Constants.hpp"
struct Physics {
	Mach mach;

	size_t num_dynamic_objects = 5;

	Physics() {
		BoxRigidBody boxRigidBody;
		boxRigidBody.size = { 1900.0f, 100.0f };
		boxRigidBody.position_current = { 970.0f , 900.0f };
		boxRigidBody.rotation_current = 0.0f;
		boxRigidBody.color = { 0.0f,255.0f * 0.92f,0.0f };
		mach.addStaticObject(boxRigidBody);

		for (size_t i = 0; i < num_dynamic_objects; i++) {
			BoxRigidBody boxRigidBody{ getRandomPosition(),getRandomSize(),0.0f,getRainbow(i) };
			mach.addDynamicObject(boxRigidBody);
		}
	}

	glm::vec2 getRandomPosition() {
		float x = 100.0f + (rand() % (int)(SCREEN_WIDTH - 200.0f));
		float y = 100.0f + (rand() % (int)(SCREEN_HEIGHT - 600.0f));
		return glm::vec2(x, y);
	}

	glm::vec2 getRandomSize() {
		float size = 10.0f + 10.0f * (rand() % 10);
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
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
			mach.getDynamicObjects()[0].accelerate({0.0f, -100.0f});
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
			mach.getDynamicObjects()[0].accelerate({ 0.0f, 100.0f });
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {
			mach.getDynamicObjects()[0].accelerate({ -100.0f,0.0f });
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {
			mach.getDynamicObjects()[0].accelerate({ 100.0f,0.0f });
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) {
			mach.getDynamicObjects()[0].angularAccelerate(5.0f);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) {
			mach.getDynamicObjects()[0].angularAccelerate(-5.0f);
		}

	}

	void update(float dt) {
		mach.update(dt);
	}


};
#endif // !__PHYSICS_HPP__
