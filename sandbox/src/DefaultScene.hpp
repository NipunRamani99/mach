#ifndef __DEFAULT_SCENE_HPP__
#define __DEFAULT_SCENE_HPP__
#include "Scene.hpp"
#include "Constants.hpp"
#include <SFML/Graphics.hpp>
#include <random>
class DefaultScene : public Scene {
private:
	bool is_key_b_pressed = false;
	bool is_key_c_pressed = false;
public:
	DefaultScene(Mach & mach) 
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
		boxRigidBody5->aabb.size = {100.0f, 1970.0f};
		mach.addDynamicObject(boxRigidBody);
		mach.addDynamicObject(boxRigidBody2);
		mach.addDynamicObject(boxRigidBody4);
		mach.addDynamicObject(boxRigidBody5);
		mach.addDynamicObject(slope1);
		mach.addDynamicObject(slope2);
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
#endif // !__DEFAULT_SCENE_HPP__
