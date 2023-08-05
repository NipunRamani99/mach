#ifndef __SCENE_MANAGER_HPP__
#define __SCENE_MANAGER_HPP__
#include "Scene.hpp"
#include "DefaultScene.hpp"
#include "RevoluteJointScene.hpp"
#include "PyramidScene.hpp"
#include "ObjectPickingDemonstrationScene.h"
#include "MouseJointScene.hpp"
#include "CarScene.hpp"
#include "include/mach.hpp"
#include <memory>
#include <SFML/Graphics.hpp>

class SceneManager {
private:
	std::unique_ptr<Scene> currentScene;
	Mach& mach;
public:
	SceneManager(Mach & mach) 
		:
		mach(mach)
	{
		currentScene = std::make_unique<DefaultScene>(mach);
		currentScene->initialize();
	}

	void processInput(sf::RenderWindow & window) {
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num1)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<DefaultScene>(mach);
			currentScene->initialize();
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num2)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<RevoluteJointScene>(mach);
			currentScene->initialize();
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num3)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<PyramidScene>(mach);
			currentScene->initialize();
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num4)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<ObjectPickingDemonstrationScene>(mach);
			currentScene->initialize();
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num5)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<MouseJointScene>(mach);
			currentScene->initialize();
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num6)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<CarScene>(mach);
			currentScene->initialize();
		}
		currentScene->processInput(window);
	}

	~SceneManager() {
		currentScene->teardown();
	}
};

#endif // !__SCENE_MANAGER_HPP__
