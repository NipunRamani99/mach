#ifndef __SCENE_MANAGER_HPP__
#define __SCENE_MANAGER_HPP__
#include "Scene.hpp"
#include "DefaultScene.hpp"
#include "RevoluteJointScene.hpp"
#include "PyramidScene.hpp"
#include "CarScene.hpp"
#include "TrainScene.hpp"
#include "include/mach.hpp"
#include <memory>
#include <SFML/Graphics.hpp>

class SceneManager {
private:
	std::unique_ptr<Scene> currentScene;
	Mach& mach;
	size_t currentSceneNumber = 0;
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
			currentSceneNumber = 0;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num2)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<RevoluteJointScene>(mach);
			currentScene->initialize();
			currentSceneNumber = 1;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num3)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<PyramidScene>(mach);
			currentScene->initialize();
			currentSceneNumber = 2;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num4)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<CarScene>(mach);
			currentScene->initialize();
			currentSceneNumber = 3;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Num5)) {
			if (currentScene) currentScene->teardown();
			currentScene = std::make_unique<TrainScene>(mach);
			currentScene->initialize();
			currentSceneNumber = 4;
		}
		currentScene->processInput(window);
	}

	size_t getCurrentSceneNumber() {
		return currentSceneNumber;
	}

	~SceneManager() {
		currentScene->teardown();
	}
};

#endif // !__SCENE_MANAGER_HPP__
