#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>

#include <imgui-SFML.h>
#include <imgui.h>

#include "rendering/renderer.hpp"
#include "Constants.hpp"
#include <include/ContactPoint.hpp>

#include "include/CircleRigidBody.hpp"
#include "include/mach.hpp"
#include "SceneManager.hpp"

class Sandbox {
private:
	sf::RenderWindow window;
	Mach mach;
	SceneManager sceneManager;
	Renderer renderer;
	sf::Color clearBackground;
	float dt;
	sf::Clock deltaClock;
	bool isSpacePressed;
	bool isZPressed;
	bool runSim;
	bool renderAABB;
	bool renderContactPoints;
	bool renderJoints;
public:
	Sandbox() :
		window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Mach Sandbox"),
		mach(),
		sceneManager(mach)
	{
		window.setFramerateLimit(0);
		clearBackground = sf::Color(49, 60, 69);
		ImGui::SFML::Init(window);
		dt = 1.0f / 60.0f;
		isSpacePressed = false;
		isZPressed = false;
		runSim = false;
		renderAABB = false;
		renderContactPoints = false;
		renderJoints = false;
	}

	void go() {
		sf::Clock deltaClock;
		while (window.isOpen()) {
			processEvents();
			updateImGui();
			updateGame(dt);
			render();
			window.display();
		}
		ImGui::SFML::Shutdown();
	}

private:
	void processEvents() {
		sf::Event event;
		while (window.pollEvent(event)) {
			ImGui::SFML::ProcessEvent(event);
			sceneManager.processInput(window);
			if (event.type == sf::Event::Closed) {
				window.close();
			}
			if (event.type == sf::Event::Resized) {
				sf::FloatRect visibleArea(0.f, 0.f, event.size.width, event.size.height);
			}
		}
	}

	void updateImGui() {
		ImGui::SFML::Update(window, deltaClock.restart());
		ImGui::Begin("Mach Sandbox");
		// ImGui code for UI elements
		ImGui::Checkbox("Render AABB", &renderAABB);
		ImGui::Checkbox("Render Contact Points", &renderContactPoints);
		ImGui::Checkbox("Render Joints", &renderJoints);
		ImGui::End();
		ImGui::SFML::Render(window);
	}

	void updateGame(float timestep) {
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Z) && !isZPressed) {
			runSim = !runSim;
			isZPressed = true;
		}
		if (!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Z)) {
			isZPressed = false;
		}
		if ((sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space) && !isSpacePressed) || runSim) {
			mach.update(timestep);
			isSpacePressed = true;
		}
		if (!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space)) {
			isSpacePressed = false;
		}
	}

	void render() {
		window.clear(clearBackground);
		auto& contactManifolds = mach.getContactList();
		auto& rigidBodies = mach.getRigidBodies();
		auto& joints = mach.getJoints();
		for (RigidBody* rigidBodies : rigidBodies) {
			renderer.renderRigidBody(window, rigidBodies);
			if(renderAABB)
				renderer.renderAABB(window, rigidBodies);
		}
		if (renderContactPoints) {
			for (auto& contactManifold : contactManifolds) {
				for (CollisionManifold& cm : contactManifolds) {
					for (ContactPoint& c : cm.contacts) {
						renderer.renderContactPoint(window, c.position);
					}
				}
			}
		}
		if (renderJoints) {
			for (Joint* j : joints) {
				renderer.renderJoint(window, j);
			}
			if (mach.getMouseJoint() != nullptr) {
				renderer.renderJoint(window, mach.getMouseJoint());
			}
		}
		ImGui::SFML::Render(window);
	}
};

int main() {
	Sandbox sandbox;
	sandbox.go();
	return 0;
}