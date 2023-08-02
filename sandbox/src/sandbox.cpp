#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>

#include <imgui-SFML.h>
#include <imgui.h>
#include "physics.hpp"
#include "rendering/renderer.hpp"
#include "Constants.hpp"
#include <include/ContactPoint.hpp>

#include "include/CircleRigidBody.hpp"
#include "include/mach.hpp"
#include "SceneManager.hpp"
int main() {
   // Physics physics;
    Renderer renderer;
    Mach mach;
    SceneManager sceneManager(mach);

    sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Mach Sandbox");
    window.setFramerateLimit(0);
    
    sf::Color clearBackground{49, 60, 69};
    ImGui::SFML::Init(window);
    float dt = 1 / 60.0f;
    sf::Clock deltaClock;
    bool isSpacePressed = false;
    bool isZPressed = false;
    bool runSim = false;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(event);
            sceneManager.processInput(window);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }   

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("Mach Sandbox");
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Z) && !isZPressed) {
			runSim = !runSim;
			isZPressed = true;
		}
        if (!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Z)) {
            isZPressed = false;
        }
        if((sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space) && !isSpacePressed )|| runSim ) {
            mach.update(dt);
            isSpacePressed = true;
        }
        if(!sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space)) {
			isSpacePressed = false;
		}
        window.clear(clearBackground);
        auto & contactManifolds = mach.getContactList();
        auto & rigidBodies = mach.getRigidBodies();
        auto & joints = mach.getJoints();
        for (size_t i = 0; i < rigidBodies.size(); i++) {
            auto& dynamicObject = *rigidBodies[i];
            ImGui::Separator();
            std::string title = "Dynamic Object %f#" + std::to_string(i);
            ImGui::Text(title.c_str(), i);
            ImGui::Text("Position: %f,%f", dynamicObject.position.x, dynamicObject.position.y);
            ImGui::Text("Rotation: %f", dynamicObject.angle);
            ImGui::Text("Linear Velocity: %f %f", dynamicObject.linear_velocity.x, dynamicObject.linear_velocity.y);
            ImGui::Text("Angular Velocity: %f", dynamicObject.angular_velocity);
        }

        for(RigidBody * rigidBodies: rigidBodies) {
			renderer.renderRigidBody(window, rigidBodies);
		}

        for (auto& contactManifold : contactManifolds) {
            for (Collisions::CollisionManifold & cm : contactManifolds) {
                for (ContactPoint& c : cm.contacts) {
                    renderer.renderContactPoint(window, c.position);
                }
            }
        }

        for (Joint* j : joints) {
            renderer.renderJoint(window, j);
        }

        //for (size_t i = 0; i < rigidBodies.size(); i++) {
        //    renderer.renderAABB(window, rigidBodies[i]);
        //}
        ImGui::End();

        ImGui::SFML::Render(window);
        
        window.display();
    }
    ImGui::SFML::Shutdown();
    return 0;
}