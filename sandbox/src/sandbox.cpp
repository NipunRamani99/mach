#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>

#include <imgui-SFML.h>
#include <imgui.h>
#include "physics.hpp"
#include "renderer/renderer.hpp"
#include "Constants.hpp"

int main() {
    Physics physics;
    Renderer renderer;

    sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Mach Sandbox");
    window.setFramerateLimit(0);
    
    sf::Color clearBackground{49, 60, 69};
    ImGui::SFML::Init(window);
    float dt = 1 / 60.0f;
    sf::Clock deltaClock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(event);
            physics.processInput(event, dt);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }   

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("Mach Sandbox");
        
        physics.update(dt);
        window.clear(clearBackground);
        auto & contactManifolds = physics.mach.getContactList();
        auto & staticObjects = physics.mach.getStaticObjects();
        auto & dynamicObjects = physics.mach.getDynamicObjects();

        for (size_t i = 0; i < dynamicObjects.size(); i++) {
            auto& dynamicObject = dynamicObjects[i];
            ImGui::Text("Dynamic Object %f", i);
            ImGui::Text("Position: %f,%f", dynamicObject.position_current.x, dynamicObject.position_current.y);
            ImGui::Text("Rotation: %f", dynamicObject.rotation_current);

        }

        for(auto & dynamicObjects: dynamicObjects) {
			renderer.render(window, dynamicObjects);

		}
        for (auto& staticObject : staticObjects) {
            renderer.render(window, staticObject);
        }
        for (auto& contactManifold : contactManifolds) {
            renderer.renderContactPoint(window, contactManifold.contact1);
            if (contactManifold.contactCount > 1) {
                renderer.renderContactPoint(window, contactManifold.contact2);
            }
        }

        ImGui::End();

        ImGui::SFML::Render(window);
        
        window.display();
        
    }

    ImGui::SFML::Shutdown();

    return 0;
}