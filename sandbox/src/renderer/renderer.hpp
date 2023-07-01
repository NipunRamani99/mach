#ifndef __RENDERER_HPP__
#define __RENDERER_HPP__
#include <include/mach.hpp>
#include <SFML/Graphics.hpp>
class Renderer {
public:
	Renderer() {}
	Renderer(Renderer& renderer) = delete;
	void render(sf::RenderWindow& window, BoxRigidBody & boxRigidBody) {
		std::vector<glm::vec2> vertices = boxRigidBody.getVertices();
		sf::VertexArray quad(sf::Quads, 4);
		for (int i = 0; i < 4; i++) {
			quad[i].position = { vertices[i].x, vertices[i].y };
			quad[i].color = sf::Color(boxRigidBody.color.r, boxRigidBody.color.g, boxRigidBody.color.b);
		}

		window.draw(quad);
	}
};
#endif