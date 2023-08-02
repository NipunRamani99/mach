#ifndef __RENDERER_HPP__
#define __RENDERER_HPP__
#include <include/mach.hpp>
#include <include/CircleRigidBody.hpp>
#include <SFML/Graphics.hpp>
#include <filesystem>
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

	void renderRigidBody(sf::RenderWindow & window, RigidBody * rigidBody) {
		if (rigidBody->type == RigidBody::Type::BOX) {
			render(window, *(BoxRigidBody*)rigidBody);
		}
		else if (rigidBody->type == RigidBody::Type::CIRCLE) {
			render(window, *(CircleRigidBody*)rigidBody);
		}
	}

	void render(sf::RenderWindow& window, CircleRigidBody& circleRigidBody) {
		sf::Vector2f position1 = sf::Vector2f(circleRigidBody.position.x, circleRigidBody.position.y);
		glm::vec2 pos2 = glm::vec2(circleRigidBody.radius, 0);
		pos2 = rotateVec2Radians(pos2, circleRigidBody.angle);
		pos2 += circleRigidBody.position;
		sf::Vector2f position2 = sf::Vector2f(pos2.x, pos2.y);
		sf::VertexArray line(sf::Lines, 2);
		line[0].position = position1;
		line[0].color = sf::Color::White;
		line[1].position = position2;
		line[1].color = sf::Color::White;
		sf::CircleShape circleShape(circleRigidBody.radius);
		circleShape.setFillColor(sf::Color(circleRigidBody.color.r, circleRigidBody.color.g, circleRigidBody.color.b, 255));
		circleShape.setPosition(circleRigidBody.position.x, circleRigidBody.position.y);
		circleShape.setOrigin(circleRigidBody.radius, circleRigidBody.radius);

		window.draw(circleShape);
		window.draw(line);
	}

	void renderContactPoint(sf::RenderWindow& window, glm::vec2 contactPoint) {
		sf::RectangleShape rectShape;
		rectShape.setSize(sf::Vector2f(5, 5));
		rectShape.setFillColor(sf::Color::Red);
		rectShape.setOrigin(2.5, 2.5);
		rectShape.setPosition(contactPoint.x, contactPoint.y);
		window.draw(rectShape);
	}

	void renderJoint(sf::RenderWindow& window, Joint * j) {
		RigidBody* b1 = j->body1;
		RigidBody* b2 = j->body2;

		glm::mat2 r1 = rotationMatrix(b1->angle);
		glm::mat2 r2 = rotationMatrix(b2->angle);

		glm::vec2 x1 = b1->position;
		glm::vec2 p1 = x1 + j->localAnchorPoint1 * r1;
		glm::vec2 x2 = b2->position;
		glm::vec2 p2 = x2 + j->localAnchorPoint2 * r2;

		sf::VertexArray line(sf::Lines, 2);

		line[0].position = { x1.x, x1.y };
		line[0].color = sf::Color::White;
		line[1].position = { x2.x, x2.y };
		line[1].color = sf::Color::White;

		window.draw(line);
	}

	void renderAABB(sf::RenderWindow& window, RigidBody* body) {
		glm::vec2 pos = body->aabb.position;
		glm::vec2 width = body->aabb.size;
		sf::RectangleShape rectShape({width.x, width.y});
		rectShape.setPosition({ pos.x,pos.y });
		rectShape.setFillColor(sf::Color(255, 0, 0, 128));
		window.draw(rectShape);
	}
};
#endif