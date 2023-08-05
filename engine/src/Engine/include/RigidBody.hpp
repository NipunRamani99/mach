#ifndef __RIGID_BODY_HPP__
#define __RIGID_BODY_HPP__
#include <glm/glm.hpp>
#include <vector>
#include "AABB.hpp"
struct PolygonData {
	std::vector<glm::vec2> vertices;
	std::vector<glm::vec2> normals;
};
class RigidBody
{
private:
public:
	enum Type {
		BOX,
		CIRCLE
	};
	PolygonData polygonData;

	glm::vec2 position = { 0.0f, 0.0f };
	glm::vec2 linear_velocity = { 0.0f, 0.0f };
	glm::vec3 color = { 1.0f,1.0f,1.0f };
	glm::vec2 force = { 0.0f,0.0f };
	float mass = 0.0f;
	float inv_mass = 0.0f;
	float restitution = 0.0f;
	float inertia = 0.0f;
	float inv_inertia = 0.0f;
	float angle = 0.0f;
	float torque = 0.0f;
	float angular_velocity = 0.0f;
	float static_friction = 0.1f;
	float dynamic_friction = 0.1f;
	bool is_static = false;
	uint32_t id = 0;
	uint32_t groupId = -1;
	Type type = BOX;
	AABB aabb;
	
	virtual ~RigidBody() {}

	inline void step(float dt) {
		updateVelocity(dt); 
		updateAngularVelocity(dt);
	}
	
	inline void updateVelocity(float dt) {
		if (is_static) {
			linear_velocity = { 0, 0 };
			return;
		}
		linear_velocity += ( force * inv_mass )*dt;
	
	}

	inline void updateAngularVelocity(float dt) {
		if (is_static) {
			angular_velocity = 0;
			return;
		}
		angular_velocity += inv_inertia*torque * dt;
	}

	inline void applyForce(float dt) {
		updateVelocity(dt);
		updateAngularVelocity(dt);
	}

	inline void integrate(float dt) {
		if (is_static) return;
		position += linear_velocity * dt;
		angle += angular_velocity * dt;
		torque = 0.0f;
		force = { 0.0f,0.0f };
	}

	float getAngularVelocity() {
		if (is_static) return 0.0f;
		else
			return angular_velocity;
	}

	glm::vec2 getVelocity() {
		if (is_static) return { 0.0f,0.0f };
		return linear_velocity;
	}

	void setVelocity(glm::vec2 vel) {
		linear_velocity = vel;
	}

	void move(glm::vec2 velocity) {
		this->position += velocity;
	}

	void moveTo(glm::vec2 pos) {
		this->position = pos;
	}

	void accelerate(glm::vec2 acceleration) {
		if (is_static) return;
		force += acceleration * mass;
	}

	virtual void calculateInertia() = 0;

	virtual void calculateAABB() = 0;

	virtual bool checkIfInside(glm::vec2 p) = 0;
};
#endif // !__RIGID_BODY_HPP__
