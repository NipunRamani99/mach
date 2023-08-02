#pragma once
#include <cmath>
#include <random>
static const size_t SCREEN_WIDTH = 1920;
static const size_t SCREEN_HEIGHT = 1080;
static const float PI = 3.141592653f;
static std::random_device rd;

float getRandomAngle() {
	return (rand() % 360) * PI / 180.0f;
}

float getRandomAngularVelocity() {
	return -1.0f + 2.0f * (rand() % 2);
}

glm::vec2 getRandomVelocity() {
	float x = 0.0f;
	float y = 0.0f;
	while (x == 0.0f) {
		x = -10.0f + (rand() % 20);
	}
	while (y == 0.0f) {
		y = -10.0f + (rand() % 20);
	}
	return glm::vec2(x, y);
}

float getRandomMass() {
	return 1.0f + 10.0f * (rand() % 10);
}

float getRandomRadius() {
	return 10.0f + 2.0f * (rand() % 10);
}

glm::vec2 getRandomPosition() {

	float x = 100.0f + (rand() % (int)(SCREEN_WIDTH - 200.0f));
	float y = 100.0f + (rand() % (int)(SCREEN_HEIGHT - 600.0f));
	return glm::vec2(x, y);
}

glm::vec2 getRandomSize() {
	float size = 50.0f + 10.0f * (rand() % 3);
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