#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
class Ball {
private:
	sf::Vector2f position;
	sf::Vector2f velocity;
	float radius;
	float mass;
public:
	Ball(sf::Vector2f firstPosition, sf::Vector2f firstVelocity, float firstRadius, float firstMass);
	void update(float dt);
	sf::Vector2f getPosition() const;
	sf::Vector2f getVelocity() const;
	float getRadius() const;
	float getMass() const;

	void draw(sf::RenderWindow& window) const;
};