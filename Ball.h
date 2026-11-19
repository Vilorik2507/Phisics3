#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include "PhysicsGroup.h"
class Ball {
private:
	sf::Vector2f position;
	sf::Vector2f velocity;
	PhysicsGroup group = PhysicsGroup::Normal;
	float radius;
	float mass;
public:
	Ball(sf::Vector2f firstPosition, sf::Vector2f firstVelocity, float firstRadius, float firstMass);
	void update(float dt);
	sf::Vector2f getPosition() const;
	sf::Vector2f getVelocity() const;
	float getRadius() const;
	float getMass() const;

	void setPosition(sf::Vector2f newPosition);
	void setVelocity(sf::Vector2f newVelocity);

	void draw(sf::RenderWindow& window) const;

	void setGroup(PhysicsGroup newGroup)
	{
		group = newGroup;
	}

	PhysicsGroup getGroup() const
	{
		return group;
	}
};