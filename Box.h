
#pragma once

#include <SFML/Graphics.hpp>
#include "PhysicsGroup.h"

class Box
{
private:
    sf::Vector2f position;
    sf::Vector2f size;
    sf::Vector2f velocity;
    PhysicsGroup group = PhysicsGroup::Normal;

    float mass;

public:
    Box(
        sf::Vector2f startPosition,
        sf::Vector2f startSize,
        sf::Vector2f startVelocity,
        float startMass
    );

    void update(float dt);
    void draw(sf::RenderWindow& window) const;

    sf::Vector2f getPosition() const;
    sf::Vector2f getSize() const;
    sf::Vector2f getVelocity() const;

    float getMass() const;

    void setPosition(sf::Vector2f newPosition);
    void setVelocity(sf::Vector2f newVelocity);

    void setGroup(PhysicsGroup newGroup)
    {
        group = newGroup;
    }

    PhysicsGroup getGroup() const
    {
        return group;
    }
};
