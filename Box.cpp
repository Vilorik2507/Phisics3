
#include "Box.h"

Box::Box(
    sf::Vector2f startPosition,
    sf::Vector2f startSize,
    sf::Vector2f startVelocity,
    float startMass
)
{
    position = startPosition;
    size = startSize;
    velocity = startVelocity;
    mass = startMass;
}

void Box::update(float dt)
{
    position += velocity * dt;
}

void Box::draw(sf::RenderWindow& window)
{
    sf::RectangleShape rectangle(size);

    rectangle.setOrigin(size.x / 2.f, size.y / 2.f);
    rectangle.setPosition(position);
    rectangle.setFillColor(sf::Color(220, 150, 60));

    window.draw(rectangle);
}

sf::Vector2f Box::getPosition() const
{
    return position;
}

sf::Vector2f Box::getSize() const
{
    return size;
}

sf::Vector2f Box::getVelocity() const
{
    return velocity;
}

float Box::getMass() const
{
    return mass;
}

void Box::setPosition(sf::Vector2f newPosition)
{
    position = newPosition;
}

void Box::setVelocity(sf::Vector2f newVelocity)
{
    velocity = newVelocity;
}
