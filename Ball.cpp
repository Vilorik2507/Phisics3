#include "Ball.h"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>

Ball::Ball(sf::Vector2f firstPosition, sf::Vector2f firstVelocity, float firstRadius, float firstMass) {
	position = firstPosition;
	velocity = firstVelocity;
	radius = firstRadius;
	mass = firstMass;
}

void Ball::update(float dt) {
	position += velocity * dt;
}
sf::Vector2f Ball::getPosition() const {
	return position;
}
sf::Vector2f Ball::getVelocity() const {
	return velocity;
}
float Ball::getRadius() const {
	return radius;
}
float Ball::getMass() const {
	return mass;
}

void Ball::setPosition(sf::Vector2f newPosition) {
	position = newPosition;
}
void Ball::setVelocity(sf::Vector2f newVelocity) {
	velocity = newVelocity;
}

void Ball::draw(sf::RenderWindow& window) const {
	sf::CircleShape circle(radius);
	circle.setPosition(position.x - radius, position.y - radius);
	circle.setFillColor(sf::Color::Cyan);
	window.draw(circle);
	
}