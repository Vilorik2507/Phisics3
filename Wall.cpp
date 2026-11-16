#include "Wall.h"

#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/Color.hpp>

Wall::Wall(sf::Vector2f firstPosition, sf::Vector2f wallSize) {
	position = firstPosition;
	size = wallSize;
}
void Wall::draw(sf::RenderWindow& window) const {
	sf::RectangleShape rectangle(size);
	rectangle.setPosition(position);
	rectangle.setFillColor(sf::Color::White);
	window.draw(rectangle);
}
sf::Vector2f Wall::getPosition() const {
	return position;
}
sf::Vector2f Wall::getSize() const {
	return size;
}