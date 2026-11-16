#pragma once
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
class Wall {
private:
	sf::Vector2f position;
	sf::Vector2f size;
public:
	Wall(sf::Vector2f firstPosition, sf::Vector2f wallSize);
	void draw(sf::RenderWindow& window) const;
	sf::Vector2f getPosition() const;
	sf::Vector2f getSize() const;
};