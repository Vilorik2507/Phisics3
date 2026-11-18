
#pragma once

#include "Ball.h"
#include "Wall.h"

#include <vector>
#include <SFML/Graphics/RenderWindow.hpp>

class World
{
private:
    std::vector<Ball> balls;
    std::vector<Wall> walls;

public:
    void addBall(const Ball& ball);
    void addWall(const Wall& wall);

    void update(float dt, float restitution);

    void draw(sf::RenderWindow& window) const;
    void updateWithCCD(float dt, float restitution);
    void updateWithFullCCD(float dt, float restitution);
};
