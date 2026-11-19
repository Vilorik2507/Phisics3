
#pragma once

#include "Ball.h"
#include "Wall.h"
#include "Box.h"
#include "MultiRate.h"

#include <vector>
#include <SFML/Graphics/RenderWindow.hpp>

class World
{
private:
    std::vector<Ball> balls;
    std::vector<Wall> walls;
    std::vector<Box> boxes;
    MultiRate multiRate;
    void updatePhysicsStep(
        float dt,
        float restitution,
        PhysicsGroup group,
        bool updateVelocity,
        bool updatePosition
    );

public:
    void addBall(const Ball& ball);
    void addWall(const Wall& wall);
    void addBox(const Box& box);

    void update(float dt, float restitution);

    void draw(sf::RenderWindow& window) const;
    void updateWithCCD(float dt, float restitution);
    void updateWithFullCCD(float dt, float restitution);
    void updateMultiRate(float frameDt, float restitution);

};
