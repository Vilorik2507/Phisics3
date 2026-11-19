
#pragma once

#include <vector>

#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>

#include "Ball.h"
#include "Box.h"
#include "Wall.h"
#include "PhysicsGroup.h"
#include "MultiRate.h"

class World
{
private:
    std::vector<Ball> balls;
    std::vector<Wall> walls;
    std::vector<Box> boxes;

    MultiRate multiRate;

    sf::Vector2f gravity = { 0.f, 980.f };

    // Теневое физическое состояние.
    // Используется для общего CCD Normal и Fast.
    std::vector<Ball> shadowBalls;
    std::vector<Box> shadowBoxes;

    bool shadowInitialized = false;

    void applyGravity(PhysicsGroup group, float dt);
    void updateGroup(PhysicsGroup group, float dt);

    void solveBallCollisions(
        float restitution,
        int iterations
    );

    void solveBoxCollisions(
        float restitution,
        int iterations
    );

    void synchronizeBodies(
        float dt,
        float restitution
    );

    void initializeShadowState();

    void updateShadowState(
        float fastDt,
        float restitution
    );

    void copyFastBodiesFromShadow();
    void copyNormalBodiesFromShadow();

public:
    void addBall(const Ball& ball);
    void addWall(const Wall& wall);
    void addBox(const Box& box);

    void update(float dt, float restitution);

    void updateWithCCD(
        float dt,
        float restitution
    );

    void updateWithFullCCD(
        float dt,
        float restitution
    );

    void updateMultiRate(
        float frameDt,
        float restitution
    );

    void draw(sf::RenderWindow& window) const;

    int getFastStepsThisFrame() const;
    int getFastStepsTotal() const;
    int getNormalStepsTotal() const;
};
