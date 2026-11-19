
#pragma once

#include <SFML/System/Vector2.hpp>

#include "Ball.h"
#include "Box.h"
#include "Wall.h"

#include <vector>

// =====================================================
// COLLISION DATA
// =====================================================

struct Collision
{
    bool isColliding = false;
    sf::Vector2f normal = { 0.f, 0.f };
    float penetration = 0.f;
};

struct BoxSweepResult
{
    bool hit = false;
    float time = 0.f;
    sf::Vector2f normal = { 0.f, 0.f };
};

struct BallSweepResult
{
    bool hit = false;
    float time = 0.f;
    sf::Vector2f normal = { 0.f, 0.f };
};

struct BoxWallSweepResult
{
    bool hit = false;
    float time = 0.f;
    sf::Vector2f normal = { 0.f, 0.f };
};

// =====================================================
// BALL - BALL
// =====================================================

Collision checkCollision(
    const Ball& a,
    const Ball& b
);

void resolveCollision(
    Ball& a,
    Ball& b,
    float restitution
);

// =====================================================
// BALL - WALL
// =====================================================

void resolveBallWallCollision(
    Ball& ball,
    const Wall& wall,
    float restitution
);

// =====================================================
// BOX - WALL
// =====================================================

Collision checkBoxWallCollision(
    const Box& box,
    const Wall& wall
);

void resolveBoxWallCollision(
    Box& box,
    const Wall& wall,
    float restitution
);

// =====================================================
// BOX - BOX
// =====================================================

Collision checkBoxBoxCollision(
    const Box& a,
    const Box& b
);

void resolveBoxBoxCollision(
    Box& a,
    Box& b,
    float restitution
);

// =====================================================
// COLLISION SOLVER HELPERS
// =====================================================

void correctBoxBoxPenetration(
    Box& a,
    Box& b,
    const Collision& collision
);

void correctBoxWallPenetration(
    Box& box,
    const Collision& collision
);

void resolveBoxBoxImpact(
    Box& a,
    Box& b,
    sf::Vector2f normal,
    float restitution
);

void resolveBoxWallImpact(
    Box& box,
    sf::Vector2f normal,
    float restitution
);

// =====================================================
// CCD: BOX - BOX
// =====================================================

BoxSweepResult checkBoxBoxCCD(
    const Box& a,
    const Box& b,
    float dt
);

// =====================================================
// CCD: BOX - WALL
// =====================================================

BoxWallSweepResult checkBoxWallCCD(
    const Box& box,
    const Wall& wall,
    float dt
);

BallSweepResult checkBallBallCCD(
    const Ball& a,
    const Ball& b,
    float dt
);

void resolveBallBallImpact(
    Ball& a,
    Ball& b,
    sf::Vector2f normal,
    float restitution
);

// =====================================================
// ITERATIVE BOX SOLVER
// =====================================================

void solveBoxCollisions(
    std::vector<Box>& boxes,
    const std::vector<Wall>& walls,
    float restitution,
    int iterations
);
