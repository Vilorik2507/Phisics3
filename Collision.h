#pragma once
#include "Ball.h"
#include "Wall.h"

struct Collision {
	bool isColliding;
	sf::Vector2f normal;
	float penetration;
};
Collision checkCollision(const Ball& a, const Ball& b);
void resolveCollision(Ball& a, Ball& b, float restitution);
void resolveBallWallCollision(Ball& ball, const Wall& wall, float restitution);