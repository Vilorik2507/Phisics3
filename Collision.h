#pragma once
#include "Ball.h"
#include "Wall.h"
#include "Box.h"
#include <vector>

struct Collision {
	bool isColliding;
	sf::Vector2f normal;
	float penetration;
};
Collision checkCollision(const Ball& a, const Ball& b);
void resolveCollision(Ball& a, Ball& b, float restitution);
void resolveBallWallCollision(Ball& ball, const Wall& wall, float restitution);
void resolveBoxWallCollision(Box& box, const Wall& wall, float restitution);
void resolveBoxBoxCollision(Box& a, Box& b, float restitution);
void updateBoxWithWallCCD(Box& box, const std::vector<Wall>& walls, float dt, float restitution);