#include "Collision.h"


#include <cmath>
#include <algorithm>

Collision checkCollision(const Ball& a, const Ball& b) {
	Collision result{ false, sf::Vector2f(0.f,0.f), 0.f };
	sf::Vector2f difference = b.getPosition() - a.getPosition();
	float distanceSquared = difference.x * difference.x + difference.y * difference.y;
	float radiusSum = a.getRadius() + b.getRadius();
	if (distanceSquared >= radiusSum * radiusSum) {//^2
		return result;
	}
	float distance = std::sqrt(distanceSquared);
	result.isColliding = true;
	if (distance > 0.0010f) {
		result.normal = difference / distance;
	}
	else {
		result.normal = sf::Vector2f(1.f, 0.f);
	}
	result.penetration = radiusSum - distance;

	return result;
}
void resolveCollision(Ball& a,Ball& b,float restitution)
{
    Collision collision = checkCollision(a, b);

    if (!collision.isColliding)
    {
        return;
    }

    sf::Vector2f normal = collision.normal;

    sf::Vector2f relativeVelocity =
        b.getVelocity() - a.getVelocity();

    float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal > 0.f)
    {
        return;
    }

    float inverseMassA = 1.f / a.getMass();
    float inverseMassB = 1.f / b.getMass();

    float impulseMagnitude =
        -(1.f + restitution) * velocityAlongNormal /
        (inverseMassA + inverseMassB);

    sf::Vector2f impulse = normal * impulseMagnitude;

    a.setVelocity(
        a.getVelocity() - impulse * inverseMassA
    );

    b.setVelocity(
        b.getVelocity() + impulse * inverseMassB
    );

    float correctionPercent = 0.8f;
    float correctionSlop = 0.01f;

    float correctionMagnitude =
        std::max(
            collision.penetration - correctionSlop,
            0.f
        ) /
        (inverseMassA + inverseMassB) *
        correctionPercent;

    sf::Vector2f correction =
        normal * correctionMagnitude;

    a.setPosition(
        a.getPosition() - correction * inverseMassA
    );

    b.setPosition(
        b.getPosition() + correction * inverseMassB
    );
}

void resolveBallWallCollision(
    Ball& ball,
    const Wall& wall,
    float restitution
)
{
    sf::Vector2f center = ball.getPosition();
    sf::Vector2f wallPosition = wall.getPosition();
    sf::Vector2f wallSize = wall.getSize();

    float radius = ball.getRadius();

    // Ближайшая к центру шарика точка стены
    float closestX = std::clamp(
        center.x,
        wallPosition.x,
        wallPosition.x + wallSize.x
    );

    float closestY = std::clamp(
        center.y,
        wallPosition.y,
        wallPosition.y + wallSize.y
    );

    sf::Vector2f difference(
        center.x - closestX,
        center.y - closestY
    );

    float distanceSquared =
        difference.x * difference.x +
        difference.y * difference.y;

    if (distanceSquared >= radius * radius)
    {
        return;
    }

    float distance = std::sqrt(distanceSquared);

    sf::Vector2f normal;
    float penetration;

    if (distance > 0.0001f)
    {
        normal = difference / distance;
        penetration = radius - distance;
    }
    else
    {
        // Центр шарика находится внутри прямоугольника.
        // Выбираем ближайшую сторону стены.

        float left = center.x - wallPosition.x;
        float right = wallPosition.x + wallSize.x - center.x;
        float top = center.y - wallPosition.y;
        float bottom = wallPosition.y + wallSize.y - center.y;

        float minimum = std::min(
            std::min(left, right),
            std::min(top, bottom)
        );

        if (minimum == left)
            normal = sf::Vector2f(-1.f, 0.f);
        else if (minimum == right)
            normal = sf::Vector2f(1.f, 0.f);
        else if (minimum == top)
            normal = sf::Vector2f(0.f, -1.f);
        else
            normal = sf::Vector2f(0.f, 1.f);

        penetration = radius + minimum;
    }

    // Исправляем положение шарика
    ball.setPosition(
        ball.getPosition() + normal * penetration
    );

    // Рассчитываем изменение скорости
    sf::Vector2f velocity = ball.getVelocity();

    float velocityAlongNormal =
        velocity.x * normal.x +
        velocity.y * normal.y;

    // Если шарик уже движется от стены,
    // отражать его скорость не нужно.
    if (velocityAlongNormal < 0.f)
    {
        velocity = velocity -
            normal * ((1.f + restitution) * velocityAlongNormal);

        ball.setVelocity(velocity);
    }
}
