#include "Collision.h"

#include <cmath>
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